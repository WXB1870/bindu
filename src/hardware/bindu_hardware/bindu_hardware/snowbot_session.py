"""Independent position-session contract. No ROS, devices or implicit renewal."""
import math
import time

from .snowbot import GROUPS, SLOT_MASKS

ARMS = frozenset(('left_arm', 'right_arm'))
CONTROL_GROUPS = ARMS | {'waist'}
# Independent wire order; QP/resource order is mapped by joint name.
CONTROL_NAMES = (*GROUPS['left_arm'], *GROUPS['right_arm'], *GROUPS['waist'])
CONTROL_MASK = sum(SLOT_MASKS[g] for g in CONTROL_GROUPS)
# ROS delivery can lag the local PDO sampler. Keep these budgets independent.
ROS_FEEDBACK_TIMEOUT_S = .04
PDO_MAX_AGE_S = .02
SLOTS = [*(f'arm_l_joint_{i}' for i in range(1, 9)),
         *(f'arm_r_joint_{i}' for i in range(1, 9)),
         'head_joint_1', 'head_joint_2', 'body_joint_1', 'body_joint_2']


def decode_observation(msg, now):
    if msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 1_000_000_000:
        raise ValueError('SNOWBOT_OBSERVATION_STAMP')
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if (not msg.boot_id or not msg.pdo_sample or
            not all(math.isfinite(x) for x in (stamp, msg.pdo_age, now)) or
            not 0 <= now-stamp <= ROS_FEEDBACK_TIMEOUT_S or
            not 0 <= msg.pdo_age <= PDO_MAX_AGE_S or
            now-stamp+msg.pdo_age > ROS_FEEDBACK_TIMEOUT_S):
        raise ValueError('SNOWBOT_PDO_STALE')
    if msg.expected_wkc <= 0 or msg.wkc != msg.expected_wkc:
        raise ValueError('SNOWBOT_PDO_WKC')
    if any(len(getattr(msg, field)) != 20 for field in ('position', 'speed', 'current', 'status')):
        raise ValueError('SNOWBOT_OBSERVATION_LAYOUT')
    q, v = {}, {}
    for i, name in enumerate(SLOTS):
        if msg.valid_mask & (1 << i):
            if not all(math.isfinite(x) for x in (msg.position[i], msg.speed[i], msg.current[i])):
                raise ValueError('SNOWBOT_OBSERVATION_NONFINITE')
            q[name], v[name] = msg.position[i], msg.speed[i]
    return q, v, stamp-msg.pdo_age


class SessionHealth:
    def __init__(self, now, steady=time.monotonic, *, execution_mode="position", reference_mode=0):
        if execution_mode not in ("position", "impedance"):
            raise ValueError("SNOWBOT_EXECUTION_MODE")
        self.execution_mode = execution_mode
        if reference_mode not in (0, 2):
            raise ValueError('SNOWBOT_REFERENCE_MODE')
        if reference_mode and execution_mode != 'position':
            raise ValueError('SNOWBOT_TARGET_REQUIRES_PURE_POSITION')
        self.reference_mode = reference_mode
        self.now, self.steady = now, steady
        self.states, self.received = {}, {}
        self.observation = None
        self.boot = self.sample = 0
        self.progress = None
        self.advances = 0
        self.invalid = ''
        self.messages = {'servo': {}}

    def observe(self, msg):
        try:
            result = decode_observation(msg, self.now())
            if self.boot and msg.boot_id != self.boot:
                raise ValueError('SNOWBOT_SERVO_RESTART_RECONNECT_REQUIRED')
            if msg.pdo_sample < self.sample:
                raise ValueError('SNOWBOT_PDO_SEQUENCE')
            if msg.pdo_sample > self.sample:
                self.advances = (min(2, self.advances+1) if self.progress is not None and
                    0 <= self.steady()-self.progress <= ROS_FEEDBACK_TIMEOUT_S else 1)
                self.progress = self.steady()
            self.boot, self.sample = msg.boot_id, msg.pdo_sample
            self.observation, self.received['observation'] = msg, self.steady()
            if (msg.valid_mask & CONTROL_MASK != CONTROL_MASK or
                    any(status & 8 for i, status in enumerate(msg.status) if CONTROL_MASK & (1 << i))):
                self.advances = 0
            self.invalid = ''
            self.messages['servo'] = dict(operation_enabled_mask=sum(
                1 << i for i, status in enumerate(msg.status) if status & 4))
            return result
        except ValueError as exc:
            self.advances = 0
            self.invalid = str(exc)
            raise

    def state(self, layer, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (msg.layer != layer or not math.isfinite(stamp) or not 0 <= self.now()-stamp <= .2 or
                stamp <= self.states.get(layer, (0., None))[0]):
            raise ValueError('SNOWBOT_SESSION_STATE_INVALID:'+layer)
        self.states[layer] = stamp, msg
        self.received[layer] = self.steady()

    def confirmed_hold(self, request):
        """Return the adopted fixed reference, never physical-stop confirmation.

        A trajectory impedance PAUSE adopts a fixed nominal reference and keeps
        its controller. Executor still requires measured position and rest.
        """
        if request is None:
            return None
        try:
            self.require(CONTROL_GROUPS, acquiring=True)
        except RuntimeError:
            return None
        obs = self.observation
        session, sequence, boot, baseline = request
        if (obs.boot_id != boot or not obs.session_selected or obs.session != session or
                obs.command_sequence != sequence or obs.session_reason not in (1, 2) or
                not getattr(obs, 'hold_valid', False) or
                not baseline <= obs.hold_sample < obs.pdo_sample):
            return None
        if self.reference_mode and (getattr(obs, 'reference_mode', 0) != 2 or
                getattr(obs, 'segment_id', 0) != sequence or obs.session_reason != 1 or
                getattr(obs, 'trajectory_phase', 0) != 4 or getattr(obs, 'hold_basis', -1) != 0 or
                not getattr(obs, 'reference_valid', False) or not getattr(obs, 'output_valid', False)):
            # Expiry/fault/revocation is not a successful PAUSE. The executor's
            # original stop budget and measured-rest checks remain authoritative.
            return None
        values = obs.hold_position
        if len(values) != len(CONTROL_NAMES) or not all(math.isfinite(q) for q in values):
            return None
        return dict(zip(CONTROL_NAMES, values))

    def require(self, groups, *, acquiring=False):
        if set(groups) != CONTROL_GROUPS:
            raise RuntimeError('SNOWBOT_POSITION_SESSION_RESOURCE_LAYOUT')
        if self.invalid:
            raise RuntimeError(self.invalid)
        obs = self.observation
        if obs is not None and any(status & 8 for i, status in enumerate(obs.status) if CONTROL_MASK & (1 << i)):
            raise RuntimeError('SNOWBOT_DRIVE_FAULT')
        if (obs is None or self.progress is None or self.advances < 2 or
                not 0 <= self.steady()-self.progress <= ROS_FEEDBACK_TIMEOUT_S or
                not 0 <= self.steady()-self.received['observation'] <= ROS_FEEDBACK_TIMEOUT_S):
            raise RuntimeError('SNOWBOT_PDO_NOT_ADVANCING')
        try:
            decode_observation(obs, self.now())
        except ValueError as exc:
            raise RuntimeError(str(exc)) from exc
        mask = sum(SLOT_MASKS[g] for g in groups)
        if obs.valid_mask & mask != mask:
            raise RuntimeError('SNOWBOT_POSITION_FEEDBACK_INVALID')
        if any(status & 8 for i, status in enumerate(obs.status) if mask & (1 << i)):
            raise RuntimeError('SNOWBOT_DRIVE_FAULT')
        for layer in ('upper', 'middle', 'servo'):
            stamp, state = self.states.get(layer, (0., None))
            if (state is None or not 0 <= self.now()-stamp <= .2 or
                    not 0 <= self.steady()-self.received[layer] <= .2 or not state.startup_complete or
                    not state.feedback_ready or not state.instance_id):
                raise RuntimeError('SNOWBOT_SESSION_LAYER_UNAVAILABLE:'+layer)
            if layer == 'upper' and state.control_source != 'topic':
                raise RuntimeError('SNOWBOT_CONTROL_SOURCE_NOT_TOPIC')
            if layer == 'servo' and state.stop_policy not in ('disable_no_load', 'measured_hold'):
                raise RuntimeError('SNOWBOT_STOP_POLICY_UNCONFIRMED')
            if layer == 'servo' and state.control_source != self.execution_mode:
                raise RuntimeError('SNOWBOT_EXECUTION_MODE_CONFLICT:'+self.execution_mode)
            if self.reference_mode and (getattr(state, 'wire_version', 0) != 4 or
                    layer == 'servo' and getattr(state, 'reference_mode', 0) != self.reference_mode):
                raise RuntimeError('SNOWBOT_TRAJECTORY_PROTOCOL_CONFLICT:'+layer)
        if self.execution_mode == 'impedance':
            # The optional vendor build places force data in this exact PDO sample.
            wrench = getattr(obs, 'wrench', ())
            age = getattr(obs, 'sensor_age', ())
            stamp = obs.header.stamp.sec+obs.header.stamp.nanosec*1e-9
            if (getattr(obs, 'sensor_valid_mask', 0) != 3 or len(wrench) != 12 or
                    obs.valid_mask != 0xfffff or not all(math.isfinite(x) for x in wrench) or
                    len(age) != 2 or not all(math.isfinite(a) and 0 <= a and
                        self.now()-stamp+a <= .02 for a in age)):
                raise RuntimeError('SNOWBOT_IMPEDANCE_SENSOR_INVALID')
        if obs.session_reason >= 2 and not acquiring:
            raise RuntimeError('SNOWBOT_SESSION_STOPPED:'+str(obs.session_reason))


class SourceEnvelope:
    """One envelope per accepted Motion; executor ticks cannot renew a target."""
    def __init__(self, session, now, *, target_stream=False):
        self.session, self.now = session, now
        # A new PAUSE authorizes only braking. Its original deadline is fixed
        # here; ordinary motion and legacy/impedance TTLs remain 200 ms.
        self.pause_valid_for = 2. if target_stream else .2
        self.sequence = 0
        self.last_id = None
        self.takeover = True

    def command(self, motion, *, pause=False):
        if not pause and motion is None:
            raise RuntimeError('SNOWBOT_SOURCE_MOTION_REQUIRED')
        if not pause and motion.command_id == self.last_id:
            return None
        source = self.now() if pause else motion.stamp
        expires = source + (self.pause_valid_for if pause else min(.2, motion.valid_for))
        if not source <= self.now() < expires:
            raise RuntimeError('SNOWBOT_SOURCE_COMMAND_EXPIRED')
        self.sequence += 1
        operation = 2 if pause else 1 if self.takeover else 0
        self.takeover = pause
        if not pause:
            self.last_id = motion.command_id
        source_ns = round(source*1e9)
        ttl_ns = (round(self.pause_valid_for*1e9) if pause else
                  min(200_000_000, round(motion.valid_for*1e9)))
        return dict(session=self.session, sequence=self.sequence,
                    source_stamp_ns=source_ns, expires_at_ns=source_ns+ttl_ns,
                    operation=operation)
