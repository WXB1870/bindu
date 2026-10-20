"""Snowbot position/feedback boundary; no ROS or trajectory generation.

Layout follows the user supplied src.zip (2026-10-13). J8 is a gripper,
read as context only. Arm packets always carry both arms, so their ownership
must be acquired together. No base/odometry interface is inferred.
"""
import math
import json
import time

from bindu_contracts.contracts import Feedback


GROUPS = {
    'waist': ['body_joint_1', 'body_joint_2'],
    'head': ['head_joint_1', 'head_joint_2'],
    'left_arm': [f'arm_l_joint_{i}' for i in range(1, 8)],
    'right_arm': [f'arm_r_joint_{i}' for i in range(1, 8)],
    'left_tool_axis': ['arm_l_joint_8'],
    'right_tool_axis': ['arm_r_joint_8'],
}
WRITABLE = frozenset(('waist', 'head', 'left_arm', 'right_arm'))
SLOT_MASKS = {'left_arm': 0x7f, 'right_arm': 0x7f00, 'head': 0x30000, 'waist': 0xc0000}
SLOT_MASKS.update(left_tool_axis=0x80, right_tool_axis=0x8000)
FEEDBACK_LAYOUT = (('l', ('left_arm', 'left_tool_axis')),
                   ('r', ('right_arm', 'right_tool_axis')),
                   ('torso', ('waist',)), ('head', ('head',)))


def feedback_schema(fields):
    """Inspect installed ROS fields; optional head/waist are never synthesized."""
    errors, available = [], []
    if fields.get('header') != 'std_msgs/Header':
        errors.append('header:std_msgs/Header')
    for prefix, groups in FEEDBACK_LAYOUT:
        size = sum(len(GROUPS[g]) for g in groups)
        required = ('position', 'speed', 'iq') if prefix in ('l', 'r') else ('position', 'speed')
        if prefix not in ('l', 'r') and not any(prefix+'_'+k in fields for k in required):
            continue
        for kind in required:
            name = prefix+'_'+kind
            if fields.get(name) not in (f'double[{size}]', f'float64[{size}]'):
                errors.append(name+f':float64[{size}]')
        if not any(prefix+'_'+kind+':' in e for e in errors for kind in required):
            available.extend(groups)
    return dict(compatible=not errors, errors=errors, available_groups=available,
                missing_groups=[g for g in GROUPS if g not in available])


class VendorHealth:
    """Separate source acknowledgment and successful PDO progress from ROS liveness."""
    def __init__(self, now, steady=time.monotonic, max_age=.2, expected_session=None):
        self.now, self.steady, self.max_age = now, steady, max_age
        self.expected_session = expected_session
        self.messages, self.received = {}, {}
        self.errors = {}
        self.sample = 0
        self.progress_at = None

    def receive(self, channel, payload):
        try:
            msg = json.loads(payload)
            if not isinstance(msg, dict) or type(msg.get('schema')) is not int or msg['schema'] != 1:
                raise ValueError('SCHEMA')
            stamp = msg.get('stamp')
            if (type(stamp) not in (float, int) or not math.isfinite(stamp) or
                    not 0 <= self.now()-stamp <= self.max_age or
                    stamp <= self.messages.get(channel, {}).get('stamp', 0)):
                raise ValueError('STAMP')
            if channel == 'source':
                if msg.get('source') not in ('topic', 'demo', 'vr'):
                    raise ValueError('SOURCE')
            elif channel == 'servo':
                for key in ('operational', 'ready', 'guard_enabled', 'guard_armed', 'guard_latched'):
                    if type(msg.get(key)) is not bool:
                        raise ValueError('BOOL:'+key)
                for key in ('sample', 'wkc', 'expected_wkc', 'fault_mask', 'present_mask',
                            'enable_requested_mask', 'operation_enabled_mask', 'guard_session'):
                    if type(msg.get(key)) is not int or msg[key] < 0:
                        raise ValueError('INT:'+key)
                age = msg.get('sample_age')
                if type(age) not in (int, float) or not math.isfinite(age) or age < 0:
                    raise ValueError('PDO_AGE')
                if msg['sample'] < self.sample:
                    raise ValueError('PDO_SEQUENCE')
                if msg['sample'] > self.sample:
                    self.sample, self.progress_at = msg['sample'], self.steady()
            else:
                raise ValueError('CHANNEL')
            self.messages[channel], self.received[channel] = msg, self.steady()
            self.errors.pop(channel, None)
        except (ValueError, TypeError) as exc:
            self.errors[channel] = str(exc)
            raise ValueError('SNOWBOT_HEALTH_INVALID:'+channel+':'+str(exc)) from exc

    def require(self, groups, *, armed=True):
        channels = ('servo', 'source')
        for channel in channels:
            msg = self.messages.get(channel, {})
            if (channel in self.errors or not msg or
                    not 0 <= self.now()-msg['stamp'] <= self.max_age or
                    not 0 <= self.steady()-self.received[channel] <= self.max_age):
                raise RuntimeError('SNOWBOT_HEALTH_UNAVAILABLE:'+channel)
        servo = self.messages['servo']
        if self.messages['source']['source'] != 'topic':
            raise RuntimeError('SNOWBOT_CONTROL_SOURCE_NOT_TOPIC')
        if (self.expected_session is not None and servo['guard_armed'] and
                servo['guard_session'] != self.expected_session):
            raise RuntimeError('SNOWBOT_GUARD_SESSION_CONFLICT')
        if (not all(servo[k] for k in ('operational', 'ready', 'guard_enabled')) or
                (armed and not servo['guard_armed']) or
                servo['guard_latched'] or not servo['expected_wkc'] or
                servo['wkc'] != servo['expected_wkc'] or servo['sample_age'] > .02 or
                self.progress_at is None or self.steady()-self.progress_at > .05):
            raise RuntimeError('SNOWBOT_SERVO_NOT_READY')
        mask = sum(SLOT_MASKS[g] for g in set(groups))
        if servo['present_mask'] & mask != mask or servo['fault_mask'] & mask:
            raise RuntimeError('SNOWBOT_DRIVE_UNAVAILABLE')


def decode_feedback(msg):
    """Accept native 16-axis and extended feedback, preserving missing axes."""
    positions, velocities = {}, {}
    for prefix, groups in FEEDBACK_LAYOUT:
        names = [n for g in groups for n in GROUPS[g]]
        q, v = getattr(msg, prefix+'_position', None), getattr(msg, prefix+'_speed', None)
        if prefix in ('torso', 'head') and q is None and v is None:
            continue
        if q is None or v is None or len(q) != len(names) or len(v) != len(names):
            raise ValueError('SNOWBOT_FEEDBACK_LAYOUT')
        positions.update(zip(names, q))
        velocities.update(zip(names, v))
    stamp = msg.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('SNOWBOT_FEEDBACK_STAMP')
    return positions, velocities, stamp.sec+stamp.nanosec*1e-9


class SnowbotIO:
    def __init__(self, profile, send, now, *, command_enabled=False,
                 simulated=False, feedback_max_age=.2, check_ready=None, check_health=None):
        if (profile.groups != GROUPS or set(profile.capabilities) != {'joint_position'} or
                profile.name not in ('snowbot_vendor_position','snowbot_real_position')):
            raise ValueError('SNOWBOT_PROFILE_REQUIRED')
        self.profile, self.send, self.now = profile, send, now
        self.command_enabled, self.simulated = command_enabled, simulated
        self.feedback_max_age = feedback_max_age
        self.check_ready = check_ready
        self.check_health = check_health
        self.owned_groups = set()
        self.writable_groups = WRITABLE
        self.coupled_group_sets = (frozenset(('left_arm', 'right_arm')),)
        self.last_feedback = Feedback(0., {}, 0., 0., (0., 0.), simulated)
        self.received_at = 0.
        self.invalid = ''
        self.last_source_stamp = 0.
        self.targets = {}
        self.reference = {}
        self.stop_failures = ()
        self.stop_targets = {}
        self.stop_requested_at = 0.
        # ExecutionState expects independently dated sources. Base is absent.
        self.feedback = self
        self.source_stamps = {}

    @property
    def missing_feedback_groups(self):
        return [g for g, names in self.profile.groups.items()
                if not set(names) <= self.last_feedback.positions.keys()]

    @property
    def reference_positions(self):
        return dict(self.reference)

    def observe(self, positions, velocities, stamp, now):
        names = set(self.profile.limits)
        required = {n for prefix, groups in FEEDBACK_LAYOUT[:2] for g in groups for n in GROUPS[g]}
        if (not required <= set(positions) <= names or set(velocities) != set(positions) or
                any(set(group) & positions.keys() and not set(group) <= positions.keys()
                    for group in self.profile.groups.values()) or
                not all(math.isfinite(v) for v in (*positions.values(), *velocities.values(), stamp, now))):
            self.invalid = 'SNOWBOT_FEEDBACK_INVALID'
            raise ValueError(self.invalid)
        if stamp <= self.last_source_stamp or not 0 <= now-stamp <= self.feedback_max_age:
            self.invalid = 'SNOWBOT_FEEDBACK_STALE_OR_UNORDERED'
            raise ValueError(self.invalid)
        self.last_source_stamp, self.received_at, self.invalid = stamp, now, ''
        self.last_feedback = Feedback(stamp, dict(positions), 0., 0., (0., 0.),
                                      self.simulated, joint_velocities=dict(velocities))
        self.source_stamps = {g: stamp for g in self.profile.groups if g not in self.missing_feedback_groups}

    def require_feedback(self, groups=()):
        now = self.now()
        if (self.invalid or not self.last_source_stamp or
                not 0 <= now-self.last_source_stamp <= self.feedback_max_age or
                not 0 <= now-self.received_at <= self.feedback_max_age):
            raise RuntimeError(self.invalid or 'SNOWBOT_FEEDBACK_UNAVAILABLE')
        missing = set(groups) & set(self.missing_feedback_groups)
        if missing:
            raise RuntimeError('SNOWBOT_FEEDBACK_INCOMPLETE:'+','.join(sorted(missing)))
        if self.check_health is not None and (self.targets or self.stop_targets):
            self.check_health(self.owned_groups or self._groups(self.targets or self.stop_targets))

    def capture(self, groups):
        if not self.command_enabled:
            raise RuntimeError('SNOWBOT_COMMANDS_DISABLED')
        if self.stop_failures:
            raise RuntimeError('SNOWBOT_STOP_UNCONFIRMED_RESTART_REQUIRED')
        self.require_feedback(groups)
        if self.check_ready is not None:
            self.check_ready(groups)
        self.owned_groups = set(groups)
        self.targets.clear()
        self.reference.clear()
        self.stop_targets.clear()

    def _groups(self, target):
        selected = {g for g, names in self.profile.groups.items() if set(names) <= set(target)}
        if not selected or set(target) != {n for g in selected for n in self.profile.groups[g]}:
            raise RuntimeError('SNOWBOT_COMPLETE_GROUP_REQUIRED')
        if not selected <= WRITABLE:
            raise RuntimeError('SNOWBOT_GRIPPER_CONTEXT_ONLY')
        return selected

    def _send(self, target):
        groups = self._groups(target)
        if {'left_arm', 'right_arm'} & groups:
            # First command captures the other owned arm; never assume zero.
            for g in ('left_arm', 'right_arm'):
                for n in GROUPS[g]:
                    target.setdefault(n, self.targets.get(n, self.last_feedback.positions[n]))
            groups.update(('left_arm', 'right_arm'))
        if any(not math.isfinite(q) or not self.profile.limits[n][0] <= q <= self.profile.limits[n][1]
               for n, q in target.items()):
            raise RuntimeError('SNOWBOT_JOINT_LIMIT')
        self.send(groups, target)

    def write_joints(self, target):
        if not self.command_enabled:
            raise RuntimeError('SNOWBOT_COMMANDS_DISABLED')
        self.require_feedback(self.owned_groups or self._groups(target))
        self._groups(target)
        if any(not math.isfinite(q) or not self.profile.limits[n][0] <= q <= self.profile.limits[n][1]
               for n, q in target.items()):
            raise RuntimeError('SNOWBOT_JOINT_LIMIT')
        packet = dict(target)
        try:
            self._send(packet)
        finally:
            # A multi-topic send can fail after one packet was delivered.
            # Keep its full footprint so cancellation also covers that packet.
            self.targets.update(packet)
        self.reference.update(packet)

    def write_base(self, velocity):
        if velocity != (0., 0.):
            raise RuntimeError('SNOWBOT_BASE_UNAVAILABLE')

    def stop(self):
        self.reference.clear()
        if not self.targets:
            if self.stop_targets:
                # A hold already sent still needs fresh feedback. A subsequent
                # fault must not erase that outstanding physical stop request.
                try:
                    self.require_feedback(self._groups(self.stop_targets))
                except RuntimeError:
                    self.stop_failures = ('vendor_hold',)
            return  # Read-only/idle shutdown never publishes a motor command.
        self.stop_requested_at = self.now()
        self.stop_targets = {}
        try:
            self.require_feedback()
        except RuntimeError:
            self.stop_failures = tuple(sorted(self._groups(self.targets)))
            self.targets.clear()
            return
        involved = self._groups(self.targets)
        failures = set(self.stop_failures)
        # Normal multi-topic writes check the whole batch before sending.
        # Stops instead try every independent endpoint, even if one is lost.
        for coupled in (('left_arm', 'right_arm'), ('waist',), ('head',)):
            groups = set(coupled) & involved
            if not groups:
                continue
            try:
                self.require_feedback(groups)
                hold = {n: self.last_feedback.positions[n] for g in coupled if g in groups for n in GROUPS[g]}
                self._send(hold)
            except RuntimeError:
                failures.update(groups)
            else:
                self.stop_targets.update(hold)
        self.stop_failures = tuple(sorted(failures))
        self.targets.clear()

    def read_feedback(self, now, dt):
        if self.invalid:
            raise RuntimeError(self.invalid)
        if self.check_health is not None and (self.targets or self.stop_targets):
            self.check_health(self.owned_groups)
        return self.last_feedback
