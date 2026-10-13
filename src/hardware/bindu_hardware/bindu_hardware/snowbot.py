"""Snowbot vendor position/feedback boundary; no ROS or trajectory generation.

Layout follows the user supplied src.zip (2026-10-13). J8 is a gripper,
read as context only. Arm packets always carry both arms, so their ownership
must be acquired together. No base/odometry interface is inferred.
"""
import math

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


def decode_feedback(msg):
    """Decode RetInformation, preserving measured velocity and source stamp."""
    positions, velocities = {}, {}
    for prefix, groups in (('l', ('left_arm', 'left_tool_axis')),
                           ('r', ('right_arm', 'right_tool_axis')),
                           ('torso', ('waist',)), ('head', ('head',))):
        names = [n for g in groups for n in GROUPS[g]]
        q, v = getattr(msg, prefix+'_position'), getattr(msg, prefix+'_speed')
        if len(q) != len(names) or len(v) != len(names):
            raise ValueError('SNOWBOT_FEEDBACK_LAYOUT')
        positions.update(zip(names, q))
        velocities.update(zip(names, v))
    stamp = msg.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('SNOWBOT_FEEDBACK_STAMP')
    return positions, velocities, stamp.sec+stamp.nanosec*1e-9


class SnowbotIO:
    def __init__(self, profile, send, now, *, command_enabled=False,
                 simulated=False, feedback_max_age=.2, check_ready=None):
        if (profile.groups != GROUPS or set(profile.capabilities) != {'joint_position'} or
                profile.name != 'snowbot_vendor_position'):
            raise ValueError('SNOWBOT_PROFILE_REQUIRED')
        self.profile, self.send, self.now = profile, send, now
        self.command_enabled, self.simulated = command_enabled, simulated
        self.feedback_max_age = feedback_max_age
        self.check_ready = check_ready
        self.writable_groups = WRITABLE
        self.coupled_group_sets = (frozenset(('left_arm', 'right_arm')),)
        self.last_feedback = Feedback(0., dict.fromkeys(profile.limits, 0.),
                                      0., 0., (0., 0.), simulated)
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
        self.source_stamps = dict.fromkeys(profile.groups, 0.)

    @property
    def reference_positions(self):
        return dict(self.reference)

    def observe(self, positions, velocities, stamp, now):
        names = set(self.profile.limits)
        if (set(positions) != names or set(velocities) != names or
                not all(math.isfinite(v) for v in (*positions.values(), *velocities.values(), stamp, now))):
            self.invalid = 'SNOWBOT_FEEDBACK_INVALID'
            raise ValueError(self.invalid)
        if stamp <= self.last_source_stamp or not 0 <= now-stamp <= self.feedback_max_age:
            self.invalid = 'SNOWBOT_FEEDBACK_STALE_OR_UNORDERED'
            raise ValueError(self.invalid)
        self.last_source_stamp, self.received_at, self.invalid = stamp, now, ''
        self.last_feedback = Feedback(stamp, dict(positions), 0., 0., (0., 0.),
                                      self.simulated, joint_velocities=dict(velocities))
        self.source_stamps = dict.fromkeys(self.profile.groups, stamp)

    def require_feedback(self):
        now = self.now()
        if (self.invalid or not self.last_source_stamp or
                not 0 <= now-self.last_source_stamp <= self.feedback_max_age or
                not 0 <= now-self.received_at <= self.feedback_max_age):
            raise RuntimeError(self.invalid or 'SNOWBOT_FEEDBACK_UNAVAILABLE')

    def capture(self, groups):
        if not self.command_enabled:
            raise RuntimeError('SNOWBOT_COMMANDS_DISABLED')
        if self.stop_failures:
            raise RuntimeError('SNOWBOT_STOP_UNCONFIRMED_RESTART_REQUIRED')
        self.require_feedback()
        if self.check_ready is not None:
            self.check_ready(groups)
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
        self.require_feedback()
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
                    self.require_feedback()
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
            hold = {n: self.last_feedback.positions[n] for g in coupled if g in groups for n in GROUPS[g]}
            try:
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
        return self.last_feedback
