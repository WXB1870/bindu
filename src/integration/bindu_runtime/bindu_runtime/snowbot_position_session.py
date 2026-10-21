"""Typed three-layer position sessions, using the independent PDO observation."""
import secrets
import json
import math
from dataclasses import dataclass, replace
from enum import Enum

from bindu_hardware.snowbot import SnowbotIO, GROUPS
from bindu_hardware.snowbot_session import CONTROL_GROUPS, CONTROL_NAMES, SLOTS, SessionHealth, SourceEnvelope


@dataclass(frozen=True)
class PreparedSessionTrajectory:
    segment: object
    effective_ns: int
    wire: dict


class SessionTrajectory:
    """Plan once outside the servo loop; keep one common clock for all axes."""
    lead_ns = 60_000_000

    def __init__(self, profile):
        # The device's existing conservative Follower bounds remain the ceiling.
        # Its faster software profile is not hardware validation. No second
        # limiter is applied after the certified trajectory is sampled.
        dynamics = dict(profile.joint_dynamics)
        for name in CONTROL_NAMES:
            dynamics[name] = dict(velocity=min(.25, profile.speed(name)),
                                  acceleration=min(.5, profile.acceleration(name)),
                                  jerk=min(162., profile.jerk(name)))
        self.profile = replace(profile, joint_dynamics=dynamics)
        self.segment = None
        self.effective_ns = 0

    def prepare(self, metadata, target, observation, now):
        from bindu_execution.interpolation import PreparedSegment, State
        effective = max(round(now*1e9)+self.lead_ns, self.effective_ns+1)
        remaining = (metadata['expires_at_ns']-effective)*1e-9
        if remaining <= .005:
            raise RuntimeError('SNOWBOT_TRAJECTORY_DEADLINE_TOO_SHORT')
        takeover = metadata['operation'] == 1
        if takeover or self.segment is None:
            if any(abs(observation.speed[SLOTS.index(n)]) > .01 for n in CONTROL_NAMES):
                raise RuntimeError('SNOWBOT_TRAJECTORY_TAKEOVER_REQUIRES_REST')
            state = State.rest(observation.position[SLOTS.index(n)] for n in CONTROL_NAMES)
        else:
            state = self.segment.sample((effective-self.effective_ns)*1e-9)
        if metadata['operation'] == 2:
            segment = PreparedSegment.stop(replace(self.profile, stop_timeout=remaining),
                                           CONTROL_NAMES, state, 0.)
        else:
            segment = PreparedSegment.target(self.profile, CONTROL_NAMES, state,
                                             tuple(target[n] for n in CONTROL_NAMES), 0.)
        description = segment.describe()
        if len(description) > 64:
            raise RuntimeError('SNOWBOT_TRAJECTORY_PIECE_CAPACITY')
        wire = dict(version=1, segment_id=metadata['sequence'], effective_at_ns=effective,
                    piece_start=[], duration=[], source_duration=[], source_elapsed=[],
                    control_count=[], controls=[])
        for start, duration, source_duration, source_elapsed, axes in description:
            wire['piece_start'].append(start)
            wire['duration'].append(duration)
            wire['source_duration'].append(source_duration)
            wire['source_elapsed'].append(source_elapsed)
            for controls in axes:
                wire['control_count'].append(len(controls))
                wire['controls'].extend(controls)
        for field, getter in (('max_velocity', self.profile.speed),
                              ('max_acceleration', self.profile.acceleration),
                              ('max_jerk', self.profile.jerk)):
            wire[field] = [getter(n) for n in CONTROL_NAMES]
        return PreparedSessionTrajectory(segment, effective, wire)

    def commit(self, proposal):
        # Only a matching servo acceptance may advance the nominal timeline.
        self.segment, self.effective_ns = proposal.segment, proposal.effective_ns


def sampled_nominal_reference(state, names, epoch, now, max_age, *, wait_until=None, steady_now=None):
    """Read the servo sample published by execution; never substitute q_goal."""
    reference = state.reference
    sampled_at = reference.header.stamp.sec+reference.header.stamp.nanosec*1e-9
    if not state.lease_active or state.lease_epoch != epoch:
        raise RuntimeError('SNOWBOT_TRAJECTORY_REFERENCE_OWNER')
    if (not math.isfinite(sampled_at) or not 0 <= now-sampled_at <= max_age or
            len(reference.name) != len(names) or len(reference.position) != len(names) or
            set(reference.name) != set(names) or not all(math.isfinite(q) for q in reference.position)):
        if wait_until is not None and steady_now is not None and steady_now < wait_until:
            return None
        raise RuntimeError('SNOWBOT_TRAJECTORY_REFERENCE_UNAVAILABLE')
    positions = dict(zip(reference.name, reference.position))
    return tuple(positions[n] for n in names), sampled_at


class PositionSessionIO(SnowbotIO):
    requesting_pause = False
    trajectory_reference = False
    servo_reference = None
    deferred_pause = False

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.writable_groups = CONTROL_GROUPS
        self.coupled_group_sets = (CONTROL_GROUPS,)
        # One atomic envelope and one PAUSE identity for waist and both arms.
        self.stop_group_sets = (('waist', 'left_arm', 'right_arm'),)

    def capture(self, groups):
        if set(groups) != CONTROL_GROUPS:
            raise RuntimeError('SNOWBOT_POSITION_SESSION_RESOURCE_LAYOUT')
        super().capture(groups)
        self.pause_request = self.pause_targets = None
        self.servo_reference = None
        self.deferred_pause = False
        self.after_capture()

    def sampled_reference(self):
        if not self.trajectory_reference:
            return None
        sample = self.servo_reference
        if (sample is None or not (self.targets or self.stop_targets) or
                not 0 <= self.now()-sample[0] <= .02):
            return (self.now(), {}, {}, {})
        return sample

    @property
    def reference_positions(self):
        if self.trajectory_reference:
            return self.sampled_reference()[1]
        return super().reference_positions

    def confirmed_stop_targets(self):
        targets = self.pause_feedback(getattr(self, 'pause_request', None))
        if targets is None:
            return None
        if any(not self.profile.limits[n][0] <= q <= self.profile.limits[n][1]
               for n, q in targets.items()):
            return {}  # Executor rejects an invalid ACK without changing its budget.
        if self.pause_targets is None:
            self.pause_targets = targets
        # A hold point is fixed for this PAUSE. Reject a changed acknowledgement;
        # measured drift must never move the expected stopping point.
        return self.pause_targets if targets == self.pause_targets else {}

    def stop(self):
        # A healthy measured pause is still possible after a target expires.
        # Bus/drive faults and invalid measurements remain blocking conditions.
        previous = self.requesting_pause
        self.requesting_pause = True
        try:
            if self.trajectory_reference and self.targets:
                self.require_feedback(CONTROL_GROUPS)
                if self.targets and self.wait_for_acceptance():
                    if not self.deferred_pause:
                        self.stop_requested_at = self.now()
                        self.stop_targets = {n: self.last_feedback.positions[n] for n in CONTROL_NAMES}
                    self.deferred_pause = True
                    return  # STOPPING waits within the original source/stop budgets.
            return super().stop()
        finally:
            self.requesting_pause = previous


class TrajectoryFault(Enum):
    NONE = 0
    REJECTED_WITH_REFERENCE = 1
    RECONNECT_REQUIRED = 2


class PositionSessionDevices:
    def __init__(self, node, profile):
        from rcl_interfaces.msg import ParameterDescriptor
        from snowbot_ctl_msgs.msg import PositionObservation, PositionSessionCommand, PositionSessionState
        from std_msgs.msg import Float64MultiArray
        from rclpy.qos import qos_profile_sensor_data
        if node.runtime_mode != 'real':
            raise ValueError('SNOWBOT_VENDOR_REQUIRES_REAL_MODE')
        self.node, self.grippers = node, None
        node.device_backend = 'snowbot_vendor'
        node.declare_parameter('command_enabled', False, descriptor=ParameterDescriptor(read_only=True))
        node.declare_parameter('vendor_namespace', '/', descriptor=ParameterDescriptor(read_only=True))
        if node.get_parameter('vendor_namespace').value != '/':
            raise ValueError('SNOWBOT_POSITION_SESSION_ROOT_NAMESPACE_REQUIRED')
        for key in ('arm_receiver_node', 'mid_receiver_node', 'servo_receiver_node'):
            node.declare_parameter(key, '', descriptor=ParameterDescriptor(read_only=True))
        self.receivers = {key: node.get_parameter(parameter).value or default for key, parameter, default in (
            ('arms', 'arm_receiver_node', '/robot_upperMonitor'),
            ('middle', 'mid_receiver_node', '/dual_arm_control_node'),
            ('head', 'servo_receiver_node', '/servo_node'))}
        self.health = SessionHealth(node.now, execution_mode=node.snowbot_execution_mode,
                                   reference_mode=getattr(node, 'snowbot_reference_mode', 0))
        self.envelopes = SourceEnvelope(secrets.randbits(64) or 1, node.now)
        self.trajectory = SessionTrajectory(profile) if self.health.reference_mode else None
        self.pending_trajectory = None
        self.trajectory_fault_kind, self.trajectory_fault_reason = TrajectoryFault.NONE, ''
        self.trajectory_instance = None
        self.io = PositionSessionIO(profile, self.send, node.now,
                            command_enabled=node.get_parameter('command_enabled').value,
                            simulated=False, check_ready=self.check_ready, check_health=self.check_health)
        self.io.source_motion = None
        self.io.trajectory_reference = bool(self.health.reference_mode)
        self.io.wait_for_acceptance = lambda: self.pending_trajectory is not None
        self.io.after_capture = lambda: self.new_session()
        self.io.pause_feedback = self.health.confirmed_hold
        self.arm_type, self.array_type = PositionSessionCommand, Float64MultiArray
        self.feedback_topic = '/bindu/position/observation'
        self.command_topics = {'arms': '/bindu/position/command'}
        self.publishers = {'arms': node.create_publisher(PositionSessionCommand, self.command_topics['arms'], 1)} if self.io.command_enabled else {}
        self.heartbeat_running = False
        node.create_subscription(PositionObservation, self.feedback_topic, self.receive, qos_profile_sensor_data)
        for layer, receiver in (('upper', 'arms'), ('middle', 'middle'), ('servo', 'head')):
            topic = '/bindu/position/'+layer+'_state'
            node.create_subscription(PositionSessionState, topic,
                lambda msg, l=layer, t=topic, r=receiver: self.receive_state(l, t, r, msg), 1)

    def identity(self, topic, expected, kind, *, subscription=False):
        method = self.node.get_subscriptions_info_by_topic if subscription else self.node.get_publishers_info_by_topic
        infos = method(topic)
        matched = [i for i in infos if i.node_namespace.rstrip('/')+'/'+i.node_name == expected and i.topic_type == kind]
        if len(matched) != 1 or not subscription and len(infos) != 1:
            raise RuntimeError('SNOWBOT_SESSION_ENDPOINT:'+topic)

    def receive_state(self, layer, topic, receiver, msg):
        try:
            self.identity(topic, self.receivers[receiver], 'snowbot_ctl_msgs/msg/PositionSessionState')
            self.health.state(layer, msg)
        except (ValueError, RuntimeError) as exc:
            self.health.states.pop(layer, None)
            self.node.get_logger().warn(str(exc), throttle_duration_sec=1.)

    def receive(self, msg):
        try:
            self.identity(self.feedback_topic, self.receivers['head'], 'snowbot_ctl_msgs/msg/PositionObservation')
            q, v, stamp = self.health.observe(msg)
            self.io.observe(q, v, stamp, self.node.now(), sample=msg.pdo_sample)
            if self.health.reference_mode:
                self.io.servo_reference = None
                sample = getattr(msg, 'sample_time_ns', 0)*1e-9
                arrays = [getattr(msg, field, ()) for field in
                          ('reference_position','reference_speed','reference_acceleration')]
                if (msg.session_selected and msg.session == self.envelopes.session and
                        getattr(msg, 'reference_mode', 0) == 1 and getattr(msg, 'reference_valid', False) and
                        getattr(msg, 'output_valid', False) and getattr(msg, 'segment_id', 0) == msg.command_sequence and
                        0 <= self.node.now()-sample <= .02 and
                        all(len(values)==len(CONTROL_NAMES) and all(math.isfinite(x) for x in values) for values in arrays)):
                    self.io.servo_reference = (sample, *[dict(zip(CONTROL_NAMES, values)) for values in arrays])
        except (ValueError, RuntimeError) as exc:
            self.io.invalid = str(exc)
            self.node.get_logger().warn(str(exc), throttle_duration_sec=1.)

    def _check_session_health(self, groups, *, acquiring=False):
        self.health.require(groups, acquiring=(acquiring or self.io.requesting_pause or bool(self.io.stop_targets)))
        obs = self.health.observation
        if obs.session_selected and obs.session != self.envelopes.session:
            # A paused/expired session can be explicitly replaced after capture.
            # During a pending stop, ignore its delayed observation and wait for
            # the exact new PAUSE identity; it cannot confirm this stop.
            if not (acquiring or self.io.requesting_pause or self.io.stop_targets) or obs.session_reason < 1:
                raise RuntimeError('SNOWBOT_SESSION_OWNER_CONFLICT')

    def _require_trajectory(self, *, allow_rejected=False):
        if self.trajectory_fault_kind is TrajectoryFault.NONE:
            return
        if (allow_rejected and self.trajectory.segment is not None and
                self.trajectory_fault_kind is TrajectoryFault.REJECTED_WITH_REFERENCE):
            return
        raise RuntimeError(self.trajectory_fault_reason)

    def check_health(self, groups, *, acquiring=False):
        self._check_session_health(groups, acquiring=acquiring)
        if self.health.reference_mode:
            self._require_trajectory(allow_rejected=(self.io.requesting_pause or self.io.sending_hold or
                                                   bool(self.io.stop_targets)))

    def new_session(self):
        self.envelopes = SourceEnvelope(secrets.randbits(64) or 1, self.node.now)
        self.io.source_motion = None
        if getattr(getattr(self, 'health', None), 'reference_mode', 0):
            self.trajectory = SessionTrajectory(self.io.profile)
            self.pending_trajectory = None
            self.trajectory_fault_kind, self.trajectory_fault_reason = TrajectoryFault.NONE, ''
            self.trajectory_instance = None

    def _settle_trajectory(self, *, ready):
        # Unknown adoption or a changed servo instance is latched for reconnect.
        # Do not bypass the existing executor's unconfirmed-stop barrier here.
        pending = self.pending_trajectory
        _, servo = self.health.states.get('servo', (0., None))
        if (self.trajectory_instance is not None and servo is not None and
                servo.instance_id != self.trajectory_instance):
            self.trajectory_fault_kind = TrajectoryFault.RECONNECT_REQUIRED
            self.trajectory_fault_reason = 'SNOWBOT_TRAJECTORY_INSTANCE_CHANGED_RECONNECT_REQUIRED'
            self.pending_trajectory = pending = None
        if pending:
            metadata, proposal, instance, command_id, sent_at = pending
            if self.node.now()*1e9 >= metadata['expires_at_ns']:
                self.trajectory_fault_kind = TrajectoryFault.RECONNECT_REQUIRED
                self.trajectory_fault_reason = 'SNOWBOT_TRAJECTORY_ACCEPTANCE_UNCONFIRMED_RECONNECT_REQUIRED'
                self.pending_trajectory = None
            elif (ready and servo is not None and servo.instance_id == instance and
                    getattr(servo, 'rejected_session', 0) == metadata['session'] and
                    getattr(servo, 'rejected_sequence', 0) == metadata['sequence']):
                known_reference = self.trajectory.segment is not None
                self.trajectory_fault_kind = (TrajectoryFault.REJECTED_WITH_REFERENCE if known_reference else
                                              TrajectoryFault.RECONNECT_REQUIRED)
                self.trajectory_fault_reason = ('SNOWBOT_TRAJECTORY_REJECTED:' if known_reference else
                    'SNOWBOT_TRAJECTORY_REJECTED_RECONNECT_REQUIRED:')+servo.reason
                self.pending_trajectory = None
            elif (ready and servo is not None and servo.instance_id == instance and servo.feedback_ready and
                    servo.startup_complete and servo.reference_mode == 1 and
                    getattr(servo, 'session', 0) == metadata['session'] and
                    getattr(servo, 'sequence', 0) == metadata['sequence'] and
                    round(self.health.states['servo'][0]*1e9) >= sent_at):
                self.trajectory.commit(proposal)
                self.pending_trajectory = None
                if metadata['operation'] == 2:
                    self.trajectory_fault_kind, self.trajectory_fault_reason = TrajectoryFault.NONE, ''
                self.node.event('SEGMENT_ACCEPTED', 'SNOWBOT_SEGMENT:'+json.dumps(dict(
                    session=metadata['session'], sequence=metadata['sequence'], segment_id=proposal.wire['segment_id'],
                    source_stamp_ns=metadata['source_stamp_ns'], expires_at_ns=metadata['expires_at_ns']),
                    separators=(',', ':')), command_id=command_id)

    def advance_session(self):
        """Advance once from the existing execution loop, never from health reads."""
        if not self.health.reference_mode:
            return
        ready = True
        try:
            self._check_session_health(CONTROL_GROUPS)
        except (RuntimeError, ValueError):
            ready = False  # Expiry still latches; stale feedback cannot acknowledge adoption.
        try:
            self._settle_trajectory(ready=ready)
            if (ready and self.io.deferred_pause and self.pending_trajectory is None and
                    self.trajectory_fault_kind in (TrajectoryFault.NONE, TrajectoryFault.REJECTED_WITH_REFERENCE)):
                self.io.deferred_pause = False
                self.io.stop()
        except (RuntimeError, ValueError) as exc:
            # Keep the engine tick running so its original stop deadlines and
            # STOP_UNCONFIRMED barrier still decide the outcome.
            self.trajectory_fault_kind = TrajectoryFault.RECONNECT_REQUIRED
            self.trajectory_fault_reason = str(exc)
            self.io.stop_failures = tuple(sorted(set(self.io.stop_failures) | {'vendor_hold'}))

    def check_ready(self, groups):
        self.check_health(groups, acquiring=True)
        for topic, kind, writer, reader in (
            ('command', 'PositionSessionCommand', None, 'arms'),
            ('upper', 'PositionSessionUpper', 'arms', 'middle'),
            ('servo', 'PositionSessionServo', 'middle', 'head')):
            name = '/bindu/position/'+topic
            self.identity(name, self.receivers[reader], 'snowbot_ctl_msgs/msg/'+kind, subscription=True)
            if writer:
                self.identity(name, self.receivers[writer], 'snowbot_ctl_msgs/msg/'+kind)
            elif self.node.count_publishers(name) != 1:
                raise RuntimeError('SNOWBOT_COMMAND_PUBLISHER_CONFLICT:'+name)
        if any(self.node.count_publishers(t) for t in ('/joint_position_cmd', '/body/motor_cmd')):
            raise RuntimeError('SNOWBOT_LEGACY_SOURCE_CONFLICT')

    def send(self, groups, target):
        if set(groups) != CONTROL_GROUPS:
            raise RuntimeError('SNOWBOT_POSITION_SESSION_RESOURCE_LAYOUT')
        # Validate before minting an envelope, so rejected requests cannot renew.
        self.check_ready(groups)
        if self.health.reference_mode:
            self._require_trajectory(allow_rejected=self.io.sending_hold)
            if self.pending_trajectory:
                if self.io.sending_hold:
                    raise RuntimeError('SNOWBOT_TRAJECTORY_ACCEPTANCE_UNCONFIRMED_RECONNECT_REQUIRED')
                return  # One in flight: latest Motion waits without minting a source.
        metadata = self.envelopes.command(self.io.source_motion, pause=self.io.sending_hold)
        if metadata is None:
            return
        metadata.update(middle_instance_id=self.health.states['middle'][1].instance_id,
                        feedback_boot_id=self.health.boot, feedback_sample=self.health.sample)
        msg = self.arm_type()
        for name, value in metadata.items():
            setattr(msg.meta, name, value)
        if self.health.reference_mode:
            motion = self.io.source_motion
            identity = (motion.command_id, motion.lease_id, self.io.profile.digest) if motion else ()
            if (len(identity) != 3 or any(not isinstance(value, str) or not value or
                    len(value.encode('utf-8')) > 128 for value in identity) or
                    type(motion.epoch) is not int or not 0 <= motion.epoch < 2**64):
                raise RuntimeError('SNOWBOT_SOURCE_IDENTITY_LAYOUT')
            msg.meta.reference_mode = self.health.reference_mode
            msg.meta.source_command_id = motion.command_id
            msg.meta.lease_id = motion.lease_id
            msg.meta.epoch = motion.epoch
            msg.meta.profile_hash = self.io.profile.digest
            try:
                proposal = self.trajectory.prepare(metadata, target, self.health.observation, self.node.now())
            except ValueError as exc:
                raise RuntimeError('SNOWBOT_TRAJECTORY_PREPARE:'+str(exc)) from exc
            for field, value in proposal.wire.items():
                setattr(msg.trajectory, field, value)
        msg.command.l_arm_position = [float(target[n]) for n in GROUPS['left_arm']]
        msg.command.r_arm_position = [float(target[n]) for n in GROUPS['right_arm']]
        msg.waist_position = [float(target[n]) for n in GROUPS['waist']]
        self.publishers['arms'].publish(msg)
        if self.health.reference_mode:
            self.trajectory_instance = self.health.states['servo'][1].instance_id
            self.pending_trajectory = (metadata, proposal, self.health.states['servo'][1].instance_id,
                                       msg.meta.source_command_id, round(self.node.now()*1e9))
            mapping = {k: metadata[k] for k in ('session', 'sequence', 'source_stamp_ns', 'expires_at_ns')}
            mapping.update(segment_id=msg.trajectory.segment_id, effective_at_ns=msg.trajectory.effective_at_ns,
                           lease_id=msg.meta.lease_id, epoch=msg.meta.epoch,
                           execution_mode=self.health.execution_mode, reference_mode=1)
            self.node.event('SEGMENT_SENT', 'SNOWBOT_SEGMENT:'+json.dumps(mapping, separators=(',', ':')),
                            command_id=msg.meta.source_command_id)
        if self.io.sending_hold:
            self.io.pause_request = (metadata['session'], metadata['sequence'],
                                     self.health.boot, self.health.sample)
            self.io.pause_targets = None

    def guarded_array(self, values, hold=False):
        raise RuntimeError('GRIPPER_INDEPENDENT_CALIBRATION_AND_PROTOCOL_REQUIRED')
