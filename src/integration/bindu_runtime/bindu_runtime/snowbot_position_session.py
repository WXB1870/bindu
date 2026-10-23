"""Typed three-layer position sessions, using the independent PDO observation."""
import json
import math

from bindu_hardware.snowbot import CommandNotSent, SnowbotIO, GROUPS
from bindu_hardware.snowbot_session import CONTROL_GROUPS, CONTROL_NAMES, ROS_FEEDBACK_TIMEOUT_S, SessionHealth, SourceEnvelope, new_session_id


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
    captured_pause_request = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.writable_groups = CONTROL_GROUPS
        self.coupled_group_sets = (CONTROL_GROUPS,)
        # One atomic envelope and one PAUSE identity for waist and both arms.
        self.stop_group_sets = (('waist', 'left_arm', 'right_arm'),)

    def validate_motion(self, motion, now):
        SourceEnvelope.validate_motion(motion, now)

    def capture(self, groups):
        if set(groups) != CONTROL_GROUPS:
            raise RuntimeError('SNOWBOT_POSITION_SESSION_RESOURCE_LAYOUT')
        previous_pause = getattr(self, 'pause_request', None) or self.captured_pause_request
        super().capture(groups)
        self.captured_pause_request = previous_pause
        self.pause_request = self.pause_targets = None
        self.servo_reference = None
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
        previous = self.requesting_pause
        self.requesting_pause = True
        try:
            if self.trajectory_reference:
                # PAUSE carries the existing session authority. Its position
                # fields are only a wire/layout footprint: the servo brakes
                # from its own reference, and its ACK supplies the hold point.
                # In particular, do not require a fresh ROS measurement to ask
                # that a following session stop producing motion.
                self.reference.clear()
                if not self.targets:
                    return  # Pending PAUSE is not renewed on repeated faults.
                self.stop_requested_at = self.now()
                target = dict(self.targets)
                try:
                    self._send(target, hold=True)
                except RuntimeError:
                    self.stop_failures = tuple(sorted(self._groups(target)))
                else:
                    self.stop_targets = target
                self.targets.clear()
                return
            return super().stop()
        finally:
            self.requesting_pause = previous


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
        self.envelopes = SourceEnvelope(new_session_id(node.now()), node.now,
                                       target_stream=self.health.reference_mode == 2)
        self.sent = {}
        self.target_instance = None
        self.source_identity = None
        self.target_fault = ''
        self.applied_sequence = 0
        self.takeover_pending = None
        self.takeover_previous_hold = None
        self.io = PositionSessionIO(profile, self.send, node.now,
                            command_enabled=node.get_parameter('command_enabled').value,
                            simulated=False, check_ready=self.check_ready, check_health=self.check_health)
        self.io.source_motion = None
        self.io.trajectory_reference = bool(self.health.reference_mode)
        self.io.stop_confirmation_window = .1 if self.health.reference_mode == 2 else 0.
        self.io.stop_confirmation_max_gap = ROS_FEEDBACK_TIMEOUT_S
        self.io.stop_requires_arrival = self.health.reference_mode != 2
        self.startup_hold_state = ('waiting' if self.io.command_enabled and
                                   self.health.reference_mode == 2 else 'disabled')
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
        if self.health.reference_mode:
            node.create_subscription(PositionSessionState, '/bindu/position/servo_receipt', self.receive_receipt, 1)
        if self.health.reference_mode == 2:
            from std_srvs.srv import Trigger
            node.create_service(Trigger, 'execution/held_reference', self.read_held_reference)

    def read_held_reference(self, request, reply):
        """Read the confirmed PAUSE point without acquiring or sending anything."""
        engine = self.node.engine
        try:
            if (self.node.closing or not engine.lease_id or engine.active or
                    engine.stopping or engine.stop_fault or self.io.stop_failures):
                raise RuntimeError('SNOWBOT_HELD_REFERENCE_REQUIRES_STOPPED_LEASE')
            identity = getattr(self.io, 'pause_request', None) or self.io.captured_pause_request
            targets = self.health.confirmed_hold(identity)
            if targets is None:
                raise RuntimeError('SNOWBOT_HELD_REFERENCE:'+self.health.hold_failure(identity))
            if any(not self.io.profile.limits[n][0] <= q <= self.io.profile.limits[n][1]
                   for n, q in targets.items()):
                raise RuntimeError('SNOWBOT_HELD_REFERENCE_LIMIT')
            obs = self.health.observation
            reply.success, reply.message = True, json.dumps(dict(
                instance_id=self.node.instance_id, profile_hash=self.io.profile.digest,
                lease_id=engine.lease_id, epoch=engine.epoch, revision=engine.revision,
                stamp=obs.header.stamp.sec+obs.header.stamp.nanosec*1e-9-obs.pdo_age,
                positions=targets, boot_id=obs.boot_id, session=obs.session,
                sequence=obs.command_sequence))
        except RuntimeError as exc:
            reply.success, reply.message = False, str(exc)
        return reply

    def receive_receipt(self, msg):
        """Takeover ACK opens its initial barrier; ordinary targets/PAUSE never wait."""
        try:
            self.identity('/bindu/position/servo_receipt', self.receivers['head'],
                          'snowbot_ctl_msgs/msg/PositionSessionState')
            sequence = msg.sequence if msg.state == 'accepted' else msg.rejected_sequence
            source = self.sent.get(sequence)
            if source is None:
                return
            metadata, instance, command_id = source
            session = msg.session if msg.state == 'accepted' else msg.rejected_session
            stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if (msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 1_000_000_000 or
                    msg.layer != 'servo' or msg.wire_version != 5 or msg.reference_mode != 2 or
                    msg.state not in ('accepted', 'rejected') or msg.instance_id != instance or
                    msg.feedback_boot_id != self.health.boot or session != metadata['session'] or
                    not metadata['source_stamp_ns'] <= stamp < metadata['expires_at_ns'] or
                    stamp > round(self.node.now()*1e9)):
                return
            self._settle_takeover(msg if msg.state == 'accepted' else None, receipt=True)
            if msg.state == 'rejected' and sequence == self.envelopes.sequence:
                self.target_fault = 'SNOWBOT_TARGET_REJECTED:'+msg.reason
            self.node.event('TARGET_'+msg.state.upper(), 'SNOWBOT_TARGET:'+json.dumps(dict(
                session=session, sequence=sequence, reason=msg.reason), separators=(',', ':')),
                command_id=command_id)
        except (ValueError, RuntimeError) as exc:
            self.node.get_logger().warn(str(exc), throttle_duration_sec=1.)

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
            advanced = self.io.observe(q, v, stamp, self.node.now(), sample=msg.pdo_sample)
            if advanced and self.grippers and self.grippers.core:
                try:
                    self.grippers.core.observe(q, v, {'left': float(msg.current[7]),
                        'right': float(msg.current[15])}, stamp)
                except ValueError as exc:
                    self.grippers.core.invalid = str(exc)
            if msg.session_selected and msg.session == self.envelopes.session:
                self.takeover_previous_hold = None
            if self.health.reference_mode:
                self.io.servo_reference = None
                sample = getattr(msg, 'sample_time_ns', 0)*1e-9
                arrays = [getattr(msg, field, ()) for field in
                          ('reference_position','reference_speed','reference_acceleration')]
                if (msg.session_selected and msg.session == self.envelopes.session and
                        getattr(msg, 'reference_mode', 0) == 2 and getattr(msg, 'reference_valid', False) and
                        getattr(msg, 'output_valid', False) and getattr(msg, 'segment_id', 0) == msg.command_sequence and
                        0 <= self.node.now()-sample <= .02 and
                        all(len(values)==len(CONTROL_NAMES) and all(math.isfinite(x) for x in values) for values in arrays)):
                    self.io.servo_reference = (sample, *[dict(zip(CONTROL_NAMES, values)) for values in arrays])
                    source = self.sent.get(msg.command_sequence)
                    if (source is not None and msg.command_sequence > self.applied_sequence and
                            getattr(msg, 'trajectory_phase', 0) in (2, 3, 4, 5) and
                            source[0]['source_stamp_ns'] <= msg.sample_time_ns < source[0]['expires_at_ns']):
                        self.applied_sequence = msg.command_sequence
                        self.node.event('TARGET_APPLIED', 'SNOWBOT_TARGET:'+json.dumps(dict(
                            session=msg.session, sequence=msg.command_sequence,
                            sample_time_ns=msg.sample_time_ns), separators=(',', ':')),
                            command_id=source[2])
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
            if not (acquiring or self.io.requesting_pause or self.io.stop_targets or
                    self._waiting_for_takeover_pdo(obs)) or obs.session_reason < 1:
                raise RuntimeError('SNOWBOT_SESSION_OWNER_CONFLICT')

    def _waiting_for_takeover_pdo(self, obs):
        # ACK may arrive before PDO changes owner. Allow only the captured old
        # hold, until actual adoption or the original TAKEOVER deadline.
        previous = self.takeover_previous_hold
        return bool(previous is not None and
            (obs.boot_id, obs.session, obs.command_sequence,
             self.health.states['servo'][1].instance_id) == previous[:4] and
            round(self.node.now()*1e9) < previous[4] and
            obs.session_reason == 1 and obs.hold_valid and
            getattr(obs, 'trajectory_phase', 0) == 4 and
            getattr(obs, 'reference_valid', False) and getattr(obs, 'output_valid', False))

    def check_health(self, groups, *, acquiring=False):
        self._check_session_health(groups, acquiring=acquiring)
        if self.health.reference_mode:
            if self.target_fault and not (self.io.requesting_pause or self.io.sending_hold):
                if not (acquiring and self._completed_pause_rejection()):
                    raise RuntimeError(self.target_fault)

    def _completed_pause_rejection(self):
        # An expired repeat of a completed PAUSE cannot poison the next capture.
        # Keep the rejection recorded; only a successful capture opens a new session.
        source = self.sent.get(self.envelopes.sequence)
        if (self.health.reference_mode != 2 or
                self.target_fault != 'SNOWBOT_TARGET_REJECTED:duplicate_or_expired_frame' or
                source is None or self.io.targets or self.io.stop_targets or self.io.stop_failures):
            return False
        metadata = source[0]
        request = getattr(self.io, 'pause_request', None)
        return bool(metadata['operation'] == 2 and
            source[1] == self.health.states['servo'][1].instance_id and
            round(self.node.now()*1e9) >= metadata['expires_at_ns'] and
            request and request[:2] == (metadata['session'], metadata['sequence']) and
            self.io.pause_targets and self.io.confirmed_stop_targets() == self.io.pause_targets)

    def new_session(self):
        self.envelopes = SourceEnvelope(new_session_id(self.node.now(), self.envelopes.session), self.node.now,
                                       target_stream=self.health.reference_mode == 2)
        self.io.source_motion = None
        self.sent = {}
        self.target_instance = None
        self.source_identity = None
        self.target_fault = ''
        self.applied_sequence = 0
        self.takeover_pending = None
        self.takeover_previous_hold = None

    def _settle_takeover(self, report=None, *, receipt=False):
        """Only takeover has a delivery barrier; normal targets never wait."""
        pending = self.takeover_pending
        if pending is None:
            return
        metadata, instance = pending
        _, current_servo = self.health.states.get('servo', (0., None))
        if current_servo is not None and current_servo.instance_id != instance:
            self.target_fault = 'SNOWBOT_TARGET_INSTANCE_CHANGED_RECONNECT_REQUIRED'
            return
        now_ns = round(self.node.now()*1e9)
        if now_ns >= metadata['expires_at_ns']:
            self.target_fault = 'SNOWBOT_TARGET_TAKEOVER_UNCONFIRMED_RECONNECT_REQUIRED'
            return
        if report is None or self.target_fault:
            return
        stamp = report.header.stamp.sec*1_000_000_000+report.header.stamp.nanosec
        if (report.header.stamp.sec >= 0 and 0 <= report.header.stamp.nanosec < 1_000_000_000 and
                getattr(report, 'feedback_boot_id', 0) == self.health.boot and
                report.instance_id == instance and getattr(report, 'session', 0) == metadata['session'] and
                getattr(report, 'sequence', 0) == metadata['sequence'] and
                metadata['source_stamp_ns'] <= stamp <= now_ns and
                report.wire_version == 5 and report.reference_mode == 2 and
                (receipt or report.selected and report.feedback_ready and report.startup_complete)):
            self.takeover_pending = None

    def advance_session(self):
        """Observe restart/rejection without gating subsequent target production."""
        if not self.health.reference_mode:
            return
        _, servo = self.health.states.get('servo', (0., None))
        self._settle_takeover(servo)
        if servo is not None:
            if self.target_instance is not None and servo.instance_id != self.target_instance:
                self.target_fault = 'SNOWBOT_TARGET_INSTANCE_CHANGED_RECONNECT_REQUIRED'
            if (getattr(servo, 'rejected_session', 0) == self.envelopes.session and
                    getattr(servo, 'rejected_sequence', 0) == self.envelopes.sequence and
                    self.envelopes.sequence in self.sent):
                self.target_fault = 'SNOWBOT_TARGET_REJECTED:'+servo.reason

    def check_ready(self, groups):
        if self.health.reference_mode == 2 and self.io.sending_hold:
            # This is a stop-only request, never a takeover or target refresh.
            # Keep the known boot/instances and all endpoint/authority checks;
            # the servo still validates its local PDO and the stop trajectory.
            sent = self.sent.get(self.envelopes.sequence)
            servo = self.health.states.get('servo', (0., None))[1]
            middle = self.health.states.get('middle', (0., None))[1]
            if (sent is None or servo is None or middle is None or
                    servo.instance_id != sent[1] or
                    middle.instance_id != sent[0]['middle_instance_id'] or
                    self.health.boot != sent[0]['feedback_boot_id']):
                raise RuntimeError('SNOWBOT_PAUSE_AUTHORITY_UNAVAILABLE')
        else:
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

    def advance_startup_hold(self, engine):
        """One write-enabled startup, using the normal lease/command/ACK path.

        Never retry a granted lease or failed enable. Subsequent squeeze actions
        acquire following sessions; they do not initiate this startup action.
        """
        from bindu_contracts.contracts import Motion
        from bindu_execution.executor import Rejected
        state = self.startup_hold_state
        if state in ('disabled', 'ready', 'failed') or self.node.closing:
            return
        now = self.node.now()
        if engine.stop_fault or self.io.stop_failures:
            self.startup_hold_state = 'failed'
            self.node.event('STARTUP_HOLD_FAILED', engine.stop_fault or 'STOP_UNCONFIRMED')
            return
        if state == 'waiting':
            try:
                lease, epoch = engine.acquire('snowbot_startup_hold',
                    ('waist', 'left_arm', 'right_arm'), 2., now)
            except (Rejected, RuntimeError):
                return  # Discovery/fresh measured feedback may not yet be ready.
            self.startup_hold_state = 'takeover'
            names = tuple(n for group in ('waist', 'left_arm', 'right_arm') for n in GROUPS[group])
            motion = Motion('startup_hold_'+lease, lease, epoch, self.io.profile.digest,
                'startup_hold', '', 'joint_target', '', now, .2, names=names,
                positions=tuple(engine.feedback.positions[n] for n in names),
                schema_version=2, resource_groups=('waist', 'left_arm', 'right_arm'), runtime_mode='real')
            try:
                engine.submit(motion, now)
                engine.tick(now)
            except (Rejected, RuntimeError) as exc:
                engine.halt(now, 'STARTUP_HOLD_FAILED', revoke=True)
                self.startup_hold_state = 'failed'
                self.node.event('STARTUP_HOLD_FAILED', str(exc))
                return
            self.node.event('STARTUP_HOLD_REQUESTED', 'MEASURED_POSITION', command_id=motion.command_id)
        elif state == 'takeover':
            if engine.stopping or self.target_fault or engine.active is None:
                self.startup_hold_state = 'failed'
                self.node.event('STARTUP_HOLD_FAILED', self.target_fault or 'TAKEOVER_NOT_CONFIRMED')
            elif self.takeover_pending is None and self.applied_sequence:
                engine.halt(now, revoke=True)
                self.startup_hold_state = 'stopping'
        elif state == 'stopping' and not engine.stopping:
            self.startup_hold_state = 'ready'
            self.node.event('STARTUP_HOLD_READY', 'POWERED_HOLD')
            self.node.get_logger().info('SNOWBOT_STARTUP_HOLD_READY: powered hold; squeeze controls following')

    def brake_before_source_expiry(self, engine):
        """Request PAUSE before a lost target producer's original TTL expires."""
        if self.health.reference_mode != 2 or not engine.active or engine.stopping:
            return
        sent = self.sent.get(self.envelopes.sequence)
        if sent and sent[0]['operation'] != 2:
            # 60 ms covers two executor ticks plus the 10 ms local preparation.
            # A stalled controller still falls under the native watchdog.
            if round(self.node.now()*1e9) >= sent[0]['expires_at_ns']-60_000_000:
                engine.halt(self.node.now(), 'COMMAND_TIMEOUT', revoke=True)

    def send(self, groups, target):
        # This adapter has one atomic publisher. Before that call, failure is
        # provably unsent; once called, even an exception leaves delivery unknown.
        self.publish_attempted = False
        try:
            return self._send_packet(groups, target)
        except RuntimeError as exc:
            if not self.publish_attempted:
                raise CommandNotSent(str(exc)) from exc
            raise

    def _send_packet(self, groups, target):
        if set(groups) != CONTROL_GROUPS:
            raise RuntimeError('SNOWBOT_POSITION_SESSION_RESOURCE_LAYOUT')
        # Validate before minting an envelope, so rejected requests cannot renew.
        if self.health.reference_mode:
            self._settle_takeover()
        self.check_ready(groups)
        if set(target) != set(CONTROL_NAMES) or any(not math.isfinite(target[n]) or
                not self.io.profile.limits[n][0] <= target[n] <= self.io.profile.limits[n][1]
                for n in CONTROL_NAMES):
            raise RuntimeError('SNOWBOT_TARGET_LAYOUT_OR_LIMIT')
        identity = None
        if self.health.reference_mode:
            motion = self.io.source_motion
            # PAUSE may follow revocation/clearing of the Motion. Retain the last
            # source identity in this session; stopping never invents ownership.
            identity = ((motion.command_id, motion.lease_id, motion.epoch, self.io.profile.digest)
                        if motion is not None else self.source_identity if self.io.sending_hold else None)
            if (identity is None or any(not isinstance(value, str) or not value or
                    len(value.encode('utf-8')) > 128 for value in (identity[0], identity[1], identity[3])) or
                    type(identity[2]) is not int or not 0 <= identity[2] < 2**64):
                raise RuntimeError('SNOWBOT_SOURCE_IDENTITY_LAYOUT')
        if self.health.reference_mode and self.takeover_pending is not None and not self.io.sending_hold:
            return  # Retain the latest original Motion in IO; no identity/TTL renewal.
        metadata = self.envelopes.command(self.io.source_motion, pause=self.io.sending_hold)
        if metadata is None:
            return
        metadata.update(middle_instance_id=self.health.states['middle'][1].instance_id,
                        feedback_boot_id=self.health.boot, feedback_sample=self.health.sample)
        msg = self.arm_type()
        for name, value in metadata.items():
            setattr(msg.meta, name, value)
        if self.health.reference_mode:
            self.source_identity = identity
            msg.meta.reference_mode = self.health.reference_mode
            msg.meta.source_command_id, msg.meta.lease_id, msg.meta.epoch, msg.meta.profile_hash = identity
            # Version 2 is a target recipe, never a curve. The servo owns its
            # local switching time; empty curve fields reject legacy packets.
            msg.trajectory.version = 2
            msg.trajectory.segment_id = metadata['sequence']
            for field, getter, ceiling in (('max_velocity', self.io.profile.speed, .25),
                    ('max_acceleration', self.io.profile.acceleration, .5),
                    ('max_jerk', self.io.profile.jerk, 162.)):
                setattr(msg.trajectory, field, [min(ceiling, getter(n)) for n in CONTROL_NAMES])
        msg.command.l_arm_position = [float(target[n]) for n in GROUPS['left_arm']]
        msg.command.r_arm_position = [float(target[n]) for n in GROUPS['right_arm']]
        msg.waist_position = [float(target[n]) for n in GROUPS['waist']]
        if self.health.reference_mode:
            instance = self.health.states['servo'][1].instance_id
            self.target_instance = instance
            if metadata['operation'] == 1:
                self.takeover_pending = (dict(metadata), instance)
                obs = self.health.observation
                self.takeover_previous_hold = (
                    (obs.boot_id, obs.session, obs.command_sequence, instance, metadata['expires_at_ns'])
                    if obs.session_selected and obs.session != metadata['session'] and
                    obs.session_reason == 1 and obs.hold_valid and
                    getattr(obs, 'trajectory_phase', 0) == 4 and
                    getattr(obs, 'reference_valid', False) and getattr(obs, 'output_valid', False)
                    else None)
            elif metadata['operation'] == 2:
                self.takeover_pending = None
            self.sent[metadata['sequence']] = (metadata, instance, msg.meta.source_command_id)
            while len(self.sent) > 32:
                del self.sent[next(iter(self.sent))]
        self.publish_attempted = True
        self.publishers['arms'].publish(msg)
        if self.health.reference_mode:
            mapping = {k: metadata[k] for k in ('session', 'sequence', 'source_stamp_ns', 'expires_at_ns')}
            mapping.update(lease_id=msg.meta.lease_id, epoch=msg.meta.epoch,
                           execution_mode='position', reference_mode=2)
            self.node.event('TARGET_SENT', 'SNOWBOT_TARGET:'+json.dumps(mapping, separators=(',', ':')),
                            command_id=msg.meta.source_command_id)
        if self.io.sending_hold:
            self.io.pause_request = (metadata['session'], metadata['sequence'],
                                     self.health.boot, self.health.sample)
            self.io.pause_targets = None

    def guarded_array(self, values, hold=False):
        raise RuntimeError('GRIPPER_INDEPENDENT_CALIBRATION_AND_PROTOCOL_REQUIRED')
