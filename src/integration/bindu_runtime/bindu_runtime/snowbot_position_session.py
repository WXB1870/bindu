"""Typed three-layer position sessions, using the independent PDO observation."""
import secrets

from rcl_interfaces.msg import ParameterDescriptor
from bindu_hardware.snowbot import SnowbotIO, GROUPS
from bindu_hardware.snowbot_session import ARMS, SessionHealth, SourceEnvelope


class PositionSessionIO(SnowbotIO):
    requesting_pause = False

    def capture(self, groups):
        super().capture(groups)
        self.after_capture()

    def stop(self):
        # A healthy measured pause is still possible after a target expires.
        # Bus/drive faults and invalid measurements remain blocking conditions.
        self.requesting_pause = True
        try:
            return super().stop()
        finally:
            self.requesting_pause = False


class PositionSessionDevices:
    def __init__(self, node, profile):
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
        self.health = SessionHealth(node.now)
        self.envelopes = SourceEnvelope(secrets.randbits(64) or 1, node.now)
        self.io = PositionSessionIO(profile, self.send, node.now,
                            command_enabled=node.get_parameter('command_enabled').value,
                            simulated=False, check_ready=self.check_ready, check_health=self.check_health)
        self.io.source_motion = None
        self.io.after_capture = lambda: self.new_session()
        self.io.writable_groups = ARMS
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
        except (ValueError, RuntimeError) as exc:
            self.io.invalid = str(exc)
            self.node.get_logger().warn(str(exc), throttle_duration_sec=1.)

    def check_health(self, groups, *, acquiring=False):
        self.health.require(groups, acquiring=(acquiring or self.io.requesting_pause or bool(self.io.stop_targets)))
        obs = self.health.observation
        if obs.session_selected and obs.session != self.envelopes.session:
            # A paused/expired session can be explicitly replaced after capture.
            if not acquiring or obs.session_reason < 1:
                raise RuntimeError('SNOWBOT_SESSION_OWNER_CONFLICT')

    def new_session(self):
        self.envelopes = SourceEnvelope(secrets.randbits(64) or 1, self.node.now)
        self.io.source_motion = None

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
        if self.node.count_publishers('/joint_position_cmd'):
            raise RuntimeError('SNOWBOT_LEGACY_SOURCE_CONFLICT')

    def send(self, groups, target):
        if set(groups) != ARMS:
            raise RuntimeError('SNOWBOT_POSITION_SESSION_ARMS_ONLY')
        # Validate before minting an envelope, so rejected requests cannot renew.
        self.check_ready(groups)
        metadata = self.envelopes.command(self.io.source_motion, pause=self.io.sending_hold)
        if metadata is None:
            return
        metadata.update(middle_instance_id=self.health.states['middle'][1].instance_id,
                        feedback_boot_id=self.health.boot, feedback_sample=self.health.sample)
        msg = self.arm_type()
        for name, value in metadata.items():
            setattr(msg.meta, name, value)
        msg.command.l_arm_position = [float(target[n]) for n in GROUPS['left_arm']]
        msg.command.r_arm_position = [float(target[n]) for n in GROUPS['right_arm']]
        self.publishers['arms'].publish(msg)

    def guarded_array(self, values, hold=False):
        raise RuntimeError('GRIPPER_INDEPENDENT_CALIBRATION_AND_PROTOCOL_REQUIRED')
