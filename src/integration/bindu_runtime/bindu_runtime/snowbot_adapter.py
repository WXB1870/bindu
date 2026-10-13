"""Opt-in Snowbot execution endpoint; vendor nodes are never launched here."""
import signal
import time
import secrets
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from bindu_execution.delegated_position import DelegatedPositionExecutor
from bindu_hardware.snowbot import GROUPS, WRITABLE, VendorHealth, SnowbotIO, decode_feedback
from .execution_node import ExecutionNode


class SnowbotDevices:
    def __init__(self, node, profile):
        if node.runtime_mode != 'real':
            raise ValueError('SNOWBOT_VENDOR_REQUIRES_REAL_MODE')
        node.device_backend = 'snowbot_vendor'
        # Optional message package: other robot assemblies do not import it.
        try:
            from snowbot_ctl_msgs.msg import JointPositionCommand, RetInformation
        except ImportError as exc:
            raise RuntimeError('SNOWBOT_CTL_MSGS_NOT_INSTALLED') from exc
        from std_msgs.msg import Float64MultiArray, String, UInt64MultiArray
        from rclpy.qos import qos_profile_sensor_data
        self.node = node
        self.session = secrets.randbits(64) or 1
        self.health = VendorHealth(node.now, expected_session=self.session)
        self.heartbeat_sequence = 0
        self.heartbeat_running = False
        self.heartbeat_type = UInt64MultiArray
        self.arm_type, self.array_type = JointPositionCommand, Float64MultiArray
        node.declare_parameter('command_enabled', False)
        node.declare_parameter('vendor_namespace', '/snowbot')
        prefix = node.get_parameter('vendor_namespace').value
        if (not isinstance(prefix, str) or not prefix.startswith('/') or
                (prefix != '/' and prefix != prefix.rstrip('/')) or '//' in prefix):
            raise ValueError('SNOWBOT_VENDOR_NAMESPACE')
        self.io = SnowbotIO(profile, self.send, node.now,
                            command_enabled=node.get_parameter('command_enabled').value,
                            simulated=node.simulation, check_ready=self.check_ready,
                            check_health=self.check_health)
        topic = lambda name: prefix.rstrip('/')+'/'+name
        self.feedback_topic = topic('dualArmCtl_fdback')
        self.command_topics = {'arms': topic('joint_position_cmd'),
                               'waist': topic('body/motor_cmd'), 'head': topic('head/motor_cmd')}
        node.declare_parameter('arm_receiver_node', '')
        node.declare_parameter('servo_receiver_node', '')
        node.declare_parameter('mid_receiver_node', '')
        arm_receiver = node.get_parameter('arm_receiver_node').value or topic('robot_upperMonitor')
        servo_receiver = node.get_parameter('servo_receiver_node').value or topic('servo_node')
        mid_receiver = node.get_parameter('mid_receiver_node').value or topic('dual_arm_control_node')
        self.bridge_routes = ((topic('upperMonitor_msg_send'), arm_receiver, mid_receiver),
                              (topic('ctl_data_pub'), mid_receiver, servo_receiver))
        self.receivers = {'arms': arm_receiver, 'waist': servo_receiver, 'head': servo_receiver}
        if any(not isinstance(v, str) or not v.startswith('/') or v.endswith('/')
               for v in (*self.receivers.values(), mid_receiver)):
            raise ValueError('SNOWBOT_RECEIVER_NODE_NAME')
        self.publishers = {
            'arms': node.create_publisher(JointPositionCommand, topic('joint_position_cmd'), 1),
            'waist': node.create_publisher(Float64MultiArray, topic('body/motor_cmd'), 1),
            'head': node.create_publisher(Float64MultiArray, topic('head/motor_cmd'), 1),
        } if self.io.command_enabled else {}
        node.create_subscription(RetInformation, self.feedback_topic, self.receive,
                                 qos_profile_sensor_data)
        self.health_topics = {'servo': topic('bindu/servo_health'), 'source': topic('bindu/control_source')}
        self.health_receivers = {'servo': servo_receiver, 'source': arm_receiver}
        for channel, health_topic in self.health_topics.items():
            node.create_subscription(String, health_topic,
                                     lambda msg, c=channel: self.receive_health(c, msg), 1)
        self.heartbeat_topic = topic('bindu/heartbeat')
        self.heartbeat_pub = node.create_publisher(UInt64MultiArray, self.heartbeat_topic, 1) if self.io.command_enabled else None
        self.heartbeat_running = self.io.command_enabled
        node.create_timer(.02, self.heartbeat)

    def receive_health(self, channel, msg):
        try:
            infos = self.node.get_publishers_info_by_topic(self.health_topics[channel])
            if (len(infos) != 1 or
                    infos[0].node_namespace.rstrip('/')+'/'+infos[0].node_name != self.health_receivers[channel]):
                raise ValueError('SNOWBOT_HEALTH_PUBLISHER_IDENTITY')
            self.health.receive(channel, msg.data)
        except ValueError as exc:
            self.health.errors[channel] = str(exc)
            self.node.get_logger().warn(str(exc), throttle_duration_sec=1.)

    def check_health(self, groups, *, armed=True):
        for channel, topic in self.health_topics.items():
            if self.node.count_publishers(topic) != 1:
                raise RuntimeError('SNOWBOT_HEALTH_UNAVAILABLE:'+channel)
        self.health.require(groups, armed=armed)

    def heartbeat(self):
        if not self.heartbeat_running:
            return
        try:
            # The guard mode is exclusive to this adapter, including idle time.
            # A wrong source, PDO fault or conflict stops heartbeat renewal.
            self.check_health(WRITABLE, armed=False)
            if (self.node.count_publishers(self.heartbeat_topic) != 1 or
                    any(self.node.count_publishers(t) != 1 for t in self.command_topics.values())):
                return
        except RuntimeError:
            return
        self.heartbeat_sequence += 1
        self.heartbeat_pub.publish(self.heartbeat_type(data=[
            self.node.get_clock().now().nanoseconds, self.heartbeat_sequence, self.session]))

    def receive(self, msg):
        try:
            writers = self.node.get_publishers_info_by_topic(self.feedback_topic)
            if (len(writers) != 1 or
                    writers[0].node_namespace.rstrip('/')+'/'+writers[0].node_name != self.health_receivers['servo']):
                raise ValueError('SNOWBOT_FEEDBACK_PUBLISHER_IDENTITY')
            q, v, stamp = decode_feedback(msg)
            self.io.observe(q, v, stamp, self.node.now())
        except ValueError as exc:
            self.io.invalid = str(exc)
            self.node.get_logger().warn(str(exc), throttle_duration_sec=1.)

    def send(self, groups, target):
        # Validate all subscribers before sending the first packet. Across the
        # vendor's three topics this is still not an atomic hardware operation.
        packets = []
        if {'left_arm', 'right_arm'} & groups:
            msg = self.arm_type()
            msg.l_arm_position = [float(target[n]) for n in GROUPS['left_arm']]
            msg.r_arm_position = [float(target[n]) for n in GROUPS['right_arm']]
            packets.append(('arms', msg))
        for group in ('waist', 'head'):
            if group in groups:
                packets.append((group, self.array_type(data=[float(target[n]) for n in GROUPS[group]])))
        self.check_ready(groups)
        for group, msg in packets:
            self.publishers[group].publish(msg)

    def check_ready(self, groups):
        self.check_health(groups)
        if (self.node.count_publishers(self.heartbeat_topic) != 1 or
                not any(v.node_namespace.rstrip('/')+'/'+v.node_name == self.receivers['head']
                        for v in self.node.get_subscriptions_info_by_topic(self.heartbeat_topic))):
            raise RuntimeError('SNOWBOT_HEARTBEAT_UNAVAILABLE')
        channels = ({'arms'} if {'left_arm', 'right_arm'} & set(groups) else set()) | (set(groups) & {'waist', 'head'})
        if 'arms' in channels:
            for topic, sender, receiver in self.bridge_routes:
                writers = self.node.get_publishers_info_by_topic(topic)
                readers = self.node.get_subscriptions_info_by_topic(topic)
                if (len(writers) != 1 or
                        writers[0].node_namespace.rstrip('/')+'/'+writers[0].node_name != sender or
                        not any(v.node_namespace.rstrip('/')+'/'+v.node_name == receiver for v in readers)):
                    raise RuntimeError('SNOWBOT_VENDOR_RECEIVER_UNAVAILABLE:'+topic)
        for group in sorted(channels):
            topic = self.command_topics[group]
            if self.node.count_publishers(topic) != 1:
                self.node.get_logger().error('SNOWBOT_COMMAND_PUBLISHER_CONFLICT:'+topic,
                                             throttle_duration_sec=1.)
                raise RuntimeError('SNOWBOT_COMMAND_PUBLISHER_CONFLICT:'+group)
            receivers = self.node.get_subscriptions_info_by_topic(topic)
            if not any(info.node_namespace.rstrip('/')+'/'+info.node_name == self.receivers[group]
                       for info in receivers):
                self.node.get_logger().error('SNOWBOT_VENDOR_RECEIVER_UNAVAILABLE:'+topic+
                                             ':expected='+self.receivers[group], throttle_duration_sec=1.)
                raise RuntimeError('SNOWBOT_VENDOR_RECEIVER_UNAVAILABLE:'+group)


def assemble_snowbot(node, profile):
    devices = SnowbotDevices(node, profile)
    node.snowbot_devices = devices
    return devices.io, {}


class SnowbotAdapter(ExecutionNode):
    def __init__(self):
        super().__init__(allow_hardware=True, device_factory=assemble_snowbot,
                         executor_factory=DelegatedPositionExecutor)
        self.get_logger().info('Snowbot adapter: command_enabled=%s; interpolation owned by vendor' %
                               self.robot_io.command_enabled)

    def fault(self, request, reply):
        reply.ok, reply.code = False, 'SNOWBOT_SIM_FAULT_UNAVAILABLE'
        return reply


def main():
    # rclpy's default SIGINT handling shuts down DDS before destroy_node();
    # then a final hold cannot be delivered. Keep this entry's context alive
    # through the bounded hold/feedback handshake. Other entries use spin().
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    requested = False
    def request_shutdown(signum, frame):
        nonlocal requested
        requested = True
    previous = {s: signal.signal(s, request_shutdown) for s in (signal.SIGINT, signal.SIGTERM)}
    executor = SingleThreadedExecutor()
    node = None
    deadline = None
    exit_code = 0
    try:
        node = SnowbotAdapter()
        executor.add_node(node)
        while rclpy.ok():
            if requested and deadline is None:
                node.robot_io.command_enabled = False  # Fence new acquisitions and writes.
                node.engine.halt(node.now(), 'SHUTDOWN', revoke=True, emergency=True)
                deadline = time.monotonic()+node.profile.stop_timeout+.5
            if requested:
                if not node.engine.stopping:
                    break
                if time.monotonic() >= deadline:
                    node.robot_io.stop_failures = ('vendor_hold',)
                    node.engine.halt(node.now(), 'STOP_UNCONFIRMED', revoke=True, emergency=True)
                    break
            executor.spin_once(timeout_sec=.02)
        if node.robot_io.stop_failures:
            node.get_logger().error('Snowbot shutdown: physical hold unconfirmed')
            exit_code = 1
        elif requested:
            node.get_logger().info('Snowbot shutdown: hold confirmed or no commanded axes')
    finally:
        executor.shutdown(timeout_sec=1.)
        if node is not None:
            node.snowbot_devices.heartbeat_running = False
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    raise SystemExit(exit_code)
