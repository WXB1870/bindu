"""Opt-in Snowbot execution endpoint; vendor nodes are never launched here."""
from bindu_execution.delegated_position import DelegatedPositionExecutor
from bindu_hardware.snowbot import GROUPS, SnowbotIO, decode_feedback
from .execution_node import ExecutionNode
from .common import spin


class SnowbotDevices:
    def __init__(self, node, profile):
        # Optional message package: other robot assemblies do not import it.
        try:
            from snowbot_ctl_msgs.msg import JointPositionCommand, RetInformation
        except ImportError as exc:
            raise RuntimeError('SNOWBOT_CTL_MSGS_NOT_INSTALLED') from exc
        from std_msgs.msg import Float64MultiArray
        from rclpy.qos import qos_profile_sensor_data
        self.node = node
        self.arm_type, self.array_type = JointPositionCommand, Float64MultiArray
        node.declare_parameter('command_enabled', False)
        node.declare_parameter('vendor_namespace', '/snowbot')
        prefix = node.get_parameter('vendor_namespace').value
        if (not isinstance(prefix, str) or not prefix.startswith('/') or
                (prefix != '/' and prefix != prefix.rstrip('/')) or '//' in prefix):
            raise ValueError('SNOWBOT_VENDOR_NAMESPACE')
        self.io = SnowbotIO(profile, self.send, node.now,
                            command_enabled=node.get_parameter('command_enabled').value,
                            simulated=node.simulation)
        topic = lambda name: prefix.rstrip('/')+'/'+name
        self.feedback_topic = topic('dualArmCtl_fdback')
        self.command_topics = {'arms': topic('joint_position_cmd'),
                               'waist': topic('body/motor_cmd'), 'head': topic('head/motor_cmd')}
        self.publishers = {
            'arms': node.create_publisher(JointPositionCommand, topic('joint_position_cmd'), 1),
            'waist': node.create_publisher(Float64MultiArray, topic('body/motor_cmd'), 1),
            'head': node.create_publisher(Float64MultiArray, topic('head/motor_cmd'), 1),
        } if self.io.command_enabled else {}
        node.create_subscription(RetInformation, self.feedback_topic, self.receive,
                                 qos_profile_sensor_data)

    def receive(self, msg):
        try:
            if self.node.count_publishers(self.feedback_topic) != 1:
                raise ValueError('SNOWBOT_FEEDBACK_PUBLISHER_COUNT')
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
        for group, _ in packets:
            if (self.publishers[group].get_subscription_count() != 1 or
                    self.node.count_publishers(self.command_topics[group]) != 1):
                raise RuntimeError('SNOWBOT_VENDOR_SUBSCRIBER_COUNT:'+group)
        for group, msg in packets:
            self.publishers[group].publish(msg)


def assemble_snowbot(node, profile):
    devices = SnowbotDevices(node, profile)
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
    spin(SnowbotAdapter, threaded=False)
