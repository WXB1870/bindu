"""Read-only SOEM ROS feedback mapping for the second wheel robot.

Subscribes only; never creates a RobotCommand publisher or touches shared memory.
It is independent of the unfinished wheel robot motion profile.
"""
import argparse
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from bindu_hardware.soem_feedback import SoemFeedbackGate


def seconds(stamp):
    if stamp.nanosec >= 1000000000 or stamp.sec < 0:
        raise ValueError('SOEM_JOINT_STAMP')
    return stamp.sec + stamp.nanosec * 1e-9


class SoemJointFeedback(Node):
    def __init__(self, mapping_path, source_state, source_status, output_topic):
        super().__init__('soem_joint_feedback')
        config = json.loads(Path(mapping_path).read_text())
        self.gate = SoemFeedbackGate(config)
        self.publisher = self.create_publisher(JointState, output_topic, qos_profile_sensor_data)
        self.create_subscription(JointState, source_state, self.on_state, qos_profile_sensor_data)
        self.create_subscription(String, source_status, self.on_status, 10)
        self.last_invalid = ''
        self.get_logger().info('Read-only SOEM feedback: %s -> %s (%d mapped joints)' %
                               (source_state, output_topic, len(self.gate.mapping.joints)))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        try:
            self.gate.observe_status(msg.data, self.now())
        except ValueError as exc:
            self._invalid(str(exc))

    def on_state(self, msg):
        try:
            self.gate.observe_joint_state(msg.name, msg.position, seconds(msg.header.stamp), self.now())
        except ValueError as exc:
            self._invalid(str(exc))
            return
        accepted = self.gate.take(self.now())
        if accepted is None:
            return
        _, positions = accepted
        out = JointState()
        out.header = msg.header
        out.name = list(positions)
        out.position = list(positions.values())
        self.publisher.publish(out)
        self.last_invalid = ''

    def _invalid(self, code):
        if code != self.last_invalid:
            self.get_logger().warn(code)
            self.last_invalid = code


def main():
    parser = argparse.ArgumentParser(description='Read-only 48-channel SOEM joint feedback mapping')
    parser.add_argument('--mapping', required=True)
    parser.add_argument('--source-state', default='/robot/joint_states')
    parser.add_argument('--source-status', default='/robot/shm_status')
    parser.add_argument('--output-topic', default='/bindu_wheel_gen2/joint_states')
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = None
    try:
        node = SoemJointFeedback(args.mapping, args.source_state, args.source_status, args.output_topic)
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
