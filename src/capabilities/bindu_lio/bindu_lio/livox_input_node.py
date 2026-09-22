#!/usr/bin/env python3
"""Livox CustomMsg -> timed PointCloud2; preserves acquisition timestamps."""
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class LivoxInput(Node):
    def __init__(self):
        super().__init__('livox_input')
        # Optional driver dependency: the driver is supplied by the hardware
        # workspace, never started or replaced by this adaptation node.
        from livox_ros_driver2.msg import CustomMsg
        self.frame = self.declare_parameter('lidar_frame', 'lidar').value
        self.lines = self.declare_parameter('scan_lines', 4).value
        self.max_points = self.declare_parameter('max_scan_points', 100000).value
        if not 1 <= self.lines <= 128 or self.max_points <= 0:
            raise ValueError('INVALID_LIVOX_LIMITS')
        self.publisher = self.create_publisher(PointCloud2, 'lio/points', qos_profile_sensor_data)
        self.create_subscription(CustomMsg, 'livox/lidar', self.receive, qos_profile_sensor_data)

    def receive(self, message):
        if (message.header.frame_id != self.frame or message.point_num != len(message.points)
                or not 0 < message.point_num <= self.max_points):
            self.get_logger().warning('LIVOX_LAYOUT_OR_FRAME_REJECTED')
            return
        # Both timestamps describe the first point, in nanoseconds. Do not hide
        # an incompatible driver clock by substituting receipt time.
        stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if abs(int(message.timebase) - stamp_ns) > 1000000:
            self.get_logger().warning('LIVOX_TIMEBASE_MISMATCH')
            return
        data = bytearray()
        for point in message.points:
            if point.line >= self.lines or (point.tag & 0x30) not in (0x00, 0x10):
                continue
            if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
                self.get_logger().warning('LIVOX_NONFINITE_POINT')
                return
            data.extend(struct.pack('<fffff', point.x, point.y, point.z,
                                    float(point.reflectivity), point.offset_time * 1e-9))
        cloud = PointCloud2()
        cloud.header = message.header
        cloud.height = 1
        cloud.width = len(data) // 20
        cloud.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                        for index, name in enumerate(('x', 'y', 'z', 'intensity', 'time'))]
        cloud.point_step = 20
        cloud.row_step = len(data)
        cloud.is_dense = True
        cloud.data = bytes(data)
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = None
    try:
        node = LivoxInput()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
