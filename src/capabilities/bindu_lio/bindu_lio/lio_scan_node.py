#!/usr/bin/env python3
"""Deskewed LIO cloud -> planar navigation scan using explicit sensor geometry."""
from collections import deque
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py.point_cloud2 import read_points_numpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from bindu_lio.localization import inverse_se3, pose_matrix
from bindu_lio.lio_localization_node import matrix_from_pose, stamp_seconds


class LioScan(Node):
    def __init__(self):
        super().__init__('lio_scan')
        self.frames = {name: self.declare_parameter(name+'_frame', default).value
                       for name, default in [('odom', 'odom'), ('base', 'base_link'), ('lidar', 'lidar')]}
        def transform(name):
            values = self.declare_parameter(name, [0.]*7).value
            if len(values) != 7:
                raise ValueError('SEVEN_VALUE_TRANSFORM_REQUIRED')
            return pose_matrix(values[:3], values[3:])
        self.base_from_lidar = inverse_se3(transform('imu_from_base')) @ transform('imu_from_lidar')
        self.minimum_height = self.declare_parameter('min_height', 0.).value
        self.maximum_height = self.declare_parameter('max_height', .25).value
        self.minimum_range = self.declare_parameter('min_range', .3).value
        self.maximum_range = self.declare_parameter('max_range', 30.).value
        self.bins = self.declare_parameter('bins', 720).value
        self.timeout = self.declare_parameter('input_timeout', .3).value
        self.pair_tolerance = self.declare_parameter('pair_tolerance', .02).value
        self.max_points = self.declare_parameter('max_scan_points', 100000).value
        values = [self.minimum_height, self.maximum_height, self.minimum_range, self.maximum_range,
                  self.timeout, self.pair_tolerance]
        if (not all(math.isfinite(v) for v in values) or self.minimum_height >= self.maximum_height
                or not 0. <= self.minimum_range < self.maximum_range or not 8 <= self.bins <= 10000
                or self.timeout <= 0. or self.pair_tolerance <= 0. or self.max_points <= 0):
            raise ValueError('INVALID_SCAN_PROJECTION')
        self.odometry = deque(maxlen=32)
        self.pending = None
        self.last_stamp = 0.
        self.last_odom_stamp = 0.
        self.publisher = self.create_publisher(LaserScan, 'navigation/scan', qos_profile_sensor_data)
        self.create_subscription(Odometry, 'lio/odom', self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'lio/cloud_registered', self.receive_cloud, qos_profile_sensor_data)
        self.create_timer(.01, self.publish_pending)
        if self.declare_parameter('publish_sensor_tf', False).value:
            from scipy.spatial.transform import Rotation
            self.static = StaticTransformBroadcaster(self)
            transform_msg = TransformStamped()
            transform_msg.header.stamp = self.get_clock().now().to_msg()
            transform_msg.header.frame_id = self.frames['base']
            transform_msg.child_frame_id = self.frames['lidar']
            p = self.base_from_lidar[:3, 3]
            q = Rotation.from_matrix(self.base_from_lidar[:3, :3]).as_quat()
            transform_msg.transform.translation.x = float(p[0])
            transform_msg.transform.translation.y = float(p[1])
            transform_msg.transform.translation.z = float(p[2])
            rotation = transform_msg.transform.rotation
            rotation.x, rotation.y, rotation.z, rotation.w = map(float, q)
            self.static.sendTransform(transform_msg)

    def fresh(self, stamp):
        return 0. <= self.get_clock().now().nanoseconds/1e9-stamp <= self.timeout

    def receive_odom(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if (message.header.frame_id != self.frames['odom'] or message.child_frame_id != self.frames['base']
                or not self.fresh(stamp) or stamp <= self.last_odom_stamp):
            return
        try:
            pose = matrix_from_pose(message.pose.pose)
        except ValueError:
            return
        self.last_odom_stamp = stamp
        self.odometry.append((stamp, pose))

    def receive_cloud(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if (message.header.frame_id == self.frames['odom'] and self.fresh(stamp)
                and stamp > self.last_stamp and 0 < message.width*message.height <= self.max_points):
            self.pending = message
            self.last_stamp = stamp

    def publish_pending(self):
        if self.pending is None or not self.odometry:
            return
        message = self.pending
        stamp = stamp_seconds(message.header.stamp)
        if not self.fresh(stamp):
            self.pending = None
            return
        odom_stamp, pose = min(self.odometry, key=lambda row: abs(row[0]-stamp))
        if abs(odom_stamp-stamp) > self.pair_tolerance:
            return
        self.pending = None
        try:
            points = read_points_numpy(message, field_names=('x', 'y', 'z'), skip_nans=False).reshape(-1, 3)
            if not np.isfinite(points).all():
                return
        except (ValueError, AssertionError, TypeError):
            return
        lidar_from_odom = inverse_se3(pose @ self.base_from_lidar)
        local = points @ lidar_from_odom[:3, :3].T + lidar_from_odom[:3, 3]
        distance = np.linalg.norm(local[:, :2], axis=1)
        keep = ((local[:, 2] >= self.minimum_height) & (local[:, 2] <= self.maximum_height)
                & (distance >= self.minimum_range) & (distance <= self.maximum_range))
        indices = np.floor((np.arctan2(local[keep, 1], local[keep, 0])+math.pi) * self.bins/(2*math.pi)).astype(int)
        indices = np.clip(indices, 0, self.bins-1)
        ranges = np.full(self.bins, np.inf)
        np.minimum.at(ranges, indices, distance[keep])
        # Absence of 3D returns in a height slice is unknown, not free space.
        ranges[~np.isfinite(ranges)] = np.nan
        scan = LaserScan()
        scan.header = message.header
        scan.header.frame_id = self.frames['lidar']
        scan.angle_min = -math.pi
        scan.angle_increment = 2*math.pi/self.bins
        scan.angle_max = scan.angle_min+(self.bins-1)*scan.angle_increment
        scan.range_min, scan.range_max = self.minimum_range, self.maximum_range
        scan.time_increment = 0.  # All points are deskewed to the same scan end.
        scan.scan_time = 0.
        scan.ranges = ranges.astype(np.float32).tolist()
        self.publisher.publish(scan)


def main():
    rclpy.init()
    node = None
    try:
        node = LioScan()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
