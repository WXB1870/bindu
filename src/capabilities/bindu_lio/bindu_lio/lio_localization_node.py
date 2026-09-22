#!/usr/bin/env python3
"""Seeded scan-to-PCD localization; one worker, bounded input, source-time TF."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from bindu_lio.localization import (
    RegistrationConfig, ScanMapRegistration, initial_map_from_odom, pose_matrix,
)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def matrix_from_pose(pose):
    p, q = pose.position, pose.orientation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


class LioLocalization(Node):
    def __init__(self):
        super().__init__('lio_localization')
        defaults = {'map_path': '', 'map_frame': 'map', 'odom_frame': 'odom',
                    'base_frame': 'base_link', 'input_timeout': .5, 'pair_tolerance': .02,
                    'max_scan_points': 100000, 'max_map_points': 2000000}
        defaults.update(vars(RegistrationConfig()))
        self.cfg = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        for name in ('input_timeout', 'pair_tolerance', 'max_scan_points', 'max_map_points'):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError('INVALID_LOCALIZATION_' + name.upper())
        if self.cfg['map_frame'] == self.cfg['odom_frame']:
            raise ValueError('DISTINCT_MAP_ODOM_FRAMES_REQUIRED')
        path = Path(self.cfg['map_path'])
        if not path.is_file() or path.suffix.lower() != '.pcd':
            raise ValueError('PCD_MAP_REQUIRED')
        import open3d as o3d
        cloud = o3d.io.read_point_cloud(str(path))
        if len(cloud.points) > self.cfg['max_map_points']:
            raise ValueError('MAP_CAPACITY_EXCEEDED')
        self.registration = ScanMapRegistration(np.asarray(cloud.points), RegistrationConfig(
            **{name: self.cfg[name] for name in vars(RegistrationConfig())}))
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='scan_map_icp')
        self.future = None
        self.odometry = deque(maxlen=64)
        self.scan = None
        self.initial_pose = None
        self.correction = None
        self.accepted_stamp = None
        self.generation = 0
        self.last_scan_stamp = 0.
        self.last_odom_stamp = 0.
        self.tf = TransformBroadcaster(self)
        self.status = self.create_publisher(String, 'lio/localization_status', 1)
        self.last_status = None
        self.create_subscription(Odometry, 'lio/odom', self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'lio/cloud_registered', self.receive_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.receive_initial, 1)
        self.create_timer(.02, self.update)
        self.report('INITIAL_POSE_REQUIRED')

    def report(self, code):
        if code != self.last_status:
            self.status.publish(String(data=code))
            self.get_logger().info(code)
            self.last_status = code

    def fresh(self, stamp):
        return 0. <= self.get_clock().now().nanoseconds/1e9-stamp <= self.cfg['input_timeout']

    def receive_initial(self, message):
        if message.header.frame_id != self.cfg['map_frame']:
            self.report('INITIAL_FRAME_REJECTED')
            return
        # Explicit seeds may use a zero stamp (RViz convention). Nonzero seeds
        # must be fresh; they do not fabricate an observation or publish TF.
        stamp = stamp_seconds(message.header.stamp)
        if stamp and not self.fresh(stamp):
            self.report('INITIAL_POSE_STALE')
            return
        try:
            self.initial_pose = matrix_from_pose(message.pose.pose)
        except ValueError as error:
            self.report(str(error))
            return
        self.generation += 1
        self.correction = None
        self.accepted_stamp = None
        self.report('INITIAL_POSE_ACCEPTED')

    def receive_odom(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if (message.header.frame_id != self.cfg['odom_frame']
                or message.child_frame_id != self.cfg['base_frame']
                or stamp <= self.last_odom_stamp or not self.fresh(stamp)):
            return
        try:
            pose = matrix_from_pose(message.pose.pose)
        except ValueError:
            return
        self.odometry.append((stamp, pose))
        self.last_odom_stamp = stamp

    def receive_scan(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if (message.header.frame_id != self.cfg['odom_frame'] or not self.fresh(stamp)
                or stamp <= self.last_scan_stamp
                or not 0 < message.width*message.height <= self.cfg['max_scan_points']):
            return
        try:
            rows = point_cloud2.read_points_numpy(message, field_names=('x', 'y', 'z'), skip_nans=False)
            points = np.asarray(rows, dtype=float).reshape(-1, 3).copy()
            if not np.isfinite(points).all():
                raise ValueError('NONFINITE_SCAN')
        except (ValueError, AssertionError, TypeError) as error:
            self.report('INVALID_SCAN:' + str(error))
            return
        self.scan = (stamp, points)
        self.last_scan_stamp = stamp

    def update(self):
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            stamp, generation = self.pending
            try:
                correction, fitness, rmse = future.result()
                if generation == self.generation and self.fresh(stamp):
                    self.correction = correction
                    self.initial_pose = None
                    self.accepted_stamp = stamp
                    self.report(f'LOCALIZED:fitness={fitness:.4f},rmse={rmse:.4f}')
            except Exception as error:
                if generation == self.generation:
                    self.accepted_stamp = None
                    self.report(str(error))
        if self.accepted_stamp is not None and not self.fresh(self.accepted_stamp):
            self.accepted_stamp = None
            self.report('REGISTRATION_STALE')
        if (self.future is None and self.scan is not None and self.odometry
                and (self.initial_pose is not None or self.correction is not None)):
            stamp, points = self.scan
            odom_stamp, odom = min(self.odometry, key=lambda row: abs(row[0]-stamp))
            if self.fresh(stamp) and abs(odom_stamp-stamp) <= self.cfg['pair_tolerance']:
                initial = (initial_map_from_odom(self.initial_pose, odom)
                           if self.initial_pose is not None else self.correction.copy())
                self.scan = None
                self.pending = (stamp, self.generation)
                self.future = self.pool.submit(self.registration.match, points, odom.copy(), initial)
        if self.accepted_stamp is not None and self.odometry:
            stamp = min(self.odometry[-1][0], self.last_scan_stamp)
            if self.fresh(stamp):
                self.publish_transform(stamp)

    def publish_transform(self, stamp):
        # Piecewise constant correction, usable only while its real registration
        # measurement remains fresh. No current-wall-time restamping on dropout.
        from scipy.spatial.transform import Rotation
        q = Rotation.from_matrix(self.correction[:3, :3]).as_quat()
        transform = TransformStamped()
        transform.header.stamp = rclpy.time.Time(seconds=stamp).to_msg()
        transform.header.frame_id = self.cfg['map_frame']
        transform.child_frame_id = self.cfg['odom_frame']
        p = self.correction[:3, 3]
        transform.transform.translation.x = float(p[0])
        transform.transform.translation.y = float(p[1])
        transform.transform.translation.z = float(p[2])
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = map(float, q)
        self.tf.sendTransform(transform)

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = LioLocalization()
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
