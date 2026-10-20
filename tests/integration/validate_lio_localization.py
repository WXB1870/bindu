#!/usr/bin/env python3
"""Actual ROS 2 PCD/ICP node, seeded localization and fail-closed TF tests."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage

from bindu_lio.localization import inverse_se3, pose_matrix, correction_distance


class Probe(Node):
    def __init__(self):
        super().__init__('icp_probe', namespace='/lio_registration_check')
        self.cloud = self.create_publisher(PointCloud2, 'lio/cloud_registered', qos_profile_sensor_data)
        self.odom = self.create_publisher(Odometry, 'lio/odom', qos_profile_sensor_data)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.transforms = []
        self.statuses = []
        self.create_subscription(TFMessage, 'tf', self.transforms.append, 100)
        self.create_subscription(String, 'lio/localization_status', lambda m: self.statuses.append(m.data), 10)

    def spin_for(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.002)

    def publish(self, points, wrong_frame=False):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='odom')
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.
        self.odom.publish(odom)
        header = Header(stamp=stamp, frame_id='unexpected' if wrong_frame else 'odom')
        self.cloud.publish(create_cloud_xyz32(header, points))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    import open3d as o3d
    points = np.asarray(o3d.io.read_point_cloud(args.map).points)
    correction = pose_matrix([.15, -.1, .05], [0., 0., math.sin(.025), math.cos(.025)])
    scan = (inverse_se3(correction) @ np.c_[points, np.ones(len(points))].T).T[:, :3]
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    log = (output/'localization.log').open('w')
    process = subprocess.Popen([sys.executable, '-m', 'bindu_lio.lio_localization_node',
        '--ros-args', '-r', '__ns:=/lio_registration_check', '-r', '/tf:=/lio_registration_check/tf',
        '-p', 'map_path:='+str(Path(args.map).resolve())], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    probe = Probe()
    results = {}
    try:
        deadline = time.monotonic()+20.
        while probe.cloud.get_subscription_count() == 0 or probe.initial.get_subscription_count() == 0:
            assert process.poll() is None and time.monotonic() < deadline, 'NODE_NOT_READY'
            probe.spin_for(.05)
        probe.publish(scan)
        probe.spin_for(.2)
        assert not probe.transforms, 'TF_BEFORE_INITIALIZATION'
        seed = PoseWithCovarianceStamped()
        seed.header.frame_id = 'map'
        seed.pose.pose.orientation.w = 1.
        probe.initial.publish(seed)
        probe.spin_for(.1)
        for _ in range(25):
            probe.publish(scan)
            probe.spin_for(.1)
        errors = []
        for batch in probe.transforms:
            for transform in batch.transforms:
                assert transform.header.frame_id == 'map' and transform.child_frame_id == 'odom'
                p, q = transform.transform.translation, transform.transform.rotation
                actual = pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                errors.append(correction_distance(correction, actual))
        assert len(errors) > 5, probe.statuses
        results['max_translation_error'] = max(e[0] for e in errors)
        results['max_rotation_error'] = max(e[1] for e in errors)
        assert results['max_translation_error'] < .02 and results['max_rotation_error'] < .01, results
        # Fresh odometry cannot keep a rejected point-cloud match alive.
        for _ in range(9):
            probe.publish(scan+100.)
            probe.spin_for(.1)
        before = len(probe.transforms)
        probe.spin_for(.2)
        results['bad_match_stops_tf'] = before == len(probe.transforms) and 'REGISTRATION_REJECTED' in probe.statuses
        assert results['bad_match_stops_tf'], probe.statuses
        # A fresh seed invalidates old corrections immediately. With wrong
        # frame scans, neither old work nor old TF may become current again.
        probe.initial.publish(seed)
        for _ in range(7):
            probe.publish(scan, wrong_frame=True)
            probe.spin_for(.1)
        results['reseed_and_wrong_frame_remain_unlocalized'] = before == len(probe.transforms)
        assert results['reseed_and_wrong_frame_remain_unlocalized']
        results['passed'] = True
    finally:
        results['statuses'] = probe.statuses
        (output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        log.close()
        probe.destroy_node()
        rclpy.shutdown()
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
