#!/usr/bin/env python3
"""ROS 2 numerical migration check with generated, time-resolved 3D scans/IMU.

No device commands, Isaac ground truth injection, or motion servers are used.
Synthetic geometry is a transport/numerical test, not a precision benchmark.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


def trajectory(t):
    t = max(0., t-2.)
    # Bounded analytic motion, initially stationary with zero acceleration.
    x = .25 * (1. - math.cos(.6*t))
    vx = .15 * math.sin(.6*t)
    ax = .09 * math.cos(.6*t) if t else 0.
    yaw = .12 * (1. - math.cos(.5*t))
    omega = .06 * math.sin(.5*t)
    return x, vx, ax, yaw, omega


def geometry():
    rng = np.random.default_rng(123)
    points = []
    for axis, coordinate in ((0, -3.), (0, 4.), (1, -2.), (1, 3.), (2, -1.), (2, 2.)):
        cloud = rng.uniform([-3., -2., -1.], [4., 3., 2.], size=(500, 3))
        cloud[:, axis] = coordinate
        points.append(cloud)
    return np.concatenate(points)


class Probe(Node):
    def __init__(self, namespace):
        super().__init__('lio_probe', namespace=namespace)
        self.odometry = []
        self.statuses = []
        self.cloud = self.create_publisher(PointCloud2, 'lio/points', qos_profile_sensor_data)
        self.imu = self.create_publisher(Imu, 'lio/imu', qos_profile_sensor_data)
        self.create_subscription(Odometry, 'lio/odom', self.odometry.append, qos_profile_sensor_data)
        self.create_subscription(String, 'lio/status', lambda m: self.statuses.append(m.data), 10)
        self.save = self.create_client(Trigger, 'lio/save_map')

    def spin_for(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.002)

    def send_imu(self, stamp, elapsed):
        _, _, acceleration, yaw, omega = trajectory(elapsed)
        message = Imu()
        message.header.stamp = rclpy.time.Time(seconds=stamp).to_msg()
        message.header.frame_id = 'imu'
        message.linear_acceleration.x = acceleration*math.cos(yaw)
        message.linear_acceleration.y = -acceleration*math.sin(yaw)
        message.linear_acceleration.z = 9.81
        message.angular_velocity.z = omega
        self.imu.publish(message)

    def send_cloud(self, stamp, elapsed, world, malformed=False):
        offsets = np.linspace(0., .099, len(world))
        points = np.zeros((len(world), 5), dtype='<f4')
        for i, offset in enumerate(offsets):
            x, _, _, yaw, _ = trajectory(elapsed+offset)
            px, py, pz = world[i]-[x, 0., 0.]
            points[i] = [math.cos(yaw)*px+math.sin(yaw)*py,
                         -math.sin(yaw)*px+math.cos(yaw)*py, pz, 1., offset]
        message = PointCloud2()
        message.header.stamp = rclpy.time.Time(seconds=stamp).to_msg()
        message.header.frame_id = 'lidar'
        message.height = 1
        message.width = len(world)
        message.point_step = 20
        message.row_step = len(world)*20
        message.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                          for i, name in enumerate(('x', 'y', 'z', 'intensity', 'missing_time' if malformed else 'time'))]
        message.data = points.tobytes()
        self.cloud.publish(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    namespace = '/lio_migration_check'
    map_path = output/'map.pcd'
    command = [args.binary, '--ros-args', '-r', '__ns:='+namespace,
               '-r', '/tf:='+namespace+'/tf', '-p', 'imu_from_lidar:=[0.,0.,0.,0.,0.,0.,1.]',
               '-p', 'imu_from_base:=[0.,0.,0.,0.,0.,0.,1.]', '-p', 'save_map_path:='+str(map_path)]
    rclpy.init()
    probe = Probe(namespace)
    log = (output/'lio.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    results = {}
    try:
        deadline = time.monotonic()+12.
        while probe.cloud.get_subscription_count() == 0 or probe.imu.get_subscription_count() == 0:
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('LIO_STARTUP_FAILED')
            probe.spin_for(.05)
        origin = time.time()-.15
        started = time.monotonic()
        imu_index, scan_index = 0, 0
        world = geometry()
        while time.monotonic()-started < 9.:
            elapsed = time.monotonic()-started
            while imu_index*.005 <= elapsed:
                probe.send_imu(origin+imu_index*.005, imu_index*.005)
                imu_index += 1
            # Send a completed scan (100 ms acquisition); no instantaneous
            # whole-cloud pose is substituted for per-point acquisition poses.
            if (scan_index+1)*.1 <= elapsed:
                if scan_index == 5:
                    probe.send_cloud(origin+scan_index*.1, scan_index*.1, world, malformed=True)
                    probe.spin_for(.015)
                probe.send_cloud(origin+scan_index*.1, scan_index*.1, world)
                scan_index += 1
            rclpy.spin_once(probe, timeout_sec=.001)
            if process.poll() is not None:
                raise RuntimeError('LIO_PROCESS_EXITED')
        probe.spin_for(.8)
        count = len(probe.odometry)
        probe.spin_for(.3)
        results['stale_stops_output'] = count == len(probe.odometry) and any('LIO_INPUT_STALE' in s for s in probe.statuses)
        assert results['stale_stops_output'], probe.statuses
        errors = []
        angles = []
        stamps = []
        trace = []
        for message in probe.odometry:
            stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
            elapsed = stamp-origin
            expected_x, _, _, expected_yaw, _ = trajectory(elapsed)
            p, q = message.pose.pose.position, message.pose.pose.orientation
            errors.append(math.dist([p.x, p.y, p.z], [expected_x, 0., 0.]))
            measured_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            angles.append(abs(measured_yaw-expected_yaw))
            stamps.append(stamp)
            trace.append({'stamp': stamp, 'position': [p.x, p.y, p.z],
                          'expected_position': [expected_x, 0., 0.],
                          'yaw': measured_yaw, 'expected_yaw': expected_yaw})
        (output/'odometry.json').write_text(json.dumps(trace, indent=2)+'\n')
        results['malformed_time_field_rejected'] = 'TIMED_CLOUD_SCHEMA_REQUIRED' in probe.statuses
        assert results['malformed_time_field_rejected']
        results['measurement_time_preserved'] = all(abs(((s-origin-.099)/.1)-round((s-origin-.099)/.1)) < 1e-4 for s in stamps)
        assert results['measurement_time_preserved']
        results.update(samples=count, max_position_error=max(errors, default=999.),
                       max_yaw_error=max(angles, default=999.), source_stamps_monotonic=all(b>a for a,b in zip(stamps,stamps[1:])))
        assert count > 40 and max(errors) < .06 and max(angles) < .03, results
        assert results['source_stamps_monotonic']
        assert probe.save.wait_for_service(timeout_sec=2.)
        responses = []
        for _ in range(2):
            future = probe.save.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(probe, future, timeout_sec=3.)
            assert future.done()
            responses.append(future.result())
        results['map_saved_and_overwrite_rejected'] = (responses[0].success and map_path.is_file()
                                                     and not responses[1].success)
        assert results['map_saved_and_overwrite_rejected']
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
