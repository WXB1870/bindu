"""RoboSense Airy point cloud -> bounded planar scan across two ROS domains.

The source domain keeps the full cloud; only LaserScan and one calibrated
static transform enter the control domain. This module never sends commands.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import numpy as np
from sensor_msgs.msg import LaserScan, PointField
from sensor_msgs_py.point_cloud2 import read_points_numpy


@dataclass(frozen=True)
class AiryScanConfig:
    source_domain: int
    control_domain: int
    source_topic: str
    scan_topic: str
    source_frame: str
    base_frame: str
    scan_frame: str
    base_from_lidar: tuple
    min_height: float
    max_height: float
    min_range: float
    max_range: float
    bins: int
    min_bins: int
    max_points: int
    max_input_age: float

    @classmethod
    def load(cls, path):
        raw = json.loads(Path(path).read_text())
        required = set(cls.__dataclass_fields__)
        if not isinstance(raw, dict) or set(raw) != required:
            raise ValueError('AIRY_CONFIG_FIELDS')
        pending = [name for name in ('base_frame', 'base_from_lidar', 'min_height', 'max_height')
                   if raw[name] is None]
        if pending:
            raise ValueError('AIRY_CALIBRATION_REQUIRED:' + ','.join(pending))
        if (any(type(raw[name]) is not int or not 0 <= raw[name] <= 232
                for name in ('source_domain', 'control_domain')) or
                raw['source_domain'] == raw['control_domain']):
            raise ValueError('AIRY_DOMAIN_CONFIG')
        if (any(not isinstance(raw[name], str) or not raw[name].startswith('/')
                or raw[name] == '/' for name in ('source_topic', 'scan_topic')) or
                not raw['scan_topic'].endswith('/navigation/scan') or
                any(not isinstance(raw[name], str) or not raw[name] or '/' in raw[name]
                    for name in ('source_frame', 'base_frame', 'scan_frame')) or
                len({raw[name] for name in ('source_frame', 'base_frame', 'scan_frame')}) != 3):
            raise ValueError('AIRY_TOPIC_OR_FRAME_CONFIG')
        transform = raw['base_from_lidar']
        if (not isinstance(transform, list) or len(transform) != 7 or
                any(type(value) not in (int, float) or not math.isfinite(value)
                    for value in transform)):
            raise ValueError('AIRY_EXTRINSIC')
        norm = math.sqrt(sum(value*value for value in transform[3:]))
        if abs(norm-1.) > 1e-3:
            raise ValueError('AIRY_EXTRINSIC')
        for name in ('min_height', 'max_height', 'min_range', 'max_range', 'max_input_age'):
            if type(raw[name]) not in (int, float) or not math.isfinite(raw[name]):
                raise ValueError('AIRY_SCAN_LIMITS')
        if (raw['min_height'] >= raw['max_height'] or
                not 0 < raw['min_range'] < raw['max_range'] or
                not 0 < raw['max_input_age'] <= 2. or
                any(type(raw[name]) is not int for name in ('bins', 'min_bins', 'max_points')) or
                not 8 <= raw['bins'] <= 10000 or
                not 1 <= raw['min_bins'] <= raw['bins'] or
                not 1 <= raw['max_points'] <= 2000000):
            raise ValueError('AIRY_SCAN_LIMITS')
        raw['base_from_lidar'] = tuple(transform)
        return cls(**raw)

    def rotation(self):
        x, y, z, w = self.base_from_lidar[3:]
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])


def cloud_stamp(message):
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('AIRY_INVALID_STAMP')
    value = stamp.sec + stamp.nanosec * 1e-9
    if value <= 0:
        raise ValueError('AIRY_INVALID_STAMP')
    return value


def project_cloud(message, config, now, last_stamp=0.):
    """Return one scan or raise; no invalid cloud refreshes the input stamp."""
    stamp = cloud_stamp(message)
    if message.header.frame_id != config.source_frame:
        raise ValueError('AIRY_FRAME_MISMATCH')
    if stamp <= last_stamp or not 0 <= now-stamp <= config.max_input_age:
        raise ValueError('AIRY_STALE_CLOUD')
    if (message.is_bigendian or not 0 < message.width*message.height <= config.max_points or
            message.point_step < 12 or message.row_step < message.point_step*message.width or
            len(message.data) != message.row_step*message.height):
        raise ValueError('AIRY_CLOUD_LAYOUT')
    fields = {field.name: field for field in message.fields}
    if len(fields) != len(message.fields):
        raise ValueError('AIRY_CLOUD_LAYOUT')
    offsets = []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if (field is None or field.datatype != PointField.FLOAT32 or field.count != 1 or
                field.offset < 0 or field.offset+4 > message.point_step):
            raise ValueError('AIRY_CLOUD_LAYOUT')
        offsets.append(field.offset)
    if len(set(offsets)) != 3 or any(abs(a-b) < 4 for a in offsets for b in offsets if a != b):
        raise ValueError('AIRY_CLOUD_LAYOUT')
    try:
        points = read_points_numpy(message, field_names=('x', 'y', 'z'), skip_nans=False).reshape(-1, 3)
    except (ValueError, TypeError, AssertionError) as exc:
        raise ValueError('AIRY_CLOUD_LAYOUT') from exc
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError('AIRY_NO_FINITE_POINTS')
    origin = np.asarray(config.base_from_lidar[:3])
    base = points @ config.rotation().T + origin
    local = base[:, :2]-origin[:2]
    distance = np.linalg.norm(local, axis=1)
    keep = ((base[:, 2] >= config.min_height) & (base[:, 2] <= config.max_height) &
            (distance >= config.min_range) & (distance <= config.max_range))
    indices = np.floor((np.arctan2(local[keep, 1], local[keep, 0])+math.pi)
                       * config.bins/(2*math.pi)).astype(int)
    indices = np.clip(indices, 0, config.bins-1)
    ranges = np.full(config.bins, np.inf)
    np.minimum.at(ranges, indices, distance[keep])
    if np.count_nonzero(np.isfinite(ranges)) < config.min_bins:
        raise ValueError('AIRY_SCAN_TOO_SPARSE')
    ranges[~np.isfinite(ranges)] = np.nan  # No return is unknown, never free space.
    scan = LaserScan()
    scan.header.stamp = message.header.stamp
    scan.header.frame_id = config.scan_frame
    scan.angle_min = -math.pi
    scan.angle_increment = 2*math.pi/config.bins
    scan.angle_max = scan.angle_min+(config.bins-1)*scan.angle_increment
    scan.range_min, scan.range_max = config.min_range, config.max_range
    scan.ranges = ranges.astype(np.float32).tolist()
    return scan


def run(config):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import StaticTransformBroadcaster

    source_context, control_context = Context(), Context()
    source = output = executor = None
    try:
        rclpy.init(context=source_context, domain_id=config.source_domain)
        rclpy.init(context=control_context, domain_id=config.control_domain)
        source = Node('bindu_airy_scan_input', context=source_context)
        tf_topic = config.scan_topic[:-len('/navigation/scan')] + '/tf_static'
        output = Node('bindu_airy_scan_output', context=control_context,
                      cli_args=['--ros-args', '-r', '/tf_static:=' + tf_topic])
        publisher = output.create_publisher(LaserScan, config.scan_topic, qos_profile_sensor_data)
        static = StaticTransformBroadcaster(output)
        transform = TransformStamped()
        transform.header.frame_id = config.base_frame
        transform.child_frame_id = config.scan_frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = config.base_from_lidar[:3]
        transform.transform.rotation.w = 1.
        static.sendTransform(transform)
        last_stamp = 0.
        last_warning = 0.

        def receive(message):
            nonlocal last_stamp, last_warning
            now = source.get_clock().now().nanoseconds * 1e-9
            try:
                scan = project_cloud(message, config, now, last_stamp)
            except ValueError as exc:
                if time.monotonic()-last_warning > 5.:
                    source.get_logger().warning(str(exc))
                    last_warning = time.monotonic()
                return
            last_stamp = cloud_stamp(message)
            publisher.publish(scan)

        source.create_subscription(PointCloud2, config.source_topic, receive, qos_profile_sensor_data)
        executor = SingleThreadedExecutor(context=source_context)
        executor.add_node(source)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    finally:
        if executor is not None:
            executor.shutdown()
        if source is not None:
            source.destroy_node()
        if output is not None:
            output.destroy_node()
        if source_context.ok():
            source_context.shutdown()
        if control_context.ok():
            control_context.shutdown()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Airy point cloud to bounded navigation scan, no motion output')
    parser.add_argument('--config', required=True)
    parser.add_argument('--check', action='store_true', help='validate configuration without starting ROS')
    args = parser.parse_args()
    try:
        config = AiryScanConfig.load(args.config)
    except (OSError, ValueError) as exc:
        parser.exit(2, str(exc) + '\n')
    if args.check:
        print('AIRY_SCAN_CONFIG_OK')
    else:
        run(config)


if __name__ == '__main__':
    main()
