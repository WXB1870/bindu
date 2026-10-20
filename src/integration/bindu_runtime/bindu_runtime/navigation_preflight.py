"""Bounded read-only navigation preflight: subscriptions and discovery only."""
import math
import time
import json
import subprocess
import tempfile
from pathlib import Path

from bindu_navigation.orin import InputWindow


def seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def run(site, stage):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import LaserScan, PointCloud2, Imu
    from nav_msgs.msg import Odometry, OccupancyGrid
    from tf2_ros import Buffer, TransformListener
    import numpy as np
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    required_packages = ['bindu_runtime', 'bindu_navigation']
    if stage != 'sensors':
        required_packages += ['nav2_map_server']
        required_packages += ['slam_toolbox', 'nav2_amcl'] if site['route'] == '2d' else ['bindu_lio']
    required_packages += site['hardware']['driver_packages']
    errors = []
    for package in required_packages:
        try: get_package_prefix(package)
        except PackageNotFoundError: errors.append('DEPENDENCY_MISSING: '+package+'；按选定路线安装/加载驱动覆盖层')
    if errors: return errors
    rclpy.init(args=['--ros-args', '-r', '/tf:='+site['tf_topic'], '-r', '/tf_static:='+site['tf_static_topic']])
    node = rclpy.create_node('bindu_navigation_preflight', namespace=site['namespace'])
    # Drivers may use either the global TF topics or an explicitly isolated graph.
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    window = InputWindow(site['checks']['max_age'], site['checks']['future_tolerance'])
    f = site['frames']; route = site['route']
    now = lambda: node.get_clock().now().nanoseconds/1e9
    required = []
    edges = {}; edge_time = {}; edge_static = {}; messages = {}
    def sub(name, kind, topic, frame, validate=lambda m: True):
        required.append(name)
        def receive(message):
            messages[name] = message
            window.observe(name, seconds(message.header.stamp), now(),
                           message.header.frame_id == frame and validate(message))
        node.create_subscription(kind, topic, receive, qos_profile_sensor_data)
    def scan_valid(m):
        return (len(m.ranges) > 8 and math.isfinite(m.angle_increment) and m.angle_increment > 0 and
                0 <= m.range_min < m.range_max and
                sum(math.isfinite(v) and m.range_min <= v <= m.range_max for v in m.ranges) >= 8)
    def odom_valid(m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        return (m.child_frame_id == f['base'] and
                all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.linear.x,v.angular.z)) and
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < 1e-3)
    # Raw LIO sensor checks can run before the estimator creates odom and scan.
    if route == '2d' or stage != 'sensors':
        sub('scan', LaserScan, site['scan_topic'], f['scan'], scan_valid)
        sub('odom', Odometry, site['odom_topic'], f['odom'], odom_valid)
    if route == 'lio':
        def cloud_valid(m):
            fields = {field.name: field for field in m.fields}
            schema = (m.width*m.height > 0 and m.point_step > 0 and len(m.data) == m.row_step*m.height and
                    not m.is_bigendian and all(key in fields and fields[key].datatype == 7 and
                    fields[key].count == 1 and fields[key].offset+4 <= m.point_step
                    for key in ('x','y','z','intensity','time')))
            if not schema: return False
            try:
                values = read_points_numpy(m, field_names=('x','y','z','intensity','time'), skip_nans=False).reshape(-1,5)
                return bool(np.isfinite(values).all() and np.all(values[:,4]>=0) and np.all(values[:,4]<=.2))
            except (ValueError,TypeError,AssertionError): return False
        def imu_valid(m):
            return (m.angular_velocity_covariance[0] != -1 and m.linear_acceleration_covariance[0] != -1 and
                    all(math.isfinite(x) for x in (m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,
                                                  m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z)))
        sub('points', PointCloud2, site['points_topic'], f['lidar'], cloud_valid)
        sub('imu', Imu, site['imu_topic'], f['imu'], imu_valid)
    probe = Path(get_package_prefix('bindu_navigation'))/'lib/bindu_navigation/navigation_tf_probe'
    if not probe.is_file():
        node.destroy_node(); rclpy.shutdown()
        return ['TF_PROBE_NOT_BUILT: 重新构建bindu_navigation，Humble Python元数据不足以检查TF发布者']
    source_log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([str(probe),site['tf_topic'],site['tf_static_topic'],
                                str(site['checks']['sample_seconds'])],stdout=source_log,stderr=subprocess.DEVNULL)
    if stage in ('mapping','localization','navigation'):
        def map_receive(m): messages['map'] = m
        node.create_subscription(OccupancyGrid, site['namespace']+'/navigation/map', map_receive,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        end = time.monotonic()+site['checks']['sample_seconds']
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        process.wait(timeout=3)
        if process.returncode: errors.append('TF_PROBE_FAILED: 检查已构建原生探针及ROS环境')
        source_log.seek(0)
        for line in source_log:
            record = json.loads(line)
            key = (record['parent'], record['child'])
            edges.setdefault(key,set()).add(record['gid'])
            edge_time[key] = record['stamp']; edge_static[key] = record['static']
        errors += window.finish(required, now())
        parents = {}
        for (parent, child), sources in edges.items():
            parents.setdefault(child, set()).add(parent)
            if len(sources) != 1: errors.append('TF_DUPLICATE_AUTHORITY: '+parent+'→'+child+'；关闭重复发布者')
        publishers = {}
        for topic in (site['tf_topic'],site['tf_static_topic']):
            for pub in node.get_publishers_info_by_topic(topic):
                publishers[bytes(pub.endpoint_gid).hex()] = pub.node_namespace.rstrip('/')+'/'+pub.node_name
        expected = {(f['odom'],f['base']): site['tf_authorities']['odom_to_base']}
        if route == 'lio': expected[f['odom'],f['base']] = site['namespace']+'/fast_lio'
        if stage in ('mapping','localization','navigation'):
            name = ('slam_toolbox' if stage=='mapping' else 'amcl') if route=='2d' else ('fast_lio' if stage=='mapping' else 'lio_localization')
            expected[f['map'],f['odom']] = site['namespace']+'/'+name
        for (parent,child), sources in edges.items():
            owner = expected.get((parent,child))
            if child == f['scan']: owner = site['tf_authorities']['base_to_scan']
            if owner and not (route=='lio' and stage=='sensors' and parent==f['odom']):
                if any(publishers.get(source) != owner for source in sources):
                    errors.append('TF_AUTHORITY_MISMATCH: '+parent+'→'+child+'，期望 '+owner+'；核对节点全名/集中配置')
        for child, values in parents.items():
            if len(values) > 1: errors.append('TF_MULTIPLE_PARENTS: '+child)
        pairs = [(f['base'], f['scan'])]
        if route == '2d' or stage != 'sensors': pairs.append((f['odom'], f['base']))
        if route == 'lio': pairs += [(f['base'],f['lidar']),(f['base'],f['imu'])]
        if stage in ('mapping','localization','navigation'): pairs.append((f['map'],f['odom']))
        for parent, child in pairs:
            try:
                transform = buffer.lookup_transform(parent, child, rclpy.time.Time())
                # Latest static transform legitimately has stamp zero.
                if parent in (f['map'],f['odom']) and not -0.25 <= now()-seconds(transform.header.stamp) <= window.max_age:
                    errors.append('TF_STALE: '+parent+'→'+child)
            except Exception:
                errors.append('TF_UNAVAILABLE: '+parent+'→'+child+'；核对发布者、域、frame、TF话题')
        if 'scan' in messages:
            m = messages['scan']
            try:
                transform = buffer.lookup_transform(f['base'], f['scan'], rclpy.time.Time.from_msg(m.header.stamp))
                t, q = transform.transform.translation, transform.transform.rotation
                actual = [t.x,t.y,t.z,q.x,q.y,q.z,q.w]; expected = site['base_from_scan']
                if (max(abs(a-b) for a,b in zip(actual[:3],expected[:3])) > .001 or
                    abs(sum(a*b for a,b in zip(actual[3:],expected[3:]))) < .999999):
                    errors.append('SCAN_TF_CALIBRATION_MISMATCH: 实测TF与集中外参不一致；可动传感器需时间对齐运动链适配')
            except Exception: errors.append('SCAN_TIME_TF_UNAVAILABLE: 采集时刻无TF，不能用接收时刻代替')
        if stage == 'sensors':
            if (f['map'],f['odom']) in edges:
                errors.append('LOCALIZATION_ALREADY_RUNNING: 启动前先退出已有建图/定位，避免重复map→odom')
            if route == 'lio' and (f['odom'],f['base']) in edges:
                errors.append('ODOM_AUTHORITY_CONFLICT: LIO将发布odom→base；先关闭现有发布者')
        if stage in ('mapping','localization','navigation'):
            m = messages.get('map')
            if m is None or m.header.frame_id != f['map'] or len(m.data) != m.info.width*m.info.height or not any(v >= 0 for v in m.data):
                errors.append('MAP_UNAVAILABLE: 等待地图；定位时检查map_server及地图frame')
            key = (f['map'],f['odom'])
            if key not in edges or edge_static.get(key) or not -0.25 <= now()-edge_time.get(key, 0) <= window.max_age:
                errors.append('LOCALIZATION_TF_STALE_OR_STATIC: 需本阶段算法持续产生map→odom；AMCL/ICP需RViz近似初值')
            services = dict(node.get_service_names_and_types())
            service = site['namespace']+('/slam_toolbox/serialize_map' if route=='2d' else '/lio/save_map')
            if stage == 'mapping' and service not in services:
                errors.append('MAP_SAVE_SERVICE_MISSING: '+service)
        print('观测消息：'+', '.join(f'{name}={count}' for name, (_,count) in window.samples.items()))
        return errors
    finally:
        if process.poll() is None: process.terminate(); process.wait(timeout=3)
        source_log.close()
        node.destroy_node(); rclpy.shutdown()
