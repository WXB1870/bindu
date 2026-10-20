"""Orin navigation site validation; no ROS imports and no device defaults."""
import json
import math
from pathlib import Path


MOTION_BLOCK = ('BASE_ADAPTER_REQUIRED: Tracer 2.0 接口按要求占位；尚无经公共执行契约的底盘适配、'
                '硬件看门狗及真实停车反馈。需实现并验收后才能人工速度控制或导航；配置不能解除此保护。')


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def transform_ok(value):
    return (isinstance(value, list) and len(value) == 7 and all(map(number, value))
            and abs(sum(v*v for v in value[3:])-1) < 1e-3)



def relative_transform(parent_from_a, parent_from_b):
    """Return a_from_b for xyz+xyzw rigid transforms without ROS dependencies."""
    def mul(a,b):
        x,y,z,w=a; X,Y,Z,W=b
        return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]
    inverse = [-v for v in parent_from_a[3:6]]+[parent_from_a[6]]
    delta = [b-a for a,b in zip(parent_from_a[:3],parent_from_b[:3])]+[0.]
    position = mul(mul(inverse,delta),parent_from_a[3:])[:3]
    return position+mul(inverse,parent_from_b[3:])


def load_site(path):
    data = json.loads(Path(path).read_text())
    if data.get('schema') != 1 or data.get('runtime_mode') != 'real':
        raise ValueError('ORIN_SITE_REQUIRED: schema=1、runtime_mode=real；不要使用仿真配方')
    return data


def map_errors(site):
    """Check saved map, including its image; no existence-only YAML acceptance."""
    import yaml
    errors = []
    path = site['map'].get('yaml')
    if not path or not Path(path).is_absolute() or not Path(path).is_file():
        return ['SAVED_MAP_REQUIRED: map.yaml 需指向本机已保存地图的绝对路径']
    try:
        cfg = yaml.safe_load(Path(path).read_text())
        image = Path(cfg['image'])
        if not image.is_absolute():
            image = Path(path).parent/image
        if not image.is_file() or image.stat().st_size == 0:
            errors.append('MAP_IMAGE_REQUIRED: 地图 YAML 引用的图像缺失或为空')
        if not number(cfg['resolution']) or cfg['resolution'] <= 0:
            errors.append('MAP_RESOLUTION_INVALID')
        if len(cfg['origin']) != 3 or not all(map(number, cfg['origin'])):
            errors.append('MAP_ORIGIN_INVALID')
        if not 0 <= cfg['free_thresh'] < cfg['occupied_thresh'] <= 1:
            errors.append('MAP_THRESHOLDS_INVALID')
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        errors.append('MAP_YAML_INVALID: '+str(exc))
    if site['route'] == 'lio':
        pcd = site['map'].get('pcd')
        if not pcd or not Path(pcd).is_absolute() or not Path(pcd).is_file():
            errors.append('PCD_MAP_REQUIRED: LIO 定位还需要同一建图批次的 PCD')
    return errors


def site_errors(site, stage='sensors', files=True):
    errors = []
    def require(ok, code):
        if not ok: errors.append(code)
    def text(value):
        return isinstance(value, str) and bool(value.strip())
    def topic(value):
        return text(value) and value.startswith('/') and ' ' not in value and '//' not in value
    driver = site.get('driver_setup')
    require(driver is None or (isinstance(driver,str) and Path(driver).is_absolute() and Path(driver).name=='local_setup.bash' and '\n' not in driver and (not files or Path(driver).is_file())), 'DRIVER_SETUP_INVALID: 仅填写本机ARM64驱动的绝对local_setup.bash路径或null')
    ns = site.get('namespace')
    require(topic(ns) and ns != '/' and not ns.endswith('/'), 'NAMESPACE_REQUIRED: 绝对命名空间且不以/结束')
    route = site.get('route')
    require(route in ('2d', 'lio'), 'ROUTE_REQUIRED: 现场确认后选择 2d 或 lio，不按历史雷达型号猜测')
    if route not in ('2d', 'lio'): return errors + ([MOTION_BLOCK] if stage == 'navigation' else [])
    f = site.get('frames', {})
    names = ('map', 'odom', 'base', 'scan') + (('lidar', 'imu') if route == 'lio' else ())
    for key in names:
        require(text(f.get(key)) and not f[key].startswith('/'), 'FRAME_REQUIRED: frames.'+key)
    require(len({f.get(k) for k in ('map', 'odom', 'base')}) == 3, 'DISTINCT_MAP_ODOM_BASE_REQUIRED')
    for key in ('scan_topic', 'odom_topic', 'tf_topic', 'tf_static_topic'):
        require(topic(site.get(key)), 'ABSOLUTE_TOPIC_REQUIRED: '+key)
    for key in ('odom_to_base', 'base_to_scan'):
        require(text(site.get('tf_authorities', {}).get(key)), 'TF_AUTHORITY_REQUIRED: '+key)
    for key in ('max_age', 'future_tolerance', 'sample_seconds'):
        value = site.get('checks', {}).get(key)
        require(number(value) and value > 0, 'CHECK_TIMING_REQUIRED: checks.'+key)
    duration = site.get('checks', {}).get('sample_seconds')
    require(number(duration) and 1 <= duration <= 30, 'CHECK_SAMPLE_SECONDS_INVALID: 采样1..30秒')
    laser = site.get('laser', {})
    require(number(laser.get('min_range')) and number(laser.get('max_range')) and
            0 <= laser['min_range'] < laser['max_range'], 'LASER_RANGE_REQUIRED: 使用驱动/实测有效范围')
    require(transform_ok(site.get('base_from_scan')), 'SCAN_CALIBRATION_REQUIRED: base_from_scan 为 xyz(m)+xyzw；不得猜外参')
    if route == 'lio':
        for key in ('points_topic', 'imu_topic'):
            require(topic(site.get(key)), 'ABSOLUTE_TOPIC_REQUIRED: '+key)
        # The current Orin route deliberately accepts the already validated timed
        # PointCloud2 contract only. Livox conversion remains the existing separate adapter.
        require(site.get('input_type') == 'timed_cloud', 'TIMED_CLOUD_REQUIRED: 点云需 x/y/z/intensity/time(float32)，time 为相对帧头的秒偏移；Livox 先适配')
        for key in ('imu_from_lidar', 'imu_from_base'):
            require(transform_ok(site.get(key)), 'LIO_CALIBRATION_REQUIRED: '+key)
        if all(transform_ok(site.get(k)) for k in ('imu_from_lidar','imu_from_base','base_from_scan')):
            derived = relative_transform(site['imu_from_base'],site['imu_from_lidar'])
            calibrated = site['base_from_scan']
            require(max(abs(a-b) for a,b in zip(derived[:3],calibrated[:3])) < .001 and
                    abs(sum(a*b for a,b in zip(derived[3:],calibrated[3:]))) > .999999,
                    'LIO_EXTRINSICS_INCONSISTENT: imu外参与base_from_scan必须来自同一标定链')
        projection = site.get('scan_projection', {})
        lo, hi = projection.get('min_height'), projection.get('max_height')
        require(number(lo) and number(hi) and lo < hi, 'LIO_HEIGHT_SLICE_REQUIRED: 实测雷达坐标高度范围（当前LIO扫描投影约定）')
        require(projection.get('publish_sensor_tf') is False, 'EXTERNAL_SENSOR_TF_REQUIRED: 由已标定设备/模型单独发布，避免重复 TF')
        if transform_ok(site.get('base_from_scan')):
            require(abs(site['base_from_scan'][3])+abs(site['base_from_scan'][4]) < 1e-6,
                    'LIO_HORIZONTAL_SCAN_REQUIRED: 当前扫描投影要求雷达z轴平行底盘z轴；倾斜安装需先适配水平扫描')
        require(f.get('scan') == f.get('lidar'), 'LIO_SCAN_FRAME_MUST_MATCH_LIDAR')
        require(site.get('tf_authorities', {}).get('odom_to_base') == 'bindu_lio', 'LIO_ODOM_AUTHORITY_REQUIRED: 关闭其他 odom→base 发布者')
        for key in ('min_range', 'max_range'):
            require(projection.get(key) == laser.get(key), 'LIO_SCAN_RANGE_MISMATCH: scan_projection.'+key)
        grid = site.get('grid', {})
        require(number(grid.get('resolution')) and grid['resolution'] > 0 and
                number(grid.get('hit_padding')) and grid['hit_padding'] >= .1 and
                type(grid.get('max_side')) is int and 16 <= grid['max_side'] <= 4096,
                'LIO_GRID_CONFIG_INVALID: 包含0.2m体素的至少0.1m半宽；最大边长16..4096格')
    if stage in ('mapping', 'localization', 'navigation'):
        prefix = site.get('map', {}).get('prefix')
        require(text(prefix) and Path(prefix).is_absolute(), 'MAP_PREFIX_REQUIRED: map.prefix 为本机新批次绝对路径，不含扩展名')
    if stage in ('localization', 'navigation') and files:
        errors.extend(map_errors(site))
    if stage == 'navigation':
        for key in ('linear_velocity', 'angular_velocity', 'linear_acceleration', 'angular_acceleration'):
            v = site.get('limits', {}).get(key)
            require(number(v) and v > 0, 'MEASURED_BASE_LIMIT_REQUIRED: limits.'+key)
        shape = site.get('footprint')
        require(isinstance(shape, list) and len(shape) >= 3 and all(
            isinstance(p, list) and len(p) == 2 and all(map(number, p)) for p in shape),
            'MEASURED_FOOTPRINT_REQUIRED: footprint 为 base 坐标系米制多边形')
        errors.append(MOTION_BLOCK)
    return errors


class InputWindow:
    """Source time diagnostics: receipt never makes an old measurement fresh."""
    def __init__(self, max_age, future_tolerance):
        self.max_age, self.future = max_age, future_tolerance
        self.samples = {}
        self.errors = set()

    def observe(self, name, stamp, now, valid=True):
        previous, count = self.samples.get(name, (None, 0))
        if not number(stamp) or stamp <= 0 or not -self.future <= now-stamp <= self.max_age:
            self.errors.add(name+': SOURCE_TIME_INVALID（核对驱动时间源及跨机时钟）')
        if previous is not None and stamp < previous:
            self.errors.add(name+': SOURCE_TIME_REGRESSION（重启后重新预检，不自动续跑）')
        if not valid: self.errors.add(name+': INVALID_DATA_OR_FRAME（核对消息、坐标系与标定）')
        self.samples[name] = (stamp, count+int(previous is None or stamp > previous))

    def finish(self, required, now):
        errors = sorted(self.errors)
        for name in required:
            stamp, count = self.samples.get(name, (None, 0))
            if count < 2: errors.append(name+': NO_ADVANCING_DATA（检查驱动、话题、域、QoS）')
            if stamp is not None and not -self.future <= now-stamp <= self.max_age:
                errors.append(name+': STREAM_STALE（最后测量已过期）')
        return errors
