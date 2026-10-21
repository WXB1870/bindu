"""Software checks and read-only robot interface checks; never send a target."""
import importlib
import json
import math
import time
import uuid
from pathlib import Path

from tools import ROOT


def check_installed_configs(paths, share):
    """Site overrides are independent; package configs must match this checkout."""
    source_share = ROOT/'src/integration/bindu_runtime'
    share = Path(share).resolve()
    for value in paths:
        if not value:
            continue
        installed = Path(value).resolve()
        if installed.is_relative_to(share):
            source = source_share/installed.relative_to(share)
            if not source.is_file() or source.read_bytes() != installed.read_bytes():
                raise ValueError('安装配置与源码不同，请重建：'+str(source))


def check_runtime(args, robot):
    """Load native libraries/typesupport and construct QP; no targets or device IO."""
    from ament_index_python.packages import get_package_share_directory
    from rclpy.type_support import check_for_type_support
    from bindu_kinematics.bimanual import BimanualQP

    check_installed_configs(
        [args['robot_config'], robot.profile_path, robot.tool_config,
         args.get('pico_config'), args.get('gripper_config')],
        get_package_share_directory('bindu_runtime'))
    modules = [
        ('bindu_interfaces.msg', ('ExecutionState', 'VRInput', 'TeleopModeState', 'GripperState')),
        ('bindu_interfaces.action', ('SelectTeleopMode', 'TeleopSession')),
        ('bindu_interfaces.srv', ('GripperControl',)),
    ]
    if args['runtime_mode'] == 'real':
        modules.append(('snowbot_ctl_msgs.msg', ('JointPositionCommand', 'RetInformation')))
        if args.get('position_protocol') == 'session_v1':
            modules.append(('snowbot_ctl_msgs.msg', ('PositionObservation', 'PositionSessionMeta',
                'PositionSessionCommand', 'PositionSessionUpper', 'PositionSessionServo', 'PositionSessionState')))
    for module, names in modules:
        package = importlib.import_module(module)
        for name in names:
            check_for_type_support(getattr(package, name))
    for module in ('pinocchio', 'casadi', 'vuer', 'bindu_execution.interpolation'):
        importlib.import_module(module)
    tool = json.loads(robot.tool_config.read_text())
    tool['urdf'] = str(robot.urdf)
    solver = BimanualQP(tool, robot.profile)
    solver.close()
    report = dict(software_dependencies='passed', robot_interface='not_checked')
    if args['runtime_mode'] == 'real':
        package = importlib.import_module('snowbot_ctl_msgs.msg')
        report['vendor_messages'] = inspect_vendor_messages(package.JointPositionCommand, package.RetInformation)
    return report


def inspect_vendor_messages(command_type, feedback_type):
    """ROS type support alone does not establish the actual field contract."""
    from bindu_hardware.snowbot import feedback_schema
    report = feedback_schema(feedback_type.get_fields_and_field_types())
    command = command_type.get_fields_and_field_types()
    for name in ('l_arm_position', 'r_arm_position'):
        if command.get(name) not in ('double[7]', 'float64[7]'):
            report['errors'].append(name+':float64[7]')
    report['compatible'] = not report['errors']
    report['full_body_feedback'] = report['compatible'] and not report['missing_groups']
    if report['errors']:
        raise ValueError('SNOWBOT_MESSAGE_SCHEMA_MISMATCH:'+','.join(report['errors']))
    return report


def vendor_endpoints(args):
    """Expected native interfaces plus the existing, separate safety contract."""
    value = args.get('vendor_namespace', '/snowbot')
    if (not isinstance(value, str) or not value.startswith('/') or '//' in value or
            value != '/' and value.endswith('/')):
        raise ValueError('SNOWBOT_VENDOR_NAMESPACE')
    prefix = value.rstrip('/')
    topic = lambda name: prefix+'/'+name
    arm = args.get('arm_receiver_node') or topic('robot_upperMonitor')
    servo = args.get('servo_receiver_node') or topic('servo_node')
    middle = args.get('mid_receiver_node') or topic('dual_arm_control_node')
    result = []
    def add(name, kind, role, node):
        result.append(dict(topic=topic(name), type=kind, role=role, node=node))
    if args.get('position_protocol') == 'session_v1':
        if value != '/':
            raise ValueError('SNOWBOT_POSITION_SESSION_ROOT_NAMESPACE_REQUIRED')
        add('bindu/position/observation', 'snowbot_ctl_msgs/msg/PositionObservation', 'publisher', servo)
        for layer, owner in (('upper', arm), ('middle', middle), ('servo', servo)):
            add('bindu/position/'+layer+'_state', 'snowbot_ctl_msgs/msg/PositionSessionState', 'publisher', owner)
        add('bindu/position/command', 'snowbot_ctl_msgs/msg/PositionSessionCommand', 'subscription', arm)
        for name, kind, sender, receiver in (
            ('upper', 'PositionSessionUpper', arm, middle), ('servo', 'PositionSessionServo', middle, servo)):
            add('bindu/position/'+name, 'snowbot_ctl_msgs/msg/'+kind, 'publisher', sender)
            add('bindu/position/'+name, 'snowbot_ctl_msgs/msg/'+kind, 'subscription', receiver)
        return result
    add('dualArmCtl_fdback', 'snowbot_ctl_msgs/msg/RetInformation', 'publisher', servo)
    for name, kind, receiver in (
        ('joint_position_cmd', 'snowbot_ctl_msgs/msg/JointPositionCommand', arm),
        ('body/motor_cmd', 'std_msgs/msg/Float64MultiArray', servo),
        ('head/motor_cmd', 'std_msgs/msg/Float64MultiArray', servo),
        ('bindu/heartbeat', 'std_msgs/msg/UInt64MultiArray', servo)):
        add(name, kind, 'subscription', receiver)
    for name, kind, sender, receiver in (
        ('upperMonitor_msg_send', 'snowbot_ctl_msgs/msg/MidNodeCtlMsgSend', arm, middle),
        ('ctl_data_pub', 'snowbot_ctl_msgs/msg/CtlInformation', middle, servo)):
        add(name, kind, 'publisher', sender)
        add(name, kind, 'subscription', receiver)
    add('bindu/servo_health', 'std_msgs/msg/String', 'publisher', servo)
    add('bindu/control_source', 'std_msgs/msg/String', 'publisher', arm)
    if args.get('gripper_config'):
        for side in ('left', 'right'):
            add('gripper/'+side+'_cmd', 'std_msgs/msg/Float64MultiArray', 'subscription', servo)
    return result


def inspect_vendor_endpoints(args, node):
    """Read graph metadata only; a topic name without its receiver is insufficient."""
    errors = []
    for endpoint in vendor_endpoints(args):
        method = (node.get_publishers_info_by_topic if endpoint['role'] == 'publisher'
                  else node.get_subscriptions_info_by_topic)
        infos = method(endpoint['topic'])
        matched = [info for info in infos
            if info.node_namespace.rstrip('/')+'/'+info.node_name == endpoint['node']]
        if (len(matched) != 1 or endpoint['role'] == 'publisher' and len(infos) != 1):
            errors.append('SNOWBOT_ENDPOINT_IDENTITY:'+endpoint['topic']+':'+endpoint['role']+':'+endpoint['node'])
        elif matched[0].topic_type != endpoint['type']:
            errors.append('SNOWBOT_ENDPOINT_TYPE:'+endpoint['topic']+':'+endpoint['type'])
        if endpoint['role'] == 'subscription':
            # A preflight observer never adds a command publisher of its own.
            writers = node.get_publishers_info_by_topic(endpoint['topic'])
            if endpoint['type'] in ('snowbot_ctl_msgs/msg/JointPositionCommand',
                                    'snowbot_ctl_msgs/msg/PositionSessionCommand',
                                    'std_msgs/msg/Float64MultiArray', 'std_msgs/msg/UInt64MultiArray'):
                execution = '/'+args['namespace'].strip('/')+'/execution'
                if any(i.node_namespace.rstrip('/')+'/'+i.node_name != execution for i in writers) or len(writers) > 1:
                    errors.append('SNOWBOT_COMMAND_PUBLISHER_CONFLICT:'+endpoint['topic'])
    return errors



def inspect_position_routes(args, node):
    """Compare local generated schemas and offered/requested QoS, without publishing."""
    from rclpy.qos import QoSCompatibility, qos_check_compatible
    endpoints = vendor_endpoints(args)
    own = node.get_namespace().rstrip('/')+'/'+node.get_name()
    errors, graph = [], {}

    def infos(topic, role):
        key = topic, role
        if key not in graph:
            read = node.get_publishers_info_by_topic if role == 'publisher' else node.get_subscriptions_info_by_topic
            graph[key] = read(topic)
        return graph[key]

    def find(topic, role, name):
        found = [i for i in infos(topic, role) if i.node_namespace.rstrip('/')+'/'+i.node_name == name]
        return found[0] if len(found) == 1 else None

    def schema(info):
        value = getattr(info, 'topic_type_hash', None)
        version, digest = getattr(value, 'version', 0), getattr(value, 'value', b'')
        if isinstance(version, int) and version > 0 and isinstance(digest, bytes) and len(digest) == 32 and any(digest):
            return version, digest
        return None

    for endpoint in endpoints:
        topic, role = endpoint['topic'], endpoint['role']
        local, peer = find(topic, 'subscription', own), find(topic, role, endpoint['node'])
        if local is None or peer is None:
            errors.append('SNOWBOT_SESSION_ROUTE_UNVERIFIED:'+topic)
            continue
        local_hash, peer_hash = schema(local), schema(peer)
        if local_hash is None or peer_hash is None:
            errors.append('SNOWBOT_ENDPOINT_TYPE_HASH_UNAVAILABLE:'+topic)
        elif local_hash != peer_hash:
            errors.append('SNOWBOT_ENDPOINT_TYPE_HASH_MISMATCH:'+topic)
        # Relays compare their actual publisher and receiver. The passive local
        # reader supplies only the local schema on those command topics.
        publisher = next((e for e in endpoints if e['topic'] == topic and e['role'] == 'publisher'), None)
        receiver = next((e for e in endpoints if e['topic'] == topic and e['role'] == 'subscription'), None)
        if role == 'publisher' and receiver:
            continue
        offered = find(topic, 'publisher', publisher['node']) if publisher else local
        requested = peer if role == 'subscription' else local
        if offered is None:
            errors.append('SNOWBOT_SESSION_ROUTE_UNVERIFIED:'+topic)
            continue
        try:
            compatibility, _ = qos_check_compatible(offered.qos_profile, requested.qos_profile)
        except (AttributeError, TypeError, ValueError):
            compatibility = QoSCompatibility.WARNING
        if compatibility != QoSCompatibility.OK:
            label = 'INCOMPATIBLE' if compatibility == QoSCompatibility.ERROR else 'UNVERIFIED'
            errors.append('SNOWBOT_ENDPOINT_QOS_'+label+':'+topic)
    return list(dict.fromkeys(errors))


def assess_robot_interface(args, node, messages, now, health):
    """Combine endpoint identity, actual measured fields and advancing PDO health."""
    from bindu_hardware.snowbot import GROUPS, decode_feedback
    errors = inspect_vendor_endpoints(args, node)
    missing = list(GROUPS)
    feedback_usable = False
    msg = messages.get('feedback')
    try:
        if msg is None:
            raise ValueError('SNOWBOT_FEEDBACK_UNAVAILABLE')
        q, v, stamp = decode_feedback(msg)
        if (not all(math.isfinite(x) for x in (*q.values(), *v.values(), stamp)) or
                not 0 <= now-stamp <= .2):
            raise ValueError('SNOWBOT_FEEDBACK_STALE_OR_INVALID')
        feedback_usable = not any(':dualArmCtl_fdback' in e or '/dualArmCtl_fdback:' in e for e in errors)
        missing = [g for g, names in GROUPS.items() if not set(names) <= q.keys()]
        if missing:
            errors.append('SNOWBOT_FEEDBACK_INCOMPLETE:'+','.join(missing))
    except (ValueError, AttributeError, TypeError) as exc:
        errors.append(str(exc))
    try:
        health.require(tuple(GROUPS), armed=False)
        if health.messages.get('servo', {}).get('native_position_guard_version') != 1:
            raise RuntimeError('SNOWBOT_NATIVE_GUARD_REQUIRED')
    except RuntimeError as exc:
        errors.append(str(exc))
    if args.get('gripper_config'):
        # Historical custom-wire guards reject the native five-element topics.
        if 'gripper_control_enabled' in health.messages.get('servo', {}):
            errors.append('GRIPPER_NATIVE_PROTOCOL_GUARD_UNVERIFIED')
        cfg = json.loads(Path(args['gripper_config']).read_text())
        if not cfg['calibrated']:
            errors.append('GRIPPER_CALIBRATION_REQUIRED')
    return dict(robot_interface='available' if not errors else 'unavailable',
                feedback_usable=feedback_usable, missing_feedback_groups=missing,
                errors=errors, scope='read-only endpoints, feedback and safety metadata; no motion acceptance')


def check_robot_interface(args, timeout=2.):
    """Subscribe/read only. Never launch a driver, publish a target or call a service."""
    if not math.isfinite(timeout) or not 0 < timeout <= 30:
        raise ValueError('SNOWBOT_INTERFACE_CHECK_TIMEOUT')
    if args.get('position_protocol') == 'session_v1':
        return check_position_session(args, timeout)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from snowbot_ctl_msgs.msg import RetInformation
    from std_msgs.msg import String
    from bindu_hardware.snowbot import VendorHealth
    rclpy.init(args=[])
    node = rclpy.create_node('bindu_interface_check_'+uuid.uuid4().hex[:8])
    now = lambda: node.get_clock().now().nanoseconds*1e-9
    health, messages = VendorHealth(now), {}
    prefix = args.get('vendor_namespace', '/snowbot').rstrip('/')
    def receive_health(channel, msg):
        try:
            health.receive(channel, msg.data)
        except ValueError:
            pass  # VendorHealth retains the channel's explicit invalid status.
    try:
        node.create_subscription(RetInformation, prefix+'/dualArmCtl_fdback',
                                 lambda msg: messages.update(feedback=msg), qos_profile_sensor_data)
        for channel, name in (('servo', 'servo_health'), ('source', 'control_source')):
            node.create_subscription(String, prefix+'/bindu/'+name,
                                     lambda msg, c=channel: receive_health(c, msg), qos_profile_sensor_data)
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline-time.monotonic())))
        return assess_robot_interface(args, node, messages, now(), health)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def check_position_session(args, timeout):
    """Observe typed metadata and PDO progress; no command publisher is created."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from snowbot_ctl_msgs.msg import (PositionObservation, PositionSessionState,
        PositionSessionCommand, PositionSessionUpper, PositionSessionServo)
    from bindu_hardware.snowbot_session import SessionHealth, CONTROL_GROUPS, decode_observation
    from bindu_hardware.snowbot import GROUPS
    rclpy.init(args=[])
    node = rclpy.create_node('bindu_position_check_'+uuid.uuid4().hex[:8])
    health = SessionHealth(lambda: node.get_clock().now().nanoseconds*1e-9)
    failures = {}
    def observe(msg):
        try:
            health.observe(msg)
            failures.pop('observation', None)
        except ValueError as exc:
            failures['observation'] = str(exc)
    def state(layer, msg):
        try:
            health.state(layer, msg)
            failures.pop(layer, None)
        except ValueError as exc:
            failures[layer] = str(exc)
    try:
        node.create_subscription(PositionObservation, '/bindu/position/observation', observe, qos_profile_sensor_data)
        for layer in ('upper', 'middle', 'servo'):
            node.create_subscription(PositionSessionState, '/bindu/position/'+layer+'_state',
                lambda msg, l=layer: state(l, msg), 1)
        # Passive readers expose this process's generated type hashes. Their
        # default QoS also describes the future command publisher's QoS.
        for topic, kind in (('command', PositionSessionCommand), ('upper', PositionSessionUpper),
                            ('servo', PositionSessionServo)):
            node.create_subscription(kind, '/bindu/position/'+topic, lambda msg: None, 1)
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.01, max(0., deadline-time.monotonic())))
        errors = inspect_vendor_endpoints(args, node) + inspect_position_routes(args, node) + list(failures.values())
        if any(node.count_publishers(t) for t in ('/joint_position_cmd', '/body/motor_cmd')):
            errors.append('SNOWBOT_LEGACY_SOURCE_CONFLICT')
        try:
            health.require(CONTROL_GROUPS, acquiring=True)
        except (RuntimeError, ValueError) as exc:
            errors.append(str(exc))
        q = {}
        try:
            q, _, _ = decode_observation(health.observation, health.now())
        except (ValueError, AttributeError):
            pass
        return dict(robot_interface='available' if not errors else 'unavailable',
            position_protocol='session_v1', feedback_usable=bool(q), errors=errors,
            missing_feedback_groups=[g for g, names in GROUPS.items() if not set(names) <= q.keys()],
            required_control_groups=sorted(CONTROL_GROUPS), deferred_resources=['grippers', 'head'],
            scope='read-only session endpoints, measured PDO progress and faults; no motion acceptance')
    finally:
        node.destroy_node()
        rclpy.shutdown()
