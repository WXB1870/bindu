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
                                    'std_msgs/msg/Float64MultiArray', 'std_msgs/msg/UInt64MultiArray'):
                execution = '/'+args['namespace'].strip('/')+'/execution'
                if any(i.node_namespace.rstrip('/')+'/'+i.node_name != execution for i in writers) or len(writers) > 1:
                    errors.append('SNOWBOT_COMMAND_PUBLISHER_CONFLICT:'+endpoint['topic'])
    return errors


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
