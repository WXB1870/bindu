#!/usr/bin/env python3
"""Snowbot deployment configuration, offline checks and read-only ROS observation."""
import argparse
import json
import hashlib
import math
import os
from pathlib import Path
import platform
import re
import ssl
import sys
import time
import uuid

from tools import ROOT
SHARE = ROOT/'src/integration/bindu_runtime'
RECIPE = SHARE/'config/deployments/snowbot_real_teleop.json'


def namespace(value):
    if value != '/' and not re.fullmatch(r'/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*', value):
        raise ValueError('命名空间格式无效：'+str(value))
    return '/'+value.strip('/') if value != '/' else '/'


def validate_recipe(data):
    """Keep this helper bounded to the four-node real teleop assembly."""
    args = data['arguments']
    expected = dict(runtime_mode='real', device_backend='snowbot_vendor',
                    single_arm_enabled=False, pico_enabled=False, recorder_enabled=False,
                    recording_mode='off', description_enabled=False, tool_display_enabled=False,
                    tool_input_source='vr', vr_enabled=True)
    if data['schema_version'] != 2 or data['robot_launch'] != 'robot_teleop.launch.py':
        raise ValueError('需要 Snowbot 真机最小配方')
    for key, value in expected.items():
        if type(args.get(key)) is not type(value) or args[key] != value:
            raise ValueError('真机最小配方不匹配：'+key)
    if type(args.get('command_enabled')) is not bool:
        raise ValueError('command_enabled 必须是布尔值')
    namespace(args['namespace'])
    vendor = namespace(args['vendor_namespace'])
    if vendor != args['vendor_namespace']:
        raise ValueError('vendor_namespace 须为绝对路径且无末尾斜线')
    for key in ('arm_receiver_node', 'servo_receiver_node', 'mid_receiver_node'):
        if args.get(key) and (namespace(args[key]) != args[key] or args[key] == '/'):
            raise ValueError('接收节点须为完整绝对名称：'+key)
    if not isinstance(args.get('host'), str) or not args['host'].strip():
        raise ValueError('VR 监听地址不能为空')
    if type(args.get('port')) is not int or not 1 <= args['port'] <= 65535:
        raise ValueError('VR 端口须在 1–65535 内')
    if bool(args.get('cert_file')) != bool(args.get('key_file')):
        raise ValueError('证书和私钥路径须同时填写')
    if args['host'] not in ('127.0.0.1', 'localhost', '::1') and not args.get('cert_file'):
        raise ValueError('局域网 VR 监听需要 TLS 证书和私钥路径')
    return args


def configure(options):
    data = json.loads(RECIPE.read_text())
    data['arguments'].update({
        key: getattr(options, key) for key in
        ('host', 'port', 'namespace', 'vendor_namespace', 'arm_receiver_node',
         'servo_receiver_node', 'mid_receiver_node')})
    data['arguments']['command_enabled'] = options.enable_commands
    if options.gripper_config:
        data['arguments']['gripper_config'] = str(Path(options.gripper_config).expanduser().resolve())
    for key in ('cert_file', 'key_file'):
        value = getattr(options, key)
        data['arguments'][key] = str(Path(value).expanduser().resolve()) if value else ''
    validate_recipe(data)
    # Exclusive creation also protects symlinks; never replace a reviewed site recipe.
    options.config.parent.mkdir(parents=True, exist_ok=True)
    with options.config.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    print(f'已生成 {options.config}；command_enabled={options.enable_commands}')


def preflight(path):
    from ament_index_python.packages import get_package_share_directory
    from bindu_runtime.teleop_deployment import load_deployment
    from bindu_runtime.robot_setup import RobotSetup

    raw = validate_recipe(json.loads(path.read_text()))
    share = Path(get_package_share_directory('bindu_runtime'))
    _, args = load_deployment(path, share)
    robot = RobotSetup.load(args['robot_config'])
    tool = json.loads(robot.tool_config.read_text())
    if robot.robot_id != 'snowbot' or tool.get('simulation_only') is not False:
        raise ValueError('需要 Snowbot 真机模型与工具配置')
    if tool.get('demo', {}).get('initial_positions_rad'):
        raise ValueError('真机工具配置包含演示初始关节')
    from tools.deployment.runtime_check import check_runtime
    report = check_runtime(args, robot)
    if args.get('gripper_config'):
        from bindu_hardware.snowbot_gripper import validate_config
        grippers = validate_config(json.loads(Path(args['gripper_config']).read_text()), robot.profile)
        if not grippers['calibrated']:
            print('夹爪配置未现场标定：状态可读，夹爪写入将拒绝。')
    if args.get('cert_file'):
        # Do not include SSL exceptions in output: they may contain private file paths.
        try:
            ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER).load_cert_chain(args['cert_file'], args['key_file'], password='')
        except (OSError, ssl.SSLError):
            raise ValueError('TLS 证书/私钥无法加载或不匹配') from None
    print(json.dumps(dict(python=sys.executable, version=platform.python_version(),
                          architecture=platform.machine(), ros=os.environ.get('ROS_DISTRO'),
                          runtime_share=str(share), robot_digest=robot.digest,
                          command_enabled=raw['command_enabled'],
                          ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
                          ros_localhost_only=os.environ.get('ROS_LOCALHOST_ONLY', '0')),
                     ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False))
    print('软件依赖与消息兼容性检查通过；机器人在线端点未检查，不能据此确认接口可用。')
    return args, robot


def ensure_stopped(args):
    """Observe the graph before launch; never terminate an existing instance."""
    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node('bindu_launch_check_'+uuid.uuid4().hex[:8])
    ns = namespace(args['namespace'])
    try:
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        conflicts = [name for name, space in node.get_node_names_and_namespaces()
                     if space == ns and name in ('execution', 'teleop_mode', 'teleop_tool', 'vr_input')]
        if conflicts:
            raise ValueError('该命名空间已有遥操作节点，请先核对并收尾：'+','.join(sorted(conflicts)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def assess_state(execution, mode, now, profile_hash):
    errors = []
    def fresh(stamp, limit):
        age = now-stamp.sec-stamp.nanosec*1e-9
        return math.isfinite(age) and 0 <= age <= limit
    if execution is None:
        errors.append('未收到 execution/state')
    else:
        if execution.simulated or execution.runtime_mode != 'real' or execution.device_backend != 'snowbot_vendor':
            errors.append('执行反馈不是 Snowbot 真实模式')
        if execution.profile_hash != profile_hash:
            errors.append('执行 profile 与现场配方不一致')
        if not fresh(execution.stamp, .5) or not fresh(execution.feedback_stamp, .2):
            errors.append('执行状态或设备反馈过期/来自未来')
        if execution.stop_failures:
            errors.append('存在停止未确认：'+','.join(execution.stop_failures))
    if mode is None:
        errors.append('未收到 teleop/mode_state')
    elif mode.runtime_mode != 'real' or mode.profile_hash != profile_hash or not fresh(mode.stamp, .5):
        errors.append('模式状态的环境、profile或时间不匹配')
    return errors


def observe(args, robot, duration):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from bindu_interfaces.msg import ExecutionState, TeleopModeState
    rclpy.init(args=[])
    node = rclpy.create_node('bindu_deployment_observer_'+uuid.uuid4().hex[:8])
    ns = namespace(args['namespace']).rstrip('/')
    received = {}
    topics = {'execution': ns+'/execution/state', 'mode': ns+'/teleop/mode_state'}
    try:
        for key, cls in (('execution', ExecutionState), ('mode', TeleopModeState)):
            node.create_subscription(cls, topics[key], lambda msg, k=key: received.update({k: msg}), qos_profile_sensor_data)
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline-time.monotonic())))
        errors = assess_state(received.get('execution'), received.get('mode'),
                              node.get_clock().now().nanoseconds*1e-9, robot.profile.digest)
        for key, name in (('execution', 'execution'), ('mode', 'teleop_mode')):
            infos = node.get_publishers_info_by_topic(topics[key])
            if len(infos) != 1 or infos[0].node_namespace.rstrip('/')+'/'+infos[0].node_name != ns+'/'+name:
                errors.append('状态发布者缺失、重名或身份不匹配：'+topics[key])
        for key, msg in received.items():
            print(f'{key}: state={getattr(msg, "state", getattr(msg, "phase", ""))}'
                  f' code={msg.code} mode={getattr(msg, "mode", "")}')
        for error in errors:
            print('阻断：'+error, file=sys.stderr)
        if not errors:
            print('已观察到新鲜的真实模式状态；此结果不等于获准运动或停稳验收。')
        return 1 if errors else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def gripper_publisher_matches(node, topic, expected):
    writers = node.get_publishers_info_by_topic(topic)
    return (len(writers)==1 and
            writers[0].node_namespace.rstrip('/')+'/'+writers[0].node_name==expected and
            writers[0].topic_type=='bindu_interfaces/msg/GripperState')


def operate_gripper(args, robot, options):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from bindu_interfaces.msg import GripperState
    from bindu_interfaces.srv import Lease, GripperControl
    cfg = json.loads(Path(args['gripper_config']).read_text())
    config_hash = hashlib.sha256(Path(args['gripper_config']).read_bytes()).hexdigest()
    if args['command_enabled'] != 'true' or not cfg['calibrated']:
        raise ValueError('夹爪操作需要允许写入且已现场标定的配置')
    if options.operation == 'authorize_loading' and not options.vr_source_id:
        raise ValueError('authorize_loading 须明确指定 --vr-source-id')
    if options.operation != 'authorize_loading' and options.vr_source_id:
        raise ValueError('--vr-source-id 仅用于 authorize_loading')
    if options.operation != 'authorize_loading' and options.side == 'both':
        raise ValueError('位置操作每次选择 left 或 right')
    if options.operation == 'position' and (options.position is None or not math.isfinite(options.position)):
        raise ValueError('position 操作须填写有限的 --position（rad）')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('bindu_gripper_operator_'+uuid.uuid4().hex[:8])
    ns = namespace(args['namespace']).rstrip('/')
    clients = [node.create_client(Lease, ns+'/gripper/lease'), node.create_client(GripperControl, ns+'/gripper/control')]
    received = {}
    def state_received(msg):
        age = node.get_clock().now().nanoseconds*1e-9-msg.stamp.sec-msg.stamp.nanosec*1e-9
        if (msg.runtime_mode=='real' and msg.profile_hash==robot.profile.digest and msg.config_hash==config_hash
                and 0 <= age <= .5):
            received[msg.side]=msg
    node.create_subscription(GripperState, ns+'/gripper/state', state_received, 10)
    lease = None
    transferred = False
    def call(client, request):
        deadline = time.monotonic()+2.
        while not client.service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not client.service_is_ready():
            raise RuntimeError('夹爪服务未就绪')
        future = client.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not future.done():
            raise RuntimeError('夹爪服务应答超时')
        reply = future.result()
        if not (reply.ok if hasattr(reply, 'ok') else reply.accepted):
            raise RuntimeError(reply.code)
        return reply
    def request(operation, sequence):
        req = GripperControl.Request(operation=operation, side='' if operation=='authorize_loading' else options.side,
            lease_id=lease.lease_id, epoch=lease.epoch, sequence=sequence, valid_for=.3,
            position=options.position or 0., load_state=options.load_state, external_support=options.external_support)
        req.stamp = node.get_clock().now().to_msg()
        return req
    try:
        sides = ('left','right') if options.side=='both' else (options.side,)
        def publisher_matches():
            return gripper_publisher_matches(node, ns+'/gripper/state', ns+'/execution')
        # DDS may discover a writer before its owning ROS node information.
        # This is a startup budget, not a relaxation of feedback/command ages.
        discovery_deadline = time.monotonic()+5.
        while not publisher_matches() and time.monotonic()<discovery_deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not publisher_matches():
            writers=node.get_publishers_info_by_topic(ns+'/gripper/state')
            observed=[info.node_namespace.rstrip('/')+'/'+info.node_name for info in writers]
            raise RuntimeError('夹爪状态发布者不匹配：expected='+ns+'/execution; observed='+repr(observed))
        owner = options.vr_source_id or node.get_name()
        lease = call(clients[0], Lease.Request(operation='acquire',owner=owner,resources=[s+'_tool_axis' for s in sides]))
        if lease.profile_hash != robot.profile.digest:
            raise RuntimeError('夹爪profile不匹配')
        call(clients[1], request('authorize_loading', 1))
        if options.operation == 'authorize_loading':
            transferred = True
            print('已将装卸控制绑定到指定VR来源；须先释放Trigger再操作，输入中断会撤权并请求保持。')
            return 0
        call(clients[1], request(options.operation, 2))
        deadline, renew_at = time.monotonic()+cfg['motion_timeout']+cfg['stop_timeout']+1., time.monotonic()+.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            state = received.get(options.side)
            if (state and state.epoch==lease.epoch and state.control_sequence>=2
                    and state.state in ('SUCCEEDED','FAILED','FAULT','CANCELED')):
                if not publisher_matches():
                    raise RuntimeError('夹爪状态发布者不匹配')
                print(f'{state.side}: {state.state} {state.code}; position={state.measured_position:.5f} rad; grasp={state.grasp_state}')
                return 0 if state.position_reached and state.state=='SUCCEEDED' else 1
            if time.monotonic() >= renew_at:
                call(clients[0], Lease.Request(operation='renew',lease_id=lease.lease_id,epoch=lease.epoch))
                renew_at=time.monotonic()+.5
        raise RuntimeError('夹爪状态等待超时')
    finally:
        try:
            if lease and not transferred:
                call(clients[0], Lease.Request(operation='release',lease_id=lease.lease_id,epoch=lease.epoch))
                end=time.monotonic()+cfg['stop_timeout']+.5
                while time.monotonic()<end:
                    rclpy.spin_once(node,timeout_sec=.02)
                    if all(s in received and received[s].epoch>lease.epoch and not received[s].lease_active and received[s].state!='STOPPING' for s in sides):
                        if any(received[s].state=='FAULT' for s in sides):
                            raise RuntimeError('夹爪停止未确认')
                        break
                else:
                    raise RuntimeError('夹爪释放后的保持未确认')
        finally:
            node.destroy_node()
            rclpy.shutdown()


def parser():
    p = argparse.ArgumentParser(description='Snowbot 最小遥操作：现场配置、构建、检查、前台启动和只读状态。通过 snowbot_teleop.sh 自动加载环境。')
    commands = p.add_subparsers(dest='command')
    workspace = Path(os.environ.get('BINDU_TELEOP_WORKSPACE', ROOT/'artifacts/real-teleop'))
    for command in ('configure', 'build', 'check', 'start', 'status', 'gripper'):
        sub = commands.add_parser(command)
        if command == 'build':
            continue
        sub.add_argument('--config', type=Path, default=workspace/'site.json')
        if command == 'configure':
            sub.add_argument('--gripper-config', default='', help='现场标定的夹爪配置；默认示例禁止夹爪写入')
            sub.add_argument('--host', default='127.0.0.1')
            sub.add_argument('--port', type=int, default=8012)
            sub.add_argument('--namespace', default='bindu_snowbot')
            sub.add_argument('--vendor-namespace', default='/snowbot')
            for key in ('cert-file', 'key-file', 'arm-receiver-node', 'servo-receiver-node', 'mid-receiver-node'):
                sub.add_argument('--'+key, default='')
            sub.add_argument('--enable-commands', action='store_true', help='仅在已获准的现场运动实验中生成允许写入的配方')
        if command == 'status':
            sub.add_argument('--seconds', type=float, default=3.)
        if command == 'gripper':
            sub.add_argument('--operation', choices=('open','close','position','authorize_loading'), required=True)
            sub.add_argument('--side', choices=('left','right','both'), default='left')
            sub.add_argument('--position', type=float)
            sub.add_argument('--load-state', choices=('empty','held'), required=True)
            sub.add_argument('--external-support', action='store_true')
            sub.add_argument('--vr-source-id', default='')
    return p


def main(argv=None):
    p = parser()
    options = p.parse_args(argv)
    if options.command is None:
        p.print_help()
        return 2
    if options.command == 'build':
        p.error('请通过 bash tools/snowbot_teleop.sh build 构建')
    try:
        options.config = options.config.expanduser().absolute()
        if options.command == 'configure':
            configure(options)
            return 0
        if options.command == 'status' and not (math.isfinite(options.seconds) and 0 < options.seconds <= 30):
            raise ValueError('--seconds 须在 0–30 秒内')
        args, robot = preflight(options.config)
        if options.command == 'gripper':
            return operate_gripper(args, robot, options)
        if options.command == 'status':
            return observe(args, robot, options.seconds)
        if options.command == 'start':
            if args.get('command_enabled') == 'true':
                from tools.deployment.runtime_check import check_robot_interface
                report = check_robot_interface(args)
                print(json.dumps(report, ensure_ascii=False))
                if report['errors']:
                    raise RuntimeError('ROBOT_INTERFACE_UNAVAILABLE:'+','.join(report['errors']))
            ensure_stopped(args)
            print('前台启动四节点；Ctrl+C 收尾。仍需显式选择模式和开启会话。', flush=True)
            os.execvp('ros2', ['ros2', 'launch', 'bindu_runtime', 'teleop_only.launch.py', 'config:='+str(options.config)])
        return 0
    except KeyboardInterrupt:
        print('操作已中断；已尝试按夹爪控制权完成保持收尾。', file=sys.stderr)
        return 130
    except (OSError, ValueError, KeyError, TypeError, ImportError, RuntimeError) as exc:
        print(f'部署检查失败：{exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
