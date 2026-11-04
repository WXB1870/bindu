#!/usr/bin/env python3
"""Snowbot deployment configuration, offline checks and read-only ROS observation."""
from tools import package_share
import argparse
import json
import hashlib
import math
import os
from pathlib import Path
import re
import sys
import time
import uuid

def recipe_path():
    return package_share('snowbot_bringup')/'config/examples/snowbot/snowbot_real_teleop.json'


def namespace(value):
    if value != '/' and not re.fullmatch(r'/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*', value):
        raise ValueError('命名空间格式无效：'+str(value))
    return '/'+value.strip('/') if value != '/' else '/'


def configure(options):
    data = json.loads(recipe_path().read_text())
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
    args = data['arguments']
    if bool(args['cert_file']) != bool(args['key_file']):
        raise ValueError('证书和私钥路径须同时填写')
    if args['host'] not in ('127.0.0.1', 'localhost', '::1') and not args['cert_file']:
        raise ValueError('局域网 VR 监听需要 TLS 证书和私钥路径')
    # Exclusive creation also protects symlinks; never replace a reviewed site recipe.
    options.config.parent.mkdir(parents=True, exist_ok=True)
    with options.config.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    print(f'已生成 {options.config}；command_enabled={options.enable_commands}')


def preflight(path, *, diagnostics=True):
    from bindu_runtime.teleop_deployment import load_deployment
    from bindu_runtime.robot_setup import RobotSetup
    _, args = load_deployment(path, package_share('bindu_runtime'))
    robot = RobotSetup.load(args['robot_config'])
    if diagnostics:
        from tools.deployment.runtime_check import check_runtime
        print(json.dumps(check_runtime(args, robot), ensure_ascii=False))
    return args, robot


def startup_lock(args):
    from snowbot_bringup.teleop.run_vr import startup_lock as acquire
    return acquire(os.environ.get('ROS_DOMAIN_ID', '0'), args['namespace'], inherit=True)


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
    from snowbot_hardware.parameters import load_mode_config, mode_config_digest
    cfg = load_mode_config(args['gripper_config'],args['runtime_mode'])
    config_hash = mode_config_digest(cfg,args['runtime_mode'])
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
    p = argparse.ArgumentParser(description='Snowbot 旧四节点专项配置、检查、前台启动和只读状态。通过 snowbot_teleop.sh 加载选定安装环境；构建使用 x86_real。')
    commands = p.add_subparsers(dest='command')
    selected_workspace = os.environ.get('BINDU_TELEOP_WORKSPACE')
    workspace = Path(selected_workspace) if selected_workspace else None
    for command in ('configure', 'build', 'check', 'start', 'status', 'gripper'):
        sub = commands.add_parser(command, **({'help':'已弃用；改用 tools.deployment.x86_real build'}
                                               if command == 'build' else {}))
        if command == 'build':
            continue
        sub.add_argument('--config', type=Path, default=workspace/'site.json' if workspace else None,
                         required=workspace is None,
                         help='专项配方文件；未设置 BINDU_TELEOP_WORKSPACE 时必须显式指定')
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
        p.error('旧专项工具不再单独构建；请使用 python3 -m tools.deployment.x86_real build --workspace /已准备的部署目录')
    try:
        options.config = options.config.expanduser().absolute()
        if options.command == 'configure':
            configure(options)
            return 0
        if options.command == 'status' and not (math.isfinite(options.seconds) and 0 < options.seconds <= 30):
            raise ValueError('--seconds 须在 0–30 秒内')
        args, robot = preflight(options.config, diagnostics=options.command == 'check')
        if options.command == 'gripper':
            return operate_gripper(args, robot, options)
        if options.command == 'status':
            return observe(args, robot, options.seconds)
        if options.command == 'start':
            print('前台启动四节点；Ctrl+C 收尾。仍需显式选择模式和开启会话。', flush=True)
            with startup_lock(args):
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
