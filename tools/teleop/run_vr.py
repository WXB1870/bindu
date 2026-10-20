#!/usr/bin/env python3
"""Start a configured VR control chain and its operator view."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import ssl
import subprocess
import sys
import tempfile
import threading
import time
import uuid

from tools import ROOT


def commands(args):
    control = ['ros2', 'launch', 'bindu_runtime', args.launch,
               *[f'{key}:={value}' for key,value in args.deployment.items() if value!=''],
               f'run_id:={args.run_id}']
    display = [sys.executable, '-m', 'tools.teleop.snowbot_live', '--dashboard',
        '--namespace', args.namespace, '--run-id', args.run_id, '--port', str(args.port),
        '--robot-config', args.deployment['robot_config'],
        '--runtime-mode', args.deployment['runtime_mode'],
        '--device-backend', args.deployment['device_backend'],
        '--output', str(args.output/'web'), '--startup-timeout', str(args.startup_timeout)]
    if args.diagnostics:
        display += ['--trace-file', str(args.output/'diagnostics.jsonl')]
    result=[control,display]
    if args.deployment.get('device_backend')=='external_simulation':
        result.insert(0,['bash',str(ROOT/'tools/run_shovel_physics.sh'),'--vr',
                         '--session',str(args.output/'physics-session.json'),'--output',str(args.output/'physics')])
    return tuple(result)


def check(args):
    """Check before starting any control nodes; never print TLS contents."""
    import importlib
    for module in ('rclpy', 'bindu_interfaces.msg', 'bindu_runtime',
                   'meshcat', 'tornado'):
        importlib.import_module(module)
    if args.cert_file:
        try:
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(str(args.cert_file), str(args.key_file), password='')
        except (OSError, ssl.SSLError):
            raise ValueError('TLS 证书/私钥无法加载或不匹配') from None
    elif args.host not in ('127.0.0.1','localhost','::1'):
        raise ValueError('局域网头显入口需要配置 TLS 证书和私钥')
    from bindu_runtime.robot_setup import RobotSetup
    from bindu_hardware.snowbot_gripper import validate_config
    import json
    robot=RobotSetup.load(args.deployment['robot_config'])
    tool=json.loads(robot.tool_config.read_text())
    pico=json.loads(Path(args.deployment['pico_config']).read_text())
    grippers=validate_config(json.loads(Path(args.deployment['gripper_config']).read_text()),robot.profile)
    simulation=args.deployment['runtime_mode']=='simulation'
    if args.deployment.get('device_backend')=='external_simulation':
        sdk=Path(os.environ.get('ISAAC_SIM_PATH',str(Path.home()/'isaacsim-6.2.0')))
        if not simulation or not (sdk/'python.sh').is_file():
            raise ValueError('Isaac物理后端要求仿真模式和可用的ISAAC_SIM_PATH')
        physics=json.loads(Path(args.deployment['physics_config']).read_text())
        from tools.simulation.snowbot_isaac import validate_live_config
        validate_live_config(physics)
    if (tool.get('simulation_only') is not False or pico.get('schema_version')!=2 or
            pico.get('simulation_only') is not simulation or
            grippers.get('simulation_only',False) is not simulation):
        raise ValueError('部署、工具、PICO 和夹爪配置范围不匹配')
    from tools.deployment.runtime_check import check_runtime
    check_runtime(args.deployment, robot)
    from tools.deployment.snowbot_teleop import ensure_stopped
    ensure_stopped(args.deployment)
    for host, port in ((args.host, args.vr_port), ('127.0.0.1', args.port)):
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind((host, port))
            except OSError as exc:
                raise RuntimeError(f'端口 {host}:{port} 不可用；请先关闭原实例。') from exc


def stop_group(process, grace=12.):
    """SIGINT gives ROS launch and execution their existing stop handshake."""
    def send(sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass
    def alive():
        process.poll()  # Reap the leader before checking descendants.
        try:
            os.killpg(process.pid, 0)
            return True
        except ProcessLookupError:
            return False
    for sig, timeout in ((signal.SIGINT, grace), (signal.SIGTERM, 4.), (signal.SIGKILL, 2.)):
        if not alive():
            return True
        send(sig)
        end = time.monotonic()+timeout
        while alive() and time.monotonic() < end:
            time.sleep(.05)
        if not alive():
            process.wait()
            return sig == signal.SIGINT
    return False


def run(args):
    os.environ['ROS_DOMAIN_ID']=str(args.domain_id)
    if args.declare_load:
        from tools.teleop.snowbot_live import declare_load
        args.robot_config=args.deployment['robot_config']
        args.runtime_mode=args.deployment['runtime_mode']
        args.run_id=args.context_run_id
        declare_load(args)
        return 0
    # Held for the full session; ports alone do not protect a shared ROS namespace.
    token = hashlib.sha256(f'{args.domain_id}/{args.namespace}'.encode()).hexdigest()[:20]
    lock_path = Path(tempfile.gettempdir())/f'bindu-vr-{os.getuid()}-{token}.lock'
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('该 ROS 域和命名空间已有启动器运行。') from exc
        check(args)
        if args.check:
            print('VR_CHECK_OK：配置、环境、TLS 与端口检查通过；未启动控制节点。')
            return 0
        args.output.mkdir(parents=True, exist_ok=False)
        env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id),
                   ROS_LOG_DIR=str(args.output/'ros-log'))
        if args.deployment.get('device_backend')=='external_simulation':
            from ament_index_python.packages import get_package_prefix
            env['BINDU_ISAAC_ROS_PREFIX']=get_package_prefix('bindu_interfaces')
        stop = threading.Event()
        handlers = {s: signal.signal(s, lambda *_: stop.set()) for s in (signal.SIGINT, signal.SIGTERM)}
        children, readers, result = [], [], 0
        try:
            resolved_commands=commands(args)
            configs={}
            from bindu_runtime.robot_setup import RobotSetup
            robot=RobotSetup.load(args.deployment['robot_config'])
            if args.deployment.get('device_backend')=='external_simulation':
                physics_session=dict(run_id=args.run_id,namespace=args.namespace,runtime_mode='simulation',
                    profile=str(robot.profile_path),profile_hash=robot.profile.digest,
                    display_urdf=str(robot.display_urdf),tool_config=str(robot.tool_config),
                    pico_config=args.deployment['pico_config'],gripper_config=args.deployment['gripper_config'],
                    physics_config=args.deployment['physics_config'])
                (args.output/'physics-session.json').write_text(json.dumps(physics_session,indent=2))
            config_paths=dict(deployment=str(args.config),**args.deployment,
                tool_config=str(robot.tool_config),profile_config=str(robot.profile_path))
            for key,value in config_paths.items():
                if key=='deployment' or key.endswith('_config'):
                    path=Path(value)
                    if path.is_file():
                        configs[key]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                                          content=json.loads(path.read_text()))
            from bindu_teleoperation.vr import receiver
            probe=Path(receiver.__file__).with_name('client_timing.js')
            manifest=dict(schema=1,run_id=args.run_id,started_at=time.time(),domain_id=args.domain_id,
                namespace=args.namespace,diagnostics=args.diagnostics,commands=resolved_commands,configs=configs,
                client_timing_probe=dict(path=str(probe),sha256=hashlib.sha256(probe.read_bytes()).hexdigest(),
                    clock_semantics='browser monotonic send time; never used as control freshness or one-way latency'),
                diagnostic_format='JSONL: received_at, received_monotonic_ns, data; ROS fields retained',
                diagnostic_chunk_bytes=128*1024*1024)
            (args.output/'session.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
            names=('physics','control','display') if len(resolved_commands)==3 else ('control','display')
            for name,command in zip(names,resolved_commands):
                options=dict(stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace') if args.diagnostics else {}
                child=subprocess.Popen(command,cwd=ROOT,env=env,start_new_session=True,**options)
                children.append(child)
                if args.diagnostics:
                    reader=threading.Thread(target=copy_output,args=(child,args.output/(name+'.log')),daemon=True)
                    reader.start();readers.append(reader)
                if name=='physics':
                    deadline=time.monotonic()+max(180.,args.startup_timeout)
                    while not (args.output/'physics/ready.json').is_file():
                        if stop.wait(.1) or child.poll() is not None or time.monotonic()>deadline:
                            raise RuntimeError('ISAAC_STARTUP_NOT_READY：请检查physics.log')
            print(f'遥操作会话 {args.run_id}，ROS 域 {args.domain_id}，/{args.namespace}', flush=True)
            print(f'日志目录：{args.output}', flush=True)
            print(f'电脑界面：http://127.0.0.1:{args.port}/\n'
                  f'头显：{"https" if args.cert_file else "http"}://{"<开发机局域网 IP>" if args.host=="0.0.0.0" else args.host}:{args.vr_port}/\n'
                  '等待 SNOWBOT_WEB_READY 后打开页面；Ctrl+C 统一停止。', flush=True)
            start = time.monotonic()
            while not stop.wait(.2):
                if any(p.poll() is not None for p in children):
                    print('子进程已退出，正在收尾本次遥操作。', file=sys.stderr)
                    result = 1
                    break
                if args.duration and time.monotonic()-start >= args.duration:
                    break
        finally:
            # Keep the display alive while the control chain confirms stopping.
            # Control first, physics last: hold confirmation still needs steps.
            ordered=children[1:]+children[:1] if args.deployment.get('device_backend')=='external_simulation' else children
            for child in ordered:
                if not stop_group(child):
                    print(f'进程组 {child.pid} 超过正常退出期限；已升级终止，不能视为停稳确认。', file=sys.stderr)
                    result = 1
            for reader in readers:reader.join(timeout=2.)
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
        return result


def copy_output(child, path):
    # Drain child output outside control threads; preserve stderr in the same log.
    with path.open('a',buffering=1) as output:
        for line in child.stdout:
            output.write(line)
            print(line,end='',flush=True)
    child.stdout.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='从部署配置启动 VR 遥操作与电脑界面。')
    parser.add_argument('--config', type=Path, default=ROOT/'src/integration/bindu_runtime/config/deployments/snowbot_vr_sim.json')
    parser.add_argument('--diagnostics', action=argparse.BooleanOptionalAction, default=True,
                        help='当前调试阶段默认记录完整输入、反馈和事件；--no-diagnostics 关闭')
    parser.add_argument('--check', action='store_true', help='仅检查环境、证书和端口')
    parser.add_argument('--physics', action='store_true', help='启用Isaac物理反馈，保留浏览器显示；仅允许仿真配置')
    parser.add_argument('--domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID','0')))
    parser.add_argument('--namespace')
    parser.add_argument('--run-id')
    parser.add_argument('--declare-load', choices=('empty','held','unknown'), help='声明当前装载状态；不启动新的会话')
    parser.add_argument('--external-support', action='store_true', help='与装载声明一起确认已有外部支撑')
    parser.add_argument('--host')
    parser.add_argument('--vr-port', type=int)
    parser.add_argument('--port', type=int, default=8013, help='电脑页面端口，仅监听本机')
    parser.add_argument('--cert-file', type=Path)
    parser.add_argument('--key-file', type=Path)
    parser.add_argument('--output', type=Path, help='新的运行日志目录，默认 log/vr/<run-id>')
    parser.add_argument('--startup-timeout', type=float, default=30.)
    parser.add_argument('--duration', type=float, default=0., help='验收用运行秒数，0 表示持续运行')
    args = parser.parse_args(argv)
    if args.check and args.declare_load:
        parser.error('--check 不能与状态声明同时使用')
    if args.external_support and not args.declare_load:
        parser.error('--external-support 须与 --declare-load 一起使用')
    args.context_run_id=args.run_id
    args.run_id=args.run_id or 'vr_'+uuid.uuid4().hex[:12]
    from ament_index_python.packages import get_package_share_directory
    from bindu_runtime.teleop_deployment import load_deployment
    args.launch,args.deployment=load_deployment(args.config,get_package_share_directory('bindu_runtime'))
    d=args.deployment
    if args.physics:
        if d.get('runtime_mode')!='simulation':parser.error('--physics 仅允许仿真配置')
        d['device_backend']='external_simulation'
    if d.get('device_backend')=='external_simulation' and not d.get('physics_config'):
        d['physics_config']=str(Path(get_package_share_directory('bindu_runtime'))/'config/robots/snowbot/snowbot_shovel_physics.json')
    if (args.launch!='robot_teleop.launch.py' or d.get('position_contract')!='true' or
            d.get('pico_enabled')!='true' or d.get('vr_enabled')!='true'):
        parser.error('此入口要求启用 position_contract、PICO 和 VR 的部署配置')
    for key,attribute,default in (('namespace','namespace','bindu_snowbot'),('host','host','127.0.0.1'),
            ('port','vr_port','8012'),('cert_file','cert_file',''),('key_file','key_file','')):
        value=getattr(args,attribute)
        if value is None:value=d.get(key,default)
        setattr(args,attribute,int(value) if attribute=='vr_port' else str(value))
        d[key]=str(value)
    if bool(args.cert_file)!=bool(args.key_file):parser.error('证书和私钥路径须同时指定')
    args.namespace = args.namespace.strip('/')
    d['namespace']=args.namespace
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', args.namespace):
        parser.error('namespace 必须为有效 ROS 命名空间')
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.run_id):
        parser.error('run-id 只允许字母、数字、下划线和短横线')
    if not 0 <= args.domain_id <= 232:
        parser.error('domain-id 必须在 0–232 内')
    if not all(1024 <= p <= 65535 for p in (args.port, args.vr_port)) or args.port == args.vr_port:
        parser.error('两个端口必须不同，且在 1024–65535 内')
    if (not math.isfinite(args.duration) or args.duration < 0 or
            not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0):
        parser.error('运行时间必须非负且有限，启动超时必须为正且有限')
    args.output = (args.output or ROOT/'log/vr'/args.run_id).resolve()
    for key in ('cert_file','key_file'):
        value=getattr(args,key)
        if value:
            value=str(Path(value).resolve());setattr(args,key,value);d[key]=value
    return args


if __name__ == '__main__':
    try:
        raise SystemExit(run(parse_args()))
    except (OSError, ImportError, RuntimeError, ValueError) as exc:
        print(f'VR 启动失败：{exc}', file=sys.stderr)
        raise SystemExit(1)
