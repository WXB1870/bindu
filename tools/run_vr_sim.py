#!/usr/bin/env python3
"""Start the existing PICO simulation and a separate read-only dashboard."""
import argparse
import fcntl
import hashlib
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

ROOT = Path(__file__).resolve().parents[1]


def commands(args):
    control = ['ros2', 'launch', 'bindu_runtime', 'robot_teleop.launch.py',
        f'namespace:={args.namespace}', f'run_id:={args.run_id}',
        'runtime_mode:=simulation', 'device_backend:=kinematic',
        'single_arm_enabled:=false', 'recorder_enabled:=false', 'recording_mode:=off',
        'description_enabled:=false', 'tool_display_enabled:=false',
        'pico_enabled:=true', 'pico_simulation_demo:=true',
        'tool_input_source:=vr', 'vr_enabled:=true',
        f'host:={args.host}', f'port:={args.vr_port}',
        f'cert_file:={args.cert_file}', f'key_file:={args.key_file}']
    display = [sys.executable, str(ROOT/'tools/snowbot_live.py'), '--dashboard',
        '--namespace', args.namespace, '--run-id', args.run_id, '--port', str(args.port),
        '--output', str(args.output/'web'), '--startup-timeout', str(args.startup_timeout)]
    if args.diagnostics:
        display += ['--trace-file', str(args.output/'diagnostics.jsonl')]
    return control, display


def check(args):
    """Check before starting any control nodes; never print TLS contents."""
    import importlib
    for module in ('rclpy', 'bindu_interfaces.msg', 'bindu_runtime',
                   'bindu_execution', 'pinocchio', 'meshcat', 'tornado', 'vuer'):
        importlib.import_module(module)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(str(args.cert_file), str(args.key_file))
    for host, port in ((args.host, args.vr_port), ('127.0.0.1', args.port)):
        with socket.socket() as probe:
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
            print('VR_SIM_CHECK_OK：环境、TLS 与端口检查通过；未启动节点。')
            return 0
        args.output.mkdir(parents=True, exist_ok=False)
        env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1',
                   ROS_LOG_DIR=str(args.output/'ros-log'))
        stop = threading.Event()
        handlers = {s: signal.signal(s, lambda *_: stop.set()) for s in (signal.SIGINT, signal.SIGTERM)}
        children, result = [], 0
        try:
            for command in commands(args):
                children.append(subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True))
            print(f'仿真会话 {args.run_id}，ROS 域 {args.domain_id}，/{args.namespace}', flush=True)
            print(f'电脑操作台：http://127.0.0.1:{args.port}/\n'
                  f'头显：https://<开发机局域网 IP>:{args.vr_port}/\n'
                  '等待 SNOWBOT_WEB_READY 后打开页面；Ctrl+C 统一停止。', flush=True)
            start = time.monotonic()
            while not stop.wait(.2):
                if any(p.poll() is not None for p in children):
                    print('子进程已退出，正在收尾本次仿真。', file=sys.stderr)
                    result = 1
                    break
                if args.duration and time.monotonic()-start >= args.duration:
                    break
        finally:
            # Keep the display alive while the control chain confirms stopping.
            for child in children:
                if not stop_group(child):
                    print(f'进程组 {child.pid} 超过正常退出期限；已升级终止，不能视为停稳确认。', file=sys.stderr)
                    result = 1
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
        return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='一键启动 PICO 短棍运动学仿真与只读电脑操作台。')
    parser.add_argument('--diagnostics', action='store_true', help='开启诊断 JSONL 落盘（默认仅内存近期事件）')
    parser.add_argument('--check', action='store_true', help='仅检查环境、证书和端口')
    parser.add_argument('--domain-id', type=int, default=184)
    parser.add_argument('--namespace', default='bindu_vr_sim')
    parser.add_argument('--run-id', default='vr_'+uuid.uuid4().hex[:12])
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--vr-port', type=int, default=8012)
    parser.add_argument('--port', type=int, default=8013, help='电脑页面端口，仅监听本机')
    tls = ROOT/'artifacts/vr-connectivity-2026-10-18/tls'
    parser.add_argument('--cert-file', type=Path, default=tls/'server.crt')
    parser.add_argument('--key-file', type=Path, default=tls/'server.key')
    parser.add_argument('--output', type=Path, help='新的运行目录，默认 artifacts/vr-sim/<run-id>')
    parser.add_argument('--startup-timeout', type=float, default=30.)
    parser.add_argument('--duration', type=float, default=0., help='验收用运行秒数，0 表示持续运行')
    args = parser.parse_args(argv)
    args.namespace = args.namespace.strip('/')
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
    args.output = (args.output or ROOT/'artifacts/vr-sim'/args.run_id).resolve()
    args.cert_file, args.key_file = args.cert_file.resolve(), args.key_file.resolve()
    return args


if __name__ == '__main__':
    try:
        raise SystemExit(run(parse_args()))
    except (OSError, ImportError, RuntimeError, ValueError) as exc:
        print(f'VR 启动失败：{exc}', file=sys.stderr)
        raise SystemExit(1)
