#!/usr/bin/env python3
"""Read-only Snowbot display and a supervised VR simulation entry point."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import threading
import copy

ROOT = Path(__file__).resolve().parents[1]


def display_tool_config(data):
    config=json.loads(Path(data['tool_config']).read_text())
    if data.get('tool_variant')=='short_rod':
        from bindu_teleoperation.simulation_fixture import short_rod_config
        config=short_rod_config(config)
    return config


def validate_state(data, now, run_id, profile_hash, names):
    if (data.get('schema') != 1 or data.get('run_id') != run_id or
            data.get('profile_hash') != profile_hash or data.get('runtime_mode') != 'simulation' or
            data.get('device_backend') != 'kinematic' or not data.get('valid')):
        raise ValueError('LIVE_STATE_IDENTITY_OR_INVALID')
    for field in ('written_at', 'feedback_stamp'):
        value = data.get(field)
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= now-value <= .5:
            raise ValueError('LIVE_STATE_STALE')
    q = data.get('q', {})
    if set(q) != set(names) or any(type(x) not in (int, float) or not math.isfinite(x) for x in q.values()):
        raise ValueError('LIVE_JOINT_LAYOUT')
    return q


class LatestState:
    """One bounded snapshot shared by ROS, model rendering and HTTP threads."""
    def __init__(self):
        self.lock = threading.Lock()
        self.value = {}
        self.error = None

    def publish(self, value):
        with self.lock:
            self.value = copy.deepcopy(value)

    def read(self):
        with self.lock:
            if self.error:
                raise RuntimeError('ROS_OBSERVER_FAILED') from self.error
            return copy.deepcopy(self.value)


def observe(args, publish=None, stop=None):
    import rclpy
    from std_msgs.msg import String
    from bindu_interfaces.msg import ExecutionState, RuntimeEvent
    from bindu_runtime.common import EVENT_QOS
    from bindu_runtime.robot_setup import RobotSetup
    robot = RobotSetup.load(args.robot_config)
    meta = dict(schema=1, run_id=args.run_id, profile_hash=robot.profile.digest,
                display_urdf=str(robot.display_urdf),
                display_sha256=hashlib.sha256(robot.display_urdf.read_bytes()).hexdigest(),
                tool_config=str(robot.tool_config), joint_names=sorted(robot.profile.limits))
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO if publish else SignalHandlerOptions.ALL)
    node = rclpy.create_node('snowbot_live_observer')
    latest = {'state': None, 'pico': {}, 'pico_time': 0.}
    if args.state_file:
        args.state_file.parent.mkdir(parents=True, exist_ok=True)
    if args.trace_file:
        args.trace_file.parent.mkdir(parents=True, exist_ok=True)
    ns = '/'+args.namespace.strip('/')
    node.create_subscription(ExecutionState, ns+'/execution/state',
        lambda msg: latest.update(state=msg), 1)
    def pico(msg):
        try:
            value = json.loads(msg.data)
            if value.get('run_id') == args.run_id and value.get('profile_hash') == robot.profile.digest:
                latest.update(pico=value, pico_time=time.time())
        except (ValueError, TypeError):
            pass
    node.create_subscription(String, ns+'/teleop/pico/state', pico, 1)
    if args.trace_file:
        def runtime_event(msg):
            if msg.run_id!=args.run_id:return
            event=dict(stamp=msg.stamp.sec+msg.stamp.nanosec*1e-9,run_id=msg.run_id,
                       component=msg.component,state=msg.state,code=msg.code,
                       task_id=msg.task_id,command_id=msg.command_id)
            with args.trace_file.with_name('runtime-events.jsonl').open('a') as stream:
                stream.write(json.dumps(event,allow_nan=False)+'\n')
        node.create_subscription(RuntimeEvent,ns+'/events',runtime_event,EVENT_QOS)
    start = time.monotonic()
    last_trace = 0.
    print('LIVE_OBSERVER_READY '+('memory' if publish else str(args.state_file)), flush=True)
    try:
        while rclpy.ok() and not (stop and stop.is_set()) and (not args.duration or time.monotonic()-start < args.duration):
            rclpy.spin_once(node, timeout_sec=.02)
            msg = latest['state']
            now = time.time()
            data = dict(meta, written_at=now, valid=False, q={}, feedback_stamp=0.,
                        execution='WAITING', pico={})
            if msg is not None:
                data.update(runtime_mode=msg.runtime_mode, device_backend=msg.device_backend,
                    feedback_stamp=msg.feedback_stamp.sec+msg.feedback_stamp.nanosec*1e-9,
                    execution=msg.state, code=msg.code, instance_id=msg.instance_id,
                    valid=(msg.simulated and msg.profile_hash == robot.profile.digest and
                           not msg.stop_failures and len(msg.joints.name) == len(msg.joints.position) and
                           len(set(msg.joints.name)) == len(msg.joints.name)),
                    q=dict(zip(msg.joints.name, msg.joints.position)))
                try:
                    validate_state(data, now, args.run_id, robot.profile.digest, robot.profile.limits)
                except ValueError:
                    data.update(valid=False, q={})
            if 0 <= now-latest['pico_time'] <= .6:
                data['pico'] = latest['pico']
                meta['tool_variant']=latest['pico'].get('tool_variant','shovel')
                data['tool_variant']=meta['tool_variant']
            if publish:
                publish(data)
            elif args.state_file:
                temporary = args.state_file.with_suffix('.tmp')
                temporary.write_text(json.dumps(data, allow_nan=False))
                temporary.replace(args.state_file)
            if args.trace_file and time.monotonic()-last_trace >= .1:
                with args.trace_file.open('a') as stream:
                    stream.write(json.dumps(data,allow_nan=False)+'\n')
                last_trace = time.monotonic()
            time.sleep(.025)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def dashboard(args):
    """ROS observation and browser serving share memory in this display process."""
    import signal
    from snowbot_web import serve
    state, stop = LatestState(), threading.Event()
    def observe_worker():
        try:
            observe(args, state.publish, stop)
        except BaseException as exc:
            with state.lock:
                state.error = exc
            stop.set()
    handlers = {s: signal.signal(s, lambda *_: stop.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    observer = threading.Thread(target=observe_worker, name='ros-observer', daemon=True)
    observer.start()
    try:
        serve(args, read_state=state.read, stop=stop)
        state.read()  # Report observer failure even when it set the stop event.
    finally:
        stop.set()
        observer.join(timeout=3.)
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        if observer.is_alive():
            raise RuntimeError('ROS_OBSERVER_SHUTDOWN_TIMEOUT')


def declare_load(args):
    """Explicit simulation-fixture declaration; never a physical grasp claim."""
    import rclpy
    from bindu_interfaces.srv import PicoContext
    data = json.loads(args.state_file.read_text())
    validate_state(data, time.time(), args.run_id, data['profile_hash'], data['joint_names'])
    pico = data['pico']
    if not pico.get('source_id') or not 0 <= time.time()-pico.get('stamp', 0.) <= .6:
        raise ValueError('LIVE_PICO_CONTEXT_STALE')
    rclpy.init(args=[])
    node = rclpy.create_node('snowbot_simulation_context')
    try:
        client = node.create_client(PicoContext, '/'+args.namespace.strip('/')+'/teleop/pico/context')
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('PICO_CONTEXT_UNAVAILABLE')
        future = client.call_async(PicoContext.Request(source_id=pico['source_id'],
            expected_generation=pico['generation'], load_state=args.declare_load,
            external_support=args.external_support))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
        if not future.done():
            raise RuntimeError('PICO_CONTEXT_TIMEOUT')
        response = future.result()
        print(response)
        if not response.success:
            raise RuntimeError(response.code)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def display(args, app):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pxr import Gf, UsdGeom, UsdLux
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.viewports import set_camera_view
    from isaac_kinematic_scene import build_scene
    import omni.ui as ui
    data = json.loads(args.state_file.read_text())
    model = Path(data['display_urdf'])
    if hashlib.sha256(model.read_bytes()).hexdigest() != data['display_sha256']:
        raise ValueError('LIVE_MODEL_CHANGED')
    config = display_tool_config(data)
    recipe = config['demo']
    validate_state(data, time.time(), args.run_id, data['profile_hash'], data['joint_names'])
    world, body, flanges = build_scene(model, args.output,
        (config['left_frame'], config['right_frame']), recipe['ground']['z_in_base_m'])
    if set(body.dof_names) != set(data['joint_names']):
        raise ValueError('LIVE_MODEL_JOINT_LAYOUT')
    stage = world.stage
    UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1100.)
    light = UsdLux.DistantLight.Define(stage, '/World/Sun')
    light.CreateIntensityAttr(1500.)
    UsdGeom.Xformable(light).AddRotateXYZOp().Set(Gf.Vec3f(-30., -35., 0.))
    shovel = UsdGeom.Xform.Define(stage, '/World/Shovel')
    shovel_transform = shovel.AddTransformOp()
    axis = np.asarray(recipe['shaft_axis_in_tool'])
    mount = np.asarray(config['left_to_tool']).reshape(4, 4)
    rear = (np.linalg.inv(mount) @ np.r_[recipe['grip_offset_m'], 1.])[:3]
    ends = recipe['shaft_from_left_grip_m']
    reference = np.eye(3)[np.argmin(abs(axis))]
    tangent = np.cross(reference, axis); tangent /= np.linalg.norm(tangent)
    rotation = np.column_stack((tangent, np.cross(axis, tangent), axis))
    shaft = UsdGeom.Cylinder.Define(stage, '/World/Shovel/Shaft')
    shaft.CreateAxisAttr('Z'); shaft.CreateRadiusAttr(recipe['shaft_radius_m'])
    shaft.CreateHeightAttr(ends[1]-ends[0]); shaft.CreateDisplayColorAttr([Gf.Vec3f(.55, .32, .12)])
    tf = np.eye(4); tf[:3, :3] = rotation; tf[:3, 3] = rear+axis*sum(ends)/2
    shaft.AddTransformOp().Set(Gf.Matrix4d(tf.T.tolist()))
    tray = UsdGeom.Cube.Define(stage, '/World/Shovel/Tray')
    tray.CreateSizeAttr(1.); tray.AddTranslateOp().Set(Gf.Vec3d(*recipe['pan_center_m']))
    tray.AddScaleOp().Set(Gf.Vec3f(*recipe['pan_size_m']))
    tray.CreateDisplayColorAttr([Gf.Vec3f(.12, .46, .85)])
    if recipe.get('geometry_kind')=='rod':
        stage.RemovePrim('/World/Shovel/Tray')
    set_camera_view(eye=[3.3, -3.6, 2.25], target=[.65, -.4, .55])
    panel = ui.Window('Snowbot | LIVE PICO feedback', width=460, height=165)
    with panel.frame:
        with ui.VStack(spacing=5):
            ui.Label('LIVE ROS feedback | Kinematic display', height=25)
            ui.Label('No load / contact dynamics', height=22)
            status = ui.Label('Waiting for fresh feedback', height=26)
            controls = ui.Label('A: hold to prepare | Both squeezes: follow', height=22)
    profile_hash, names = data['profile_hash'], data['joint_names']
    first_instance = data.get('instance_id')
    last_seq, count, maximum_readback = None, 0, 0.
    start, last_save = time.monotonic(), 0.
    print('SNOWBOT_LIVE_GUI_READY', flush=True)
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    captured = False
    while app.is_running() and (not args.duration or time.monotonic()-start < args.duration):
        try:
            value = json.loads(args.state_file.read_text())
            q = validate_state(value, time.time(), args.run_id, profile_hash, names)
            if value.get('instance_id') != first_instance:
                raise ValueError('LIVE_EXECUTOR_CHANGED_RESTART_VIEWER')
            seq = value['feedback_stamp']
            if last_seq is not None and seq < last_seq:
                raise ValueError('LIVE_FEEDBACK_TIME_REVERSED')
            pico = value.get('pico', {})
            if seq != last_seq:
                positions = np.array([q[name] for name in body.dof_names])
                body.set_joint_positions(positions)
                body.set_joint_velocities(np.zeros_like(positions))
                body.apply_action(ArticulationAction(joint_positions=positions))
                world.step(render=False)
                maximum_readback = max(maximum_readback, float(np.max(abs(body.get_joint_positions()-positions))))
                last_seq = seq; count += 1
            status.text = value['execution']+' | '+pico.get('phase', 'waiting')+' | '+pico.get('reason', '')
            p, wxyz = flanges[config['left_frame']].get_world_pose()
            pose = np.eye(4); pose[:3, 3] = p
            pose[:3, :3] = Rotation.from_quat(np.r_[wxyz[1:], wxyz[0]]).as_matrix()
            shovel_transform.Set(Gf.Matrix4d((pose @ mount).T.tolist()))
            UsdGeom.Imageable(shovel).GetVisibilityAttr().Set('inherited' if pico.get('load_state') == 'held' else 'invisible')
        except (ValueError, OSError, KeyError, TypeError) as exc:
            status.text = 'STALE / INVALID: '+str(exc)
            UsdGeom.Imageable(shovel).GetVisibilityAttr().Set('invisible')
        world.render()
        if not captured and time.monotonic()-start > 3.:
            capture_viewport_to_file(get_active_viewport(), str(args.output/'live-view.png'))
            captured = True
        if time.monotonic()-last_save >= 1.:
            (args.output/'display-status.json').write_text(json.dumps(dict(
                run_id=args.run_id, updated_at=time.time(), applied_frames=count,
                max_joint_readback_error_rad=maximum_readback, status=status.text,
                mode='read-only live kinematic display', robot_commands_published=0)))
            last_save = time.monotonic()
        time.sleep(.01)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observe', action='store_true')
    parser.add_argument('--web', action='store_true',help='Local Meshcat model and diagnostic dashboard')
    parser.add_argument('--dashboard', action='store_true', help='ROS + browser display in one process, without a state file')
    parser.add_argument('--port',type=int,default=8013)
    parser.add_argument('--declare-load', choices=('empty','held','unknown'),
                        help='Explicit simulation fixture declaration through existing PicoContext')
    parser.add_argument('--external-support', action='store_true')
    parser.add_argument('--state-file', type=Path)
    parser.add_argument('--trace-file', type=Path, help='Local diagnostic JSONL, at most 10 Hz; separate from training recording')
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--namespace', default='bindu_vr_sim')
    parser.add_argument('--robot-config', default=str(ROOT/'src/integration/bindu_runtime/config/robots/snowbot/robot_snowbot.json'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--startup-timeout', type=float, default=30.)
    args, kit_args = parser.parse_known_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error('startup-timeout must be positive and finite')
    if args.dashboard:
        if args.observe or args.web or args.declare_load or args.state_file or not args.output:
            parser.error('--dashboard requires --output and cannot use legacy file modes')
        args.output.mkdir(parents=True, exist_ok=True)
        dashboard(args)
        return
    if args.state_file is None:
        parser.error('legacy observation / file display requires --state-file')
    if args.observe:
        observe(args)
        return
    if args.declare_load:
        declare_load(args)
        return
    if args.output is None:
        parser.error('Isaac display requires --output')
    if args.web:
        args.output.mkdir(parents=True,exist_ok=True)
        from snowbot_web import serve
        serve(args)
        return
    args.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp
    sys.argv = [sys.argv[0]]+kit_args
    app = SimulationApp({'headless': False, 'fast_shutdown': True, 'window_width':1600, 'window_height':1000})
    try:
        display(args, app)
    except BaseException:
        import traceback
        traceback.print_exc(); sys.stdout.flush(); sys.stderr.flush(); os._exit(1)
    app.close()


if __name__ == '__main__':
    main()
