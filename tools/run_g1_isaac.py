#!/usr/bin/env python3
"""Run the provisional G1 PhysX device backend in Isaac Sim 6.0 (GUI by default)."""
import argparse
from collections import deque
import gc
import hashlib
import json
import math
import os
import signal
import sys
from pathlib import Path
import time
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def build_scene(model, cfg, room=None):
    """Keep imported meshes/inertia; author joints explicitly from the pinned URDF.

    The local 6.0 RC importer produced split articulation roots and world-parent
    joints for this massless-root URDF. Normalize using checked URDF frames rather
    than silently relying on those generated physics relationships.
    """
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdShade, UsdLux
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.viewports import set_camera_view
    usd = model / 'usd/g1_physics/g1_physics.usda'
    import_record = model / 'usd/import.json'
    model_hash = hashlib.sha256((model/'g1_physics.urdf').read_bytes()).hexdigest()
    if not usd.exists() or not import_record.exists() or json.loads(import_record.read_text()).get('urdf_sha256') != model_hash:
        enable_extension('isaacsim.asset.importer.urdf')
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        usd = Path(URDFImporter(URDFImporterConfig(urdf_path=str(model/'g1_physics.urdf'),
            usd_path=str(model/'usd'), allow_self_collision=False)).import_urdf())
        import_record.write_text(json.dumps({'urdf_sha256': model_hash})+'\n')
    world = World(stage_units_in_meters=1., physics_dt=cfg['physics_dt'], rendering_dt=cfg['physics_dt'])
    world.scene.add_default_ground_plane()
    if room:
        from isaac_navigation_scene import add_room
        add_room(world, room)
    stage = world.stage
    scene_api = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(world.get_physics_context().prim_path))
    scene_api.CreateEnableExternalForcesEveryIterationAttr(cfg.get('external_forces_every_iteration', False))
    UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000.)
    robot = stage.DefinePrim('/G1', 'Xform')
    robot.GetReferences().AddReference(str(usd))
    urdf = ET.parse(model/'g1_physics.urdf').getroot()
    names = {link.get('name') for link in urdf.findall('link')}
    links = {prim.GetName(): prim for prim in Usd.PrimRange(robot) if prim.GetName() in names}
    if set(links) != names:
        raise RuntimeError('PHYSICS_LINK_LAYOUT_MISMATCH')
    visual_source = model / 'upstream_usd/payloads/base.usda'
    source_stage = Usd.Stage.Open(str(visual_source))
    source_links = {p.GetName(): p for p in source_stage.Traverse() if p.GetName() in names
                    and '/visuals/' not in str(p.GetPath()) and '/collisions/' not in str(p.GetPath())}
    source_cache = UsdGeom.XformCache()
    restored_visuals = 0
    for link in urdf.findall('link'):
        name = link.get('name')
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            stem = Path(mesh.get('filename')).stem
            owner, shape = name, stem.replace('_left', '').replace('_right', '')
            if stem in ('chassis', 'mid360'): owner = 'base_link'
            elif stem == 'torso': owner = 'leg_link5'
            elif stem.startswith('d405'):
                owner = ('left' if name.startswith('left') else 'right') + '_arm_link7'
            source_path = f'/galbot_one_golf/{owner}/visuals/{shape}'
            if not source_stage.GetPrimAtPath(source_path):
                source_path = f'/galbot_one_golf/{owner}/visuals/{stem}'
            if not source_stage.GetPrimAtPath(source_path):
                raise RuntimeError('MISSING_USD_VISUAL:'+source_path)
            target = stage.DefinePrim(str(links[name].GetPath())+'/restored_'+stem, 'Xform')
            target.GetReferences().AddReference(str(visual_source), source_path)
            target.SetInstanceable(False)
            # Upstream fixed links were merged; URDF link transforms already
            # locate this geometry, so do not apply those merged offsets twice.
            xform = UsdGeom.Xformable(target)
            xform.ClearXformOpOrder()
            # Preserve upstream mirrored right-arm meshes and camera offsets,
            # expressed relative to their own URDF link, not a merged parent.
            matrix = (source_cache.GetLocalToWorldTransform(source_stage.GetPrimAtPath(source_path)) *
                      source_cache.GetLocalToWorldTransform(source_links[name]).GetInverse())
            xform.AddTransformOp().Set(matrix)
            if not any(p.IsA(UsdGeom.Mesh) for p in Usd.PrimRange(target, Usd.TraverseInstanceProxies())):
                raise RuntimeError('EMPTY_USD_VISUAL:'+source_path)
            restored_visuals += 1
    print('RESTORED_VISUALS', restored_visuals, flush=True)
    cache = UsdGeom.XformCache()
    transforms = {name: cache.GetLocalToWorldTransform(prim) for name, prim in links.items()}
    for prim in list(Usd.PrimRange(robot)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    stage.GetPrimAtPath('/G1/Physics').SetActive(False)
    for name, prim in links.items():
        matrix = Gf.Matrix4d(transforms[name])
        matrix.SetTranslateOnly(matrix.ExtractTranslation()+Gf.Vec3d(0, 0, cfg['spawn_z']))
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(matrix)
        xform.SetResetXformStack(True)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        if not prim.HasAPI(UsdPhysics.MassAPI) or not UsdPhysics.MassAPI(prim).GetMassAttr().Get():
            mass = UsdPhysics.MassAPI.Apply(prim)
            mass.CreateMassAttr(.0001)  # Numerical carrier for massless fixed URDF frames.
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.000001))
    root = links['base_link']
    UsdPhysics.ArticulationRootAPI.Apply(root)
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(root)
    articulation.CreateEnabledSelfCollisionsAttr(cfg['self_collision'])
    articulation.CreateSolverPositionIterationCountAttr(cfg['solver_position_iterations'])
    articulation.CreateSolverVelocityIterationCountAttr(cfg['solver_velocity_iterations'])
    # Synthetic support balls approximate freely rolling casters; no lateral grip.
    material = UsdShade.Material.Define(stage, '/World/SupportMaterial')
    mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    mat.CreateStaticFrictionAttr(0.); mat.CreateDynamicFrictionAttr(0.); mat.CreateRestitutionAttr(0.)
    for name, prim in links.items():
        if name.startswith('sim_support_'):
            for shape in Usd.PrimRange(prim):
                if shape.HasAPI(UsdPhysics.CollisionAPI):
                    UsdShade.MaterialBindingAPI.Apply(shape).Bind(material, materialPurpose='physics')
    max_frame_error = 0.
    for joint in urdf.findall('joint'):
        fixed = joint.get('type') == 'fixed'
        api = (UsdPhysics.FixedJoint if fixed else UsdPhysics.RevoluteJoint).Define(stage, '/G1/Joints/'+joint.get('name'))
        parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
        api.CreateBody0Rel().SetTargets([links[parent].GetPath()])
        api.CreateBody1Rel().SetTargets([links[child].GetPath()])
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0, 0, 0]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0, 0, 0]
        axis = [float(v) for v in joint.find('axis').get('xyz').split()] if not fixed else [1, 0, 0]
        rotation = (Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(rpy[0])) *
                    Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(rpy[1])) *
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(rpy[2])))
        align = Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.Vec3d(*axis))
        q0, q1 = Gf.Quatf((align*rotation).GetQuat()), Gf.Quatf(align.GetQuat())
        api.CreateLocalPos0Attr(Gf.Vec3f(*xyz)); api.CreateLocalPos1Attr(Gf.Vec3f(0))
        api.CreateLocalRot0Attr(q0); api.CreateLocalRot1Attr(q1)
        a = Gf.Matrix4d().SetRotate(q0)*transforms[parent]
        b = Gf.Matrix4d().SetRotate(q1)*transforms[child]
        error = max(float(np.max(np.abs(np.array(a)[:3, :3]-np.array(b)[:3, :3]))),
                    float((transforms[parent].Transform(Gf.Vec3d(*xyz))-transforms[child].ExtractTranslation()).GetLength()))
        max_frame_error = max(max_frame_error, error)
        if error > 1e-5:
            raise RuntimeError('PHYSICS_JOINT_FRAME_MISMATCH:'+joint.get('name'))
        if not fixed:
            api.CreateAxisAttr('X')
            limit = joint.find('limit')
            if joint.get('type') == 'revolute':
                api.CreateLowerLimitAttr(math.degrees(float(limit.get('lower'))))
                api.CreateUpperLimitAttr(math.degrees(float(limit.get('upper'))))
            wheel = joint.get('name').startswith('sim_')
            drive = UsdPhysics.DriveAPI.Apply(api.GetPrim(), 'angular')
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(0. if wheel else cfg['joint_stiffness'])
            drive.CreateDampingAttr(cfg['wheel_damping'] if wheel else cfg['joint_damping'])
            drive.CreateMaxForceAttr(float(limit.get('effort')))
            drive.CreateTargetPositionAttr(0.); drive.CreateTargetVelocityAttr(0.)
    body = world.scene.add(SingleArticulation(prim_path=str(root.GetPath()), name='g1'))
    set_camera_view(eye=[2.7, 2.7, 2.0], target=[0., 0., .9])
    world.reset()
    return world, body, max_frame_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=ROOT/'artifacts/g1-physics-model')
    parser.add_argument('--namespace', default='/bindu_g1_physics')
    parser.add_argument('--navigation-scene', type=Path, help='Optional robot-independent room JSON')
    parser.add_argument('--navigation-robot', type=Path, help='Required with scene: frames, footprint and laser extrinsics')
    parser.add_argument('--navigation-sensors', type=Path, help='Optional wheel odometry/noise configuration; external SLAM/AMCL supplies map and map->odom')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float, default=0., help='Wall seconds after warmup, 0 until window closes')
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/g1-physics-run')
    parser.add_argument('--no-ros', action='store_true', help='View the stationary model without the control bridge')
    parser.add_argument('--command-delay', type=float, default=0., help='Extra simulated command delivery delay in seconds (0..1)')
    args = parser.parse_args()
    if not math.isfinite(args.command_delay) or not 0 <= args.command_delay <= 1:
        parser.error('--command-delay must be finite and between 0 and 1 seconds')
    args.model = args.model.resolve(); args.output.mkdir(parents=True, exist_ok=True)
    cfg = json.loads((args.model/'physics.json').read_text())
    from isaacsim import SimulationApp
    # Use Kit's supported default: the local RC crashes during full extension
    # teardown even after ROS is destroyed. Flush our evidence before closing.
    app = SimulationApp({'headless': args.headless, 'width': 1280, 'height': 900, 'fast_shutdown': True})
    # The RC's SIGINT handler unloads plugins while Python still uses them.
    # Finish the loop, destroy ROS, then close Kit exactly once instead.
    quitting = [False]
    def request_quit(signum, frame):
        quitting[0] = True
    signal.signal(signal.SIGINT, request_quit)
    signal.signal(signal.SIGTERM, request_quit)
    node = None; failed = False
    try:
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction
        room = json.loads(args.navigation_scene.read_text()) if args.navigation_scene else None
        nav_robot = json.loads(args.navigation_robot.read_text()) if args.navigation_robot else None
        sensor_cfg = json.loads(args.navigation_sensors.read_text()) if args.navigation_sensors else {'mode':'ideal'}
        if room and not nav_robot: raise ValueError('NAVIGATION_ROBOT_CONFIG_REQUIRED')
        world, body, frame_error = build_scene(args.model, cfg, room)
        sensors = None
        # Start inside one-sided limits: gravity/solver error at q=0 must not
        # be hidden by clipping measured feedback or relaxing execution limits.
        initial_positions = dict(cfg['initial_joint_positions'])
        if nav_robot:
            initial_positions.update(nav_robot.get('transport_positions', {}))
        initial = np.array([initial_positions.get(n, 0.) for n in body.dof_names])
        body.set_joint_positions(initial)
        body.apply_action(ArticulationAction(joint_positions=initial))
        for i in range(120):
            world.step(render=i % 4 == 0)
        state = {'code': 'READY', 'received': 0, 'rejected': 0, 'expired': 0}
        names = body.dof_names
        joint_names = [n for n in names if not n.startswith('sim_')]
        indices = {n: names.index(n) for n in names}
        wheel_indices = [indices['sim_left_wheel_joint'], indices['sim_right_wheel_joint']]
        wheel_odometry = None
        if sensor_cfg['mode']=='wheel':
            from bindu_hardware.robot_io.odometry import DifferentialOdometry
            wheel_odometry = DifferentialOdometry(cfg['wheel_radius']*sensor_cfg['wheel_radius_scale'],
                cfg['wheel_separation']*sensor_cfg['wheel_separation_scale'])
            wheel_odometry.update(*map(float,body.get_joint_positions()[wheel_indices]),world.current_time)
        if len(joint_names) != 19 or len(names) != 21:
            raise RuntimeError('PHYSICS_DOF_LAYOUT_MISMATCH')
        simulator_id = uuid.uuid4().hex
        def hold(group):
            if group == 'base':
                body.apply_action(ArticulationAction(joint_velocities=np.zeros(2), joint_indices=wheel_indices))
            else:
                selected = [indices[n] for n in profile.groups[group]]
                body.apply_action(ArticulationAction(joint_positions=body.get_joint_positions()[selected], joint_indices=selected))
        if not args.no_ros:
            import rclpy
            from rclpy.signals import SignalHandlerOptions
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from bindu_contracts.profile import Profile
            from bindu_hardware.drivers.physics import PhysicsCommandGate
            from bindu_interfaces.msg import SimulationCommand, SimulationFeedback
            from geometry_msgs.msg import Pose, PoseArray
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysicsSchemaTools
            from builtin_interfaces.msg import Time
            profile = Profile.load(args.model/'g1_physics_sim.json')
            joint_names = [n for group in profile.groups.values() for n in group]
            gate = PhysicsCommandGate(profile, simulator_id, cfg['command_timeout'])
            # Keep our quit flag authoritative. rclpy's default SIGINT handler
            # can invalidate the context halfway through publishing a sample.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node('isaac_physics', namespace=args.namespace, cli_args=['--ros-args',
                '-r', '/tf:='+args.namespace+'/tf', '-r', '/tf_static:='+args.namespace+'/tf_static'])
            if room:
                from isaac_navigation_scene import NavigationSensors
                from std_msgs.msg import String
                from isaacsim.core.utils.viewports import set_camera_view
                if sensor_cfg['mode']=='lio':
                    from isaac_lio_sensors import LioSensors
                    sensors = LioSensors(node, sensor_cfg, '/G1')
                else:
                    sensors = NavigationSensors(node, world, room, nav_robot, '/G1', sensor_cfg)
                    sensors.change(String(data='clear'))
                set_camera_view(eye=[6.,-7.,6.], target=[1.5,0.,.3])
            def now(): return node.get_clock().now().nanoseconds/1e9
            def receive(command):
                try:
                    group = gate.accept(command, now())
                    if command.operation == 'hold':
                        hold(group)
                    elif group == 'base':
                        v, w = command.velocity.linear.x, command.velocity.angular.z
                        speed = np.array([v-w*cfg['wheel_separation']/2, v+w*cfg['wheel_separation']/2])/cfg['wheel_radius']
                        body.apply_action(ArticulationAction(joint_velocities=speed, joint_indices=wheel_indices))
                    else:
                        body.apply_action(ArticulationAction(joint_positions=np.array(command.positions),
                            joint_indices=[indices[n] for n in command.joint_names]))
                    state['received'] += 1; state['code'] = 'OK'
                except ValueError as exc:
                    state['rejected'] += 1; state['code'] = str(exc)
            pending_commands = deque()
            def enqueue(command):
                pending_commands.append((time.monotonic()+args.command_delay, command))
            subscription = node.create_subscription(SimulationCommand, 'simulation/command',
                enqueue if args.command_delay else receive, 8)
            publisher = node.create_publisher(SimulationFeedback, 'simulation/feedback', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            # Independent PhysX link transforms, not FK reconstructed from q.
            # PoseArray order is base, left flange, right flange, in world.
            import omni.physics.tensors as tensors
            from isaacsim.core.experimental.utils import stage as stage_utils
            diagnostic_sim = tensors.create_simulation_view('numpy', stage_id=stage_utils.get_stage_id(world.stage))
            diagnostic_sim.set_subspace_roots('/')
            sleep_stage = stage_utils.get_stage_id(world.stage)
            sleep_body = PhysicsSchemaTools.sdfPathToInt(body.prim_path)
            diagnostic_body = diagnostic_sim.create_articulation_view(body.prim_path)
            link_order = ['base_link','left_arm_end_effector_mount_link','right_arm_end_effector_mount_link']
            link_names = list(diagnostic_body.get_metatype(0).link_names)
            link_indices = [link_names.index(n) for n in link_order]
            pose_publisher = node.create_publisher(PoseArray, 'simulation/link_poses', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Kit's long-lived startup heap took 236 ms to scan in generation 2,
        # interrupting real feedback beyond the executor's 200 ms deadline.
        # Collect before accepting control, then exclude that stable heap from
        # cyclic scans. Runtime allocations still use normal automatic GC.
        gc.collect()
        gc.freeze()
        metadata = {'simulator_id': simulator_id, 'dof_names': names, 'max_joint_frame_error': frame_error,
                    'gc_frozen_startup_objects': gc.get_freeze_count(),
                    'link_pose_order': link_order if node else [],
                    'navigation_scene': room, 'navigation_robot': nav_robot,
                    'odometry': sensor_cfg if room else None,
                    'command_delay_s': args.command_delay,
                    'gui': not args.headless, 'model': str(args.model), 'simulation_only': True}
        (args.output/'scene.json').write_text(json.dumps(metadata, indent=2)+'\n')
        world.stage.GetRootLayer().Export(str((args.output/'scene.usda').resolve()))
        # Capture the actual viewport, not a generated illustration.
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        world.step(render=True)
        capture_viewport_to_file(get_active_viewport(), str((args.output/'scene.png').resolve()))
        print('BINDU_G1_PHYSICS_READY', json.dumps(metadata), flush=True)
        collection_started = {}
        def collection_timing(phase, info):
            generation = info['generation']
            if phase == 'start':
                collection_started[generation] = time.monotonic()
            else:
                elapsed = time.monotonic()-collection_started.pop(generation, time.monotonic())
                if elapsed > .05:
                    print('SLOW_GARBAGE_COLLECTION', json.dumps({'wall_time':time.time(),
                        'duration_ms':elapsed*1000, **info}), flush=True)
        gc.callbacks.append(collection_timing)
        started = time.monotonic(); step = 0; last_physics_time = world.current_time
        while not quitting[0] and app.is_running() and (args.duration <= 0 or time.monotonic()-started < args.duration):
            loop_started = time.monotonic()
            deadline = loop_started+cfg['physics_dt']
            if world.current_time < last_physics_time:
                raise RuntimeError('SIM_TIMELINE_RESET_REQUIRES_RESTART')
            if node:
                for _ in range(8):
                    rclpy.spin_once(node, timeout_sec=0.)
                while pending_commands and pending_commands[0][0] <= time.monotonic():
                    # Validate original timestamps on delivery, including hold;
                    # the local watchdog remains immediate and independent.
                    receive(pending_commands.popleft()[1])
                for group in gate.expired(now()):
                    hold(group); state['expired'] += 1; state['code'] = 'SIM_COMMAND_TIMEOUT'
            after_receive = time.monotonic()
            previous_time = world.current_time
            world.step(render=step % 4 == 0)
            after_step = time.monotonic()
            after_feedback_read = after_feedback_publish = after_sensors = after_step
            step += 1
            last_physics_time = world.current_time
            if node and world.current_time > previous_time:
                # Wall clock deliberately matches the existing Bindu executor.
                # A stopped physics loop cannot refresh this measurement stamp.
                stamp_ns = node.get_clock().now().nanoseconds
                msg = SimulationFeedback(stamp=Time(sec=stamp_ns//1000000000, nanosec=stamp_ns%1000000000),
                    simulation_time=world.current_time, simulator_id=simulator_id, sequence=step,
                    profile_hash=profile.digest, ready=True, code=state['code'])
                msg.joints.header.stamp = msg.stamp
                msg.joints.name = joint_names
                q, dq = body.get_joint_positions(), body.get_joint_velocities()
                msg.joints.position = [float(q[indices[n]]) for n in joint_names]
                msg.joints.velocity = [float(dq[indices[n]]) for n in joint_names]
                position, quaternion = body.get_world_pose()
                msg.base_pose.position.x, msg.base_pose.position.y, msg.base_pose.position.z = map(float, position)
                qw, qx, qy, qz = map(float, quaternion)
                msg.base_pose.orientation.w = qw; msg.base_pose.orientation.x = qx
                msg.base_pose.orientation.y = qy; msg.base_pose.orientation.z = qz
                yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
                velocity = body.get_linear_velocity()
                angular = body.get_angular_velocity()
                sleeping = get_physx_simulation_interface().is_sleeping(sleep_stage, sleep_body)
                raw_velocity = [float(v) for v in velocity] + [float(v) for v in angular]
                if state.get('sleeping') != sleeping:
                    print('PHYSICS_SLEEP_STATE', json.dumps({'wall_time':time.time(),'sleeping':sleeping,
                        'raw_velocity':raw_velocity,'position':[float(v) for v in position]}), flush=True)
                state['sleeping'] = sleeping
                state['raw_base_velocity'] = raw_velocity
                # PhysX can retain the last solver base velocity while an
                # articulation sleeps. Sleep status, never command state or
                # numerical tolerance, is the authority for zero motion.
                if sleeping: velocity = np.zeros(3); angular = np.zeros(3)
                msg.base_velocity.linear.x = float(math.cos(yaw)*velocity[0]+math.sin(yaw)*velocity[1])
                msg.base_velocity.linear.y = float(-math.sin(yaw)*velocity[0]+math.cos(yaw)*velocity[1])
                msg.base_velocity.angular.z = float(angular[2])
                after_feedback_read = time.monotonic()
                publisher.publish(msg)
                after_feedback_publish = time.monotonic()
                if sensors and sensor_cfg['mode']=='lio':
                    sensors.publish(msg)
                elif sensors and step % 4 == 0:
                    odometry = wheel_odometry.update(*map(float,q[wheel_indices]),world.current_time) if wheel_odometry else None
                    sensors.publish(msg, odometry)
                after_sensors = time.monotonic()
                if step % 6 == 0:
                    link_poses = PoseArray()
                    link_poses.header.stamp = msg.stamp; link_poses.header.frame_id = 'world'
                    transforms = diagnostic_body.get_link_transforms()[0]
                    for index in link_indices:
                        values = transforms[index]
                        pose = Pose()
                        pose.position.x,pose.position.y,pose.position.z = map(float,values[:3])
                        pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = map(float,values[3:])
                        link_poses.poses.append(pose)
                    pose_publisher.publish(link_poses)
            if step == 120:
                capture_viewport_to_file(get_active_viewport(), str((args.output/'scene.png').resolve()))
            if step % 120 == 0:
                (args.output/'heartbeat.json').write_text(json.dumps({'wall_time':time.time(),
                    'simulation_time':world.current_time,'playing':world.is_playing(),'steps':step,
                    'received':state['received'],'rejected':state['rejected'],'code':state['code'],
                    'sleeping':state.get('sleeping'),'raw_base_velocity':state.get('raw_base_velocity')})+'\n')
            if time.monotonic()-loop_started>.05:
                print('SLOW_SIMULATION_STEP',json.dumps({'wall_time':time.time(),'step':step,
                    'rendered':(step-1)%4==0,
                    'receive_ms':(after_receive-loop_started)*1000,'physics_render_ms':(after_step-after_receive)*1000,
                    'feedback_read_publish_ms':(after_feedback_publish-after_step)*1000,
                    'feedback_read_ms':(after_feedback_read-after_step)*1000,
                    'feedback_publish_ms':(after_feedback_publish-after_feedback_read)*1000,
                    'sensors_ms':(after_sensors-after_feedback_publish)*1000,
                    'diagnostics_ms':(time.monotonic()-after_sensors)*1000,
                    'feedback_ms':(time.monotonic()-after_step)*1000,'playing':world.is_playing()}),flush=True)
            time.sleep(max(0., deadline-time.monotonic()))
        (args.output/'run.json').write_text(json.dumps({**state, 'steps': step, 'wall_seconds': time.monotonic()-started}, indent=2)+'\n')
        gc.callbacks.remove(collection_timing)
    except BaseException:
        import traceback
        traceback.print_exc()
        failed = True
    finally:
        gc.unfreeze()
        if node:
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
        sys.stdout.flush(); sys.stderr.flush()
        if failed:
            # Kit fast shutdown exits with 0, which would mask a Python failure.
            # All Bindu output files use completed writes; preserve failure status.
            os._exit(1)
        app.close()


if __name__ == '__main__':
    main()
