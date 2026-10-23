#!/usr/bin/env python3
"""Isaac Sim 6.2 / PhysX: dynamic Snowbot with a contact-grasped wooden shovel.

Replays verified references or runs the production QP/executor on measured joints.
Only the initial state is set; every subsequent state comes from physics.
No ROS/hardware transport is created. This is an empty-shovel load experiment,
using CAD jaw contacts by default. Jaw effort, friction and low-level drive
parameters are simulation assumptions, not measured hardware parameters.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from tools.simulation.shovel_physics import load_motion, wooden_parts, compound_mass, grip_wrench, internal_wrench, MeasuredJointVelocity, joint_drive_parameters

from tools import ROOT
sys.path.insert(0, str(ROOT/'src/capabilities/bindu_kinematics'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pose(position, wxyz):
    result = np.eye(4)
    result[:3, 3] = position
    result[:3, :3] = Rotation.from_quat(np.r_[wxyz[1:], wxyz[0]]).as_matrix()
    return result


def quaternion(matrix):
    xyzw = Rotation.from_matrix(matrix).as_quat()
    return np.r_[xyzw[3], xyzw[:3]]


def validate_grasp_config(cfg):
    model = cfg.get('grasp_model', 'proxy')
    if model not in ('proxy', 'contact'):
        raise ValueError('UNKNOWN_GRASP_MODEL')
    if model == 'contact':
        settings = cfg['contact_grasp']
        fields = ('joint_stiffness_nm_rad','joint_damping_nm_s_rad','max_effort_nm','axial_effort_nm','contact_offset_m')
        if (any(not np.isfinite(settings[k]) or settings[k] <= 0 for k in fields) or
                settings['axial_effort_nm'] > settings['max_effort_nm'] or
                settings['contact_offset_m'] > .001):
            raise ValueError('INVALID_CONTACT_GRASP_DRIVE')
        for key in ('closed_positions_rad','open_positions_rad'):
            if (set(settings[key]) != {'arm_l_joint_8','arm_r_joint_8'} or
                    not np.isfinite(list(settings[key].values())).all()):
                raise ValueError('INVALID_CONTACT_GRASP_JOINTS')


def build_scene(motion, cfg, output):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, PhysicsSchemaTools, UsdLux
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, RigidPrim
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.viewports import set_camera_view
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    source = Path(motion['display_urdf'])
    tree = ET.parse(source)
    prefix = 'package://bindu_description/'
    for mesh in tree.findall('.//mesh'):
        uri = mesh.get('filename')
        if not uri.startswith(prefix):
            raise ValueError('UNEXPECTED_MESH_URI:'+uri)
        mesh.set('filename', str(source.parent.parent/uri.removeprefix(prefix)))
    absolute = output/'model.urdf'
    tree.write(absolute, encoding='utf-8', xml_declaration=True)
    usd = URDFImporter(URDFImporterConfig(urdf_path=str(absolute), usd_path=str(output/'usd'),
        fix_base=True, merge_fixed_joints=False, allow_self_collision=False,
        joint_drive_type='force', joint_target_type='position',
        override_joint_stiffness=cfg['joint_stiffness_nm_rad'],
        override_joint_damping=cfg['joint_damping_nm_s_rad'])).import_urdf()
    dt = cfg['physics_dt_s']
    world = World(stage_units_in_meters=1., physics_dt=dt, rendering_dt=dt*round(1/(cfg['gui_render_hz']*dt)))
    stage = world.stage
    model = stage.DefinePrim('/World/Robot', 'Xform')
    model.GetReferences().AddReference(str(usd))
    variant = model.GetVariantSets().GetVariantSet('Physics')
    if not variant.IsValid() or not variant.SetVariantSelection('physx'):
        raise ValueError('PHYSX_ASSET_VARIANT_REQUIRED')
    roots = [p for p in Usd.PrimRange(model) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise ValueError('ARTICULATION_ROOT_LAYOUT:'+str([str(p.GetPath()) for p in roots]))
    art = PhysxSchema.PhysxArticulationAPI.Apply(roots[0])
    art.CreateSolverPositionIterationCountAttr(cfg['solver_position_iterations'])
    art.CreateSolverVelocityIterationCountAttr(cfg['solver_velocity_iterations'])
    body = world.scene.add(SingleArticulation(str(roots[0].GetPath()), name='snowbot'))
    # PhysX uses float32 joint constraints. Keep numerical penetration of its
    # hard stops inside the public URDF bounds; never clip measured feedback.
    inset = float(cfg.get('joint_limit_inset_rad', 0.))
    if not np.isfinite(inset) or not 0 <= inset <= .001:
        raise ValueError('PHYSICS_JOINT_LIMIT_INSET')
    physical_limits = {}
    for prim in Usd.PrimRange(model):
        if not prim.IsA(UsdPhysics.RevoluteJoint):continue
        joint = UsdPhysics.RevoluteJoint(prim)
        lo, hi = joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()
        if lo is None or hi is None:continue
        lo, hi = np.deg2rad([lo, hi])
        if hi-lo <= 2*inset:raise ValueError('PHYSICS_JOINT_LIMIT_WIDTH')
        physical_limits[prim.GetName()] = [float(lo+inset),float(hi-inset)]
        if inset:
            joint.GetLowerLimitAttr().Set(float(np.rad2deg(lo+inset)))
            joint.GetUpperLimitAttr().Set(float(np.rad2deg(hi-inset)))
    # Robot self-collision is checked independently against CAD. Tool/robot
    # contact stays enabled in contact mode; legacy proxy mode is explicit.
    links = {p.GetName(): p for p in Usd.PrimRange(model) if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    contact_grasp = cfg.get('grasp_model', 'proxy') == 'contact'
    grip_colliders = []
    if contact_grasp:
        # Edit only generated USD. Keep the original CAD and URDF unchanged.
        jaw_names = {f'arm_{s}_link_{n}' for s in 'lr' for n in (7, 8)}
        for prim in list(Usd.PrimRange(model)):
            if prim.IsInstance() and any(n in str(prim.GetPath()).split('/') for n in jaw_names):
                prim.SetInstanceable(False)
        for prim in Usd.PrimRange(model):
            if (prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Mesh) and
                    any(n in str(prim.GetPath()).split('/') for n in jaw_names)):
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexDecomposition')
                decomp = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
                decomp.CreateMaxConvexHullsAttr(64)
                decomp.CreateHullVertexLimitAttr(64)
                decomp.CreateVoxelResolutionAttr(500000)
                decomp.CreateErrorPercentageAttr(.1)
                decomp.CreateShrinkWrapAttr(True)
                contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                contact.CreateContactOffsetAttr(cfg['contact_grasp']['contact_offset_m'])
                contact.CreateRestOffsetAttr(0.)
                grip_colliders.append(str(prim.GetPath()))
        if len(grip_colliders) != 4:
            raise ValueError('GRIP_COLLIDER_LAYOUT:'+str(grip_colliders))
    # Massless URDF frames remain Xforms in 6.2. Apply each grasp to its
    # physical ancestor and retain the exact fixed-frame transform.
    joints_by_child = {j.find('child').get('link'):j for j in tree.findall('joint')}
    flanges, flange_mounts = [], []
    for side in ('left', 'right'):
        name = motion[side+'_frame']
        mount = np.eye(4)
        while name not in links:
            joint = joints_by_child[name]
            if joint.get('type') != 'fixed':
                raise ValueError('NONRIGID_FLANGE_ANCESTOR:'+name)
            origin = joint.find('origin')
            fixed = np.eye(4)
            if origin is not None:
                fixed[:3,3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                fixed[:3,:3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            mount = fixed @ mount
            name = joint.find('parent').get('link')
        flanges.append(links[name])
        flange_mounts.append(mount)
    tool = UsdGeom.Xform.Define(stage, '/World/Shovel')
    tool_prim = tool.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(tool_prim)
    rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(tool_prim)
    rigid_api.CreateLinearDampingAttr(0.)
    rigid_api.CreateAngularDampingAttr(0.)
    parts = wooden_parts(motion['recipe'], cfg['material'])
    mass, com, inertia = compound_mass(parts)
    eigenvalues, principal = np.linalg.eigh(inertia)
    if np.linalg.det(principal) < 0:
        principal[:, 0] *= -1
    api = UsdPhysics.MassAPI.Apply(tool_prim)
    api.CreateMassAttr(mass)
    api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(*eigenvalues))
    api.CreatePrincipalAxesAttr(Gf.Quatf(quaternion(principal)[0], Gf.Vec3f(*quaternion(principal)[1:])))
    material = UsdShade.Material.Define(stage, '/World/WoodMaterial')
    mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    for key, creator in (('static_friction', mat.CreateStaticFrictionAttr),
                         ('dynamic_friction', mat.CreateDynamicFrictionAttr),
                         ('restitution', mat.CreateRestitutionAttr)):
        creator(cfg['material'][key])
    for part in parts:
        path = '/World/Shovel/'+part['name']
        if part['kind'] == 'box':
            shape = UsdGeom.Cube.Define(stage, path)
            shape.CreateSizeAttr(1.)
            scale = part['size']
        else:
            shape = UsdGeom.Cylinder.Define(stage, path)
            shape.CreateRadiusAttr(part['radius'])
            shape.CreateHeightAttr(part['length'])
            shape.CreateAxisAttr('Z')
            scale = np.ones(3)
        tf = np.eye(4)
        tf[:3, :3] = part['rotation'] @ np.diag(scale)
        tf[:3, 3] = part['center']
        shape.AddTransformOp().Set(Gf.Matrix4d(tf.T.tolist()))
        shape.CreateDisplayColorAttr([Gf.Vec3f(.55, .33, .15)])
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
        contact.CreateContactOffsetAttr(.001)
        contact.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material, materialPurpose='physics')
    for path in grip_colliders:
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(material, materialPurpose='physics')
    if not contact_grasp:
        UsdPhysics.FilteredPairsAPI.Apply(tool_prim).CreateFilteredPairsRel().SetTargets(
            [link.GetPath() for link in links.values()])
    z = motion['recipe']['ground']['z_in_base_m']-cfg.get('diagnostic_ground_drop_m',0.)
    PhysicsSchemaTools.addGroundPlane(stage, '/World/Ground', 'Z', 5., Gf.Vec3f(0., 0., z), Gf.Vec3f(.5))
    from tools.analysis.shovel_geometry import bin_boxes
    for name, size, center in bin_boxes(motion['recipe']):
        cube = UsdGeom.Cube.Define(stage, '/World/Bin/'+name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.25, .4, .3)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    view = world.scene.add(RigidPrim([str(p.GetPath()) for p in flanges]+['/World/Shovel'], name='grips'))
    contact_paths = [str(p.GetPath()) for p in stage.Traverse()
                     if p.HasAPI(UsdPhysics.CollisionAPI) and
                     (str(p.GetPath()).startswith('/World/Ground/') or str(p.GetPath()).startswith('/World/Bin/'))]
    contacts = {'environment': world.scene.add(RigidPrim('/World/Shovel', name='tool_contacts',
        contact_filter_prim_paths_expr=contact_paths, max_contact_count=1024, disable_stablization=False))}
    if contact_grasp:
        for side, short in (('left', 'l'), ('right', 'r')):
            contacts[side] = world.scene.add(RigidPrim('/World/Shovel', name=side+'_contacts',
                contact_filter_prim_paths_expr=[str(links[f'arm_{short}_link_{n}'].GetPath()) for n in (7,8)],
                max_contact_count=2048, disable_stablization=False))
    UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000.)
    set_camera_view(eye=[3., 2.4, 2.1], target=[.6, -.3, .8])
    world.get_physics_context().set_gravity(-9.81)
    from isaacsim.core.simulation_manager import SimulationManager
    SimulationManager.enable_fabric(cfg.get('use_fabric', True))
    world.reset()
    if SimulationManager.get_active_physics_engine() != 'physx':
        raise ValueError('PHYSX_BACKEND_REQUIRED')
    names = body.dof_names
    limits = {j.get('name'):j.find('limit') for j in tree.findall('joint') if j.get('type') != 'fixed'}
    if set(names) != set(limits):
        raise ValueError('PHYSICS_JOINT_LAYOUT')
    controller = body.get_articulation_controller()
    kp, kd, drive_efforts = joint_drive_parameters(names, limits, cfg)
    controller.set_gains(kp, kd)
    controller.set_max_efforts(drive_efforts)
    gains = controller.get_gains()
    if not np.allclose(gains[0], kp) or not np.allclose(gains[1], kd):
        raise ValueError('DRIVE_GAIN_READBACK')
    if contact_grasp:
        settings = cfg['contact_grasp']
        kp, kd = np.array(gains[0]), np.array(gains[1])
        efforts = drive_efforts.copy()
        for n in settings['closed_positions_rad']:
            i = names.index(n)
            for key in ('closed_positions_rad','open_positions_rad'):
                if not float(limits[n].get('lower')) <= settings[key][n] <= float(limits[n].get('upper')):
                    raise ValueError('GRIP_JOINT_LIMIT:'+n)
            kp[i], kd[i] = settings['joint_stiffness_nm_rad'], settings['joint_damping_nm_s_rad']
            efforts[i] = min(efforts[i], settings['max_effort_nm'])
        controller.set_gains(kp, kd)
        controller.set_max_efforts(efforts)
        gains = controller.get_gains()
    if not np.isclose(view.get_masses()[2], mass, rtol=1e-5):
        raise ValueError('TOOL_MASS_READBACK')
    if list(view.prim_paths) != [str(p.GetPath()) for p in flanges]+['/World/Shovel']:
        raise ValueError('RIGID_VIEW_ORDER')
    source = dict(joint_limit_inset_rad=inset,physical_joint_limits_rad=physical_limits,
                  mass_kg=mass, center_of_mass_in_tool_m=com.tolist(), inertia_at_com_kg_m2=inertia.tolist(),
                  parts=[dict(name=p['name'], mass_kg=p['mass']) for p in parts],
                  dof_names=names, drive_effort_limits_nm=drive_efforts.tolist(),
                  gains_readback=[g.tolist() for g in gains],
                  articulation_root=str(roots[0].GetPath()), flanges=[str(p.GetPath()) for p in flanges],
                  flange_from_physical_body=[m.tolist() for m in flange_mounts],
                  environment_contact_paths=contact_paths)
    source['native_mass_kg'] = float(view.get_masses()[2])
    source['fabric_enabled'] = SimulationManager.is_fabric_enabled()
    source['native_com_in_tool_m'] = np.asarray(view.get_coms()[0]).reshape(-1,3)[2].tolist()
    source['grip_colliders'] = grip_colliders
    source['tool_robot_contacts_enabled'] = contact_grasp
    if not np.allclose(source['native_com_in_tool_m'], com, atol=1e-6):
        raise ValueError('TOOL_COM_READBACK')
    return world, body, view, contacts, source


def contact_wrench(contacts, com, dt):
    """Sum independently measured PhysX normal and friction contact impulses."""
    def indices(counts, starts):
        groups = [np.arange(int(s), int(s+c)) for c,s in zip(np.ravel(counts),np.ravel(starts)) if c]
        return np.concatenate(groups) if groups else np.array([],dtype=int)
    magnitudes, points, normals, separations, counts, starts = contacts.get_contact_force_data(dt=dt)
    selected = indices(counts, starts)
    normal = np.asarray(magnitudes).reshape(-1,1)[selected]*np.asarray(normals).reshape(-1,3)[selected]
    normal_points = np.asarray(points).reshape(-1,3)[selected]
    friction, friction_points, counts, starts = contacts.get_friction_data(dt=dt)
    selected = indices(counts, starts)
    friction = np.asarray(friction).reshape(-1,3)[selected]
    friction_points = np.asarray(friction_points).reshape(-1,3)[selected]
    fn, ft = normal.sum(axis=0), friction.sum(axis=0)
    torque = (np.cross(normal_points-com, normal).sum(axis=0)+
              np.cross(friction_points-com, friction).sum(axis=0))
    return fn+ft, torque, fn, ft


def grasp_contact_wrenches(contacts, tool, offsets, com, dt):
    """Actual PhysX contact loads on the tool, expressed at the grasp points."""
    forces, torques, points, separation = [], [], [], []
    for side, offset in zip(('left', 'right'), offsets):
        point = tool[:3,3]+tool[:3,:3] @ offset
        force, moment, _, _ = contact_wrench(contacts[side], com, dt)
        forces.append(force); torques.append(moment-np.cross(point-com,force)); points.append(point)
        _, _, _, gaps, counts, starts = contacts[side].get_contact_force_data(dt=dt)
        for count, start in zip(np.ravel(counts), np.ravel(starts)):
            if count:
                separation.extend(np.ravel(gaps)[int(start):int(start+count)].tolist())
    return forces, torques, points, min(separation, default=0.)


def simulate(app, args, motion, cfg):
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.simulation_manager import SimulationManager
    output = args.output
    world, body, view, contacts, source = build_scene(motion, cfg, output)
    names = body.dof_names
    samples = motion['samples']
    times = np.array([s['t'] for s in samples])
    plan = np.array([[s['q'][n] for n in names] for s in samples])
    dense_path = args.motion.parent/'joint_plan.npz'
    if dense_path.is_file():
        dense = np.load(dense_path)
        dense_names = list(dense['names'])
        dense_q = dense['executed']
        dense_times = np.arange(len(dense_q))*float(dense['dt'])
        if (not set(dense_names).issubset(names) or not np.isfinite(dense_q).all() or
                abs(dense_times[-1]-times[-1]) > 1e-6):
            raise ValueError('DENSE_REFERENCE_LAYOUT')
        for j, name in enumerate(dense_names):
            if not np.allclose(np.interp(times, dense_times, dense_q[:, j]), plan[:, names.index(name)], atol=1e-10, rtol=0):
                raise ValueError('DENSE_REFERENCE_MISMATCH:'+name)
        reference = np.tile(plan[0], (len(dense_q), 1))
        for j, name in enumerate(dense_names):
            reference[:, names.index(name)] = dense_q[:, j]
        times, plan = dense_times, reference
        source['joint_reference_sha256'] = digest(dense_path)
    source['joint_reference_sample_period_s'] = float(times[1]-times[0])
    source_config = json.loads(Path(motion['config_path']).read_text())
    mounts = [np.array(source_config[side+'_to_tool']).reshape(4, 4)[:3, :3] for side in ('left', 'right')]
    recipe = motion['recipe']
    contact_grasp = cfg.get('grasp_model', 'proxy') == 'contact'
    cfg['shaft_axis_in_tool'] = np.array(recipe['shaft_axis_in_tool'])
    tool_offsets = [np.array(recipe[side+'_grip_in_tool_m']) for side in ('left', 'right')]
    hand_offsets = [np.array(recipe.get(side+'_grip_offset_m', recipe['grip_offset_m']))
                    for side in ('left', 'right')]
    # A known axial grip calibration error is carried with the right flange.
    hand_offsets[1] += mounts[1] @ cfg['shaft_axis_in_tool']*cfg.get('right_grip_axial_offset_m', 0.)
    body.set_joint_positions(plan[0])
    body.set_joint_velocities(np.zeros(len(names)))
    body.apply_action(ArticulationAction(joint_positions=plan[0]))
    home = np.array(motion['home'])
    view.set_world_poses(home[None, :3, 3], quaternion(home[:3, :3])[None, :], indices=[2])
    view.set_velocities(np.zeros((1, 6)), indices=[2])
    initial_xyz, initial_quat = view.get_world_poses()
    (output/'initial_state.json').write_text(json.dumps({
        'rigid_positions_m':initial_xyz.tolist(), 'rigid_quaternions_wxyz':initial_quat.tolist(),
        'tool_home':home.tolist(), 'q':body.get_joint_positions().tolist(),
        'source':source},indent=2)+'\n')
    # Propagate the single initial joint write to native link transforms.
    native_start = SimulationManager.get_num_physics_steps()
    world.step(render=False)
    if SimulationManager.get_num_physics_steps()-native_start != 1:
        raise ValueError('CALIBRATION_STEP_COUNT')
    freefall_acceleration = view.get_velocities()[2, :3]/cfg['physics_dt_s']
    if not contact_grasp and not np.allclose(freefall_acceleration, [0., 0., -9.81], atol=.01):
        raise ValueError('FREE_FALL_CALIBRATION:'+str(freefall_acceleration))
    source['initial_native_acceleration_m_s2' if contact_grasp else 'freefall_acceleration_m_s2'] = freefall_acceleration.tolist()
    dt, settle = cfg['physics_dt_s'], cfg['settle_s']
    warmup = cfg.get('control_warmup_s',0.) if args.closed_loop else 0.
    duration = min(args.duration, times[-1]) if args.duration is not None else times[-1]
    online = None
    responses = []
    mode = 'rigid'
    qref = plan[0].copy()
    next_control = 0.
    stopped_since = None
    input_path = args.motion.parent/'tool_inputs.json'
    if args.closed_loop:
        from tools.simulation.shovel_control import ControlClient
        frames = json.loads(input_path.read_text())
        if not frames or any(b['stamp'] <= a['stamp'] for a,b in zip(frames,frames[1:])):
            raise ValueError('CONTROL_INPUT_ORDER')
        prefix_count = round(warmup/.01)
        if not np.isclose(prefix_count*.01,warmup):raise ValueError('CONTROL_WARMUP_GRID')
        # Known ready goal only, no future motion or future target derivatives.
        prefix = [dict(frames[0],stamp=round(i*.01,9),seq=i) for i in range(prefix_count)]
        frames = prefix+[dict(f,stamp=round(f['stamp']+warmup,9),seq=f['seq']+prefix_count) for f in frames]
        feedback = None
        if args.load_feedback:
            feedback = dict(cfg['load_feedback'],simulation_only=True,
                **{k:cfg[k] for k in ('grip_translation_stiffness_n_m','grip_translation_damping_ns_m',
                                     'grip_rotation_stiffness_nm_rad','grip_rotation_damping_nm_s_rad')})
        online = ControlClient(motion['config_path'],output,feedback)
        source['online_controller'] = online.hello
        source['tool_inputs_sha256'] = digest(input_path)
    input_cursor = -1
    sensor_readings = None
    encoder_velocity = MeasuredJointVelocity()
    source['feedback_velocity_source'] = cfg.get('feedback_velocity_source','native')
    count = round((settle+warmup+duration+(6. if online else 0.))/dt)
    com_local, _ = view.get_coms()
    com_local = np.asarray(com_local).reshape(-1, 3)
    rows = []
    previous_mode = 'rigid'
    last_drive_mode = None
    static_forces, static_torques = [], []
    started = time.monotonic()
    max_step_residual = 0.
    max_axial_couple = 0.
    minimum_contact_separation = 0.
    initial_tool = None
    max_projected_effort = np.zeros(len(names))
    npz_path = output/'samples.npz'
    recorder = None
    step_times, render_times, render_stamps = [], [], []
    idle_wall = 0.
    next_gui_render = started
    if args.record_video:
        from tools.simulation.shovel_video import ViewportRecorder
        recorder = ViewportRecorder(output, args.video_fps)
    try:
        for step in range(count+1):
            if not app.is_running():
                raise RuntimeError('SIMULATION_INTERRUPTED')
            elapsed = step*dt
            control_t = round(max(0., elapsed-settle),9)
            t = round(control_t-warmup,9)
            if recorder and 0. <= t <= duration and recorder.due(t):
                recorder.capture(world, t)
            q = body.get_joint_positions()
            qd = body.get_joint_velocities()
            if online and elapsed >= settle:
                response = online.poll(control_t)
                if response is not None:
                    responses.append(response)
                    qref = np.array([response['targets'][n] for n in names])
                    mode = response['grasp_mode']
                    if response['stopped']:
                        stopped_since = t if stopped_since is None else stopped_since
                    else:
                        stopped_since = None
            elif not online:
                qref = np.array([np.interp(t, times, plan[:, j]) for j in range(len(names))])
                mode = next(k['mode'] for k in reversed(recipe['grasp_schedule']) if t+1e-9 >= k['t'])
            if getattr(args, 'grasp_test', None) == 'hold':
                qref = plan[0].copy()
            if contact_grasp:
                settings = cfg['contact_grasp']
                released = getattr(args, 'grasp_test', None) == 'release' and t >= 1.
                positions = settings['open_positions_rad'] if released else settings['closed_positions_rad']
                for n, value in positions.items():
                    qref[names.index(n)] = value
                if last_drive_mode != mode:
                    limits = np.array(source['drive_effort_limits_nm'])
                    for n in settings['closed_positions_rad']:
                        limits[names.index(n)] = min(limits[names.index(n)], settings['max_effort_nm'])
                    if mode == 'right_axial':
                        limits[names.index('arm_r_joint_8')] = settings['axial_effort_nm']
                    body.get_articulation_controller().set_max_efforts(limits)
                    last_drive_mode = mode
            body.apply_action(ArticulationAction(joint_positions=qref))
            xyz, quat = view.get_world_poses()
            transforms = [pose(p, q) for p, q in zip(xyz, quat)]
            hands = [tf @ np.array(mount) for tf, mount in zip(transforms[:2], source['flange_from_physical_body'])]
            velocities = view.get_velocities()
            coms = np.array([tf[:3, 3]+tf[:3, :3] @ offset for tf, offset in zip(transforms, com_local)])
            if not online:
                mode = next(k['mode'] for k in reversed(recipe['grasp_schedule']) if t+1e-9 >= k['t'])
            if mode != previous_mode and mode == 'rigid':
                # Re-lock the current physical relative angle, no pose write.
                mounts[1] = hands[1][:3, :3].T @ transforms[2][:3, :3]
            previous_mode = mode
            if initial_tool is None and t >= 0:
                initial_tool = transforms[2].copy()
            errors = [float(np.linalg.norm(hand[:3,3]+hand[:3,:3] @ ho-
                transforms[2][:3,3]-transforms[2][:3,:3] @ to))
                for hand,ho,to in zip(hands,hand_offsets,tool_offsets)]
            forces, torques, points = [], [], []
            if contact_grasp:
                forces, torques, points, _ = grasp_contact_wrenches(contacts, transforms[2], tool_offsets, coms[2], dt)
            else:
                for side in range(2):
                    f, tau, point, _ = grip_wrench(hands[side], transforms[2], velocities[side], velocities[2],
                        coms[side], coms[2], hand_offsets[side], tool_offsets[side], mounts[side], cfg,
                        axial=side == 1 and mode == 'right_axial')
                    forces.append(f); torques.append(tau); points.append(point)
            shaft_world = transforms[2][:3,:3] @ cfg['shaft_axis_in_tool']
            decomposition = internal_wrench(forces, torques, points, coms[2],
                released_axis=shaft_world if mode=='right_axial' and not contact_grasp else None)
            if args.wrench_stop_at is None or t < args.wrench_stop_at:
                sensor_readings = {}
                for i,side in enumerate(('left','right')):
                    f = -forces[i]
                    moment = -torques[i]+np.cross(points[i]-hands[i][:3,3],f)
                    sensor_readings[side] = dict(stamp=t,frame_id=source_config[side+'_frame'],
                        wrench=np.r_[hands[i][:3,:3].T @ f,hands[i][:3,:3].T @ moment].tolist())
                    sensor_readings[side]['stamp'] = control_t
            if online and elapsed >= settle and control_t+1e-9 >= next_control and online.pending is None:
                cutoff = warmup+min(duration, args.input_stop_at if args.input_stop_at is not None else duration)
                while input_cursor+1 < len(frames) and frames[input_cursor+1]['stamp'] <= min(control_t,cutoff):
                    input_cursor += 1
                frame = frames[input_cursor] if input_cursor >= 0 else None
                measured_qd = (encoder_velocity.update(control_t,q,qd)
                    if source['feedback_velocity_source']=='position_difference' else qd)
                online.send(control_t,dict(zip(names,map(float,q))),dict(zip(names,map(float,measured_qd))),frame,
                    cancel=args.cancel_at is not None and t >= args.cancel_at,
                    freeze_feedback=args.feedback_stop_at is not None and t >= args.feedback_stop_at,wrenches=sensor_readings)
                next_control = control_t+.01
            if mode == 'right_axial':
                shaft_world = transforms[2][:3, :3] @ cfg['shaft_axis_in_tool']
                max_axial_couple = max(max_axial_couple, abs(float(torques[1] @ shaft_world)))
            # Accumulate both grips on the tool, one opposite wrench per hand.
            applied_forces = np.vstack((-np.array(forces), np.sum(forces, axis=0)))
            applied_torques = np.vstack(([-tau+np.cross(point-com, -f)
                for f, tau, point, com in zip(forces, torques, points, coms[:2])],
                decomposition['net_torque']))
            if not contact_grasp:
                view.apply_forces_and_torques_at_pos(forces=applied_forces.astype(np.float32),
                    torques=applied_torques.astype(np.float32), positions=coms.astype(np.float32), is_global=True)
            joint_error = float(np.max(abs(q-qref)))
            if not np.isfinite(np.r_[q, velocities.ravel(), applied_forces.ravel(), applied_torques.ravel()]).all():
                raise ValueError('NONFINITE_PHYSICS')
            # Jaw error is expected while squeezing a blocked rigid rod. Arm
            # servo tracking and tool loss remain separate checks.
            if contact_grasp:
                arm_indices = [i for i,n in enumerate(names) if n not in cfg['contact_grasp']['closed_positions_rad']]
                joint_error = float(np.max(abs(q[arm_indices]-qref[arm_indices])))
            diagnostic_release = getattr(args, 'grasp_test', None) == 'release'
            if joint_error > cfg['max_joint_error_rad'] or (not diagnostic_release and max(errors) > cfg['max_grip_error_m']):
                raise ValueError(f'PHYSICS_TRACKING_LOST:t={t:.4f},grip={max(errors)},joint={joint_error}')
            native_before = SimulationManager.get_num_physics_steps()
            step_started = time.monotonic()
            world.step(render=False)
            step_times.append(time.monotonic()-step_started)
            now = time.monotonic()
            if args.gui and now >= next_gui_render:
                # World.step(render=True) advances rendering_dt/physics_dt
                # steps in 6.2. Render separately without advancing physics.
                render_stamps.append(time.monotonic())
                world.render()
                render_times.append(time.monotonic()-render_stamps[-1])
                # UI refresh follows wall time. A slow physics/diagnostic
                # loop must not reduce a 30 Hz viewport to a few frames/s.
                next_gui_render = render_stamps[-1]+1/cfg['gui_render_hz']
                idle_started = time.monotonic()
                time.sleep(max(0., elapsed-(idle_started-started)))
                idle_wall += time.monotonic()-idle_started
            if SimulationManager.get_num_physics_steps()-native_before != 1:
                raise ValueError('UNEXPECTED_NATIVE_STEP_COUNT')
            effort = body.get_measured_joint_efforts()
            if effort is None or not np.isfinite(effort).all():
                raise ValueError('MISSING_PHYSICS_JOINT_EFFORT')
            max_projected_effort = np.maximum(max_projected_effort, abs(effort))
            next_velocity = view.get_velocities()[2]
            cf, ct, cn, friction = contact_wrench(contacts['environment'], coms[2], dt)
            if contact_grasp:
                forces, torques, points, gap = grasp_contact_wrenches(contacts, transforms[2], tool_offsets, coms[2], dt)
                minimum_contact_separation = min(minimum_contact_separation, gap)
                decomposition = internal_wrench(forces, torques, points, coms[2])
            balance = decomposition['net_force']+cf+np.array([0., 0., -9.81*source['mass_kg']])
            residual = np.linalg.norm(source['mass_kg']*(next_velocity[:3]-velocities[2, :3])/dt-balance)
            max_step_residual = max(max_step_residual, float(residual))
            if elapsed >= settle-1 and elapsed < settle:
                static_forces.append(decomposition['net_force']+cf)
                static_torques.append(decomposition['net_torque']+ct)
            rows.append(np.r_[elapsed-settle-warmup, np.array(forces).ravel(), np.array(torques).ravel(),
                decomposition['wrenches'].ravel(), decomposition['axial_squeeze_n'], errors, joint_error,
                transforms[2][:3, 3], velocities[2], decomposition['net_force'], decomposition['net_torque'],
                residual, float(mode == 'right_axial'), q, qref, cf, ct, cn, friction, quat[2],qd])
            if step % max(1, round(5/dt)) == 0:
                print('PHYSICS', args.case, round(elapsed-settle-warmup, 3), 'force_N',
                      [round(float(np.linalg.norm(f)), 2) for f in forces], 'squeeze_N',
                      round(decomposition['axial_squeeze_n'], 2),
                      'control', {k:responses[-1][k] for k in ('accepted','code','stopped')} if responses else None, flush=True)
            if online and stopped_since is not None and t-stopped_since >= .5:
                break
    finally:
        if online:
            online.close()
        if rows:
            np.savez_compressed(npz_path, samples=np.array(rows), joint_names=names)
        if recorder:
            recorder.close()
    data = np.array(rows)
    active = data[:, 0] >= 0
    steady = np.mean(static_forces, axis=0)
    static_error = np.linalg.norm(steady+np.array([0., 0., -9.81*source['mass_kg']]))
    static_torque_error = np.linalg.norm(np.mean(static_torques, axis=0))
    status = ('passed' if static_error < cfg['static_force_balance_tolerance_n'] and
              static_torque_error < cfg['static_torque_balance_tolerance_nm'] and
              max_step_residual < cfg['max_discrete_newton_residual_n'] else 'failed_balance')
    def stats(values):
        return dict(mean=float(np.mean(values)), rms=float(np.sqrt(np.mean(values**2))),
                    p95=float(np.percentile(values, 95)), peak=float(np.max(values)))
    loop_wall = time.monotonic()-started
    performance = dict(loop_wall_s=loop_wall, simulated_s=len(rows)*dt,
        real_time_factor=len(rows)*dt/loop_wall,
        physics_step_wall_s=stats(np.array(step_times)),
        physics_wall_total_s=float(sum(step_times)),
        render_wall_total_s=float(sum(render_times)), idle_wall_s=idle_wall,
        python_and_io_wall_s=loop_wall-sum(step_times)-sum(render_times)-idle_wall,
        render_frames=len(render_times),
        render_interval_wall_s=stats(np.diff(render_stamps)) if len(render_stamps)>1 else None)
    result = dict(status=status, case=args.case, duration_s=duration,
        grasp_model=cfg.get('grasp_model','proxy'), minimum_contact_separation_m=minimum_contact_separation,
        controller_mode=('measured_qp_load' if args.load_feedback else 'measured_qp') if online else 'reference_replay',
        control_warmup_s=warmup,
        physics_steps=SimulationManager.get_num_physics_steps()-native_start,
        calibration_steps=1, recorded_grasp_steps=len(rows),
        wall_s=time.monotonic()-started, physics_dt_s=dt, source=source,
        performance=performance,
        static_force_balance_error_n=float(static_error), static_torque_balance_error_nm=float(static_torque_error),
        max_discrete_newton_residual_n=max_step_residual,
        grip_force_n={side:stats(np.linalg.norm(data[active, 1+3*i:4+3*i], axis=1)) for i, side in enumerate(('left', 'right'))},
        grip_moment_nm={side:stats(np.linalg.norm(data[active, 7+3*i:10+3*i], axis=1)) for i, side in enumerate(('left', 'right'))},
        internal_force_n={side:stats(np.linalg.norm(data[active, 13+6*i:16+6*i], axis=1)) for i, side in enumerate(('left', 'right'))},
        internal_moment_nm={side:stats(np.linalg.norm(data[active, 16+6*i:19+6*i], axis=1)) for i, side in enumerate(('left', 'right'))},
        axial_squeeze_abs_n=stats(abs(data[active, 25])), axial_squeeze_signed_range_n=[float(data[active,25].min()), float(data[active,25].max())],
        max_grip_error_m=float(data[active, 26:28].max()), max_joint_error_rad=float(data[active,28].max()),
        right_axial_max_direct_shaft_couple_nm=max_axial_couple,
        max_solver_projected_joint_effort_nm=dict(zip(names, map(float, max_projected_effort))),
        environment_contact_force_n=stats(np.linalg.norm(data[active,46+2*len(names):49+2*len(names)],axis=1)),
        columns={'time_s':0, 'grip_forces_world_n':[1,7], 'grip_couples_world_nm':[7,13],
            'internal_wrenches_world_n_nm':[13,25], 'axial_squeeze_n':25, 'grip_errors_m':[26,28],
            'max_joint_error_rad':28, 'tool_position_world_m':[29,32], 'tool_velocity_com_world':[32,38],
            'net_grip_force_world_n':[38,41], 'net_grip_torque_at_com_world_nm':[41,44],
            'discrete_newton_residual_n':44, 'right_axial':45, 'q':[46,46+len(names)], 'qref':[46+len(names),46+2*len(names)],
            'environment_contact_force_world_n':[46+2*len(names),49+2*len(names)],
            'environment_contact_moment_at_com_world_nm':[49+2*len(names),52+2*len(names)],
            'normal_contact_force_world_n':[52+2*len(names),55+2*len(names)],
            'friction_contact_force_world_n':[55+2*len(names),58+2*len(names)],
            'tool_quaternion_world_wxyz':[58+2*len(names),62+2*len(names)]},
        assumptions={'wood':'homogeneous rigid compound; no elastic bending or failure',
            'grips':'finite stiffness wrench proxies; no finger contact/friction limit or jaw squeeze force',
            'base':'fixed to world; not a mobile-base stability test', 'snow':'empty shovel, no snow or cutting resistance',
            'robot_collision':'self-collision off; tool-robot filtered; tool-ground/bin enabled',
            'control':('actual joint position/velocity feedback to production QP/session/executor; no tool ground truth feedback'
                       if online else 'force-limited joint reference replay, no online feedback to QP'),
            'internal_metric':'grasp-mode constrained nullspace, moments scaled by actual grip spacing; axial squeeze separately invariant',
            'sample_time':'pre-step state and wrench, discrete Newton residual uses same-step velocity increment'},
        provenance={'motion':str(args.motion), 'motion_sha256':digest(args.motion),
                    'config_sha256':args.source_hashes['config'], 'runner_sha256':args.source_hashes['runner'],
                    'math_sha256':args.source_hashes['math'],
                    'controller_sha256':online.hello['controller_sha256'] if online else None,
                    'samples_sha256':digest(npz_path), 'isaac_version':args.isaac_version})
    # Config contains a NumPy axis only for spring calculations.
    result['columns']['qd_rad_s'] = [62+2*len(names),62+3*len(names)]
    result['configuration'] = {k:v for k,v in cfg.items() if k != 'shaft_axis_in_tool'}
    if contact_grasp:
        result['assumptions'].update(grips='force-limited jaw joints; native mesh-decomposition contact and Coulomb friction; no grasp springs',
            robot_collision='self-collision off; all tool-robot contacts enabled; jaw CAD convex decomposition',
            internal_metric='full grasp-map nullspace of measured contact wrenches; actual friction couples retained')
        if args.grasp_test:
            pose_data = data[data[:,0]>=0,29:32]
            held = data[(data[:,0]>=0)&(data[:,0]<(1. if args.grasp_test=='release' else duration+dt))]
            held_error = float(np.linalg.norm(held[:,29:32]-home[:3,3],axis=1).max())
            drift = float(np.linalg.norm(held[:,29:32]-held[0,29:32],axis=1).max())
            drop = float(pose_data[0,2]-pose_data[-1,2])
            passed = (held_error < .006 and drift < .002 and minimum_contact_separation > -.001 and
                      (args.grasp_test != 'release' or drop > .05))
            result['grasp_test'] = dict(kind=args.grasp_test,
                held_position_error_m=held_error, held_position_drift_m=drift,
                final_vertical_drop_m=drop, passed=passed)
            if not passed:
                result['status'] = status = 'failed_grasp_test'
    if online:
        result['controller'] = dict(last=responses[-1] if responses else None,
            replies=len(responses), max_ipc_wall_s=max((r['ipc_wall_s'] for r in responses),default=None),
            max_worker_wall_s=max((r['worker_wall_s'] for r in responses),default=None),
            simulated_end_s=float(data[-1,0]), stop_observed_for_s=0. if stopped_since is None else t-stopped_since,
            fault_injection=dict(input_stop_at=args.input_stop_at,feedback_stop_at=args.feedback_stop_at,
                                 wrench_stop_at=args.wrench_stop_at,cancel_at=args.cancel_at))
        last = responses[-1] if responses else {}
        expected = ('CANCELED' if args.cancel_at is not None else 'FEEDBACK_STALE' if args.feedback_stop_at is not None
                    else 'GRIP_LOAD_STALE_OR_FRAME' if args.wrench_stop_at is not None else 'TOOL_INPUT_TIMEOUT')
        expected_unconfirmed = args.feedback_stop_at is not None and last.get('stop_fault') == 'STOP_FEEDBACK_TIMEOUT' and not last.get('stopped')
        control_ok = last.get('code') == expected and last.get('accepted',0) > 0 and (last.get('stopped') or expected_unconfirmed)
        result['controller']['expected_stop_code'] = expected
        result['controller']['stop_confirmed'] = bool(last.get('stopped'))
        result['controller']['outcome'] = ('expected_stop_unconfirmed' if expected_unconfirmed else 'completed') if control_ok else 'failed'
        if not control_ok:
            result['status'] = status = 'failed_control'
    (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    world.stage.GetRootLayer().Export(str(output/'scene.usda'))
    print('RESULT', json.dumps(result), flush=True)
    if status != 'passed':
        raise RuntimeError(status)
    return world


def capture_paused(app, world, output):
    """Capture the completed scene while physics is paused (no extra steps)."""
    import asyncio
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    world.pause()
    path = output/'viewport.png'
    capture = capture_viewport_to_file(get_active_viewport(), str(path))
    pending = asyncio.ensure_future(capture.wait_for_result())
    deadline = time.monotonic()+20.
    while not pending.done() and app.is_running() and time.monotonic() < deadline:
        app.update()
    import omni.kit.renderer_capture
    renderer = omni.kit.renderer_capture.acquire_renderer_capture_interface()
    while not path.is_file() and app.is_running() and time.monotonic() < deadline:
        renderer.wait_async_capture()
        app.update()
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError('VIEWPORT_CAPTURE_FAILED')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('motion', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=ROOT/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_shovel_physics.json')
    parser.add_argument('--case', default='baseline')
    parser.add_argument('--duration', type=float, help='Optional shorter simulated motion interval for diagnostics')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--closed-loop', action='store_true', help='Feed measured PhysX joints to production QP/executor')
    parser.add_argument('--load-feedback', action='store_true', help='Estimate grip deflection from calibrated virtual F/T; simulation only')
    parser.add_argument('--input-stop-at', type=float, help='Stop new operator packets at simulation time')
    parser.add_argument('--feedback-stop-at', type=float, help='Freeze controller feedback and its timestamp')
    parser.add_argument('--cancel-at', type=float, help='Request a latched shared-executor stop')
    parser.add_argument('--wrench-stop-at', type=float, help='Freeze virtual force sensor packets and timestamps')
    parser.add_argument('--keep-open', action='store_true', help='Keep the completed GUI scene paused for inspection')
    parser.add_argument('--record-video', action='store_true', help='Embed no overlays: record the live viewport at simulation-time intervals')
    parser.add_argument('--video-fps', type=int, default=20, help='Recording rate; must divide the physics step grid (default 20)')
    parser.add_argument('--grasp-test', choices=('hold','release'), help='Static contact diagnostic; release opens both jaws at t=1 s')
    parser.add_argument('--grasp-model', choices=('contact','proxy'), help='Explicit contact model or legacy spring comparison')
    # Kit portable arguments are supplied by an isolated SDK launcher.
    args, kit_args = parser.parse_known_args()
    if args.record_video and not args.gui:
        parser.error('--record-video requires --gui')
    if not 1 <= args.video_fps <= 60:
        parser.error('--video-fps must be between 1 and 60')
    for name in ('motion', 'output', 'config'):
        setattr(args, name, getattr(args, name).resolve())
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be finite and positive')
    if (args.load_feedback or any(getattr(args,n) is not None for n in ('input_stop_at','feedback_stop_at','cancel_at','wrench_stop_at'))) and not args.closed_loop:
        parser.error('feedback options and fault injection require --closed-loop')
    if any(v is not None and (not np.isfinite(v) or v < 0) for v in
           (args.input_stop_at,args.feedback_stop_at,args.cancel_at,args.wrench_stop_at)):
        parser.error('fault times must be finite and nonnegative')
    if sum(v is not None for v in (args.input_stop_at,args.feedback_stop_at,args.cancel_at,args.wrench_stop_at)) > 1:
        parser.error('inject one fault per run')
    if args.wrench_stop_at is not None and not args.load_feedback:
        parser.error('wrench faults require --load-feedback')
    cfg = json.loads(args.config.read_text())
    if args.grasp_model:
        cfg['grasp_model'] = args.grasp_model
    validate_grasp_config(cfg)
    if cfg.get('grasp_model') == 'contact' and args.load_feedback:
        parser.error('contact grasps cannot use the spring inverse load observer')
    if args.grasp_test and (args.closed_loop or cfg.get('grasp_model') != 'contact'):
        parser.error('--grasp-test requires contact grasps and reference mode')
    if args.grasp_test:
        # The ready blade is only 13 mm above the normal floor. Lower the
        # diagnostic floor so a release can demonstrate actual free fall.
        cfg['diagnostic_ground_drop_m'] = .3
    if cfg.get('feedback_velocity_source','native') not in ('native','position_difference'):
        raise ValueError('PHYSICS_VELOCITY_SOURCE')
    if cfg.get('simulation_only') is not True or args.case not in cfg['cases']:
        raise ValueError('SIMULATION_CASE_REQUIRED')
    override = cfg.pop('cases')[args.case]
    cfg.update({k:v for k,v in override.items() if k not in ('density_kg_m3', 'grip_stiffness_scale')})
    frame_steps = 1/(args.video_fps*cfg['physics_dt_s'])
    if args.record_video and not np.isclose(frame_steps, round(frame_steps)):
        parser.error('video frame interval must be an integer number of physics steps')
    if 'density_kg_m3' in override:
        cfg['material']['density_kg_m3'] = override['density_kg_m3']
    for name in ('translation', 'rotation'):
        cfg[f'grip_{name}_stiffness_'+('n_m' if name == 'translation' else 'nm_rad')] *= override.get('grip_stiffness_scale', 1.)
    motion = load_motion(args.motion)
    if cfg.get('grasp_model') == 'contact' and not motion['recipe'].get('check_grasp_contacts'):
        raise ValueError('CONTACT_GRASP_REQUIRES_CHECKED_GRIP_GEOMETRY')
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        raise FileExistsError(args.output)
    args.isaac_version = os.environ.get('ISAAC_VERSION', 'unknown')
    args.source_hashes = {name:digest(path) for name,path in (
        ('config',args.config),('runner',__file__),('math',ROOT/'tools/simulation/shovel_physics.py'))}
    snapshots = args.output/'sources';snapshots.mkdir()
    for path in (Path(__file__),ROOT/'tools/simulation/shovel_physics.py',args.config):
        (snapshots/path.name).write_bytes(path.read_bytes())
    if args.record_video:
        (snapshots/'shovel_video.py').write_bytes((ROOT/'tools/simulation/shovel_video.py').read_bytes())
    from isaacsim import SimulationApp
    sys.argv = [sys.argv[0]]+kit_args
    app = SimulationApp({'headless':not args.gui, 'fast_shutdown':True, 'width':1280, 'height':900})
    try:
        import isaacsim
        args.isaac_version = str(Path(isaacsim.__file__).resolve())
        # Record runtime's own version, never infer from the launcher's name.
        from isaacsim.core.version import get_version
        args.isaac_version = list(get_version())
        world = simulate(app, args, motion, cfg)
        if args.gui:
            capture_paused(app, world, args.output)
        if args.gui and args.keep_open:
            print('GUI_PAUSED_AFTER_COMPLETED_EXPERIMENT', flush=True)
            while app.is_running():
                app.update()
                time.sleep(.01)
    except BaseException as exc:
        traceback.print_exc()
        (args.output/'failure.json').write_text(json.dumps({'error':str(exc), 'isaac_version':args.isaac_version})+'\n')
        sys.stdout.flush(); sys.stderr.flush()
        app.close(exit_code=1)
        os._exit(1)
    sys.stdout.flush(); sys.stderr.flush()
    app.close()


if __name__ == '__main__':
    main()
