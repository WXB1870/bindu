#!/usr/bin/env python3
"""Check planned joint and flange poses in Isaac Sim; no video or ROS commands.

Run with Isaac Sim's python.sh in an environment without ROS PYTHONPATH or
LD_LIBRARY_PATH. Joint positions are set with zero gravity: this validates the
URDF import and pose propagation, not a position servo, payload or contact.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import traceback
import xml.etree.ElementTree as ET


def verify(motion_path, output):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pxr import Gf, PhysicsSchemaTools, Usd, UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.types import ArticulationAction

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/capabilities/bindu_kinematics'))
    from shovel_geometry import check_dump
    motion = json.loads(motion_path.read_text())
    source = Path(motion['display_urdf'])
    if motion['schema'] != 2 or hashlib.sha256(source.read_bytes()).hexdigest() != motion['display_sha256']:
        raise ValueError('MOTION_MODEL_IDENTITY')
    if not motion.get('source_sha256'):
        raise ValueError('MISSING_SOURCE_FINGERPRINTS')
    for path, digest in motion['source_sha256'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise ValueError('MOTION_SOURCE_IDENTITY:'+path)
    if not motion['samples']:
        raise ValueError('EMPTY_MOTION')
    times = np.array([sample['t'] for sample in motion['samples']])
    fps = motion['recipe']['sample_hz']
    duration = motion['recipe']['keyframes'][-1]['t']
    if (not np.isfinite(times).all() or not np.isfinite([fps,duration]).all() or fps <= 0 or
            len(times) != round(duration*fps)+1 or not np.allclose(times,np.arange(len(times))/fps)):
        raise ValueError('INVALID_MOTION_TIMELINE')
    tree = ET.parse(source)
    package = source.parent.parent
    for mesh in tree.findall('.//mesh'):
        uri = mesh.get('filename')
        prefix = f'package://{package.name}/'
        if uri.startswith(prefix):
            mesh.set('filename', str(package/uri.removeprefix(prefix)))
        elif not Path(uri).is_absolute():
            raise ValueError('UNRESOLVED_ISAAC_MESH:'+uri)
    roots = {link.get('name') for link in tree.findall('link')} - {
        joint.find('child').get('link') for joint in tree.findall('joint')}
    if len(roots) != 1:
        raise ValueError('URDF_REQUIRES_ONE_ROOT')
    absolute = output/'model.urdf'
    tree.write(absolute, encoding='utf-8', xml_declaration=True)
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    usd = URDFImporter(URDFImporterConfig(urdf_path=str(absolute), usd_path=str(output/'usd'),
                                        allow_self_collision=False)).import_urdf()
    world = World(stage_units_in_meters=1., physics_dt=.01, rendering_dt=.05)
    model = world.stage.DefinePrim('/World/Robot', 'Xform')
    model.GetReferences().AddReference(str(usd))
    links = {prim.GetName(): prim for prim in Usd.PrimRange(model)
             if prim.HasAPI(UsdPhysics.RigidBodyAPI)}
    root = roots.pop()
    frames = (motion['left_frame'], motion['right_frame'])
    if not {root, *frames}.issubset(links):
        raise ValueError('ISAAC_LINK_LAYOUT')
    body = world.scene.add(SingleArticulation(str(links[root].GetPath()), name='robot'))
    flanges = {name: world.scene.add(SingleRigidPrim(str(links[name].GetPath()), name=name))
               for name in frames}
    ground_z = float(motion['recipe']['ground']['z_in_base_m'])
    if not np.isfinite(ground_z):
        raise ValueError('NONFINITE_GROUND')
    # Procedural plane: no network/default-environment asset dependency.
    PhysicsSchemaTools.addGroundPlane(world.stage, '/World/Ground', 'Z', 5.,
                                      Gf.Vec3f(0.,0.,ground_z), Gf.Vec3f(.5,.5,.5))
    world.get_physics_context().set_gravity(0.)
    world.reset()
    names = body.dof_names
    maximum = dict(joint_rad=0., flange_translation_m=0., flange_rotation_rad=0., right_grip_m=0.)
    left_to_tool = np.array(motion['left_to_tool']).reshape(4, 4)
    grip_offset = np.r_[motion['recipe']['grip_offset_m'], 1.]
    right_in_tool = np.r_[motion['recipe']['right_grip_in_tool_m'], 1.]
    if not all(np.isfinite(value).all() for value in (left_to_tool, grip_offset, right_in_tool)):
        raise ValueError('NONFINITE_TOOL_GEOMETRY')
    pan = np.array(motion['recipe']['pan_center_m'])
    half = np.array(motion['recipe']['pan_size_m'])/2
    if not np.isfinite(np.r_[pan,half]).all() or min(half) <= 0:
        raise ValueError('INVALID_PAN_GEOMETRY')
    corners = np.array([np.r_[pan+half*np.array([x,y,z]),1.] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    min_clearance, ready_clearance = float('inf'), None
    flange_start, max_flange_motion = None, 0.
    measured_samples = []
    for index, sample in enumerate(motion['samples']):
        if set(sample['q']) != set(names):
            raise ValueError('ISAAC_JOINT_LAYOUT')
        q = np.array([sample['q'][name] for name in names])
        if not np.isfinite(q).all():
            raise ValueError('NONFINITE_MOTION')
        body.set_joint_positions(q)
        body.set_joint_velocities(np.zeros_like(q))
        body.apply_action(ArticulationAction(joint_positions=q))
        # A joint-buffer readback alone does not propagate poses to links.
        world.step(render=index % 20 == 0)
        measured_q = body.get_joint_positions()
        if not np.isfinite(measured_q).all():
            raise ValueError('NONFINITE_ISAAC_JOINTS')
        maximum['joint_rad'] = max(maximum['joint_rad'], float(np.max(np.abs(measured_q-q))))
        actual = {}
        for name, link in flanges.items():
            position, wxyz = link.get_world_pose()
            expected = np.array(sample['flanges'][name])
            if (not np.isfinite(position).all() or not np.isfinite(wxyz).all() or
                    not np.isfinite(expected).all()):
                raise ValueError('NONFINITE_ISAAC_POSE')
            pose = np.eye(4)
            pose[:3, 3] = position
            pose[:3, :3] = Rotation.from_quat(np.r_[wxyz[1:], wxyz[0]]).as_matrix()
            actual[name] = pose
            maximum['flange_translation_m'] = max(maximum['flange_translation_m'],
                                                  float(np.linalg.norm(position-expected[:3, 3])))
            angle = Rotation.from_matrix(expected[:3, :3].T @ pose[:3, :3]).magnitude()
            maximum['flange_rotation_rad'] = max(maximum['flange_rotation_rad'], float(angle))
        actual_tool = actual[frames[0]] @ left_to_tool
        measured_samples.append({'t':sample['t'],'tool':actual_tool.tolist()})
        clearance = float((corners @ actual_tool.T)[:,2].min())-ground_z
        min_clearance = min(min_clearance, clearance)
        if index == 0:
            ready_clearance = clearance
            flange_start = actual[frames[0]][:3,3].copy()
        max_flange_motion = max(max_flange_motion, float(np.linalg.norm(actual[frames[0]][:3,3]-flange_start)))
        error = (actual[frames[1]] @ grip_offset - actual_tool @ right_in_tool)[:3]
        maximum['right_grip_m'] = max(maximum['right_grip_m'], float(np.linalg.norm(error)))
        if index % 100 == 0:
            print('POSE_CHECK', index, sample['t'], maximum, flush=True)
    if (maximum['joint_rad'] > .002 or maximum['flange_translation_m'] > .002 or
            maximum['flange_rotation_rad'] > .005 or maximum['right_grip_m'] > .002):
        raise ValueError('ISAAC_POSE_PROPAGATION:'+json.dumps(maximum))
    gate = motion['recipe']['acceptance']
    if (min_clearance < gate['min_ground_clearance_m'] or ready_clearance > gate['max_ready_clearance_m'] or
            max_flange_motion < .01):
        raise ValueError('ISAAC_GROUND_OR_LINK_MOTION')
    result = {'dump': check_dump(measured_samples,motion['recipe']), 'ground_z_in_base_m': ground_z, 'min_pan_ground_clearance_m': min_clearance,
              'ready_pan_ground_clearance_m': ready_clearance, 'max_flange_motion_m': max_flange_motion,
              'status': 'passed', 'samples': len(motion['samples']), 'maximum_errors': maximum,
              'motion_sha256': hashlib.sha256(motion_path.read_bytes()).hexdigest(),
              'motion': str(motion_path), 'mode': 'zero-gravity joint-position kinematic replay'}
    (output/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return world, body, flanges, motion


def preview(app, context, output):
    """Play checked samples at their configured trajectory timing."""
    import time
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pxr import Gf, UsdGeom, UsdLux
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.viewports import set_camera_view
    import omni.ui as ui
    from shovel_geometry import bin_boxes
    world,body,flanges,motion = context
    stage,recipe = world.stage,motion['recipe']
    UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1100.)
    light = UsdLux.DistantLight.Define(stage,'/World/Sun')
    light.CreateIntensityAttr(1500.)
    UsdGeom.Xformable(light).AddRotateXYZOp().Set(Gf.Vec3f(-30.,-35.,0.))
    def cube(path,size,center,color):
        obj=UsdGeom.Cube.Define(stage,path)
        obj.CreateSizeAttr(1.)
        obj.AddTranslateOp().Set(Gf.Vec3d(*center))
        obj.AddScaleOp().Set(Gf.Vec3f(*size))
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    for name,size,center in bin_boxes(recipe):
        cube('/World/ReceivingBin/'+name,size,center,(.25,.5,.3))
    tool=UsdGeom.Xform.Define(stage,'/World/Shovel')
    transform=tool.AddTransformOp()
    axis=np.asarray(recipe['shaft_axis_in_tool'])
    rear=np.asarray(recipe['left_grip_in_tool_m'])
    ends=recipe['shaft_from_left_grip_m']
    reference=np.array([1.,0.,0.]) if abs(axis[0])<.9 else np.array([0.,1.,0.])
    tangent=np.cross(reference,axis);tangent/=np.linalg.norm(tangent)
    rotation=np.column_stack((tangent,np.cross(axis,tangent),axis))
    def shaft(path,center,radius,length,color):
        obj=UsdGeom.Cylinder.Define(stage,path)
        obj.CreateAxisAttr('Z');obj.CreateRadiusAttr(radius);obj.CreateHeightAttr(length)
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        tf=np.eye(4);tf[:3,:3]=rotation;tf[:3,3]=center
        obj.AddTransformOp().Set(Gf.Matrix4d(tf.T.tolist()))
    shaft('/World/Shovel/Shaft',rear+axis*sum(ends)/2,recipe['shaft_radius_m'],ends[1]-ends[0],(.55,.32,.12))
    shaft('/World/Shovel/RearGrip',rear,.019,.065,(.1,.85,.35))
    shaft('/World/Shovel/FrontGrip',np.asarray(recipe['right_grip_in_tool_m']),.019,.065,(1.,.45,.05))
    # Visual tray stays inside the conservative solid envelope checked offline.
    size=np.asarray(recipe['pan_size_m']);center=np.asarray(recipe['pan_center_m']);wall=.012
    cube('/World/Shovel/Tray',[size[0],size[1],wall],center+[0,0,-size[2]/2+wall/2],(.12,.46,.85))
    for sign in (-1,1):
        cube('/World/Shovel/Side'+str(sign).replace('-','N'),[size[0],wall,size[2]],
             center+[0,sign*(size[1]-wall)/2,0],(.12,.46,.85))
    cube('/World/Shovel/Back',[wall,size[1],size[2]],center+[-(size[0]-wall)/2,0,0],(.12,.46,.85))
    playback={'t':0.,'playing':True,'seek':True}
    samples=motion['samples'];fps=recipe['sample_hz'];duration=samples[-1]['t']
    positions=np.asarray([[s['q'][n] for n in body.dof_names] for s in samples])
    flange=flanges[motion['left_frame']];mount=np.asarray(motion['left_to_tool']).reshape(4,4)
    def camera(eye): set_camera_view(eye=eye,target=[.65,-.4,.55])
    def toggle():
        playback['playing']=not playback['playing'];pause.text='Pause' if playback['playing'] else 'Play'
    def seek(t): playback.update(t=t,seek=True)
    camera([3.3,-3.6,2.25])
    panel=ui.Window('Snowbot shovel playback',width=440,height=310)
    phase_text={'准备':'Ready','铲入':'Push','抬铲':'Lift','保持':'Hold','侧向扬起':'Transfer',
                '翻铲倒雪':'Tip into bin','倒雪保持':'Dump hold','扶正铲头':'Restore pan',
                '转回前方':'Return swing','转回侧方':'Return to side','下放回位':'Lower','完成':'Complete','转送箱口':'Move over bin'}
    with panel.frame:
        with ui.ScrollingFrame():
            with ui.VStack(spacing=5):
                ui.Label('Side grip / accelerated motion / receiving bin',height=22)
                ui.Label('1x trajectory time | Kinematic replay',height=20)
                ui.Label('No snow / load / contact dynamics',height=20)
                label=ui.Label('Ready',height=24)
                with ui.HStack(height=30,spacing=5):
                    pause=ui.Button('Pause',clicked_fn=toggle)
                    ui.Button('Restart',clicked_fn=lambda:seek(0.))
                with ui.HStack(height=30,spacing=5):
                    ui.Button('Ready',clicked_fn=lambda:seek(0.))
                    ui.Button('Raised',clicked_fn=lambda:seek(recipe['acceptance']['lift_seconds']))
                    if 'dump' in recipe:
                        ui.Button('Dump',clicked_fn=lambda:seek(recipe['dump']['hold_seconds'][0]))
                with ui.HStack(height=30,spacing=5):
                    ui.Button('Oblique',clicked_fn=lambda:camera([3.3,-3.6,2.25]))
                    ui.Button('Front',clicked_fn=lambda:camera([4.,-.4,1.25]))
                    ui.Button('Top',clicked_fn=lambda:camera([.65,-.4,4.]))
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    snapshot_times = {'ready':.2}
    if 'dump' in recipe:
        snapshot_times['dump'] = recipe['dump']['hold_seconds'][0]+.2
    captured = set()
    previous=time.monotonic();last_index=-1;cycles=0
    print('SHOVEL_GUI_READY',flush=True)
    while app.is_running():
        now=time.monotonic();dt=min(now-previous,.1);previous=now
        if playback['playing'] and not playback['seek']:
            playback['t']+=dt
            if playback['t']>duration:
                playback['t']%=duration;cycles+=1
                print('SHOVEL_GUI_LOOP',cycles,flush=True)
        index=min(round(playback['t']*fps),len(samples)-1)
        if index!=last_index or playback['seek']:
            if not world.is_playing(): world.play()
            q=positions[index]
            body.set_joint_positions(q);body.set_joint_velocities(np.zeros_like(q))
            body.apply_action(ArticulationAction(joint_positions=q))
            world.step(render=False)
            position,wxyz=flange.get_world_pose()
            tf=np.eye(4);tf[:3,3]=position
            tf[:3,:3]=Rotation.from_quat(np.r_[wxyz[1:],wxyz[0]]).as_matrix()
            transform.Set(Gf.Matrix4d((tf@mount).T.tolist()))
            label.text=f'{samples[index]["t"]:.2f} / {duration:.2f} s | {phase_text.get(samples[index]["phase"],"Motion")}'
            last_index=index;playback['seek']=False
        world.render()
        for name,t in snapshot_times.items():
            if name not in captured and playback['t'] >= t:
                capture_viewport_to_file(get_active_viewport(),str(output/(name+'.png')))
                captured.add(name)
        time.sleep(.005)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('motion', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gui', action='store_true', help='After verification, keep an interactive 1x playback window open')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise FileExistsError(output)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': not args.gui, 'fast_shutdown': True, 'window_width':1600, 'window_height':1000})
    try:
        context = verify(args.motion.resolve(), output)
        if args.gui:
            preview(app, context, output)
    except BaseException:
        # Kit's atexit fast shutdown replaces uncaught exceptions with exit 0.
        # Preserve the diagnostic and failure status before that handler runs.
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    app.close()


if __name__ == '__main__':
    main()
