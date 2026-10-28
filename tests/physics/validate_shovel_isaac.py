#!/usr/bin/env python3
"""Check joint/link poses in Isaac Sim; optional viewport video, no ROS commands.

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
    from pxr import Gf, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.types import ArticulationAction

    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/capabilities/bindu_kinematics'))
    from tools.analysis.shovel_geometry import check_dump
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
    duration = motion.get('duration_s',motion['recipe']['keyframes'][-1]['t'])
    if duration < motion['recipe']['keyframes'][-1]['t']:
        raise ValueError('MOTION_TRUNCATED_INPUT')
    if 'tool_input_path' in motion and hashlib.sha256(Path(motion['tool_input_path']).read_bytes()).hexdigest()!=motion['tool_input_sha256']:
        raise ValueError('MOTION_INPUT_IDENTITY')
    if (not np.isfinite(times).all() or not np.isfinite([fps,duration]).all() or fps <= 0 or
            len(times) != round(duration*fps)+1 or not np.allclose(times,np.arange(len(times))/fps)):
        raise ValueError('INVALID_MOTION_TIMELINE')
    from tools.simulation.isaac_kinematic_scene import build_scene
    frames = (motion['left_frame'], motion['right_frame'])
    ground_z = float(motion['recipe']['ground']['z_in_base_m'])
    if not np.isfinite(ground_z):
        raise ValueError('NONFINITE_GROUND')
    world, body, flanges = build_scene(source, output, frames, ground_z)
    names = body.dof_names
    maximum = dict(joint_rad=0., flange_translation_m=0., flange_rotation_rad=0., right_grip_m=0.)
    left_to_tool = np.array(motion['left_to_tool']).reshape(4, 4)
    grip_offset = np.r_[motion['recipe'].get('right_grip_offset_m', motion['recipe']['grip_offset_m']), 1.]
    right_in_tool = np.r_[motion['recipe']['right_grip_in_tool_m'], 1.]
    config = json.loads(Path(motion['config_path']).read_text())
    axis = np.array(motion['recipe']['shaft_axis_in_tool'])
    right_mount = np.array(config['right_to_tool']).reshape(4,4)
    if axis.shape != (3,) or not np.isfinite(axis).all() or not np.isclose(np.linalg.norm(axis),1.) or not np.isfinite(right_mount).all():
        raise ValueError('NONFINITE_AXIAL_GEOMETRY')
    helper = np.eye(3)[np.argmin(abs(axis))]
    transverse = np.cross(helper,axis); transverse /= np.linalg.norm(transverse)
    grip_rotation = np.column_stack((transverse,np.cross(axis,transverse),axis))
    measured_rolls = []
    maximum.update(right_axis_rad=0., right_roll_readback_rad=0.)
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
        world.step(render=False)
        if index % 20 == 0:
            world.render()
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
        schedule = motion['recipe'].get('grasp_schedule', [{'t':0.,'mode':'rigid'}])
        mode = next(k['mode'] for k in reversed(schedule) if sample['t']+1e-9 >= k['t'])
        if sample.get('grasp_mode','rigid') != mode:
            raise ValueError('ISAAC_GRASP_SCHEDULE_MISMATCH')
        grip_error = (actual_tool[:3,:3] @ grip_rotation).T @ actual[frames[1]][:3,:3] @ right_mount[:3,:3] @ grip_rotation
        axis_error = float(np.arccos(np.clip(grip_error[2,2],-1.,1.)))
        roll = float(np.arctan2(grip_error[1,0],grip_error[0,0]))
        measured_rolls.append(roll)
        maximum['right_axis_rad'] = max(maximum['right_axis_rad'],axis_error)
        if 'right_grip_roll_rad' in sample:
            if not np.isfinite(sample['right_grip_roll_rad']):
                raise ValueError('NONFINITE_GRIP_ROLL')
            difference = roll-sample['right_grip_roll_rad']
            maximum['right_roll_readback_rad'] = max(maximum['right_roll_readback_rad'],
                abs(float(np.arctan2(np.sin(difference),np.cos(difference)))))
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
            maximum['flange_rotation_rad'] > .005 or maximum['right_grip_m'] > .002 or
            maximum['right_axis_rad'] > config['grasp_rotation_tolerance_rad'] or
            maximum['right_roll_readback_rad'] > .005):
        raise ValueError('ISAAC_POSE_PROPAGATION:'+json.dumps(maximum))
    gate = motion['recipe']['acceptance']
    if (min_clearance < gate['min_ground_clearance_m'] or ready_clearance > gate['max_ready_clearance_m'] or
            max_flange_motion < .01):
        raise ValueError('ISAAC_GROUND_OR_LINK_MOTION')
    result = {'dump': check_dump(measured_samples,motion['recipe']), 'ground_z_in_base_m': ground_z, 'min_pan_ground_clearance_m': min_clearance,
              'ready_pan_ground_clearance_m': ready_clearance, 'max_flange_motion_m': max_flange_motion,
              'status': 'passed', 'samples': len(motion['samples']), 'maximum_errors': maximum,
              'right_grip_roll_range_rad': [min(measured_rolls),max(measured_rolls)],
              'grasp_schedule': motion['recipe'].get('grasp_schedule',[]),
              'validator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'motion_sha256': hashlib.sha256(motion_path.read_bytes()).hexdigest(),
              'motion': str(motion_path), 'mode': 'zero-gravity joint-position kinematic replay'}
    (output/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return world, body, flanges, motion


def preview(app, context, output, record_video=False, playback_once=False):
    """Play checked samples at their configured trajectory timing."""
    import time
    import asyncio
    import shutil
    import subprocess
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pxr import Gf, UsdGeom, UsdLux
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.viewports import set_camera_view
    import omni.ui as ui
    from tools.analysis.shovel_geometry import bin_boxes
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
    # Annotation glyphs, not modeled jaws: the gold stripe follows the shaft;
    # the palm pointer follows the actual right flange and can turn relative
    # to that stripe. No invented gripper opening or physical slip animation.
    shaft('/World/Shovel/RollStripe',rear+axis*sum(ends)/2+tangent*.020,
          .003,ends[1]-ends[0],(1.,.85,.05))
    config=json.loads(Path(motion['config_path']).read_text())
    right_mount=np.array(config['right_to_tool']).reshape(4,4)
    pointer=UsdGeom.Xform.Define(stage,'/World/RightPalmMarker')
    pointer_transform=pointer.AddTransformOp()
    cube('/World/RightPalmMarker/Pointer',[.12,.014,.014],[.06,0.,0.],(.1,1.,.45))
    pointer_color=UsdGeom.Gprim(stage.GetPrimAtPath('/World/RightPalmMarker/Pointer')).GetDisplayColorAttr()
    right_grip=np.eye(4);right_grip[:3,:3]=right_mount[:3,:3] @ rotation
    right_grip[:3,3]=recipe.get('right_grip_offset_m', recipe['grip_offset_m'])
    playback={'t':0.,'playing':True,'seek':True}
    samples=motion['samples'];fps=recipe['sample_hz'];duration=samples[-1]['t']
    positions=np.asarray([[s['q'][n] for n in body.dof_names] for s in samples])
    flange=flanges[motion['left_frame']];mount=np.asarray(motion['left_to_tool']).reshape(4,4)
    def camera(eye): set_camera_view(eye=eye,target=[.65,-.4,.55])
    def restart():
        playback.update(t=0.,seek=True,playing=True);pause.text='Pause'
    def toggle():
        if playback['t'] >= duration:
            restart();return
        playback['playing']=not playback['playing'];pause.text='Pause' if playback['playing'] else 'Play'
    def seek(t): playback.update(t=t,seek=True)
    camera([3.3,-3.6,2.25])
    loose_change=next((k for k in recipe.get('grasp_schedule',[]) if k['mode']=='right_axial'),None)
    panel=ui.Window('Snowbot shovel playback',width=510,height=390)
    phase_text={'准备':'Ready','铲入':'Push','抬铲':'Lift','保持':'Hold','侧向扬起':'Transfer',
                '翻铲倒雪':'Tip into bin','倒雪保持':'Dump hold','扶正铲头':'Restore pan',
                '转回前方':'Return swing','转回侧方':'Return to side','下放回位':'Lower','完成':'Complete','转送箱口':'Move over bin'}
    with panel.frame:
        with ui.ScrollingFrame():
            with ui.VStack(spacing=5):
                ui.Label('Side grip / both hands locked / receiving bin' if loose_change is None else
                         'Side grip / right axial support / receiving bin',height=22)
                ui.Label('1x trajectory time | Kinematic replay',height=20)
                ui.Label('No snow / load / contact dynamics',height=20)
                ui.Label('Gold stripe: shaft | Pointer: actual right palm',height=20)
                ui.Label('Recorded pose + tight input; gripper not actuated',height=20)
                ui.Label('Plays once; Restart / seek resets the pose fixture',height=20)
                label=ui.Label('Ready',height=24)
                grasp_label=ui.Label('Both hands locked',height=24)
                roll_label=ui.Label('Right-hand / shaft relative roll: 0 deg',height=24)
                with ui.HStack(height=30,spacing=5):
                    pause=ui.Button('Pause',clicked_fn=toggle)
                    ui.Button('Restart',clicked_fn=restart)
                with ui.HStack(height=30,spacing=5):
                    ui.Button('Ready',clicked_fn=lambda:seek(0.))
                    ui.Button('Raised',clicked_fn=lambda:seek(recipe['acceptance']['lift_seconds']))
                    if 'dump' in recipe:
                        ui.Button('Dump',clicked_fn=lambda:seek(recipe['dump']['hold_seconds'][0]))
                if loose_change is not None:
                    with ui.HStack(height=30,spacing=5):
                        ui.Button('Loose grip',clicked_fn=lambda:seek(loose_change['t']))
                        ui.Button('Re-lock',clicked_fn=lambda:seek(recipe['grasp_schedule'][-1]['t']))
                with ui.HStack(height=30,spacing=5):
                    ui.Button('Oblique',clicked_fn=lambda:camera([3.3,-3.6,2.25]))
                    ui.Button('Front',clicked_fn=lambda:camera([4.,-.4,1.25]))
                    ui.Button('Top',clicked_fn=lambda:camera([.65,-.4,4.]))
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    snapshot_times = {'ready':.2, 'return':duration}
    if 'dump' in recipe:
        snapshot_times['dump'] = recipe['dump']['hold_seconds'][0]+.2
    if loose_change is not None and 'dump' in recipe:
        snapshot_times['loose']=(loose_change['t']+recipe['dump']['hold_seconds'][0])/2
        snapshot_times['relock']=recipe['grasp_schedule'][-1]['t']+.2
    captured = set()
    frames_dir = output/'video_frames'
    video_index = 0
    if record_video:
        if not shutil.which('ffmpeg'):
            raise RuntimeError('VIDEO_FFMPEG_UNAVAILABLE')
        frames_dir.mkdir()
        viewport = get_active_viewport()
        viewport.resolution = (1280,720)
        playback['playing'] = False
        for _ in range(20): world.render()
    previous=time.monotonic();last_index=-1
    print('SHOVEL_GUI_READY',flush=True)
    while app.is_running():
        if record_video:
            # Fixed timestamps and completed captures avoid dropped frames or
            # changing playback speed on a slow renderer.
            playback.update(t=video_index/fps, seek=True, playing=False)
        now=time.monotonic();dt=min(now-previous,.1);previous=now
        if playback['playing'] and not playback['seek']:
            playback['t']+=dt
            if playback['t']>duration:
                playback['t']=duration;playback['playing']=False;pause.text='Play'
                print('SHOVEL_GUI_COMPLETE; Restart resets the initial fixture',flush=True)
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
            right_position,right_wxyz=flanges[motion['right_frame']].get_world_pose()
            right_tf=np.eye(4);right_tf[:3,3]=right_position
            right_tf[:3,:3]=Rotation.from_quat(np.r_[right_wxyz[1:],right_wxyz[0]]).as_matrix()
            pointer_transform.Set(Gf.Matrix4d((right_tf @ right_grip).T.tolist()))
            loose=samples[index].get('grasp_mode')=='right_axial'
            pointer_color.Set([Gf.Vec3f(*((1.,.45,.05) if loose else (.1,1.,.45)))])
            grasp_label.text=('tight=false | RIGHT AXIAL SUPPORT' if loose else 'tight=true | BOTH HANDS LOCKED')
            roll_label.text=f'Right-hand / shaft relative roll: {np.rad2deg(samples[index].get("right_grip_roll_rad",0.)):.1f} deg'
            label.text=f'{samples[index]["t"]:.2f} / {duration:.2f} s | {phase_text.get(samples[index]["phase"],"Motion")}'
            last_index=index;playback['seek']=False
        world.render()
        for name,t in snapshot_times.items():
            if name not in captured and playback['t'] >= t:
                capture_viewport_to_file(get_active_viewport(),str(output/(name+'.png')))
                captured.add(name)
        if playback_once and not record_video and playback['t'] >= duration:
            world.pause()
            for _ in range(3):
                world.render()
            break
        if record_video:
            path = frames_dir/f'{video_index:06d}.png'
            capture = capture_viewport_to_file(viewport, str(path))
            pending = asyncio.ensure_future(capture.wait_for_result())
            deadline = time.monotonic()+20.
            while not pending.done() and app.is_running() and time.monotonic()<deadline:
                app.update()
            if not pending.done():
                raise RuntimeError('VIDEO_CAPTURE_TIMEOUT')
            if 'LdrColor' not in pending.result():
                raise RuntimeError('VIDEO_CAPTURE_NO_COLOR')
            # Viewport completion schedules the GPU readback; wait for the
            # renderer's asynchronous file write before advancing the pose.
            import omni.kit.renderer_capture
            renderer_capture=omni.kit.renderer_capture.acquire_renderer_capture_interface()
            for _ in range(3):
                renderer_capture.wait_async_capture()
                app.update()
            while not path.is_file() and time.monotonic()<deadline:
                app.update()
                time.sleep(.002)
            if not path.is_file() or not path.stat().st_size:
                raise RuntimeError('VIDEO_CAPTURE_EMPTY')
            video_index += 1
            if video_index % 100 == 0:
                print(f'SHOVEL_VIDEO_FRAMES {video_index}/{len(samples)}',flush=True)
            if video_index == len(samples): break
        else:
            time.sleep(.005)
    if record_video:
        if video_index != len(samples):
            raise RuntimeError('VIDEO_INTERRUPTED')
        video=output/'snowbot_shovel_pose_input.mp4'
        with (output/'ffmpeg.log').open('w') as log:
            subprocess.run(['ffmpeg','-nostdin','-v','warning','-framerate',str(fps),
                '-i','video_frames/%06d.png','-c:v','libx264','-threads','2',
                '-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',video.name],
                cwd=output,stdout=log,stderr=subprocess.STDOUT,check=True)
        metadata={'status':'passed','frames':video_index,'fps':fps,'duration_s':video_index/fps,
                  'trajectory_duration_s':duration,'playback_speed':1.,'resolution':[1280,720],
                  'motion_sha256':json.loads((output/'validation.json').read_text())['motion_sha256'],
                  'video_path':str(video),'video_sha256':hashlib.sha256(video.read_bytes()).hexdigest(),
                  'kinematic_replay':True,'snow_contact_dynamics':False,'text_overlay':False}
        (output/'video.json').write_text(json.dumps(metadata,indent=2)+'\n')
        print('SHOVEL_VIDEO_COMPLETE '+str(video),flush=True)


def task_feedback_worker(settings_path):
    """System-Python QP/native reference worker; no ROS node or device IO."""
    import numpy as np
    settings=json.loads(Path(settings_path).read_text());root=Path(settings['root'])
    sys.path[:0]=[str(root)]+[str(root/'src'/p) for p in (
        'shared/bindu_contracts','capabilities/bindu_kinematics','control/bindu_execution')]
    import bindu_kinematics,bindu_execution
    bindu_kinematics.__path__.insert(0,str(Path(settings['candidate'])/'src/capabilities/bindu_kinematics/bindu_kinematics'))
    bindu_execution.__path__.insert(0,settings['execution_native'])
    from bindu_kinematics.bimanual import BimanualQP,DeferredSolve
    from bindu_contracts.profile import Profile
    from bindu_execution.interpolation import State,PreparedSegment
    data=json.loads(Path(settings['input']).read_text());profile=Profile.load(settings['profile'])
    solver=None
    for line in sys.stdin:
        request=json.loads(line)
        try:
            if request['op']=='capture':
                if solver is not None:solver.close()
                cfg=dict(data['tool'],urdf=settings['urdf'],library=settings['library'],
                    tracking_feedback=request['mode'],reference_kinematics=True,
                    demo={**data['tool']['demo'],'display_urdf':settings['display_urdf']})
                cfg.update(settings.get('qp_overrides',{}))
                if request.get('waist_goal_offset_rad') is not None:
                    shared=data['profile']['groups']['waist']
                    cfg.update(shared_target_weight=.3,shared_targets={n:request['q'][n]+
                        (request['waist_goal_offset_rad'] if i==0 else 0.) for i,n in enumerate(shared)})
                solver=BimanualQP(cfg,profile);solver.capture(request['q'])
                reference=np.array([request['q'][n] for n in solver.names]);last=0.;segment=None
                response=dict(names=solver.names,tracking_point=solver.tracking_point.tolist())
            elif request['op']=='sample':
                state=State.rest(reference) if segment is None else segment.sample(request['t'])
                response=dict(q=list(state.q),v=list(state.v))
            else:
                now=request['t'];state=State.rest(reference) if segment is None else segment.sample(now)
                target=np.array(request['target']).reshape(4,4)
                measured=solver._q(request['q']);actual=np.empty((4,4));relative=np.empty((4,4))
                solver.lib.bq_tool_pose(solver.handle,solver._ptr(measured),solver.nq,solver._ptr(actual))
                solver.lib.bq_relative_pose(solver.handle,solver._ptr(measured),solver.nq,solver._ptr(relative))
                deferred=False
                try:
                    positions,metrics=solver.solve(request['q'],target,reference=reference,
                        dt=min(now-last,solver.max_interval),target_stamp=now)
                    # Continue from the native reference's q/v/a, never measured q.
                    segment=PreparedSegment.target(profile,solver.names,state,positions,now)
                    reference=np.array(positions);last=now
                except DeferredSolve:
                    deferred=True;metrics=(0.,0.,0.)
                response=dict(q=list(state.q),v=list(state.v),goal=reference.tolist(),
                    actual=actual.ravel().tolist(),relative=relative.ravel().tolist(),
                    scale=metrics[2],deferred=deferred)
        except Exception as exc:
            response=dict(error=type(exc).__name__+':'+str(exc))
        print(json.dumps(response),flush=True)


def compare_task_feedback(settings_path, output):
    """Closed-loop QP -> native continuation -> fitted PhysX -> measured QP.

    Unloaded fixture, reconstructed log targets. Does not exercise vendor
    recipe-3 prediction, DDS, grasp contact forces or physical stopping.
    """
    import subprocess
    import select
    import numpy as np
    from scipy.spatial.transform import Rotation,Slerp
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import UsdPhysics,UsdGeom
    settings=json.loads(Path(settings_path).read_text());root=Path(settings['root'])
    sys.path.insert(0,str(root))
    from tools.simulation.run_shovel_physics import build_scene
    data=json.loads(Path(settings['input']).read_text())
    fitted=json.loads(Path(settings['drives']).read_text())
    cfg=json.loads(Path(settings['physics']).read_text());cfg.update(physics_dt_s=.004,
        joint_drive_overrides=fitted['joint_drive_overrides'])
    matrix=json.loads(Path(settings['matrix']).read_text())
    motion=dict(display_urdf=settings['display_urdf'],recipe=matrix['recipe'],
        left_frame=data['tool']['left_frame'],right_frame=data['tool']['right_frame'])
    world,body,view,contacts,source=build_scene(motion,cfg,output)
    prim=world.stage.GetPrimAtPath('/World/Shovel')
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
    view.set_world_poses(np.array([[0.,0.,-10.]]),np.array([[1.,0.,0.,0.]]),indices=[2])
    UsdGeom.Imageable(prim).MakeInvisible()
    names=list(body.dof_names);lookup={n:i for i,n in enumerate(names)};dt=cfg['physics_dt_s']
    env=os.environ.copy();env['PYTHONPATH']='/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages'
    env['LD_LIBRARY_PATH']='/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu'
    env['AMENT_PREFIX_PATH']=':'.join(str(p) for p in (root/'install').iterdir() if (p/'share/ament_index').exists())+':/opt/ros/jazzy'
    results=[]
    for job in settings['jobs']:
        case=next(c for c in data['cases'] if c['epoch']==job['epoch'])
        initial=np.array([case['initial'][n] for n in names]);rows=[];failure=None
        body.set_joint_positions(initial);body.set_joint_velocities(np.zeros(len(names)))
        body.apply_action(ArticulationAction(joint_positions=initial,joint_velocities=np.zeros(len(names))))
        times=np.array([r['t'] for r in case['targets']]);poses=np.array([r['target'] for r in case['targets']]).reshape(-1,4,4)
        rotations=Slerp(times,Rotation.from_matrix(poses[:,:3,:3]))
        stderr=(output/(job['label']+'.stderr.log')).open('w')
        worker=subprocess.Popen(['/usr/bin/python3','-u',str(Path(__file__).resolve()),
            '--task-feedback-worker',str(settings_path)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stderr,text=True,env=env)
        def ask(request):
            worker.stdin.write(json.dumps(request)+'\n');worker.stdin.flush()
            if not select.select([worker.stdout],[],[],30.)[0]:raise TimeoutError('TASK_FEEDBACK_WORKER')
            response=json.loads(worker.stdout.readline())
            if 'error' in response:raise RuntimeError(response['error'])
            return response
        native_start=SimulationManager.get_num_physics_steps();started=__import__('time').monotonic()
        try:
            info=ask(dict(op='capture',mode=job['mode'],q=case['initial'],
                waist_goal_offset_rad=job.get('waist_goal_offset_rad')))
            active=[lookup[n] for n in info['names']];qref=initial.copy();vref=np.zeros(len(names))
            history_q=[];history_v=[];response=None
            delays=dict(fitted.get('joint_response_delay_s',{}))
            delays.update({n:job.get('waist_delay_s',0.) for n in names if n.startswith('body_joint_')})
            duration=job.get('duration_s',case['duration_s']+1.)
            for tick in range(int(duration/dt)+1):
                t=round(tick*dt,9);q=np.asarray(body.get_joint_positions()).copy()
                qd=np.asarray(body.get_joint_velocities()).copy()
                if tick and tick%5==0:
                    at=float(np.clip(t,times[0],times[-1]));target=np.eye(4)
                    target[:3,:3]=rotations(at).as_matrix()
                    target[:3,3]=[np.interp(at,times,poses[:,j,3]) for j in range(3)]
                    if job.get('input_kind')=='probe':
                        # Separate input sensitivity from the recorded low tool
                        # path: lift 80 mm, then a small smooth horizontal cycle.
                        home=np.array(case['capture']['tool_pose']);point=np.array(info['tracking_point'])
                        u=float(np.clip(t,0.,1.));lift=.08*(10*u**3-15*u**4+6*u**5)
                        phase=float(np.clip((t-1.)/4.,0.,1.));wave=np.sin(2*np.pi*phase)*np.sin(np.pi*phase)**2
                        target[:3,:3]=Rotation.from_rotvec([0.,0.,.08*wave]).as_matrix()@home[:3,:3]
                        control=home[:3,3]+home[:3,:3]@point+np.array([.03*wave,0.,lift])
                        target[:3,3]=control-target[:3,:3]@point
                    response=ask(dict(op='solve',t=t,q=dict(zip(names,q.tolist())),target=target.ravel().tolist()))
                    qref[active]=response['q'];vref[active]=response['v']
                    actual=np.array(response['actual']).reshape(4,4)
                    point=np.r_[info['tracking_point'],1.]
                    rows.append(dict(t=t,q=q[active].tolist(),goal=response['goal'],reference=qref[active].tolist(),
                        position_error_m=float(np.linalg.norm((actual@point-target@point)[:3])),
                        rotation_error_rad=float(Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude()),
                        relative=response['relative'],scale=response['scale'],deferred=response['deferred']))
                if not tick or tick%5:
                    sample=ask(dict(op='sample',t=t));qref[active]=sample['q'];vref[active]=sample['v']
                history_q.append(qref.copy());history_v.append(vref.copy())
                target=qref.copy();velocity=np.zeros(len(names))
                for j,n in enumerate(names):
                    at=max(0.,tick-delays.get(n,0.)/dt);before=int(at);fraction=at-before
                    after=min(before+1,tick)
                    target[j]=(1-fraction)*history_q[before][j]+fraction*history_q[after][j]
                    if fitted.get('velocity_feedforward',False):
                        velocity[j]=(1-fraction)*history_v[before][j]+fraction*history_v[after][j]
                body.apply_action(ArticulationAction(joint_positions=target,joint_velocities=velocity))
                world.step(render=False)
        except Exception as exc:failure=str(exc)
        finally:
            worker.stdin.close();worker.wait(timeout=10);stderr.close()
        position=np.array([r['position_error_m'] for r in rows]);rotation=np.array([r['rotation_error_rad'] for r in rows])
        result=dict(job=job,error=failure,samples=len(rows),position_rms_m=float(np.sqrt(np.mean(position**2))) if len(rows) else None,
            rotation_rms_rad=float(np.sqrt(np.mean(rotation**2))) if len(rows) else None,
            physics_steps=SimulationManager.get_num_physics_steps()-native_start,wall_s=__import__('time').monotonic()-started)
        (output/(job['label']+'.json')).write_text(json.dumps(dict(result=result,rows=rows))+'\n')
        results.append(result);print('TASK_FEEDBACK_RESULT',json.dumps(result),flush=True)
        (output/'comparison.json').write_text(json.dumps(dict(results=results,source=source,
            fitted_parameters_sha256=hashlib.sha256(Path(settings['drives']).read_bytes()).hexdigest(),
            simulation_only=True,hardware_commands=False,waist_response_identified=False,
            execution='PreparedSegment.target from continuous q/v/a; vendor recipe-3 not exercised',
            fixture='unloaded, tool parked; targets reconstructed from log snapshots'),indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    # In comparison mode there is no motion positional: otherwise argparse
    # consumes an unknown Kit option's value (e.g. --portable-root) as motion.
    comparison=any(arg.split('=',1)[0]=='--task-feedback-comparison' for arg in sys.argv[1:])
    if comparison:parser.set_defaults(motion=None)
    else:parser.add_argument('motion',type=Path)
    parser.add_argument('--task-feedback-comparison',type=Path,
                        help='Closed-loop QP comparison using existing fitted PhysX response settings')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gui', action='store_true', help='After verification, keep an interactive 1x playback window open')
    parser.add_argument('--video', action='store_true', help='Capture every checked sample at 1x timing to MP4, then exit')
    parser.add_argument('--playback-once', action='store_true', help='Close the GUI after one checked playback')
    args, kit_args = parser.parse_known_args()
    if args.playback_once and not args.gui:
        parser.error('--playback-once requires --gui')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise FileExistsError(output)
    from isaacsim import SimulationApp
    sys.argv = [sys.argv[0]]+kit_args
    app = SimulationApp({'headless': not args.gui, 'fast_shutdown': True, 'window_width':1600, 'window_height':1000})
    try:
        if args.task_feedback_comparison:
            compare_task_feedback(args.task_feedback_comparison.resolve(),output)
        else:
            if args.motion is None:parser.error('motion is required without --task-feedback-comparison')
            context = verify(args.motion.resolve(), output)
            if args.gui or args.video:
                preview(app, context, output, record_video=args.video, playback_once=args.playback_once)
    except BaseException:
        # Kit's atexit fast shutdown replaces uncaught exceptions with exit 0.
        # Preserve the diagnostic and failure status before that handler runs.
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    app.close()


if __name__ == '__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--task-feedback-worker':
        task_feedback_worker(sys.argv[2])
    else:main()
