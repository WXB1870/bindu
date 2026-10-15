#!/usr/bin/env python3
"""Generate mapped tool input and check its causal execution with the shared QP/executor.

The recipe owns robot names, ready posture, grasp and motion. This tool does not
publish commands. Positions are metres/radians in the fixed URDF base frame.
Rotation is world yaw, local pitch, then roll about the shaft. The rear grip
is the pivot. Only the input generator knows the recipe; execution has no lookahead.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from bindu_contracts.profile import Profile
from bindu_kinematics.bimanual import BimanualQP
from shovel_geometry import bin_boxes, check_dump
from bindu_kinematics.tool_geometry import collision_check, tracking_acceptance, coupling_metrics

ROOT = Path(__file__).resolve().parents[1]


def validate_recipe(config):
    def finite(value):
        if isinstance(value, dict):
            return all(finite(v) for v in value.values())
        if isinstance(value, list):
            return all(finite(v) for v in value)
        return not isinstance(value, (int, float)) or np.isfinite(value)
    if not finite(config):
        raise ValueError('NONFINITE_SHOVEL_CONFIG')
    if config.get('simulation_only') is not True:
        raise ValueError('SHOVEL_REQUIRES_SIMULATION')
    recipe, design = config['demo'], config['demo']['selected_design']
    tracking_acceptance(recipe,0.,0.)
    if ('shared_targets' in config or
            any('shared_targets_rad' in key for key in recipe['keyframes'])):
        raise ValueError('SHOVEL_REQUIRES_TOOL_POSE_INPUT')
    axis = np.asarray(recipe['shaft_axis_in_tool'])
    ends = recipe['shaft_from_left_grip_m']
    grips = [(np.linalg.inv(BimanualQP._transform(config[side+'_to_tool'])) @
              np.r_[recipe.get(side+'_grip_offset_m', recipe['grip_offset_m']), 1.])[:3]
             for side in ('left', 'right')]
    if (not np.isclose(ends[1]-ends[0], design['shaft_length_m']) or
            not np.isclose(np.linalg.norm(axis), 1.) or
            not np.isclose(np.rad2deg(np.arctan2(axis[2], -axis[0])), design['ready_elevation_deg']) or
            not np.allclose(grips[1]-grips[0], -axis*design['grip_spacing_m'], atol=1e-6) or
            not np.allclose(grips[0]+axis*ends[0], np.zeros(3), atol=1e-6) or
            not np.allclose(grips[0], recipe['pivot_in_tool_m'], atol=1e-6)):
        raise ValueError('SHOVEL_DESIGN_MISMATCH')
    for value in recipe['shaft_contacts'].values():
        expected = -design['grip_spacing_m'] if value['grip']=='right' else 0.
        if value['grip'] not in ('left','right') or not np.isclose(value['center_m'], expected):
            raise ValueError('SHOVEL_CONTACT_MISMATCH')
    bin_boxes(recipe)
    keys, gate = recipe['keyframes'], recipe['acceptance']
    schedule = recipe.get('grasp_schedule', [{'t': 0., 'mode': 'rigid'}])
    if (not schedule or schedule[0] != {'t': 0., 'mode': 'rigid'} or
            any(set(k) != {'t','mode'} or k['mode'] not in ('rigid','right_axial') for k in schedule) or
            any(b['t'] <= a['t'] or b['mode'] == a['mode'] for a,b in zip(schedule,schedule[1:])) or
            any(k['mode'] == 'right_axial' for k in schedule) and 'right_axial_grasp' not in config):
        raise ValueError('INVALID_SHOVEL_GRASP_SCHEDULE')
    if 'right_axial_grasp' in config and (
            not np.allclose(config['right_axial_grasp']['axis_in_tool'], axis) or
            not np.allclose(config['right_axial_grasp']['grip_in_right_flange_m'],
                            recipe.get('right_grip_offset_m', recipe['grip_offset_m']))):
        raise ValueError('SHOVEL_AXIAL_GEOMETRY_MISMATCH')
    forbidden = {'planning_limits','joint_filter_sigma_s','grasp_switch_hold_s',
                 'grasp_switch_ramp_s','finish_hold_s','finish_ramp_s'}
    if forbidden.intersection(recipe):
        raise ValueError('SHOVEL_OFFLINE_CONDITION_FORBIDDEN')
    if 'dump' in recipe:
        if 'receiving_bin' not in recipe or not np.isclose(recipe['receiving_bin']['bottom_z_m'], recipe['ground']['z_in_base_m']):
            raise ValueError('INVALID_RECEIVING_BIN_GROUND')
        start, end = recipe['dump']['hold_seconds']
        for seconds in (start, end):
            matches = [k for k in keys if k['t'] == seconds]
            if len(matches) != 1 or not np.isclose(matches[0].get('shaft_roll_rad', 0.), np.deg2rad(design['dump_roll_deg'])):
                raise ValueError('SHOVEL_DUMP_KEYFRAME_MISMATCH')
    expected = ((gate['lift_seconds'], [0., -np.deg2rad(design['lift_deg'])]),
                (gate['swing_seconds'], [-np.deg2rad(design['outward_swing_deg']), -np.deg2rad(design['lift_deg'])]))
    for seconds, angles in expected:
        matches = [k for k in keys if k['t']==seconds]
        if len(matches)!=1 or not np.allclose(matches[0]['yaw_pitch_rad'], angles):
            raise ValueError('SHOVEL_KEYFRAME_DESIGN_MISMATCH')


def target_at(keyframes, home, pivot, seconds, shaft_axis=None):
    if not np.isfinite(seconds) or seconds < -1e-9 or seconds > keyframes[-1]['t']+1e-9:
        raise ValueError('TIME_OUTSIDE_RECIPE')
    seconds = np.clip(seconds, 0., keyframes[-1]['t'])
    for first, last in zip(keyframes, keyframes[1:]):
        if seconds <= last['t']:
            u = np.clip((seconds-first['t'])/(last['t']-first['t']), 0., 1.)
            blend = u**3*(10.-15.*u+6.*u*u)
            xyz = (1-blend)*np.array(first['translation_m']) + blend*np.array(last['translation_m'])
            angles = (1-blend)*np.array(first['yaw_pitch_rad']) + blend*np.array(last['yaw_pitch_rad'])
            target = home.copy()
            target[:3, :3] = (Rotation.from_euler('z', angles[0]).as_matrix() @
                              home[:3, :3] @ Rotation.from_euler('y', angles[1]).as_matrix())
            roll = (1-blend)*first.get('shaft_roll_rad', 0.) + blend*last.get('shaft_roll_rad', 0.)
            if roll:
                if shaft_axis is None:
                    raise ValueError('MISSING_SHAFT_ROLL_AXIS')
                target[:3, :3] = target[:3, :3] @ Rotation.from_rotvec(np.asarray(shaft_axis)*roll).as_matrix()
            anchor = home[:3, 3] + home[:3, :3] @ pivot
            target[:3, 3] = anchor + xyz - target[:3, :3] @ pivot
            return target
    raise ValueError('TIME_OUTSIDE_RECIPE')


def grasp_mode_at(recipe, seconds):
    return next(k['mode'] for k in reversed(recipe.get('grasp_schedule', [{'t':0.,'mode':'rigid'}]))
                if seconds+1e-9 >= k['t'])


def run_input_stream(solver, profile, config, initial, inputs):
    """Causal local execution of mapped teleoperation packets.

    No recipe, phases, future targets or target duration enter this loop.
    It uses the production Executor and simulated device drivers. End of input
    exercises the ordinary freshness watchdog, then records the stopped tail.
    """
    from bindu_contracts.contracts import Motion
    from bindu_teleoperation.bimanual_session import ToolPoseSession, AcceptedReference
    from bindu_execution.executor import Executor
    from bindu_hardware.robot_io.system import RobotIO
    from bindu_hardware.drivers.simulated_joints import SimJointDriver
    from bindu_hardware.drivers.simulated_base import SimBaseDriver
    drivers={g:SimJointDriver(names,profile.max_speed,initial) for g,names in profile.groups.items()}
    io=RobotIO(profile,drivers,SimBaseDriver())
    executor=Executor(profile,io)
    session=ToolPoseSession(config,0,-1.)
    reference=AcceptedReference(config.get('max_integration_interval',.1))
    dt=profile.control_period
    lease,epoch=executor.acquire(session.identity,solver.groups,2.,0.)
    frames=iter(inputs)
    latest=None
    exhausted=False
    closing=False
    last_generation=0
    records=[];events=[];packets=[]
    max_solve=0.
    next_renewal=0.
    for step in range(360000):
        t=round(step*dt,9)
        executor.tick(t)
        measured=dict(initial);measured.update(executor.feedback.positions)
        v=tuple((measured[n]-records[-1]['q'][n])/dt for n in solver.names) if records else (0.,)*len(solver.names)
        if executor.events and any('EMERGENCY' in e.code or e.state=='FAILED' for e in executor.events):
            raise ValueError('SHOVEL_EXECUTION_FAILED:'+executor.events[-1].code)
        if t>=next_renewal:
            executor.renew(lease,epoch,t);next_renewal=t+.3
        if not exhausted:
            try:
                latest=next(frames)
                if abs(latest.stamp-t)>1e-8:
                    raise ValueError('SHOVEL_INPUT_GRID')
                session.ingest(latest,t)
                packets.append({'stamp':latest.stamp,'seq':latest.seq,'source_id':latest.source_id,
                    'frame_id':latest.frame_id,'pose':list(latest.pose),'tight':latest.tight})
            except StopIteration:
                exhausted=True
        if latest is None:
            raise ValueError('EMPTY_TOOL_INPUT')
        if exhausted and t-latest.stamp>config['input_max_age'] and not closing:
            executor.halt(t)
            closing=True
            events.append({'t':t,'event':'input_timeout_stop'})
        if not closing:
            if session.grasp_generation != last_generation:
                executor.halt(t)
                session.reanchor();reference.reset()
                last_generation=session.grasp_generation
                events.append({'t':t,'event':'grasp_requested','mode':session.grasp_mode})
            if executor.stopping is None and (session.anchor_tool is not None or max(map(abs,v))<=.005):
                if session.anchor_tool is None:
                    session.capture(solver.capture(measured,session.grasp_mode))
                    reference.capture([measured[n] for n in solver.names],t)
                    events.append({'t':t,'event':'grasp_applied','mode':session.grasp_mode,
                                   'speed_rad_s':float(max(map(abs,v)))})
                else:
                    baseline,interval=reference.step(t)
                    positions,metrics=solver.solve(measured,session.target,reference=baseline,dt=interval,target_stamp=latest.stamp)
                    executor.submit(Motion(str(step),lease,epoch,profile.digest,'shovel_input',str(step),
                        'joint_target','',t,config['command_max_age'],names=solver.names,
                        positions=positions,schema_version=2,resource_groups=solver.groups),t)
                    reference.accept(positions,t)
                    max_solve=max(max_solve,metrics[0])
        records.append({'t':t,'q':measured,'grasp_mode':solver.grasp_mode})
        if closing and executor.stopping is None and t>=events[-1]['t']+.1 and step%5==0:
            break
    else:
        raise ValueError('TOOL_STREAM_DID_NOT_STOP')
    executor.halt(t,revoke=True)
    return records,events,packets,max_solve


def generate(config_path, description, output):
    config = json.loads(config_path.read_text())
    validate_recipe(config)
    recipe = dict(config['demo'])
    profile = Profile.load(config_path.parent/recipe['profile'])
    config['urdf'] = str(description/config['urdf'])
    display = description/recipe['display_urdf']
    dt, fps = config['sample_period'], recipe['sample_hz']
    stride = round(1./(fps*dt))
    keys = recipe['keyframes']
    if (not np.isclose(dt, profile.control_period) or stride < 1 or
            not np.isclose(stride*fps*dt, 1.) or keys[0]['t'] != 0 or
            not np.isclose(keys[-1]['t']*fps, round(keys[-1]['t']*fps)) or
            any(b['t'] <= a['t'] for a, b in zip(keys, keys[1:]))):
        raise ValueError('INVALID_SHOVEL_TIMELINE')
    output.mkdir(parents=True, exist_ok=True)
    if (output/'motion.json').exists():
        raise FileExistsError(output/'motion.json')
    model = pin.buildModelFromUrdf(config['urdf'])
    data = model.createData()
    frames = [config['left_frame'], config['right_frame']]
    ids = [model.getFrameId(n) for n in frames]
    solver = BimanualQP(config, profile)
    source_paths = (config_path, Path(config['urdf']), display,
                    config_path.parent/recipe['profile'], Path(__file__).resolve(),
                    Path(__import__('shovel_geometry').__file__).resolve(),
                    Path(__import__('bindu_kinematics.tool_geometry',fromlist=['']).__file__).resolve(),
                    Path(__import__('bindu_kinematics.bimanual', fromlist=['__file__']).__file__),
                    Path(solver.lib._name))
    source_paths += tuple(Path(module.__file__).resolve() for module in (
        __import__('bindu_teleoperation.bimanual_session',fromlist=['']),
        __import__('bindu_execution.executor',fromlist=['']),
        __import__('bindu_execution.interpolation',fromlist=['']),
        __import__('bindu_execution._native',fromlist=['']),
        __import__('bindu_contracts.teleoperation',fromlist=['']),
        __import__('bindu_contracts.profile',fromlist=['']),
        __import__('bindu_hardware.drivers.simulated_joints',fromlist=['']),
        __import__('bindu_hardware.robot_io.system',fromlist=['']),
        __import__('bindu_hardware.robot_io.feedback',fromlist=[''])))
    sources = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths}
    try:
        q = dict(recipe['initial_positions_rad'])
        if set(q) != set(solver.model_names):
            raise ValueError('READY_POSE_JOINT_LAYOUT')
        if any(not profile.limits[n][0] <= value <= profile.limits[n][1] for n, value in q.items()):
            raise ValueError('READY_POSE_JOINT_LIMIT')
        home = solver.capture(q)
        if abs(np.arctan2(home[1,0],home[0,0])-np.deg2rad(recipe['selected_design']['azimuth_deg'])) > .001:
            raise ValueError('READY_AZIMUTH_MISMATCH')
        pivot = np.array(recipe['pivot_in_tool_m'])
        index = {n: model.joints[model.getJointId(n)].idx_q for n in q}
        left_to_tool = np.array(config['left_to_tool']).reshape(4, 4)
        grip_offsets = [np.r_[recipe.get(side+'_grip_offset_m', recipe['grip_offset_m']), 1.]
                        for side in ('left', 'right')]
        samples, dense_samples = [], []
        max_speed, max_relative, max_tracking, max_solve = 0., 0., 0., 0.
        max_tracking_angle = 0.
        start_grips = None
        max_grip_error = 0.
        min_margin = float('inf')
        initial_positions = np.array([q[n] for n in solver.names])
        history = [initial_positions.copy() for _ in range(3)]
        max_acceleration, max_jerk = 0., 0.
        from bindu_contracts.teleoperation import ToolPoseFrame
        def inputs():
            for step in range(round(keys[-1]['t']/dt)+1):
                seconds=round(step*dt,9)
                desired=target_at(keys,home,pivot,seconds,recipe['shaft_axis_in_tool'])
                yield ToolPoseFrame('synthetic_shovel',step,seconds,config['input_frame'],
                    tuple(desired.ravel()),grasp_mode_at(recipe,seconds)=='rigid')
        records,stream_events,input_packets,max_solve=run_input_stream(solver,profile,config,q,inputs())
        planned_positions=np.asarray([[row['q'][n] for n in solver.names] for row in records])
        np.savez_compressed(output/'joint_plan.npz',executed=planned_positions,
                            names=solver.names,dt=dt,stage='shared_executor_feedback')
        (output/'tool_inputs.json').write_text(json.dumps(input_packets)+'\n')
        (output/'stream_events.json').write_text(json.dumps(stream_events,indent=2)+'\n')
        applied=[{'t':e['t'],'mode':e['mode']} for e in stream_events if e['event']=='grasp_applied']
        recipe['input_grasp_schedule']=recipe['grasp_schedule']
        recipe['grasp_schedule']=applied
        print('STREAM_COMPLETE',len(records),flush=True)
        q = dict(recipe['initial_positions_rad'])
        expected_relative = (BimanualQP._transform(config['left_to_tool']) @
                             np.linalg.inv(BimanualQP._transform(config['right_to_tool'])))
        grasp_mode = 'rigid'
        transitions = []
        max_axis_error = 0.
        previous_relative = expected_relative.copy()
        for step, positions in enumerate(planned_positions):
            seconds = round(step*dt, 9)
            desired = target_at(keys, home, pivot, min(seconds,keys[-1]['t']), recipe['shaft_axis_in_tool'])
            speed = max(abs(v-q[n])/dt for n, v in zip(solver.names, positions))
            max_speed = max(max_speed, speed)
            if any(abs(v-q[n])/dt > profile.speed(n)+1e-6 for n,v in zip(solver.names,positions)):
                raise ValueError('SHOVEL_SPEED_LIMIT')
            q.update(zip(solver.names, positions))
            margin = min(min(q[n]-profile.limits[n][0], profile.limits[n][1]-q[n])
                         for n in solver.names)
            min_margin = min(min_margin, margin)
            history.append(np.array(positions))
            if len(history) > 4:
                history.pop(0)
            if len(history) >= 3:
                acceleration = np.abs(np.diff(history, n=2, axis=0)[-1]/dt**2)
                max_acceleration = max(max_acceleration, float(np.max(acceleration)))
                if any(v > profile.acceleration(n)+1e-6 for n, v in zip(solver.names, acceleration)):
                    raise ValueError(f'SHOVEL_ACCELERATION_LIMIT t={seconds:.3f} max={acceleration.max():.6g}')
            if len(history) == 4:
                jerk = np.abs(np.diff(history, n=3, axis=0)[-1]/dt**3)
                max_jerk = max(max_jerk, float(np.max(jerk)))
                if any(v > profile.jerk(n)+1e-6 for n, v in zip(solver.names, jerk)):
                    raise ValueError(f'SHOVEL_JERK_LIMIT t={seconds:.3f} max={jerk.max():.6g} joint={solver.names[int(np.argmax(jerk))]} margin={min_margin:.5f} speed={speed:.5f}')
            state = pin.neutral(model)
            for name, value in q.items():
                state[index[name]] = value
            pin.framesForwardKinematics(model, data, state)
            poses = [data.oMf[i].homogeneous.copy() for i in ids]
            actual_relative = np.linalg.inv(poses[0]) @ poses[1]
            mode = records[step]['grasp_mode']
            if mode != grasp_mode:
                # Re-lock at the measured current pose, never the pre-slip 6D
                # reference. Measured joints must be stopped at the switch.
                if speed > .005:
                    raise ValueError(f'SHOVEL_GRASP_SWITCH_NOT_STOPPED t={seconds:.3f} speed={speed:.6g}')
                expected_relative = previous_relative.copy()
                transitions.append({'t': round(seconds,6), 'mode': mode, 'speed_rad_s': speed})
                grasp_mode = mode
            relative_error = pin.log6(pin.SE3(np.linalg.inv(expected_relative) @ actual_relative)).vector
            if mode == 'rigid':
                max_relative = max(max_relative, float(np.linalg.norm(relative_error)))
                if max_relative > config['max_relative_error']:
                    raise ValueError('SHOVEL_EXECUTED_CLOSURE')
            previous_relative = actual_relative
            tool = poses[0] @ left_to_tool
            roll = 0.
            axis_error = 0.
            if solver.axial is not None:
                grip = np.linalg.inv(tool) @ poses[1] @ solver.right_grip
                axis_error = float(np.arccos(np.clip(grip[:3,2] @ solver.tool_grip[:3,2], -1., 1.)))
                max_axis_error = max(max_axis_error, axis_error)
                if axis_error > config['grasp_rotation_tolerance_rad']:
                    raise ValueError('SHOVEL_EXECUTED_SHAFT_AXIS')
                relative_grip = np.linalg.inv(solver.tool_grip) @ grip
                roll = float(np.arctan2(relative_grip[1,0],relative_grip[0,0]))
            grips = [(np.linalg.inv(tool) @ pose @ offset)[:3]
                     for pose, offset in zip(poses, grip_offsets)]
            if start_grips is None:
                start_grips = np.array(grips)
            max_grip_error = max(max_grip_error, float(np.max(np.linalg.norm(grips-start_grips, axis=1))))
            max_tracking = max(max_tracking, float(np.linalg.norm(tool[:3, 3]-desired[:3, 3])))
            max_tracking_angle = max(max_tracking_angle, float(Rotation.from_matrix(tool[:3,:3].T @ desired[:3,:3]).magnitude()))
            sample = {'t': round(seconds, 6), 'q': dict(q), 'tool': tool.tolist(),
                      'grasp_mode': mode, 'right_grip_roll_rad': roll, 'right_axis_error_rad': axis_error,
                      'flanges':dict(zip(frames,(p.tolist() for p in poses)))}
            dense_samples.append(sample)
            if step % stride == 0:
                phase = next((a['phase'] for a, b in zip(keys, keys[1:])
                              if a['t'] <= seconds < b['t']), keys[-1]['phase'])
                samples.append(dict(sample, phase=phase,
                                    flanges=dict(zip(frames, (p.tolist() for p in poses)))))
        axis = np.array(recipe['shaft_axis_in_tool'])
        if (not np.isclose(np.linalg.norm(axis), 1.) or
                np.linalg.norm(np.cross(start_grips[1]-start_grips[0], axis)) > .001):
            raise ValueError('HANDS_NOT_ON_ONE_SHAFT')
        recipe['left_grip_in_tool_m'] = start_grips[0].tolist()
        recipe['right_grip_in_tool_m'] = start_grips[1].tolist()
        positions = np.array([s['tool'] for s in dense_samples])[:, :3, 3]
        gate = recipe['acceptance']
        push = float((np.array(samples[round(gate['push_seconds']*fps)]['tool'])[:3, 3]-home[:3, 3]) @ home[:3, 0])
        lift = float(np.max(positions[:, 2])-home[2, 3])
        return_error = float(np.linalg.norm(positions[-1]-home[:3, 3]))
        return_angle = Rotation.from_matrix(home[:3, :3].T @ np.array(samples[-1]['tool'])[:3, :3]).magnitude()
        ranges = {n: [min(s['q'][n] for s in samples), max(s['q'][n] for s in samples)] for n in solver.names}
        # An acceptance recipe declares measurable task outcomes, not a robot name.
        pan = np.array(recipe['pan_center_m'])
        half = np.array(recipe['pan_size_m'])/2
        corners = [pan+half*np.array([x, y, z]) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
        ground_z = recipe['ground']['z_in_base_m']
        clearance = min(float((np.array(s['tool']) @ np.r_[p, 1.])[2])-ground_z
                        for s in dense_samples for p in corners)
        ready_clearance = min(float((home @ np.r_[p, 1.])[2])-ground_z for p in corners)
        tracking=tracking_acceptance(recipe,max_tracking,max_tracking_angle)
        coupling=coupling_metrics(dense_samples,config,applied)
        return_within_reference=bool(return_error<=gate['max_return_error_m'] and
                                     return_angle<=gate['max_return_angle_rad'])
        if not coupling['passed']:
            raise ValueError('BIMANUAL_COUPLING_ERROR:'+json.dumps(coupling))
        if not tracking['passed']:
            raise ValueError(f'SHOVEL_TRACKING_ERROR position={max_tracking:.6g} rotation={max_tracking_angle:.6g}')
        if (push < gate['min_push_m'] or lift < gate['min_lift_m'] or
                max_grip_error > gate['max_grip_error_m'] or
                clearance < gate['min_ground_clearance_m'] or
                ready_clearance > gate['max_ready_clearance_m']):
            raise ValueError('SHOVEL_MOTION_ACCEPTANCE_FAILED:'+json.dumps(dict(push_m=push,lift_m=lift,return_error_m=return_error,max_grip_error_m=max_grip_error,clearance_m=clearance,ready_clearance_m=ready_clearance)))
        if tracking['policy']=='required' and not return_within_reference:
            raise ValueError('SHOVEL_RETURN_POSE_FAILED')
        if (min_margin < gate['min_joint_margin_rad'] or
                any(ranges[n][1]-ranges[n][0] < value for n, value in gate['required_joint_excursion_rad'].items())):
            raise ValueError('SHOVEL_POSTURE_RESERVE_FAILED')
        if any(not low <= s['q'][name] <= high for s in dense_samples
               for name,(low,high) in gate['posture_limits_rad'].items()):
            raise ValueError('SHOVEL_POSTURE_LIMIT')
        final_speed = float(np.max(np.abs(planned_positions[-1]-planned_positions[-2]))/dt)
        if final_speed > .005:
            raise ValueError('SHOVEL_FINISH_NOT_STOPPED')
        # Include the transition from the last QP point to a stationary hold.
        for _ in range(3):
            history.append(history[-1].copy())
            history.pop(0)
            acceleration = np.abs(np.diff(history, n=2, axis=0)[-1]/dt**2)
            jerk = np.abs(np.diff(history, n=3, axis=0)[-1]/dt**3)
            max_acceleration = max(max_acceleration, float(acceleration.max()))
            max_jerk = max(max_jerk, float(jerk.max()))
            if (any(v > profile.acceleration(n)+1e-6 for n,v in zip(solver.names,acceleration)) or
                    any(v > profile.jerk(n)+1e-6 for n,v in zip(solver.names,jerk))):
                raise ValueError('SHOVEL_HOLD_DYNAMICS_LIMIT')
        design = recipe['selected_design']
        lift_sample = np.array(dense_samples[round(gate['lift_seconds']/dt)]['tool'])
        swing_sample = np.array(dense_samples[round(gate['swing_seconds']/dt)]['tool'])
        lift_angle = -Rotation.from_matrix(home[:3,:3].T @ lift_sample[:3,:3]).as_euler('xyz')[1]
        swing_angle = np.arctan2(home[1,0],home[0,0])-np.arctan2(swing_sample[1,0],swing_sample[0,0])
        swing_angle = np.arctan2(np.sin(swing_angle),np.cos(swing_angle))
        phase_angles_within_reference=bool(
            abs(lift_angle-np.deg2rad(design['lift_deg']))<=gate['max_phase_angle_error_rad'] and
            abs(swing_angle-np.deg2rad(design['outward_swing_deg']))<=gate['max_phase_angle_error_rad'])
        if tracking['policy']=='required' and not phase_angles_within_reference:
            raise ValueError('SHOVEL_PHASE_ANGLE_FAILED')
        dump_result = check_dump(dense_samples, recipe)
        validation = {'ready_shaft_axis_base': (home[:3,:3] @ axis).tolist(),
                      'ready_grips_base_m': [(home @ np.r_[g,1.])[:3].tolist() for g in start_grips],
                      'duration_s': records[-1]['t'], 'input_duration_s': keys[-1]['t'],
                      'execution_profile': profile.name, 'input_contract': 'ToolPoseFrame',
                      'execution_mode':'causal shared Executor, ideal joint feedback',
                      'stream_events':stream_events, 'dump': dump_result, 'status': 'passed', 'control_steps':len(records),
                      'samples': len(samples), 'push_m': push, 'lift_m': lift,
                      'lift_angle_deg': float(np.rad2deg(lift_angle)),
                      'outward_swing_deg': float(np.rad2deg(swing_angle)),
                      'return_error_m': return_error, 'max_grip_error_m': max_grip_error,
                      'return_angle_rad': return_angle, 'min_joint_margin_rad': min_margin,
                      'min_pan_ground_clearance_m': clearance, 'max_joint_speed_rad_s': max_speed,
                      'ready_pan_ground_clearance_m': ready_clearance,
                      'final_joint_speed_rad_s': final_speed,
                      'max_joint_acceleration_rad_s2': max_acceleration, 'max_joint_jerk_rad_s3': max_jerk,
                      'max_relative_se3_error': max_relative,
                      'max_right_axis_error_rad': max_axis_error,
                      'grasp_transitions': transitions,
                      'right_grip_roll_range_rad': [min(s['right_grip_roll_rad'] for s in dense_samples),
                                                   max(s['right_grip_roll_rad'] for s in dense_samples)],
                      'tracking_acceptance':tracking,'coupling':coupling,
                      'return_within_reference':return_within_reference,
                      'phase_angles_within_reference':phase_angles_within_reference,
                      'max_target_lag_m': max_tracking,
                      'max_target_rotation_error_rad': max_tracking_angle,
                      'max_qp_solve_us': max_solve,
                      'joint_ranges_rad': ranges}
    finally:
        solver.close()
    print('MOTION_CHECK_COMPLETE; checking full CAD geometry',len(dense_samples),flush=True)
    validation['geometry'] = collision_check(display, dense_samples, recipe)
    for name, digest in validation['geometry'].pop('source_sha256').items():
        if name in sources and sources[name] != digest:
            raise ValueError('SOURCE_CHANGED_DURING_PLAN:'+name)
        sources[name] = digest
    if any(hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest for name,digest in sources.items()):
        raise ValueError('SOURCE_CHANGED_DURING_PLAN')
    payload = {'schema': 2, 'units': 'm, rad, s', 'frame': 'fixed URDF base',
               'duration_s':records[-1]['t'],'input_contract':'ToolPoseFrame',
               'tool_input_path':str(output/'tool_inputs.json'),
               'tool_input_sha256':hashlib.sha256((output/'tool_inputs.json').read_bytes()).hexdigest(),
               'source_sha256': sources,
               'config_path': str(config_path), 'config_sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(),
               'display_urdf': str(display), 'display_sha256': hashlib.sha256(display.read_bytes()).hexdigest(),
               'left_frame': frames[0], 'right_frame': frames[1], 'left_to_tool': config['left_to_tool'],
               'home': home.tolist(), 'recipe': recipe, 'samples': samples}
    (output/'motion.json').write_text(json.dumps(payload, ensure_ascii=False)+'\n')
    validation['motion_sha256'] = hashlib.sha256((output/'motion.json').read_bytes()).hexdigest()
    (output/'plan_validation.json').write_text(json.dumps(validation, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(validation, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=ROOT/'src/integration/bindu_runtime/config/bimanual_snowbot_shovel.json')
    parser.add_argument('--description', type=Path, default=ROOT/'src/hardware/bindu_description/urdf')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    generate(args.config.resolve(), args.description.resolve(), args.output.resolve())


if __name__ == '__main__':
    main()
