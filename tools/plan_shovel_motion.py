#!/usr/bin/env python3
"""Generate and check an offline rigid-shovel trajectory with the existing QP.

The recipe owns robot names, ready posture, grasp and motion. This tool does not
publish commands. Positions are metres/radians in the fixed URDF base frame.
Rotation is world yaw, local pitch, then roll about the shaft. The rear grip
is the pivot. Optional offline joint smoothing is validated after filtering.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from bindu_contracts.profile import Profile
from bindu_kinematics.bimanual import BimanualQP
from shovel_geometry import bin_boxes, check_dump
from bindu_kinematics.tool_geometry import collision_check

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
    recipe, design = config['demo'], config['demo']['selected_design']
    axis = np.asarray(recipe['shaft_axis_in_tool'])
    ends = recipe['shaft_from_left_grip_m']
    offset = np.r_[recipe['grip_offset_m'], 1.]
    grips = [(np.linalg.inv(BimanualQP._transform(config[side+'_to_tool'])) @ offset)[:3]
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
    if 'planning_limits' in recipe:
        limits = recipe['planning_limits']
        if (config.get('simulation_only') is not True or
                set(limits) != {'max_speed','max_acceleration','max_jerk'} or
                any(not isinstance(v,(int,float)) or v <= 0 for v in limits.values())):
            raise ValueError('INVALID_SHOVEL_PLANNING_LIMITS')
    bin_boxes(recipe)
    keys, gate = recipe['keyframes'], recipe['acceptance']
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


def generate(config_path, description, output):
    config = json.loads(config_path.read_text())
    validate_recipe(config)
    recipe = dict(config['demo'])
    profile = Profile.load(config_path.parent/recipe['profile'])
    if 'planning_limits' in recipe:
        profile = replace(profile, **recipe['planning_limits'])
    config['urdf'] = str(description/config['urdf'])
    display = description/recipe['display_urdf']
    dt, fps = config['sample_period'], recipe['sample_hz']
    stride = round(1./(fps*dt))
    keys = recipe['keyframes']
    if (not np.isclose(dt, profile.control_period) or stride < 1 or
            not np.isclose(stride*fps*dt, 1.) or keys[0]['t'] != 0 or
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
        grip_offset = np.r_[recipe['grip_offset_m'], 1.]
        samples, dense_samples = [], []
        max_speed, max_relative, max_tracking, max_solve = 0., 0., 0., 0.
        start_grips = None
        max_grip_error = 0.
        min_margin = float('inf')
        initial_positions = np.array([q[n] for n in solver.names])
        history = [initial_positions.copy() for _ in range(3)]
        max_acceleration, max_jerk = 0., 0.
        raw_positions = []
        for step in range(round(keys[-1]['t']/dt)+1):
            seconds = step*dt
            desired = target_at(keys, home, pivot, seconds, recipe['shaft_axis_in_tool'])
            if 'shared_targets_rad' in keys[0]:
                first,last = next((a,b) for a,b in zip(keys,keys[1:]) if seconds <= b['t'])
                u = np.clip((seconds-first['t'])/(last['t']-first['t']),0.,1.)
                blend = u**3*(10.-15.*u+6.*u*u)
                solver.set_shared_targets({n:(1-blend)*first['shared_targets_rad'][n]+blend*last['shared_targets_rad'][n]
                                           for n in solver.shared_names})
            try:
                positions, metrics = solver.solve(q, desired,
                    reference=[q[n] for n in solver.names], dt=dt)
            except ValueError as exc:
                raise ValueError(f'{exc} t={seconds:.3f}') from exc
            q.update(zip(solver.names, positions))
            raw_positions.append(positions)
            max_solve = max(max_solve, metrics[0])
        from scipy.ndimage import gaussian_filter1d
        raw_positions = np.asarray(raw_positions)
        sigma = float(recipe.get('joint_filter_sigma_s', 0.))
        if not np.isfinite(sigma) or sigma < 0:
            raise ValueError('INVALID_SHOVEL_JOINT_FILTER')
        planned_positions = gaussian_filter1d(raw_positions, sigma/dt, axis=0, mode='nearest', truncate=4.) if sigma else raw_positions
        raw_velocity = np.diff(np.vstack((initial_positions,raw_positions)),axis=0)/dt
        raw_acceleration = np.diff(np.vstack((np.zeros_like(initial_positions),raw_velocity)),axis=0)/dt
        raw_jerk = np.diff(np.vstack((np.zeros_like(initial_positions),raw_acceleration)),axis=0)/dt
        raw_peaks = dict(speed_rad_s=float(np.abs(raw_velocity).max()), acceleration_rad_s2=float(np.abs(raw_acceleration).max()), jerk_rad_s3=float(np.abs(raw_jerk).max()))
        q = dict(recipe['initial_positions_rad'])
        expected_relative = (BimanualQP._transform(config['left_to_tool']) @
                             np.linalg.inv(BimanualQP._transform(config['right_to_tool'])))
        for step, positions in enumerate(planned_positions):
            seconds = step*dt
            desired = target_at(keys, home, pivot, seconds, recipe['shaft_axis_in_tool'])
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
            relative_error = pin.log6(pin.SE3(np.linalg.inv(expected_relative) @ actual_relative)).vector
            max_relative = max(max_relative, float(np.linalg.norm(relative_error)))
            if max_relative > config['max_relative_error']:
                raise ValueError('SHOVEL_FILTERED_CLOSURE')
            tool = poses[0] @ left_to_tool
            grips = [(np.linalg.inv(tool) @ pose @ grip_offset)[:3] for pose in poses]
            if start_grips is None:
                start_grips = np.array(grips)
            max_grip_error = max(max_grip_error, float(np.max(np.linalg.norm(grips-start_grips, axis=1))))
            max_tracking = max(max_tracking, float(np.linalg.norm(tool[:3, 3]-desired[:3, 3])))
            sample = {'t': round(seconds, 6), 'q': dict(q), 'tool': tool.tolist()}
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
        if (push < gate['min_push_m'] or lift < gate['min_lift_m'] or
                return_error > gate['max_return_error_m'] or
                max_grip_error > gate['max_grip_error_m'] or
                clearance < gate['min_ground_clearance_m'] or
                ready_clearance > gate['max_ready_clearance_m']):
            raise ValueError('SHOVEL_MOTION_ACCEPTANCE_FAILED')
        if (min_margin < gate['min_joint_margin_rad'] or return_angle > gate['max_return_angle_rad'] or
                any(ranges[n][1]-ranges[n][0] < value for n, value in gate['required_joint_excursion_rad'].items())):
            raise ValueError('SHOVEL_POSTURE_RESERVE_FAILED')
        if any(not low <= s['q'][name] <= high for s in dense_samples
               for name,(low,high) in gate['posture_limits_rad'].items()):
            raise ValueError('SHOVEL_POSTURE_LIMIT')
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
        if (abs(lift_angle-np.deg2rad(design['lift_deg'])) > gate['max_phase_angle_error_rad'] or
                abs(swing_angle-np.deg2rad(design['outward_swing_deg'])) > gate['max_phase_angle_error_rad']):
            raise ValueError('SHOVEL_PHASE_ANGLE_FAILED')
        dump_result = check_dump(dense_samples, recipe)
        validation = {'ready_shaft_axis_base': (home[:3,:3] @ axis).tolist(),
                      'ready_grips_base_m': [(home @ np.r_[g,1.])[:3].tolist() for g in start_grips],
                      'duration_s': keys[-1]['t'], 'planning_limits': recipe.get('planning_limits'),
                      'raw_qp_peaks': raw_peaks, 'joint_filter_sigma_s': sigma, 'dump': dump_result, 'status': 'passed', 'control_steps': round(keys[-1]['t']/dt)+1,
                      'samples': len(samples), 'push_m': push, 'lift_m': lift,
                      'lift_angle_deg': float(np.rad2deg(lift_angle)),
                      'outward_swing_deg': float(np.rad2deg(swing_angle)),
                      'return_error_m': return_error, 'max_grip_error_m': max_grip_error,
                      'return_angle_rad': return_angle, 'min_joint_margin_rad': min_margin,
                      'min_pan_ground_clearance_m': clearance, 'max_joint_speed_rad_s': max_speed,
                      'ready_pan_ground_clearance_m': ready_clearance,
                      'max_joint_acceleration_rad_s2': max_acceleration, 'max_joint_jerk_rad_s3': max_jerk,
                      'max_relative_se3_error': max_relative, 'max_target_lag_m': max_tracking,
                      'max_qp_solve_us': max_solve,
                      'joint_ranges_rad': ranges}
    finally:
        solver.close()
    validation['geometry'] = collision_check(display, dense_samples, recipe)
    for name, digest in validation['geometry'].pop('source_sha256').items():
        if name in sources and sources[name] != digest:
            raise ValueError('SOURCE_CHANGED_DURING_PLAN:'+name)
        sources[name] = digest
    if any(hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest for name,digest in sources.items()):
        raise ValueError('SOURCE_CHANGED_DURING_PLAN')
    payload = {'schema': 2, 'units': 'm, rad, s', 'frame': 'fixed URDF base',
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
