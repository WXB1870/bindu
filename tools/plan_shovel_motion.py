#!/usr/bin/env python3
"""Generate and check an offline rigid-shovel trajectory with the existing QP.

The recipe owns robot names, ready posture, grasp and motion. This tool does not
publish commands. Positions are metres/radians in the fixed URDF base frame.
Rotation is world yaw followed by local pitch; the rear grip is the pivot.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from bindu_contracts.profile import Profile
from bindu_kinematics.bimanual import BimanualQP

ROOT = Path(__file__).resolve().parents[1]


def target_at(keyframes, home, pivot, seconds):
    for first, last in zip(keyframes, keyframes[1:]):
        if seconds <= last['t']:
            u = np.clip((seconds-first['t'])/(last['t']-first['t']), 0., 1.)
            blend = u**3*(10.-15.*u+6.*u*u)
            xyz = (1-blend)*np.array(first['translation_m']) + blend*np.array(last['translation_m'])
            angles = (1-blend)*np.array(first['yaw_pitch_rad']) + blend*np.array(last['yaw_pitch_rad'])
            target = home.copy()
            target[:3, :3] = (Rotation.from_euler('z', angles[0]).as_matrix() @
                              home[:3, :3] @ Rotation.from_euler('y', angles[1]).as_matrix())
            anchor = home[:3, 3] + home[:3, :3] @ pivot
            target[:3, 3] = anchor + xyz - target[:3, :3] @ pivot
            return target
    raise ValueError('TIME_OUTSIDE_RECIPE')


def collision_check(urdf, samples, recipe, collect_failures=False):
    """CAD mesh self-intersection and tool clearance at the configured sample rate.

    Adjacent mechanical segments (tree distance <= 2) and intended shaft/hand
    contact are excluded explicitly. Empty CAD meshes provide no geometry.
    This is a discrete geometric check, not a contact or dynamics model.
    """
    import coal as fcl

    tree = ET.parse(urdf)
    skipped = []
    for mesh in tree.findall('.//mesh'):
        uri = mesh.get('filename')
        prefix = f'package://{urdf.parent.parent.name}/'
        if uri.startswith(prefix):
            mesh.set('filename', str(urdf.parent.parent / uri.removeprefix(prefix)))
        elif not Path(uri).is_absolute():
            raise ValueError('UNRESOLVED_COLLISION_MESH:'+uri)
    for link in tree.findall('link'):
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            path = Path(mesh.get('filename')) if mesh is not None else None
            if path is not None and path.stat().st_size == 84 and path.read_bytes()[80:] == b'\0\0\0\0':
                skipped.append(link.get('name'))
                link.remove(collision)
    with tempfile.TemporaryDirectory(prefix='bindu-shovel-') as directory:
        absolute = Path(directory)/'collision.urdf'
        tree.write(absolute)
        model = pin.buildModelFromUrdf(str(absolute))
        geometry = pin.buildGeomFromUrdf(model, str(absolute), pin.GeometryType.COLLISION)
    parents = {j.find('child').get('link'): j.find('parent').get('link') for j in tree.findall('joint')}

    def ancestors(name):
        chain = [name]
        while name in parents:
            name = parents[name]
            chain.append(name)
        return chain

    names = [model.frames[g.parentFrame].name for g in geometry.geometryObjects]
    for a, first in enumerate(names):
        for b in range(a+1, len(names)):
            second = names[b]
            if geometry.geometryObjects[a].parentJoint == geometry.geometryObjects[b].parentJoint:
                continue
            left, right = ancestors(first), ancestors(second)
            distance = min(left.index(n)+right.index(n) for n in set(left) & set(right))
            if distance > 2:
                geometry.addCollisionPair(pin.CollisionPair(a, b))
    data, geometry_data = model.createData(), pin.GeometryData(geometry)
    index = {n: model.joints[model.getJointId(n)].idx_q for n in samples[0]['q']}
    shaft_axis = np.array(recipe['shaft_axis_in_tool'])
    ends = recipe['shaft_from_left_grip_m']
    if (not np.isclose(np.linalg.norm(shaft_axis), 1.) or ends[1] <= ends[0] or
            recipe['shaft_radius_m'] <= 0 or min(recipe['pan_size_m']) <= 0):
        raise ValueError('INVALID_SHOVEL_GEOMETRY')
    shaft = fcl.Capsule(recipe['shaft_radius_m'], ends[1]-ends[0])
    reference = np.array([1., 0., 0.]) if abs(shaft_axis[0]) < .9 else np.array([0., 1., 0.])
    transverse = np.cross(reference, shaft_axis)
    transverse /= np.linalg.norm(transverse)
    rotation = np.column_stack((transverse, np.cross(shaft_axis, transverse), shaft_axis))
    excluded = set(recipe['shaft_contact_links'])
    if not excluded.issubset(names):
        raise ValueError('UNKNOWN_SHAFT_CONTACT_LINK')
    pan = fcl.Box(*recipe['pan_size_m'])
    failures = []
    for sample in samples:
        q = pin.neutral(model)
        for name, value in sample['q'].items():
            q[index[name]] = value
        pin.computeCollisions(model, data, geometry, geometry_data, q, False)
        failure = None
        for pair, result in zip(geometry.collisionPairs, geometry_data.collisionResults):
            if result.isCollision():
                failure = f'SELF_COLLISION: {names[pair.first]} / {names[pair.second]}'
                break
        tool = np.array(sample['tool'])
        center = np.array(recipe['left_grip_in_tool_m']) + shaft_axis*sum(ends)/2
        transform = fcl.Transform3s(tool[:3, :3] @ rotation, tool[:3, 3]+tool[:3, :3] @ center)
        pan_transform = fcl.Transform3s(tool[:3, :3], tool[:3, 3]+tool[:3, :3] @ np.array(recipe['pan_center_m']))
        for part, shape, pose, contacts in (('SHAFT', shaft, transform, excluded), ('PAN', pan, pan_transform, set())):
            if failure:
                break
            for name, obj, placement in zip(names, geometry.geometryObjects, geometry_data.oMg):
                if name in contacts:
                    continue
                result = fcl.CollisionResult()
                fcl.collide(shape, pose, obj.geometry,
                            fcl.Transform3s(placement.rotation, placement.translation),
                            fcl.CollisionRequest(), result)
                if result.isCollision():
                    failure = f'{part}_COLLISION: {name}'
                    break
        if failure:
            if not collect_failures:
                raise ValueError(f"{failure} t={sample['t']}")
            failures.append({'sample': sample['t'], 'reason': failure})
    return {'samples': len(samples), 'mesh_pairs': len(geometry.collisionPairs),
            'excluded_tree_distance': 2, 'shaft_contact_links': sorted(excluded),
            'empty_meshes': skipped, 'intersections': len(failures), 'failures': failures}


def generate(config_path, description, output):
    config = json.loads(config_path.read_text())
    recipe = dict(config['demo'])
    profile = Profile.load(config_path.parent/recipe['profile'])
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
    try:
        q = dict(recipe['initial_positions_rad'])
        if set(q) != set(solver.model_names):
            raise ValueError('READY_POSE_JOINT_LAYOUT')
        if any(not profile.limits[n][0] <= value <= profile.limits[n][1] for n, value in q.items()):
            raise ValueError('READY_POSE_JOINT_LIMIT')
        home = solver.capture(q)
        pivot = np.array(recipe['pivot_in_tool_m'])
        index = {n: model.joints[model.getJointId(n)].idx_q for n in q}
        left_to_tool = np.array(config['left_to_tool']).reshape(4, 4)
        grip_offset = np.r_[recipe['grip_offset_m'], 1.]
        samples, max_speed, max_relative, max_tracking, max_solve = [], 0., 0., 0., 0.
        start_grips = None
        max_grip_error = 0.
        min_margin = float('inf')
        history = []
        max_acceleration, max_jerk = 0., 0.
        for step in range(round(keys[-1]['t']/dt)+1):
            seconds = step*dt
            desired = target_at(keys, home, pivot, seconds)
            positions, metrics = solver.solve(q, desired)
            speed = max(abs(v-q[n])/dt for n, v in zip(solver.names, positions))
            max_speed, max_solve = max(max_speed, speed), max(max_solve, metrics[0])
            max_relative = max(max_relative, metrics[1])
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
                    raise ValueError('SHOVEL_ACCELERATION_LIMIT')
            if len(history) == 4:
                jerk = np.abs(np.diff(history, n=3, axis=0)[-1]/dt**3)
                max_jerk = max(max_jerk, float(np.max(jerk)))
                if any(v > profile.jerk(n)+1e-6 for n, v in zip(solver.names, jerk)):
                    raise ValueError('SHOVEL_JERK_LIMIT')
            state = pin.neutral(model)
            for name, value in q.items():
                state[index[name]] = value
            pin.framesForwardKinematics(model, data, state)
            poses = [data.oMf[i].homogeneous.copy() for i in ids]
            tool = poses[0] @ left_to_tool
            grips = [(np.linalg.inv(tool) @ pose @ grip_offset)[:3] for pose in poses]
            if start_grips is None:
                start_grips = np.array(grips)
            max_grip_error = max(max_grip_error, float(np.max(np.linalg.norm(grips-start_grips, axis=1))))
            max_tracking = max(max_tracking, float(np.linalg.norm(tool[:3, 3]-desired[:3, 3])))
            if step % stride == 0:
                phase = next((a['phase'] for a, b in zip(keys, keys[1:])
                              if a['t'] <= seconds < b['t']), keys[-1]['phase'])
                samples.append({'t': round(seconds, 6), 'phase': phase, 'q': dict(q),
                                'tool': tool.tolist(), 'flanges': dict(zip(frames, (p.tolist() for p in poses)))})
        axis = np.array(recipe['shaft_axis_in_tool'])
        if (not np.isclose(np.linalg.norm(axis), 1.) or
                np.linalg.norm(np.cross(start_grips[1]-start_grips[0], axis)) > .001):
            raise ValueError('HANDS_NOT_ON_ONE_SHAFT')
        recipe['left_grip_in_tool_m'] = start_grips[0].tolist()
        recipe['right_grip_in_tool_m'] = start_grips[1].tolist()
        positions = np.array([s['tool'] for s in samples])[:, :3, 3]
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
        clearance = min(float((np.array(s['tool']) @ np.r_[p, 1.])[2]) for s in samples for p in corners)
        ready_clearance = min(float((home @ np.r_[p, 1.])[2]) for p in corners)
        if (push < gate['min_push_m'] or lift < gate['min_lift_m'] or
                return_error > gate['max_return_error_m'] or
                max_grip_error > gate['max_grip_error_m'] or
                clearance < gate['min_ground_clearance_m'] or
                ready_clearance > gate['max_ready_clearance_m']):
            raise ValueError('SHOVEL_MOTION_ACCEPTANCE_FAILED')
        if (min_margin < gate['min_joint_margin_rad'] or return_angle > gate['max_return_angle_rad'] or
                any(ranges[n][1]-ranges[n][0] < value for n, value in gate['required_joint_excursion_rad'].items())):
            raise ValueError('SHOVEL_POSTURE_RESERVE_FAILED')
        validation = {'status': 'passed', 'control_steps': round(keys[-1]['t']/dt)+1,
                      'samples': len(samples), 'push_m': push, 'lift_m': lift,
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
    validation['geometry'] = collision_check(display, samples, recipe)
    payload = {'schema': 1, 'units': 'm, rad, s', 'frame': 'fixed URDF base',
               'config_path': str(config_path), 'config_sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(),
               'display_urdf': str(display), 'display_sha256': hashlib.sha256(display.read_bytes()).hexdigest(),
               'left_frame': frames[0], 'right_frame': frames[1], 'left_to_tool': config['left_to_tool'],
               'home': home.tolist(), 'recipe': recipe, 'samples': samples}
    (output/'motion.json').write_text(json.dumps(payload, ensure_ascii=False)+'\n')
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
