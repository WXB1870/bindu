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
    keys, gate = recipe['keyframes'], recipe['acceptance']
    expected = ((gate['lift_seconds'], [0., -np.deg2rad(design['lift_deg'])]),
                (gate['swing_seconds'], [-np.deg2rad(design['outward_swing_deg']), -np.deg2rad(design['lift_deg'])]))
    for seconds, angles in expected:
        matches = [k for k in keys if k['t']==seconds]
        if len(matches)!=1 or not np.allclose(matches[0]['yaw_pitch_rad'], angles):
            raise ValueError('SHOVEL_KEYFRAME_DESIGN_MISMATCH')


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
    """Discrete CAD and tool clearance; only bounded grip bands allow contact.

    The ground is expressed in the fixed URDF base frame and cross-checked
    against configured supporting links. Missing meshes require explicit
    conservative proxies. This is not a contact/dynamics or continuous check.
    """
    import coal as fcl

    if not samples:
        raise ValueError('EMPTY_COLLISION_SAMPLES')
    tree = ET.parse(urdf)
    skipped, proxies = [], recipe.get('empty_mesh_proxies', {})
    asset_hashes = {str(urdf): hashlib.sha256(urdf.read_bytes()).hexdigest()}
    for mesh in tree.findall('.//mesh'):
        uri = mesh.get('filename')
        prefix = f'package://{urdf.parent.parent.name}/'
        if uri.startswith(prefix):
            uri = str(urdf.parent.parent / uri.removeprefix(prefix))
            mesh.set('filename', uri)
        elif not Path(uri).is_absolute():
            raise ValueError('UNRESOLVED_COLLISION_MESH:'+uri)
        asset_hashes[uri] = hashlib.sha256(Path(uri).read_bytes()).hexdigest()
    for link in tree.findall('link'):
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            path = Path(mesh.get('filename')) if mesh is not None else None
            if path is not None and path.stat().st_size == 84 and path.read_bytes()[80:] == b'\0\0\0\0':
                name = link.get('name')
                if name not in proxies:
                    raise ValueError('MISSING_COLLISION_GEOMETRY:'+name)
                proxy = proxies[name]
                size, center = np.asarray(proxy['size_m']), np.asarray(proxy['center_m'])
                if size.shape != (3,) or center.shape != (3,) or not np.isfinite(np.r_[size,center]).all() or min(size) <= 0:
                    raise ValueError('INVALID_EMPTY_MESH_PROXY')
                skipped.append(name)
                geometry_element = collision.find('geometry')
                geometry_element.remove(mesh)
                ET.SubElement(geometry_element, 'box', size=' '.join(map(str,size)))
                origin = collision.find('origin')
                if origin is None:
                    origin = ET.SubElement(collision, 'origin')
                origin.set('xyz',' '.join(map(str,center)))
                origin.set('rpy','0 0 0')
    if set(proxies) != set(skipped):
        raise ValueError('UNUSED_EMPTY_MESH_PROXY')
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
    if any(not model.existJointName(n) for n in samples[0]['q']):
        raise ValueError('COLLISION_JOINT_LAYOUT')
    index = {n: model.joints[model.getJointId(n)].idx_q for n in samples[0]['q']}
    if set(index) != set(model.names[1:]):
        raise ValueError('COLLISION_JOINT_LAYOUT')
    shaft_axis = np.asarray(recipe['shaft_axis_in_tool'], dtype=float)
    ends = np.asarray(recipe['shaft_from_left_grip_m'], dtype=float)
    radius = float(recipe['shaft_radius_m'])
    clearance_limit = float(recipe['acceptance']['min_tool_robot_clearance_m'])
    ground = recipe['ground']
    ground_z = float(ground['z_in_base_m'])
    support_tolerance = float(ground['support_tolerance_m'])
    if (not np.isfinite(np.r_[shaft_axis, ends, radius, clearance_limit, ground_z, support_tolerance,
                              recipe['pan_size_m'], recipe['pan_center_m']]).all() or
            not np.isclose(np.linalg.norm(shaft_axis), 1.) or ends.shape != (2,) or ends[1] <= ends[0] or
            radius <= 0 or min(recipe['pan_size_m']) <= 0 or clearance_limit <= 0 or support_tolerance <= 0):
        raise ValueError('INVALID_SHOVEL_GEOMETRY')
    reference = np.array([1., 0., 0.]) if abs(shaft_axis[0]) < .9 else np.array([0., 1., 0.])
    transverse = np.cross(reference, shaft_axis)
    transverse /= np.linalg.norm(transverse)
    rotation = np.column_stack((transverse, np.cross(shaft_axis, transverse), shaft_axis))
    contacts = recipe['shaft_contacts']
    if not set(contacts).issubset(names):
        raise ValueError('UNKNOWN_SHAFT_CONTACT_LINK')
    segments = {}
    for name in set(names):
        spans = [tuple(ends)]
        if name in contacts:
            center, half_length = contacts[name]['center_m'], contacts[name]['half_length_m']
            if (not np.isfinite([center, half_length]).all() or half_length <= 0 or
                    center-half_length <= ends[0] or center+half_length >= ends[1]):
                raise ValueError('INVALID_SHAFT_CONTACT_BAND')
            # Cylinder cut faces bound each allowed contact band precisely;
            # unlike skipping a hand mesh this still checks the rest of the rod.
            spans = [(ends[0], center-half_length), (center+half_length, ends[1])]
        segments[name] = [(fcl.Cylinder(radius, b-a) if name in contacts else fcl.Capsule(radius, b-a),
                           np.array(recipe['left_grip_in_tool_m'])+shaft_axis*(a+b)/2)
                          for a, b in spans]
    pan = fcl.Box(*recipe['pan_size_m'])
    # AABB distance is a conservative lower bound. Skip an exact mesh query
    # only when it cannot violate the gate or reduce the recorded global minimum.
    bounds = {}
    def world_bounds(shape, rotation, translation):
        key = id(shape)
        if key not in bounds:
            shape.computeLocalAABB()
            low, high = np.array(shape.aabb_local.min_), np.array(shape.aabb_local.max_)
            bounds[key] = ((low+high)/2, (high-low)/2)
        center, half = bounds[key]
        return rotation @ center+translation, np.abs(rotation) @ half
    failures, worst = [], None
    exact_queries, bounded_queries = 0, 0
    minimum = float('inf')
    support_heights = {}
    distance_request = fcl.DistanceRequest()
    for sample_index, sample in enumerate(samples):
        if set(sample['q']) != set(index):
            raise ValueError('COLLISION_JOINT_LAYOUT')
        q = pin.neutral(model)
        for name, value in sample['q'].items():
            q[index[name]] = value
        tool = BimanualQP._transform(sample['tool'])
        if not np.isfinite(q).all() or not np.isfinite(sample['t']):
            raise ValueError('NONFINITE_COLLISION_SAMPLE')
        pin.computeCollisions(model, data, geometry, geometry_data, q, False)
        failure = None
        for pair, result in zip(geometry.collisionPairs, geometry_data.collisionResults):
            if result.isCollision():
                failure = f'SELF_COLLISION: {names[pair.first]} / {names[pair.second]}'
                break
        if sample_index == 0:
            for name, obj, placement in zip(names, geometry.geometryObjects, geometry_data.oMg):
                if name in ground['support_links']:
                    vertices = np.asarray(obj.geometry.vertices())
                    height = float(np.min((vertices @ placement.rotation.T + placement.translation)[:, 2]))
                    support_heights[name] = min(height, support_heights.get(name, float('inf')))
            if (set(support_heights) != set(ground['support_links']) or
                    any(abs(height-ground_z) > support_tolerance for height in support_heights.values())):
                raise ValueError('GROUND_SUPPORT_MISMATCH')
        pan_pose = fcl.Transform3s(tool[:3, :3], tool[:3, 3]+tool[:3, :3] @ np.array(recipe['pan_center_m']))
        for name, obj, placement in zip(names, geometry.geometryObjects, geometry_data.oMg):
            body_pose = fcl.Transform3s(placement.rotation, placement.translation)
            body_center, body_half = world_bounds(obj.geometry, placement.rotation, placement.translation)
            parts = [('PAN', pan, pan_pose)]
            parts += [('SHAFT', shape, fcl.Transform3s(tool[:3, :3] @ rotation,
                       tool[:3, 3]+tool[:3, :3] @ center)) for shape, center in segments[name]]
            for part, shape, pose in parts:
                part_center, part_half = world_bounds(shape, pose.getRotation(), pose.getTranslation())
                lower_bound = np.linalg.norm(np.maximum(np.abs(part_center-body_center)-part_half-body_half, 0.))-1e-9
                if lower_bound >= max(clearance_limit, minimum):
                    bounded_queries += 1
                    continue
                exact_queries += 1
                result = fcl.DistanceResult()
                distance = float(fcl.distance(shape, pose, obj.geometry, body_pose, distance_request, result))
                if not np.isfinite(distance):
                    raise ValueError('NONFINITE_COLLISION_DISTANCE')
                if distance < minimum:
                    minimum = distance
                    worst = {'part': part, 'link': name, 't': sample['t']}
                if distance < clearance_limit and failure is None:
                    failure = f'{part}_CLEARANCE: {name} distance={distance:.6g}'
        if sample_index % 1000 == 0:
            print('GEOMETRY_CHECK', sample_index, sample['t'], minimum, flush=True)
        if failure:
            if not collect_failures:
                raise ValueError(f"{failure} t={sample['t']}")
            failures.append({'sample': sample['t'], 'reason': failure})
    return {'samples': len(samples), 'mesh_pairs': len(geometry.collisionPairs),
            'excluded_tree_distance': 2, 'shaft_contacts': contacts,
            'empty_mesh_proxies': proxies, 'support_heights_m': support_heights,
            'min_tool_robot_clearance_m': minimum, 'closest_tool_robot_pair': worst,
            'exact_distance_queries': exact_queries, 'aabb_bounded_queries': bounded_queries,
            'source_sha256': asset_hashes, 'violations': len(failures), 'failures': failures}


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
        for step in range(round(keys[-1]['t']/dt)+1):
            seconds = step*dt
            desired = target_at(keys, home, pivot, seconds)
            positions, metrics = solver.solve(q, desired,
                reference=[q[n] for n in solver.names], dt=dt)
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
        validation = {'status': 'passed', 'control_steps': round(keys[-1]['t']/dt)+1,
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
