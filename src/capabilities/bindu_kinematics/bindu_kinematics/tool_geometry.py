"""Configured receiving-bin geometry shared by offline checks and Isaac views.

All positions are in the fixed robot base frame. The bin is an open box, not a
solid obstacle across its mouth. This module has no robot or simulator imports.
"""
import numpy as np


def pan_corners(recipe):
    center = np.asarray(recipe['pan_center_m'])
    half = np.asarray(recipe['pan_size_m']) / 2
    return np.array([np.r_[center+half*np.array([x,y,z]),1.]
                     for x in (-1,1) for y in (-1,1) for z in (-1,1)])


def bin_boxes(recipe):
    """Return (name, full size, world center) for four walls and the floor."""
    if 'receiving_bin' not in recipe:
        return []
    box = recipe['receiving_bin']
    center = np.asarray(box['center_xy_m'], dtype=float)
    size = np.asarray(box['inner_size_xy_m'], dtype=float)
    h, w, z = (float(box[n]) for n in ('height_m','wall_thickness_m','bottom_z_m'))
    if (center.shape != (2,) or size.shape != (2,) or
            not np.isfinite(np.r_[center,size,h,w,z]).all() or min(*size,h,w) <= 0 or w >= h):
        raise ValueError('INVALID_RECEIVING_BIN')
    x,y = center
    a,b = size
    return [('bottom',[a+2*w,b+2*w,w],[x,y,z+w/2]),
            ('front',[w,b+2*w,h],[x+(a+w)/2,y,z+h/2]),
            ('rear',[w,b+2*w,h],[x-(a+w)/2,y,z+h/2]),
            ('left',[a,w,h],[x,y+(b+w)/2,z+h/2]),
            ('right',[a,w,h],[x,y-(b+w)/2,z+h/2])]


def check_dump(samples, recipe):
    """Require a downward-facing pan over the open mouth during the dump hold.

    Projection containment is a geometric zero-horizontal-velocity envelope,
    not a snow friction, ballistic flight or mass-transfer prediction.
    """
    if 'dump' not in recipe:
        return None
    bin_boxes(recipe)  # validate dimensions before checking projections
    box, gate = recipe['receiving_bin'], recipe['dump']
    start,end = gate['hold_seconds']
    transfer_start, transfer_end = gate['transfer_seconds']
    if not 0 <= transfer_start <= start < end <= transfer_end:
        raise ValueError('INVALID_DUMP_WINDOW')
    margin, clearance = gate['mouth_margin_m'], gate['min_rim_clearance_m']
    tilt = gate['min_pan_tilt_deg']
    if not np.isfinite([start,end,transfer_start,transfer_end,margin,clearance,tilt]).all() or not (margin > 0 and clearance > 0 and 90 < tilt < 180):
        raise ValueError('INVALID_DUMP_ACCEPTANCE')
    center, half = np.asarray(box['center_xy_m']), np.asarray(box['inner_size_xy_m'])/2
    corner = pan_corners(recipe)
    in_transfer = [s for s in samples if transfer_start <= s['t'] <= transfer_end]
    hold = [s for s in samples if start <= s['t'] <= end]
    if (len(hold)<2 or len(in_transfer)<2 or hold[0]['t']>start+1e-6 or hold[-1]['t']<end-1e-6 or
            in_transfer[0]['t']>transfer_start+1e-6 or in_transfer[-1]['t']<transfer_end-1e-6):
        raise ValueError('MISSING_DUMP_SAMPLES')
    poses = np.asarray([s['tool'] for s in in_transfer])
    held = np.asarray([s['tool'] for s in hold])
    if not np.isfinite(poses).all() or not np.isfinite(held).all():
        raise ValueError('NONFINITE_DUMP_POSE')
    points = np.einsum('nij,kj->nki',poses,corner)
    mouth_margin = float(np.min(half - np.abs(points[:,:,:2]-center)))
    rim_clearance = float(points[:,:,2].min()-(box['bottom_z_m']+box['height_m']))
    min_tilt = float(np.rad2deg(np.arccos(np.clip(held[:,2,2],-1.,1.))).min())
    if mouth_margin < margin:
        raise ValueError('DUMP_MISSES_BIN')
    if rim_clearance < clearance:
        raise ValueError('DUMP_RIM_CLEARANCE')
    if min_tilt < tilt:
        raise ValueError('DUMP_PAN_NOT_INVERTED')
    return {'hold_samples':len(hold),'transfer_samples':len(in_transfer),
            'min_pan_tilt_deg':min_tilt,'min_mouth_margin_m':mouth_margin,
            'min_rim_clearance_m':rim_clearance,
            'scope':'pan envelope above bin; no snow or contact dynamics'}


def make_collision_checker(urdf, recipe, conservative_convex=False):
    """Discrete CAD and tool clearance; only bounded grip bands allow contact.

    The ground is expressed in the fixed URDF base frame and cross-checked
    against configured supporting links. Missing meshes require explicit
    conservative proxies. This is not a contact/dynamics or continuous check.
    """
    import coal as fcl
    import hashlib
    import tempfile
    import xml.etree.ElementTree as ET
    from pathlib import Path
    import pinocchio as pin
    from .bimanual import BimanualQP

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
    # Outer convex hulls contain every mesh triangle. Used only for online
    # tool/body and bin queries; mesh self-collision pairs stay unchanged.
    query_shapes = []
    for obj in geometry.geometryObjects:
        shape=obj.geometry
        if conservative_convex and hasattr(shape,'vertices'):
            from scipy.spatial import ConvexHull
            vertices=np.asarray(shape.vertices())
            hull=ConvexHull(vertices)
            remap={old:new for new,old in enumerate(hull.vertices)}
            points=fcl.StdVec_Vec3s()
            for v in vertices[hull.vertices]: points.append(v)
            triangles=fcl.StdVec_Triangle()
            for face,equation in zip(hull.simplices,hull.equations):
                a,b,c=face
                if np.cross(vertices[b]-vertices[a],vertices[c]-vertices[a]) @ equation[:3] < 0:
                    b,c=c,b
                triangles.append(fcl.Triangle(remap[a],remap[b],remap[c]))
            shape=fcl.Convex(points,triangles)
        query_shapes.append(shape)
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
    index = {n: model.joints[model.getJointId(n)].idx_q for n in model.names[1:]}
    shaft_axis = np.asarray(recipe['shaft_axis_in_tool'], dtype=float)
    ends = np.asarray(recipe['shaft_from_left_grip_m'], dtype=float)
    radius = float(recipe['shaft_radius_m'])
    clearance_limit = float(recipe['acceptance']['min_tool_robot_clearance_m'])
    ground = recipe['ground']
    ground_z = float(ground['z_in_base_m'])
    floor_clearance = float(recipe['acceptance']['min_ground_clearance_m'])
    if not np.isfinite(floor_clearance) or floor_clearance < 0:
        raise ValueError('INVALID_GROUND_CLEARANCE')
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
    bin_parts = [(name, fcl.Box(*size), fcl.Transform3s(np.eye(3), np.array(center)))
                 for name, size, center in bin_boxes(recipe)]
    bin_limit = recipe.get('receiving_bin', {}).get('min_clearance_m', .02)
    if not np.isfinite(bin_limit) or bin_limit <= 0:
        raise ValueError('INVALID_BIN_CLEARANCE')
    full_shaft = fcl.Capsule(radius, ends[1]-ends[0])
    shaft_center = np.array(recipe['left_grip_in_tool_m'])+shaft_axis*sum(ends)/2
    # AABB distance is a conservative lower bound. Skip an exact mesh query
    # only when it cannot violate the gate or reduce the recorded global minimum.
    bounds = {}
    def world_bounds(shape, rotation, translation):
        key = id(shape)
        if key not in bounds:
            shape.computeLocalAABB()
            low, high = np.array(shape.aabb_local.min_), np.array(shape.aabb_local.max_)
            bounds[key] = (shape, (low+high)/2, (high-low)/2)
        _, center, half = bounds[key]
        return rotation @ center+translation, np.abs(rotation) @ half
    def check(samples, collect_failures=False):
        if not samples:
            raise ValueError('EMPTY_COLLISION_SAMPLES')
        bin_minimum, bin_worst = float('inf'), None
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
            corners = (tool @ pan_corners(recipe).T).T
            shaft_points = np.array(recipe['left_grip_in_tool_m']) + ends[:,None]*shaft_axis
            # The real/visual shaft is a flat-ended cylinder. A capsule's
            # artificial end sphere would extend below the shovel attachment.
            axis_z = float((tool[:3,:3] @ shaft_axis)[2])
            radial_height = radius*np.sqrt(max(0.,1.-axis_z*axis_z))
            shaft_heights = (shaft_points @ tool[:3,:3].T + tool[:3,3])[:,2] - radial_height
            ground_failure = min(corners[:,2].min(),shaft_heights.min()) < ground_z+floor_clearance
            pan_pose = fcl.Transform3s(tool[:3, :3], tool[:3, 3]+tool[:3, :3] @ np.array(recipe['pan_center_m']))
            # The same body poses are checked against the tool and all five
            # bin walls. Compute their world bounds once per measured pose.
            bodies = [(name,shape,fcl.Transform3s(place.rotation,place.translation),
                       world_bounds(shape,place.rotation,place.translation))
                      for name,shape,place in zip(names,query_shapes,geometry_data.oMg)]
            pan_bounds = world_bounds(pan,pan_pose.getRotation(),pan_pose.getTranslation())
            for name, body_shape, body_pose, (body_center, body_half) in bodies:
                parts = [('PAN', pan, pan_pose)]
                parts += [('SHAFT', shape, fcl.Transform3s(tool[:3, :3] @ rotation,
                           tool[:3, 3]+tool[:3, :3] @ center)) for shape, center in segments[name]]
                for part, shape, pose in parts:
                    part_center, part_half = pan_bounds if part == 'PAN' else world_bounds(shape, pose.getRotation(), pose.getTranslation())
                    lower_bound = np.linalg.norm(np.maximum(np.abs(part_center-body_center)-part_half-body_half, 0.))-1e-9
                    if lower_bound >= max(clearance_limit, minimum):
                        bounded_queries += 1
                        continue
                    exact_queries += 1
                    result = fcl.DistanceResult()
                    distance = float(fcl.distance(shape, pose, body_shape, body_pose, distance_request, result))
                    if not np.isfinite(distance):
                        raise ValueError('NONFINITE_COLLISION_DISTANCE')
                    if distance < minimum:
                        minimum = distance
                        worst = {'part': part, 'link': name, 't': sample['t']}
                    if distance < clearance_limit and failure is None:
                        failure = f'{part}_CLEARANCE: {name} distance={distance:.6g}'
            # No grip exemptions apply to the receiving bin. Check every body mesh,
            # the full shaft and the pan against its four walls and floor.
            shaft_pose = fcl.Transform3s(tool[:3,:3] @ rotation,tool[:3,3]+tool[:3,:3] @ shaft_center)
            scene_parts = bodies + [('PAN',pan,pan_pose,pan_bounds),
                ('SHAFT',full_shaft,shaft_pose,world_bounds(full_shaft,shaft_pose.getRotation(),shaft_pose.getTranslation()))]
            for wall,box,box_pose in bin_parts:
                c1,h1 = world_bounds(box,box_pose.getRotation(),box_pose.getTranslation())
                for name,shape,pose,(c2,h2) in scene_parts:
                    bound = np.linalg.norm(np.maximum(np.abs(c1-c2)-h1-h2,0.))-1e-9
                    if bound >= max(bin_limit,bin_minimum):
                        continue
                    distance = float(fcl.distance(shape,pose,box,box_pose,distance_request,fcl.DistanceResult()))
                    if not np.isfinite(distance):
                        raise ValueError('NONFINITE_BIN_DISTANCE')
                    if distance < bin_minimum:
                        bin_minimum,bin_worst = distance, {'wall':wall,'part':name,'t':sample['t']}
                    if distance < bin_limit and failure is None:
                        failure = f'BIN_CLEARANCE: {wall} / {name} distance={distance:.6g}'
            failure = failure or ('TOOL_BELOW_GROUND' if ground_failure else None)
            if failure:
                if not collect_failures:
                    raise ValueError(f"{failure} t={sample['t']}")
                failures.append({'sample': sample['t'], 'reason': failure})
        return {'samples': len(samples), 'tool_body_representation':'convex_outer' if conservative_convex else 'cad_mesh',
                'mesh_pairs': len(geometry.collisionPairs),
                'excluded_tree_distance': 2, 'shaft_contacts': contacts,
                'empty_mesh_proxies': proxies, 'support_heights_m': support_heights,
                'min_tool_robot_clearance_m': minimum, 'closest_tool_robot_pair': worst,
                'exact_distance_queries': exact_queries, 'aabb_bounded_queries': bounded_queries,
                'min_bin_clearance_m': bin_minimum if bin_parts else None, 'closest_bin_pair': bin_worst,
                'source_sha256': asset_hashes, 'violations': len(failures), 'failures': failures}
    return check


def collision_check(urdf, samples, recipe, collect_failures=False):
    return make_collision_checker(urdf,recipe)(samples,collect_failures)
