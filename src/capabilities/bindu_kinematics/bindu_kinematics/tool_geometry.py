"""Measured tracking/coupling checks and configured tool/bin geometry.

All positions are in the fixed robot base frame. The bin is an open box, not a
solid obstacle across its mouth. Robot geometry dependencies are loaded lazily.
"""
import numpy as np


class GeometryBudgetExceeded(TimeoutError):
    """A candidate search ran out of time; not proof of safe geometry."""


class InitialSelfCollisionRecovery:
    """Bound a shallow initial overlap; every existing contact must retreat."""
    def __init__(self, config):
        bounds={'max_penetration_m':(0.,.001),'joint_sample_step_rad':(0.,.002),
                'max_exit_joint_travel_rad':(0.,.03),'max_joint_speed_rad_s':(0.,.1)}
        if not isinstance(config,dict) or set(config)!=set(bounds):
            raise ValueError('PREPARE_RECOVERY_CONFIG')
        for key,(low,high) in bounds.items():
            value=config[key]
            if type(value) not in (float,int) or not np.isfinite(value) or not low<value<=high:
                raise ValueError('PREPARE_RECOVERY_CONFIG:'+key)
        if config['joint_sample_step_rad']>config['max_exit_joint_travel_rad']:
            raise ValueError('PREPARE_RECOVERY_CONFIG:joint_sample_step_rad')
        self.config=dict(config)
        self.initial=None;self.previous=None;self.initial_contacts={};self.contacts={}
        self.cleared_at=None

    def observe(self, joints, contacts):
        if any(not np.isfinite(v) or v<0 for v in contacts.values()):
            raise ValueError('PREPARE_RECOVERY_NONFINITE_CONTACT')
        if self.initial is None:
            if any(v>self.config['max_penetration_m'] for v in contacts.values()):
                raise ValueError('PREPARE_RECOVERY_PENETRATION_LIMIT')
            self.initial=dict(joints);self.initial_contacts=dict(contacts)
        else:
            if max(abs(joints[n]-self.previous[n]) for n in joints)>self.config['joint_sample_step_rad']+1e-10:
                raise ValueError('PREPARE_RECOVERY_SAMPLE_STEP')
            if not set(contacts)<=set(self.contacts):
                raise ValueError('PREPARE_RECOVERY_NEW_CONTACT')
            if any(v>self.contacts[pair]+1e-9 for pair,v in contacts.items()):
                raise ValueError('PREPARE_RECOVERY_DEEPENING')
        travel=max(abs(joints[n]-self.initial[n]) for n in joints)
        if contacts and travel>self.config['max_exit_joint_travel_rad']+1e-10:
            raise ValueError('PREPARE_RECOVERY_EXIT_NOT_CLEAR')
        if not contacts and self.cleared_at is None:self.cleared_at=travel
        self.previous=dict(joints);self.contacts=dict(contacts)

    def report(self):
        if self.contacts:raise ValueError('PREPARE_RECOVERY_EXIT_NOT_CLEAR')
        return dict(self.config,initial_contacts={str(k):v for k,v in self.initial_contacts.items()},
                    cleared_at_joint_travel_rad=self.cleared_at) if self.initial_contacts else None


def tracking_acceptance(recipe, position_m, rotation_rad):
    """Keep task tracking quality separate from relative-grasp safety."""
    gate=recipe['acceptance'];policy=gate.get('tool_tracking_policy','required')
    values=np.asarray([position_m,rotation_rad,gate['max_target_lag_m'],gate['max_target_rotation_error_rad']])
    if policy not in ('required','report_only') or not np.isfinite(values).all() or np.any(values<0):
        raise ValueError('INVALID_TOOL_TRACKING_ACCEPTANCE')
    within=bool(position_m<=gate['max_target_lag_m'] and rotation_rad<=gate['max_target_rotation_error_rad'])
    return dict(policy=policy,within_reference=within,passed=within or policy=='report_only',
                reference_position_m=gate['max_target_lag_m'],reference_rotation_rad=gate['max_target_rotation_error_rad'])


def coupling_metrics(samples, config, captures):
    """Measured grasp-point/axis errors; re-lock captures the current angle."""
    from bisect import bisect_right
    from .bimanual import BimanualQP
    transform=BimanualQP._transform
    if not samples or not captures:raise ValueError('EMPTY_COUPLING_TRACE')
    times=[c['t'] for c in captures]
    if (not np.isfinite(times).all() or any(b<=a for a,b in zip(times,times[1:])) or
            any(c['mode'] not in ('rigid','right_axial') for c in captures)):
        raise ValueError('INVALID_COUPLING_CAPTURES')
    mount=transform(config['left_to_tool']);right_mount=transform(config['right_to_tool'])
    axis=np.asarray(config['right_axial_grasp']['axis_in_tool'])
    point=np.asarray(config['right_axial_grasp']['grip_in_right_flange_m'])
    if axis.shape!=(3,) or point.shape!=(3,) or not np.isfinite(np.r_[axis,point]).all() or not np.isclose(np.linalg.norm(axis),1.):
        raise ValueError('INVALID_COUPLING_GEOMETRY')
    nominal_point=(np.linalg.inv(right_mount) @ np.r_[point,1.])[:3]
    right_axis=right_mount[:3,:3] @ axis
    position_limit=config['demo']['acceptance']['max_grip_error_m']
    rotation_limit=config['grasp_rotation_tolerance_rad']
    if not np.isfinite([position_limit,rotation_limit]).all() or min(position_limit,rotation_limit)<=0:
        raise ValueError('INVALID_COUPLING_LIMITS')
    modes={};observed=set();last_capture=-1;previous=float('-inf');reference=None;count=0
    for sample in samples:
        t=sample['t']
        if not np.isfinite(t) or t<previous:raise ValueError('INVALID_COUPLING_TIME')
        previous=t;i=bisect_right(times,t)-1
        if i<0:continue
        left=transform(sample['flanges'][config['left_frame']]);right=transform(sample['flanges'][config['right_frame']])
        relative=np.linalg.inv(left) @ right
        if i!=last_capture:reference=relative.copy();last_capture=i
        observed.add(i)
        tool=left @ mount
        position=float(np.linalg.norm((np.linalg.inv(tool) @ right @ np.r_[point,1.])[:3]-nominal_point))
        tilt=float(np.arccos(np.clip((tool[:3,:3] @ axis) @ (right[:3,:3] @ right_axis),-1.,1.)))
        mode=captures[i]['mode']
        metric=modes.setdefault(mode,dict(samples=0,max_grip_position_m=0.,max_shaft_axis_rad=0.,
            max_rigid_rotation_rad=0.,max_rigid_flange_translation_m=0.))
        metric['samples']+=1;count+=1
        metric['max_grip_position_m']=max(metric['max_grip_position_m'],position)
        metric['max_shaft_axis_rad']=max(metric['max_shaft_axis_rad'],tilt)
        if mode=='rigid':
            angle=float(np.arccos(np.clip((np.trace(reference[:3,:3].T @ relative[:3,:3])-1.)/2.,-1.,1.)))
            metric['max_rigid_rotation_rad']=max(metric['max_rigid_rotation_rad'],angle)
            metric['max_rigid_flange_translation_m']=max(metric['max_rigid_flange_translation_m'],
                float(np.linalg.norm(relative[:3,3]-reference[:3,3])))
    if not count:raise ValueError('EMPTY_COUPLING_MATCHES')
    if observed!=set(range(len(captures))):raise ValueError('MISSING_COUPLING_CAPTURE_SAMPLES')
    passed=all(m['max_grip_position_m']<=position_limit and m['max_shaft_axis_rad']<=rotation_limit and
               m['max_rigid_rotation_rad']<=rotation_limit for m in modes.values())
    return dict(passed=passed,samples=count,modes=modes,grip_position_limit_m=position_limit,
                rotation_limit_rad=rotation_limit,reference='first measured sample at each confirmed capture',
                axial_roll='free; rigid re-lock captures current rotation')


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


def make_collision_checker(urdf, recipe, conservative_convex=False, *, tool_present=True,
                           validation_only=False):
    """Discrete CAD and tool clearance; only bounded grip bands allow contact.

    The ground is expressed in the fixed URDF base frame and cross-checked
    against configured supporting links. Missing meshes require explicit
    conservative proxies. This is not a contact/dynamics or continuous check.
    """
    if type(tool_present) is not bool or type(validation_only) is not bool:
        raise ValueError('INVALID_TOOL_PRESENCE')
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
    check_grasp_contacts = recipe.get('check_grasp_contacts', False)
    grasp_penetration = float(recipe.get('max_grasp_penetration_m', 0.0005))
    # Contact meshes have their own radial modelling tolerance. It must stay
    # inside the rod, and never exempts other body/tool collision pairs.
    if (type(check_grasp_contacts) is not bool or not np.isfinite(grasp_penetration) or
            not 0 <= grasp_penetration <= .005 or grasp_penetration >= float(recipe['shaft_radius_m'])):
        raise ValueError('INVALID_GRASP_CONTACT_CHECK')
    contact_names = set(recipe['shaft_contacts'])
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
    ground_policy=recipe['acceptance'].get('ground_policy','required')
    geometry_kind=recipe.get('geometry_kind','shovel')
    if geometry_kind not in ('shovel','rod'):
        raise ValueError('INVALID_TOOL_GEOMETRY_KIND')
    has_pan=geometry_kind=='shovel'
    if ground_policy not in ('required','report_only','disabled'):
        raise ValueError('INVALID_GROUND_POLICY')
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
    def shaft_queries(contact_bands):
        segments = {}
        segment_cache = {}
        for name in set(names):
            spans = [tuple(ends)]
            if name in contact_bands:
                center, half_length = contact_bands[name]['center_m'], contact_bands[name]['half_length_m']
                if (not np.isfinite([center, half_length]).all() or half_length <= 0 or
                        center-half_length < ends[0]-1e-9 or center+half_length > ends[1]+1e-9):
                    raise ValueError('INVALID_SHAFT_CONTACT_BAND')
                # Cylinder cut faces bound each allowed contact band precisely;
                # unlike skipping a hand mesh this still checks the rest of the rod.
                if not check_grasp_contacts:
                    spans = [(ends[0], max(ends[0],center-half_length)),
                             (min(ends[1],center+half_length), ends[1])]
            segments[name] = []
            for a, b in spans:
                if b-a<=1e-12:continue  # Contact bands may meet either physical end.
                key = (a, b, name in contact_bands)
                if key not in segment_cache:
                    query_radius = radius-grasp_penetration if check_grasp_contacts and name in contact_bands else radius
                    segment_cache[key] = (fcl.Cylinder(query_radius, b-a) if name in contact_bands or geometry_kind=='rod' else fcl.Capsule(radius, b-a),
                        np.array(recipe['left_grip_in_tool_m'])+shaft_axis*(a+b)/2)
                segments[name].append(segment_cache[key])
        tool_parts = ([('PAN', pan, np.asarray(recipe['pan_center_m']))] if has_pan else []) + [
            ('SHAFT', shape, center) for shape, center in segment_cache.values()]
        tool_ids = {id(shape): i for i, (_, shape, _) in enumerate(tool_parts)}
        body_queries = [(i, j) for i, name in enumerate(names)
                        for j in ([0] if has_pan else [])+[tool_ids[id(shape)] for shape, _ in segments[name]]]
        body_indices, part_indices = np.array(body_queries).T
        return tool_parts, body_queries, body_indices, part_indices
    pan = fcl.Box(*recipe['pan_size_m'])
    bin_parts = [(name, fcl.Box(*size), fcl.Transform3s(np.eye(3), np.array(center)))
                 for name, size, center in bin_boxes(recipe)]
    bin_limit = recipe.get('receiving_bin', {}).get('min_clearance_m', .02)
    if not np.isfinite(bin_limit) or bin_limit <= 0:
        raise ValueError('INVALID_BIN_CLEARANCE')
    full_shaft = (fcl.Cylinder if geometry_kind=='rod' else fcl.Capsule)(radius, ends[1]-ends[0])
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
    # Share shaft pieces and query layout until the next measured grasp capture.
    tool_parts, body_queries, body_indices, part_indices = shaft_queries(contacts)
    corners_local = pan_corners(recipe)
    shaft_points = np.array(recipe['left_grip_in_tool_m']) + ends[:, None]*shaft_axis
    bin_bounds = [world_bounds(box, pose.getRotation(), pose.getTranslation())
                  for _, box, pose in bin_parts]

    from collections import OrderedDict
    validated=OrderedDict()

    def set_contact_centers(centers):
        nonlocal contacts, tool_parts, body_queries, body_indices, part_indices
        if not set(centers).issubset(contacts):
            raise ValueError('UNKNOWN_SHAFT_CONTACT_LINK')
        updated={name:dict(band,center_m=centers.get(name,band['center_m']))
                 for name,band in contacts.items()}
        layout=shaft_queries(updated)  # Validate before changing the active layout.
        for _,shape,_ in tool_parts:
            bounds.pop(id(shape),None)
        contacts=updated
        tool_parts,body_queries,body_indices,part_indices=layout
        validated.clear()  # Identical poses must be rechecked against the new bands.

    def check(samples, collect_failures=False, *, deadline=None, initial_self_collision_recovery=None):
        recovery=(InitialSelfCollisionRecovery(initial_self_collision_recovery)
                  if initial_self_collision_recovery is not None else None)
        if recovery is not None and tool_present:
            raise ValueError('PREPARE_RECOVERY_REQUIRES_UNLOADED_PATH')
        fast=validation_only and not collect_failures and recovery is None
        if not samples:
            raise ValueError('EMPTY_COLLISION_SAMPLES')
        bin_minimum, bin_worst = float('inf'), None
        failures, worst = [], None
        exact_queries, bounded_queries = 0, 0
        minimum = float('inf')
        grasp_minimum = float('inf')
        support_heights = {}
        ground_minimum,ground_warnings=float('inf'),0
        distance_request = fcl.DistanceRequest()
        intersection_request = fcl.CollisionRequest()
        intersection_request.enable_contact = False
        # Only intersection is consumed here. GJK can stop after proving
        # positive separation; full closest-point refinement on every CAD
        # triangle is unnecessary. The inner cylinder/tolerance is unchanged.
        intersection_request.distance_upper_bound = 0.
        if recovery is not None:
            recovery_request=fcl.CollisionRequest()
            recovery_request.enable_contact=True;recovery_request.num_max_contacts=100
        for sample_index, sample in enumerate(samples):
            if set(sample['q']) != set(index):
                raise ValueError('COLLISION_JOINT_LAYOUT')
            q = pin.neutral(model)
            for name, value in sample['q'].items():
                q[index[name]] = value
            tool = BimanualQP._transform(sample['tool'])
            if not np.isfinite(q).all() or not np.isfinite(sample['t']):
                raise ValueError('NONFINITE_COLLISION_SAMPLE')
            # Exact full-state keys, scoped to this immutable CAD context.
            # No rounding, TTL extension, or reuse across model/config changes.
            key=q.tobytes()+tool.tobytes()
            if fast and key in validated:
                validated.move_to_end(key)
                continue
            if deadline is not None:
                import time
                if time.perf_counter() >= deadline:
                    raise GeometryBudgetExceeded('GEOMETRY_VALIDATION_BUDGET')
            pin.computeCollisions(model, data, geometry, geometry_data, q, False)
            failure = None
            initial_contacts={}
            for pair, result in zip(geometry.collisionPairs, geometry_data.collisionResults):
                if result.isCollision():
                    if recovery is None:
                        failure = f'SELF_COLLISION: {names[pair.first]} / {names[pair.second]}'
                        break
                    first,second=pair.first,pair.second
                    detail=fcl.CollisionResult()
                    fcl.collide(geometry.geometryObjects[first].geometry,
                        fcl.Transform3s(geometry_data.oMg[first].rotation,geometry_data.oMg[first].translation),
                        geometry.geometryObjects[second].geometry,
                        fcl.Transform3s(geometry_data.oMg[second].rotation,geometry_data.oMg[second].translation),
                        recovery_request,detail)
                    if not 0<detail.numContacts()<100:
                        raise ValueError('PREPARE_RECOVERY_CONTACT_UNRESOLVED')
                    initial_contacts[(names[first],names[second])]=max(abs(c.penetration_depth) for c in detail.getContacts())
            if recovery is not None:recovery.observe(sample['q'],initial_contacts)
            if fast and failure:
                raise ValueError(f"{failure} t={sample['t']}")
            if sample_index == 0:
                for name, obj, placement in zip(names, geometry.geometryObjects, geometry_data.oMg):
                    if name in ground['support_links']:
                        vertices = np.asarray(obj.geometry.vertices())
                        height = float(np.min((vertices @ placement.rotation.T + placement.translation)[:, 2]))
                        support_heights[name] = min(height, support_heights.get(name, float('inf')))
                if (set(support_heights) != set(ground['support_links']) or
                        any(abs(height-ground_z) > support_tolerance for height in support_heights.values())):
                    raise ValueError('GROUND_SUPPORT_MISMATCH')
            if fast and not tool_present and not bin_parts:
                # Self-collision and support checks above are the complete
                # geometry contract for this empty scene. Do not construct
                # tool/body distance bounds that no query will consume.
                validated[key]=True
                if len(validated)>256:validated.popitem(last=False)
                continue
            corners = (tool @ corners_local.T).T
            # The real/visual shaft is a flat-ended cylinder. A capsule's
            # artificial end sphere would extend below the shovel attachment.
            axis_z = float((tool[:3,:3] @ shaft_axis)[2])
            radial_height = radius*np.sqrt(max(0.,1.-axis_z*axis_z))
            shaft_heights = (shaft_points @ tool[:3,:3].T + tool[:3,3])[:,2] - radial_height
            tool_clearance=float(min(corners[:,2].min() if has_pan else float('inf'),shaft_heights.min())-ground_z)
            if tool_present:
                ground_minimum=min(ground_minimum,tool_clearance)
                ground_warnings+=int(tool_clearance<floor_clearance and ground_policy!='disabled')
            ground_failure = tool_present and tool_clearance<floor_clearance and ground_policy=='required'
            pan_pose = fcl.Transform3s(tool[:3, :3], tool[:3, 3]+tool[:3, :3] @ np.array(recipe['pan_center_m']))
            # The same body poses are checked against the tool and all five
            # bin walls. Compute their world bounds once per measured pose.
            bodies = [(name,shape,fcl.Transform3s(place.rotation,place.translation),
                       world_bounds(shape,place.rotation,place.translation))
                      for name,shape,place in zip(names,query_shapes,geometry_data.oMg)]
            parts = [(part, shape, pan_pose if part == 'PAN' else fcl.Transform3s(
                        tool[:3, :3] @ rotation, tool[:3, 3]+tool[:3, :3] @ center))
                     for part, shape, center in tool_parts]
            part_bounds = [world_bounds(shape, pose.getRotation(), pose.getTranslation())
                           for _, shape, pose in parts]
            part_centers, part_halves = np.asarray(part_bounds).transpose(1, 0, 2)
            body_centers, body_halves = np.asarray([body[3] for body in bodies]).transpose(1, 0, 2)
            lower_bounds = np.linalg.norm(np.maximum(
                np.abs(part_centers[part_indices]-body_centers[body_indices])-
                part_halves[part_indices]-body_halves[body_indices], 0.), axis=1)-1e-9
            for (body_index, part_index), lower_bound in (zip(body_queries, lower_bounds) if tool_present else ()):
                name = bodies[body_index][0]
                grasp_query = check_grasp_contacts and name in contacts and parts[part_index][0] == 'SHAFT'
                limit = -grasp_penetration if grasp_query else clearance_limit
                current_minimum = grasp_minimum if grasp_query else minimum
                if lower_bound >= max(limit, current_minimum):
                    bounded_queries += 1
                    continue
                exact_queries += 1
                name, body_shape, body_pose, _ = bodies[body_index]
                part, shape, pose = parts[part_index]
                if grasp_query:
                    body_shape = geometry.geometryObjects[body_index].geometry
                    # Mesh signed-distance depth depends on which triangle is
                    # returned and can miss a palm intersecting the rod core.
                    # An inner cylinder robustly rejects overlap deeper than
                    # the configured radial contact tolerance, without a band
                    # exemption or a convex hull sealing the jaw opening.
                    result = fcl.CollisionResult()
                    fcl.collide(shape,pose,body_shape,body_pose,intersection_request,result)
                    distance = -grasp_penetration-1e-9 if result.isCollision() else 0.
                else:
                    result = fcl.DistanceResult()
                    distance = float(fcl.distance(shape, pose, body_shape, body_pose, distance_request, result))
                if not np.isfinite(distance):
                    raise ValueError('NONFINITE_COLLISION_DISTANCE')
                if grasp_query:
                    grasp_minimum = min(grasp_minimum, distance)
                elif distance < minimum:
                    minimum = distance
                    worst = {'part': part, 'link': name, 't': sample['t']}
                if distance < limit and failure is None:
                    kind = 'GRASP_PENETRATION' if grasp_query else part+'_CLEARANCE'
                    failure = (f'{kind}: {name} inner_cylinder_intersection=true tolerance_m={grasp_penetration:.6g}'
                               if grasp_query else f'{kind}: {name} distance={distance:.6g}')
                    if fast:
                        raise ValueError(f"{failure} t={sample['t']}")
            # No grip exemptions apply to the receiving bin. Check every body mesh,
            # the full shaft and the pan against its four walls and floor.
            shaft_pose = fcl.Transform3s(tool[:3,:3] @ rotation,tool[:3,3]+tool[:3,:3] @ shaft_center)
            scene_parts = bodies + ([('PAN',pan,pan_pose,part_bounds[0])] if has_pan else []) + [
                ('SHAFT',full_shaft,shaft_pose,world_bounds(full_shaft,shaft_pose.getRotation(),shaft_pose.getTranslation()))]
            if not tool_present: scene_parts = bodies
            if bin_parts:
                scene_centers, scene_halves = np.asarray([part[3] for part in scene_parts]).transpose(1, 0, 2)
                wall_centers, wall_halves = np.asarray(bin_bounds).transpose(1, 0, 2)
                bin_lower_bounds = np.linalg.norm(np.maximum(
                    np.abs(wall_centers[:, None]-scene_centers)-
                    wall_halves[:, None]-scene_halves, 0.), axis=2)-1e-9
                for (wall, box, box_pose), lower_bounds in zip(bin_parts, bin_lower_bounds):
                    for (name, shape, pose, _), bound in zip(scene_parts, lower_bounds):
                        if bound >= max(bin_limit, bin_minimum):
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
            elif fast:
                validated[key]=True
                if len(validated)>256:validated.popitem(last=False)
        recovery_report=recovery.report() if recovery is not None else None
        if fast:
            return {'samples':len(samples),'validation_only':True}
        return {'samples': len(samples), 'geometry_kind':geometry_kind,
                'self_collision_recovery':recovery_report,
                'tool_body_representation':'convex_outer' if conservative_convex else 'cad_mesh',
                'mesh_pairs': len(geometry.collisionPairs),
                'excluded_tree_distance': 2, 'shaft_contacts': contacts,
                'grasp_contact_checked': check_grasp_contacts,
                'grasp_contact_test': 'inner_cylinder_mesh_intersection' if check_grasp_contacts else 'legacy_band_exemption',
                'grasp_penetration_limit_m': grasp_penetration if check_grasp_contacts else None,
                'empty_mesh_proxies': proxies, 'support_heights_m': support_heights,
                'ground_policy':ground_policy,'ground_warning_samples':ground_warnings,
                'min_tool_ground_clearance_m':ground_minimum if tool_present else None,
                'min_tool_robot_clearance_m': minimum, 'closest_tool_robot_pair': worst,
                'exact_distance_queries': exact_queries, 'aabb_bounded_queries': bounded_queries,
                'min_bin_clearance_m': bin_minimum if bin_parts else None, 'closest_bin_pair': bin_worst,
                'source_sha256': asset_hashes, 'violations': len(failures), 'failures': failures}
    check.set_contact_centers=set_contact_centers
    check.validation_only=validation_only
    return check


def collision_check(urdf, samples, recipe, collect_failures=False):
    return make_collision_checker(urdf,recipe)(samples,collect_failures)
