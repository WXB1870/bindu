"""Wooden rigid shovel and compliant grasp math, independent of Isaac/ROS.

All poses are SE(3), metres/radians; wrenches are force then moment in world.
The tray uses disjoint bottom, sides and back, not its solid CAD envelope.
Wood density changes mass/inertia; this model does not simulate wood flexure.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


class MeasuredJointVelocity:
    """Velocity of measured encoder positions at their acquisition timestamps.

    PhysX instantaneous DOF velocity can disagree with stabilized position
    increments. No commands or planned positions enter this measurement model.
    Raw native velocity remains in the experiment archive for comparison.
    """
    def __init__(self):self.previous=None

    def update(self,stamp,positions,initial_velocity):
        positions=np.asarray(positions,dtype=float)
        if not np.isfinite(np.r_[stamp,positions]).all():raise ValueError('ENCODER_NONFINITE')
        if self.previous is None:velocity=np.asarray(initial_velocity,dtype=float)
        else:
            previous_stamp,previous=self.previous;dt=stamp-previous_stamp
            if not 0 < dt <= .1 or positions.shape!=previous.shape:raise ValueError('ENCODER_TIME_OR_LAYOUT')
            velocity=(positions-previous)/dt
        if velocity.shape!=positions.shape or not np.isfinite(velocity).all():raise ValueError('ENCODER_VELOCITY')
        self.previous=(stamp,positions.copy())
        return velocity


def load_motion(path):
    path = Path(path).resolve()
    motion = json.loads(path.read_text())
    if motion.get('schema') != 2 or not motion.get('samples'):
        raise ValueError('INVALID_PHYSICS_MOTION')
    for name, digest in motion['source_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError('MOTION_SOURCE_IDENTITY:'+name)
    for source, digest in ((motion['display_urdf'], motion['display_sha256']),
                           (motion['config_path'], motion['config_sha256']),
                           (motion['tool_input_path'], motion['tool_input_sha256'])):
        if hashlib.sha256(Path(source).read_bytes()).hexdigest() != digest:
            raise ValueError('MOTION_SOURCE_IDENTITY:'+source)
    times = np.array([s['t'] for s in motion['samples']])
    names = list(motion['samples'][0]['q'])
    q = np.array([[s['q'][n] for n in names] for s in motion['samples']])
    if (not np.isfinite(times).all() or not np.isfinite(q).all() or times[0] != 0 or
            np.any(np.diff(times) <= 0) or abs(times[-1]-motion['duration_s']) > 1e-6):
        raise ValueError('INVALID_PHYSICS_TIMELINE')
    return motion


def wooden_parts(recipe, material):
    density = float(material['density_kg_m3'])
    wall = float(material['tray_wall_m'])
    size = np.asarray(recipe['pan_size_m'], dtype=float)
    center = np.asarray(recipe['pan_center_m'], dtype=float)
    radius = float(recipe['shaft_radius_m'])
    ends = np.asarray(recipe['shaft_from_left_grip_m'], dtype=float)
    axis = np.asarray(recipe['shaft_axis_in_tool'], dtype=float)
    left = np.asarray(recipe['left_grip_in_tool_m'], dtype=float)
    if (not np.isfinite(np.r_[density, wall, size, center, radius, ends, axis, left]).all() or
            density <= 0 or radius <= 0 or wall <= 0 or np.any(size <= 2*wall) or
            ends.shape != (2,) or ends[1] <= ends[0] or not np.isclose(np.linalg.norm(axis), 1.)):
        raise ValueError('INVALID_WOOD_GEOMETRY')
    reference = np.eye(3)[np.argmin(abs(axis))]
    transverse = np.cross(reference, axis)
    transverse /= np.linalg.norm(transverse)
    basis = np.column_stack((transverse, np.cross(axis, transverse), axis))
    length = ends[1]-ends[0]
    mass = density*np.pi*radius**2*length
    axial = mass*radius**2/2
    radial = mass*(3*radius**2+length**2)/12
    parts = [dict(name='Shaft', kind='cylinder', radius=radius, length=length,
                  rotation=basis, center=left+axis*np.mean(ends), mass=mass,
                  inertia=basis @ np.diag([radial, radial, axial]) @ basis.T)]
    def box(name, dims, offset):
        dims = np.asarray(dims)
        m = density*np.prod(dims)
        inertia = m/12*np.diag([dims[1]**2+dims[2]**2,
                                dims[0]**2+dims[2]**2, dims[0]**2+dims[1]**2])
        parts.append(dict(name=name, kind='box', size=dims, center=center+offset,
                          rotation=np.eye(3), mass=m, inertia=inertia))
    x, y, z = size
    box('Bottom', [x, y, wall], np.array([0., 0., -(z-wall)/2]))
    for sign in (-1, 1):
        box('Side'+str(sign).replace('-', 'N'), [x, wall, z-wall],
            np.array([0., sign*(y-wall)/2, wall/2]))
    box('Back', [wall, y-2*wall, z-wall], np.array([-(x-wall)/2, 0., wall/2]))
    return parts


def compound_mass(parts):
    mass = sum(p['mass'] for p in parts)
    com = sum(p['mass']*p['center'] for p in parts)/mass
    inertia = np.zeros((3, 3))
    for p in parts:
        r = p['center']-com
        inertia += p['inertia']+p['mass']*(np.dot(r, r)*np.eye(3)-np.outer(r, r))
    if not np.isfinite(inertia).all() or np.linalg.eigvalsh(inertia).min() <= 0:
        raise ValueError('INVALID_COMPOUND_INERTIA')
    return mass, com, inertia


def grip_wrench(hand, tool, hand_velocity, tool_velocity, hand_com, tool_com,
                hand_offset, tool_offset, rotation_mount, cfg, axial=False):
    """Finite 6D spring; axial right grip transmits zero shaft-axis couple.

    Equal and opposite wrenches act at the same midpoint to conserve total
    momentum even with finite translational deflection. Velocities are COM
    linear/world angular velocities, not velocities of the body origins.
    """
    hp = hand[:3, 3]+hand[:3, :3] @ hand_offset
    tp = tool[:3, 3]+tool[:3, :3] @ tool_offset
    point = (hp+tp)/2
    hv = hand_velocity[:3]+np.cross(hand_velocity[3:], point-hand_com)
    tv = tool_velocity[:3]+np.cross(tool_velocity[3:], point-tool_com)
    force = cfg['grip_translation_stiffness_n_m']*(hp-tp)+cfg['grip_translation_damping_ns_m']*(hv-tv)
    desired = hand[:3, :3] @ rotation_mount
    if axial:
        a, b = tool[:3, :3] @ cfg['shaft_axis_in_tool'], hand[:3, :3] @ rotation_mount @ cfg['shaft_axis_in_tool']
        cross = np.cross(a, b)
        norm = np.linalg.norm(cross)
        if norm < 1e-12 and np.dot(a, b) < 0:
            raise ValueError('ANTIPARALLEL_GRIP_AXIS')
        error = cross*(np.arctan2(norm, np.dot(a, b))/norm) if norm > 1e-12 else np.zeros(3)
        projector = np.eye(3)-np.outer(a, a)
    else:
        error = Rotation.from_matrix(desired @ tool[:3, :3].T).as_rotvec()
        projector = np.eye(3)
    torque = (cfg['grip_rotation_stiffness_nm_rad']*error+
              cfg['grip_rotation_damping_nm_s_rad']*projector @ (hand_velocity[3:]-tool_velocity[3:]))
    return force, torque, point, np.linalg.norm(hp-tp)


def internal_wrench(forces, moments, points, com, released_axis=None):
    """Nullspace of grasp map with moment divided by measured grip spacing.

    The full minimum-norm decomposition depends on this length convention.
    Axial squeeze is also returned: a self-equilibrating equal/opposite force
    along the actual grip line, positive in compression, negative in tension.
    """
    forces, moments, points = map(np.asarray, (forces, moments, points))
    line = points[1]-points[0]
    length = np.linalg.norm(line)
    if not np.isfinite(np.r_[forces.ravel(), moments.ravel(), points.ravel(), com]).all() or length < 1e-6:
        raise ValueError('INVALID_GRASP_WRENCH')
    blocks = [np.block([[np.eye(3), np.zeros((3, 3))],
                        [skew(p-com)/length, np.eye(3)]]) for p in points]
    grasp = np.hstack(blocks)
    scaled = np.concatenate([np.r_[f, m/length] for f, m in zip(forces, moments)])
    external = grasp @ scaled
    basis = np.eye(12)
    if released_axis is not None:
        axis = np.asarray(released_axis,dtype=float)
        if axis.shape != (3,) or not np.isfinite(axis).all() or not np.isclose(np.linalg.norm(axis),1.):
            raise ValueError('INVALID_RELEASED_AXIS')
        if abs(moments[1] @ axis) > 1e-8:
            raise ValueError('RELEASED_AXIS_COUPLE')
        transverse = np.cross(axis,np.eye(3)[np.argmin(abs(axis))]);transverse/=np.linalg.norm(transverse)
        basis = np.zeros((12,11));basis[:9,:9]=np.eye(9)
        basis[9:,9:] = np.column_stack((transverse,np.cross(axis,transverse)))
    constrained = grasp @ basis
    coordinates = basis.T @ scaled
    internal = basis @ (coordinates-constrained.T @ np.linalg.solve(constrained @ constrained.T,external))
    physical = internal.reshape(2, 6).copy()
    physical[:, 3:] *= length
    return dict(wrenches=physical, axial_squeeze_n=float((forces[0]-forces[1]) @ (line/length)/2),
                net_force=external[:3], net_torque=external[3:]*length,
                null_residual=float(np.linalg.norm(grasp @ internal)), length=length)


def analyze(directory):
    """Export engineering figures from checked, same-input simulation runs.

    Chart contract: three time-series panels compare left/right load, axial
    squeeze and grip deflection at every native step (>=48,000 points/run).
    A separate bar chart compares peak axial force for discrete assumptions.
    Static Matplotlib PNGs are local scientific experiment artifacts; blue and
    orange plus solid/dashed lines distinguish hands without color alone.
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory = Path(directory).resolve()
    runs = {}
    for path in sorted(directory.glob('*/result.json')):
        result = json.loads(path.read_text())
        if ('smoke' in path.parent.name or result['status'] != 'passed' or
                result.get('controller_mode','reference_replay') != 'reference_replay'):
            continue
        if hashlib.sha256((path.parent/'samples.npz').read_bytes()).hexdigest() != result['provenance']['samples_sha256']:
            raise ValueError('SAMPLE_IDENTITY:'+str(path))
        runs[result['case']] = (path, result)
    baseline_path, baseline = runs['baseline']
    for _, result in runs.values():
        if (result['provenance']['motion_sha256'] != baseline['provenance']['motion_sha256'] or
                result['duration_s'] != baseline['duration_s']):
            raise ValueError('INCOMPARABLE_PHYSICS_RUNS')
    data = np.load(baseline_path.parent/'samples.npz')['samples']
    data = data[data[:, 0] >= 0]
    t = data[:, 0]
    blue, orange = '#2870A7', '#C57922'
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':10, 'axes.spines.top':False,
                         'axes.spines.right':False, 'axes.labelcolor':'#262626', 'text.color':'#262626'})
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, layout='constrained')
    fig.suptitle('Snowbot wooden shovel: dynamic grasp loads', fontsize=15)
    axes[0].set_title(f"Empty shovel, {baseline['source']['mass_kg']:.3f} kg; PhysX, dt={baseline['physics_dt_s']*1000:g} ms; fixed base", fontsize=10)
    for i, (label, color, style) in enumerate((('Left', blue, '-'), ('Right', orange, '--'))):
        axes[0].plot(t, np.linalg.norm(data[:,1+3*i:4+3*i], axis=1), color=color, ls=style, label=label, lw=1.2)
        axes[2].plot(t, data[:,26+i]*1000, color=color, ls=style, label=label, lw=1.2)
    axes[0].set_ylabel('Grip force magnitude (N)')
    axes[0].plot(t,data[:,94],color='#777777',ls=':',lw=1.,label='Ground/bin normal Z')
    axes[1].plot(t, data[:,25], color=blue, lw=1.2)
    axes[1].axhline(0., color='#777777', lw=.7)
    axes[1].set_ylabel('Axial internal force (N)\n+ compression / - tension')
    axes[1].margins(y=.25)
    axes[2].set_ylabel('Grip deflection (mm)')
    axes[2].set_xlabel('Motion time (s); startup settling excluded')
    loose = data[:,45] > .5
    if loose.any():
        for ax in axes:
            ax.axvspan(t[loose][0], t[loose][-1], color='#dddddd', alpha=.4)
        axes[1].text(float(t[loose].mean()), .97, 'Right axial rotation released', transform=axes[1].get_xaxis_transform(), ha='center', va='top', fontsize=9)
    for ax in axes:
        ax.grid(axis='y', color='#dddddd', lw=.5)
        ax.set_xlim(0, t[-1])
    axes[0].set_ylim(bottom=0)
    axes[2].set_ylim(bottom=0)
    axes[0].legend(frameon=False, ncol=3, loc='lower left',fontsize=9)
    fig.savefig(directory/'loads.png', dpi=160)
    plt.close(fig)
    summary = {'runs':{case:r for case, (_,r) in runs.items()}}
    if 'half_dt' in runs:
        half = np.load(runs['half_dt'][0].parent/'samples.npz')['samples']
        half = half[half[:,0] >= 0]
        comparison = {}
        for key, values, halfvalues in (
            ('axial_squeeze_n', data[:,25], half[:,25]),
            ('left_force_n', np.linalg.norm(data[:,1:4],axis=1), np.linalg.norm(half[:,1:4],axis=1)),
            ('right_force_n', np.linalg.norm(data[:,4:7],axis=1), np.linalg.norm(half[:,4:7],axis=1))):
            error = np.interp(t,half[:,0],halfvalues)-values
            comparison[key] = {'rms_difference':float(np.sqrt(np.mean(error**2))),
                               'peak_difference':float(np.max(abs(error))),
                               'peak_magnitude_relative_change':float(abs(np.max(abs(halfvalues))-np.max(abs(values)))/max(np.max(abs(values)),1e-9))}
        summary['step_convergence'] = comparison
    fig, ax = plt.subplots(figsize=(10,4.8),layout='constrained')
    labels = list(runs)
    values = [runs[c][1]['axial_squeeze_abs_n']['peak'] for c in labels]
    ax.barh(labels,values,color=blue,height=.6)
    for i, value in enumerate(values):
        ax.text(value+.01*max(values),i,f'{value:.2f} N',va='center',fontsize=10)
    ax.set_xlim(0,max(values)*1.2)
    ax.invert_yaxis()
    ax.set_xlabel('Peak absolute axial internal force (N)')
    ax.set_title('Sensitivity to wood density, grasp stiffness and calibration\nSame empty-shovel reference; no material strength or finger-force model')
    ax.grid(axis='x',color='#dddddd',lw=.5)
    ax.set_axisbelow(True)
    fig.savefig(directory/'sensitivity.png',dpi=160)
    plt.close(fig)
    (directory/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({case:{'mass_kg':r['source']['mass_kg'], 'axial_peak_n':r['axial_squeeze_abs_n']['peak'],
                      'force_peak_n':{s:r['grip_force_n'][s]['peak'] for s in ('left','right')}}
                      for case,(_,r) in runs.items()},indent=2))


def score(directory):
    """Independent truth/FK/CAD scoring; run in system Python with ROS deps.

    All native steps score pose error, with latest causal absolute input and
    no time alignment. Full CAD queries sample at 20 Hz; this is explicitly
    distinct from continuous collision detection and active PhysX contacts.
    """
    import pinocchio as pin
    from bindu_kinematics.tool_geometry import collision_check
    directory = Path(directory).resolve()
    result = json.loads((directory/'result.json').read_text())
    motion = json.loads(Path(result['provenance']['motion']).read_text())
    if hashlib.sha256(Path(result['provenance']['motion']).read_bytes()).hexdigest()!=result['provenance']['motion_sha256']:
        raise ValueError('SCORING_MOTION_IDENTITY')
    archive = np.load(directory/'samples.npz')
    if hashlib.sha256((directory/'samples.npz').read_bytes()).hexdigest() != result['provenance']['samples_sha256']:
        raise ValueError('SAMPLE_IDENTITY')
    data = archive['samples']; names = list(archive['joint_names'])
    data = data[(data[:,0]>=0)&(data[:,0]<=result['duration_s']+1e-9)]
    t = data[:,0]
    inputs = json.loads(Path(motion['tool_input_path']).read_text())
    if hashlib.sha256(Path(motion['tool_input_path']).read_bytes()).hexdigest()!=motion['tool_input_sha256']:
        raise ValueError('SCORING_INPUT_IDENTITY')
    stamps = np.array([x['stamp'] for x in inputs])
    targets = np.array([x['pose'] for x in inputs]).reshape(-1,4,4)
    wanted = targets[np.maximum(np.searchsorted(stamps,t,side='right')-1,0)]
    error = np.linalg.norm(data[:,29:32]-wanted[:,:3,3],axis=1)
    def stats(values):
        return dict(rms=float(np.sqrt(np.mean(values**2))),peak=float(np.max(values)),end=float(values[-1]))
    gates = motion['recipe']['acceptance']
    output = dict(position_error_m=stats(error),position_peak_time_s=float(t[np.argmax(error)]),
                  target_position_gate_m=gates['max_target_lag_m'],actual_end_s=float(t[-1]),
                  input_sha256=motion['tool_input_sha256'],scoring_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    quat = result['columns'].get('tool_quaternion_world_wxyz')
    if quat:
        tool = np.tile(np.eye(4),(len(data),1,1)); tool[:,:3,3] = data[:,29:32]
        q = data[:,quat[0]:quat[1]]
        tool[:,:3,:3] = Rotation.from_quat(q[:,[1,2,3,0]]).as_matrix()
        angle = Rotation.from_matrix(tool[:,:3,:3] @ np.transpose(wanted[:,:3,:3],(0,2,1))).magnitude()
        output['orientation_error_rad'] = stats(angle)
        model = pin.buildModelFromUrdf(motion['display_urdf']); state = model.createData()
        index = {n:model.joints[model.getJointId(n)].idx_q for n in names}
        frame = model.getFrameId(motion['left_frame']); mount = np.array(motion['left_to_tool']).reshape(4,4)
        fk, fkref = [], []
        for row in data:
            poses = []
            for offset in (46,46+len(names)):
                q = pin.neutral(model)
                for j,n in enumerate(names):q[index[n]]=row[offset+j]
                pin.forwardKinematics(model,state,q);pin.updateFramePlacements(model,state)
                poses.append((state.oMf[frame].homogeneous @ mount)[:3,3].copy())
            fk.append(poses[0]);fkref.append(poses[1])
        output['joint_servo_position_component_m'] = stats(np.linalg.norm(np.array(fk)-fkref,axis=1))
        output['grasp_compliance_position_component_m'] = stats(np.linalg.norm(tool[:,:3,3]-fk,axis=1))
        stride = max(1,round(.05/result['physics_dt_s']))
        samples = [dict(t=float(t[i]),q=dict(zip(names,map(float,data[i,46:46+len(names)]))),
                        tool=tool[i].ravel().tolist()) for i in range(0,len(data),stride)]
        output['geometry'] = collision_check(Path(motion['display_urdf']),samples,motion['recipe'],collect_failures=True)
        output['geometry_sample_period_s'] = stride*result['physics_dt_s']
        output['task_tracking_passed'] = bool(error.max() <= gates['max_target_lag_m'] and angle.max() <= gates['max_target_rotation_error_rad'])
        output['return_pose_passed'] = bool(error[-1]<=gates['max_return_error_m'] and angle[-1]<=gates['max_return_angle_rad'])
        np.savez_compressed(directory/'tracking.npz',time_s=t,position_error_m=error,orientation_error_rad=angle)
        output['tracking_samples_sha256'] = hashlib.sha256((directory/'tracking.npz').read_bytes()).hexdigest()
    controller = result.get('controller')
    output['full_motion_completed'] = bool(t[-1]+1e-9 >= result['duration_s'])
    output['experiment_accepted'] = bool(output['full_motion_completed'] and output.get('task_tracking_passed',False) and output.get('return_pose_passed',False) and
        output.get('geometry',{}).get('violations',1)==0 and (controller is None or controller.get('outcome')=='completed'))
    if controller:
        responses = json.loads((directory/'controller_responses.json').read_text())
        all_data = archive['samples']
        maximum = 0.
        warmup = result.get('control_warmup_s',0.)
        sensor_rows=[r for r in responses if warmup<=r['stamp']<=warmup+result['duration_s'] and r.get('wrenches')]
        output['wrist_sensor_100hz']={side:dict(
            force_n=stats(np.linalg.norm(np.array([r['wrenches'][side]['wrench'][:3] for r in sensor_rows]),axis=1)),
            moment_nm=stats(np.linalg.norm(np.array([r['wrenches'][side]['wrench'][3:] for r in sensor_rows]),axis=1)))
            for side in ('left','right')} if sensor_rows else {}
        for r in responses:
            i = int(np.argmin(abs(all_data[:,0]-(r['feedback_stamp']-warmup))))
            for n,q in r['qp_feedback_q'].items():maximum=max(maximum,abs(q-all_data[i,46+names.index(n)]))
        output['controller_measured_q_record_disagreement_rad'] = maximum
        observations = [r for r in responses if r.get('tool_observation') is not None]
        if observations:
            obs_t = np.array([r['stamp']-warmup for r in observations])
            obs = np.array([r['tool_observation'] for r in observations]).reshape(-1,4,4)
            actual = np.column_stack([np.interp(obs_t,all_data[:,0],all_data[:,29+j]) for j in range(3)])
            output['force_observer_position_error_m'] = stats(np.linalg.norm(obs[:,:3,3]-actual,axis=1))
        stopped = [r for r in responses if r['stopped']]
        if stopped:
            hold = all_data[all_data[:,0]>=stopped[0]['stamp']-warmup]
            output['stop_confirmed_at_s'] = stopped[0]['stamp']-warmup
            output['hold_tool_position_drift_m'] = float(np.linalg.norm(hold[:,29:32]-hold[0,29:32],axis=1).max())
    (directory/'tracking.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps({k:v for k,v in output.items() if k!='geometry'},indent=2))
    print('CAD_VIOLATIONS',output.get('geometry',{}).get('violations'))


def audit_stop(directory):
    """Fault-run audit using independent actual physics after controller stop."""
    directory=Path(directory).resolve();result=json.loads((directory/'result.json').read_text())
    if hashlib.sha256((directory/'samples.npz').read_bytes()).hexdigest()!=result['provenance']['samples_sha256']:
        raise ValueError('SAMPLE_IDENTITY')
    data=np.load(directory/'samples.npz')['samples'];tail=data[data[:,0]>=data[-1,0]-.5]
    controller=result['controller'];responses=json.loads((directory/'controller_responses.json').read_text())
    closing=[r for r in responses if r['closing']]
    if not closing:raise ValueError('NO_STOP_REQUEST')
    a,b=result['columns']['q'];q_drift=float(np.ptp(tail[:,a:b],axis=0).max())
    tool_drift=float(np.linalg.norm(tail[:,29:32]-tail[0,29:32],axis=1).max())
    qa,qb=result['columns']['tool_quaternion_world_wxyz'];quat=tail[:,qa:qb]
    rotations=Rotation.from_quat(quat[:,[1,2,3,0]])
    angle=float((rotations*rotations[0].inv()).magnitude().max())
    last=controller['last'];frozen=controller['fault_injection']['feedback_stop_at'] is not None
    output=dict(stop_code=last['code'],stop_fault=last['stop_fault'],stop_confirmed=last['stopped'],
        stop_requested_task_s=closing[0]['stamp']-result.get('control_warmup_s',0.),
        independent_tail_duration_s=float(tail[-1,0]-tail[0,0]),
        independent_joint_drift_rad=q_drift,independent_tool_drift_m=tool_drift,independent_tool_rotation_drift_rad=angle,
        accepted_commands_after_stop=max(r['accepted'] for r in closing)-closing[0]['accepted'])
    if 'qd_rad_s' in result['columns']:
        a,b=result['columns']['qd_rad_s'];output['raw_native_velocity_peak_rad_s']=float(abs(tail[:,a:b]).max())
    feedback_ok=(not last['stopped'] and last['stop_fault']=='STOP_FEEDBACK_TIMEOUT' and
        responses[-1]['feedback_stamp']==closing[0]['feedback_stamp']) if frozen else (last['stopped'] and not last['stop_fault'])
    output['passed_expectations']=bool(result['status']=='passed' and controller['outcome'] in ('completed','expected_stop_unconfirmed') and
        feedback_ok and output['accepted_commands_after_stop']==0 and q_drift<.002 and tool_drift<.005 and angle<.03)
    (directory/'stop-audit.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2))
    if not output['passed_expectations']:raise ValueError('STOP_AUDIT_FAILED')


def compare_feedback(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory = Path(directory).resolve()
    runs = {}
    for path in sorted(directory.glob('*/tracking.json')):
        result = json.loads((path.parent/'result.json').read_text())
        score_data = json.loads(path.read_text())
        mode = result.get('controller_mode')
        if (mode in ('measured_qp','measured_qp_load') and score_data.get('full_motion_completed',False) and
                result.get('controller',{}).get('outcome')=='completed' and result['case']=='baseline'):
            runs[mode]=(path.parent,result,score_data)
    if set(runs) != {'measured_qp','measured_qp_load'}:raise ValueError('NEED_TWO_COMPLETE_FEEDBACK_RUNS')
    if len({s['input_sha256'] for _,_,s in runs.values()})!=1:raise ValueError('DIFFERENT_OPERATOR_INPUTS')
    fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True,layout='constrained')
    fig.suptitle('Measured feedback: same empty-shovel operator input',fontsize=15)
    for mode,label,color,style in (('measured_qp','Joints + native velocity','#C57922','--'),
                                  ('measured_qp_load','F/T estimate + encoder velocity','#2870A7','-')):
        path,result,summary=runs[mode]
        if hashlib.sha256((path/'tracking.npz').read_bytes()).hexdigest()!=summary['tracking_samples_sha256']:
            raise ValueError('TRACKING_SAMPLE_IDENTITY')
        track=np.load(path/'tracking.npz');t=track['time_s']
        data=np.load(path/'samples.npz')['samples'];data=data[(data[:,0]>=0)&(data[:,0]<=result['duration_s']+1e-9)]
        for ax,y in zip(axes,(track['position_error_m']*1000,track['orientation_error_rad'],data[:,25],np.linalg.norm(data[:,86:89],axis=1))):
            ax.plot(t,y,color=color,ls=style,lw=1.,label=label)
    axes[0].axhline(20.,color='#777777',ls=':',lw=1.)
    axes[1].axhline(.03,color='#777777',ls=':',lw=1.)
    for ax,label in zip(axes,('Position error (mm)','Orientation error (rad)','Axial internal force (N)','Ground/bin contact (N)')):
        ax.set_ylabel(label);ax.grid(axis='y',color='#dddddd',lw=.5);ax.spines[['top','right']].set_visible(False)
    axes[0].legend(loc='upper center',bbox_to_anchor=(.5,1.32),ncol=2,frameon=False)
    axes[-1].set_xlabel('Operator motion time (s); preparation recorded separately')
    axes[-1].set_xlim(0,t[-1])
    fig.savefig(directory/'feedback-comparison.png',dpi=160);plt.close(fig)
    (directory/'feedback-comparison.json').write_text(json.dumps({mode:dict(directory=str(p),
        metrics=s,grip_force_n=r['grip_force_n'],grip_moment_nm=r['grip_moment_nm'],
        internal_force_n=r['internal_force_n'],internal_moment_nm=r['internal_moment_nm'],
        axial_squeeze_abs_n=r['axial_squeeze_abs_n'],controller=r['controller'])
        for mode,(p,r,s) in runs.items()},indent=2)+'\n')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Analyze verified shovel physics runs and export figures')
    parser.add_argument('--score',action='store_true',help='Independent true tool pose/FK/CAD scoring in ROS system Python')
    parser.add_argument('--feedback-comparison',action='store_true')
    parser.add_argument('--audit-stop',action='store_true')
    parser.add_argument('directory',type=Path)
    arguments = parser.parse_args()
    (score if arguments.score else audit_stop if arguments.audit_stop else compare_feedback if arguments.feedback_comparison else analyze)(arguments.directory)
