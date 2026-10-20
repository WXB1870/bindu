#!/usr/bin/env python3
"""Compare configured shovel dimensions and front/side grasp workspaces offline.

Samples a connected scoop, rear-hand pivot lift and outward swing. Feasible IK
samples are checked against CAD meshes and the tool geometry. This is a local
task workspace comparison, not a proof of global reachability or payload.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from bindu_contracts.profile import Profile
from tools.analysis.plan_shovel_motion import ROOT, collision_check, validate_recipe


def search(config_path, description):
    config = json.loads(config_path.read_text())
    validate_recipe(config)
    recipe = config['demo']
    settings = recipe['design_search']
    profile = Profile.load(config_path.parent/recipe['profile'])
    model = pin.buildModelFromUrdf(str(description/config['urdf']))
    data = model.createData()
    names = [n for group in config['resource_groups'] for n in profile.groups[group]]
    indices = [model.joints[model.getJointId(n)].idx_q for n in names]
    frames = [model.getFrameId(config[side+'_frame']) for side in ('left', 'right')]
    q0 = pin.neutral(model)
    initial = recipe['initial_positions_rad']
    for name, value in initial.items():
        q0[model.joints[model.getJointId(name)].idx_q] = value
    lower = np.array([profile.limits[n][0]+settings['ik_limit_margin_rad'] for n in names])
    upper = np.array([profile.limits[n][1]-settings['ik_limit_margin_rad'] for n in names])
    elevation = np.deg2rad(settings['ready_elevation_deg'])
    axis = np.array([-np.cos(elevation), 0., np.sin(elevation)])
    lift = settings['lift_deg']
    swing = settings['outward_swing_deg']
    controls = [(0., 0., 0.)]
    controls += [(settings['push_m']*u, 0., 0.) for u in (.25, .5, .75, 1.)]
    controls += [(settings['push_m']-settings['retract_m']*a/lift, a, 0.) for a in np.linspace(0, lift, 12)[1:]]
    controls += [(settings['push_m']-settings['retract_m'], lift, -a) for a in np.linspace(0, swing, 5)[1:]]
    candidates = []
    for length, spacing, azimuth in itertools.product(settings['shaft_lengths_m'],
                                                     settings['grip_spacings_m'],
                                                     settings['azimuths_deg']):
        rear = axis*(length-settings['rear_overhang_m'])
        front = rear-axis*spacing
        relative = []
        for point, side in ((rear, 'left'), (front, 'right')):
            # Keep the configured hand approach when comparing new dimensions.
            # Replacing it by opposite palms would compare a different grasp.
            approach = np.linalg.inv(np.array(config[side+'_to_tool']).reshape(4,4))[:3,0]
            approach = approach-axis*np.dot(approach,axis)
            approach /= np.linalg.norm(approach)
            transform = np.eye(4)
            transform[:3, :3] = np.column_stack((approach, np.cross(axis, approach), axis))
            transform[:3, 3] = point-transform[:3, :3] @ np.array(
                recipe.get(side+'_grip_offset_m', recipe['grip_offset_m']))
            relative.append(transform)
        anchor = np.array([settings['rear_grip_x_m'],
                           np.sign(azimuth)*settings['rear_grip_side_offset_m'],
                           recipe['ground']['z_in_base_m']+settings['blade_start_clearance_m']+rear[2]])
        forward = Rotation.from_euler('z', np.deg2rad(azimuth)).as_matrix() @ np.array([1., 0., 0.])
        seed = np.clip(q0[indices], lower, upper)
        samples = []
        for push, angle, turn in controls:
            tool = np.eye(4)
            tool[:3, :3] = (Rotation.from_euler('z', np.deg2rad(azimuth+turn)).as_matrix() @
                            Rotation.from_euler('y', -np.deg2rad(angle)).as_matrix())
            tool[:3, 3] = anchor + forward*push - tool[:3, :3] @ rear
            goals = [tool @ transform for transform in relative]

            def residual(values):
                state = q0.copy()
                state[indices] = values
                pin.framesForwardKinematics(model, data, state)
                error = []
                for frame, goal in zip(frames, goals):
                    error.extend(3.*(data.oMf[frame].translation-goal[:3, 3]))
                    error.extend(pin.log3(goal[:3, :3].T @ data.oMf[frame].rotation))
                return np.r_[error, .0005*values]

            solved = least_squares(residual, seed, bounds=(lower, upper), max_nfev=180,
                                   ftol=1e-10, xtol=1e-10, gtol=1e-10)
            error = float(np.linalg.norm(residual(solved.x)[:12]))
            seed = solved.x
            state = q0.copy()
            state[indices] = solved.x
            samples.append({'t': len(samples), 'tool': tool.tolist(), 'ik_residual': error,
                            'q': {n: float(state[model.joints[model.getJointId(n)].idx_q]) for n in initial},
                            'joint_margin_rad': min(min(v-profile.limits[n][0], profile.limits[n][1]-v)
                                                    for n, v in zip(names, solved.x))})
        shape = dict(recipe, shaft_axis_in_tool=axis.tolist(), left_grip_in_tool_m=rear.tolist(),
                     shaft_from_left_grip_m=[-(length-settings['rear_overhang_m']), settings['rear_overhang_m']])
        shape['shaft_contacts'] = {name:dict(value, center_m=-spacing if value['grip']=='right' else 0.)
                                  for name,value in recipe['shaft_contacts'].items()}
        geometry = collision_check(description/recipe['display_urdf'], samples, shape, collect_failures=True)
        row = {'shaft_length_m': length, 'grip_spacing_m': spacing, 'azimuth_deg': azimuth,
               'ik_ok': all(s['ik_residual'] < .002 for s in samples),
               'posture_ok': all(s['joint_margin_rad'] >= recipe['acceptance']['min_joint_margin_rad'] and
                                 all(low <= s['q'][n] <= high for n,(low,high) in
                                     recipe['acceptance']['posture_limits_rad'].items()) for s in samples),
               'min_joint_margin_deg': np.rad2deg(min(s['joint_margin_rad'] for s in samples)),
               'lift_m': samples[-1]['tool'][2][3]-samples[0]['tool'][2][3],
               # Ideal vertical point-load statics; it is not a grip-force rating.
               'front_grip_force_per_unit_tip_weight':
                   ((length-settings['rear_overhang_m'])*np.cos(elevation)+recipe['pan_center_m'][0]) /
                   (spacing*np.cos(elevation)),
               'geometry': geometry, 'samples': samples}
        for name in recipe['acceptance']['required_joint_excursion_rad']:
            row.setdefault('shared_joint_ranges_deg', {})[name] = [
                np.rad2deg(min(s['q'][name] for s in samples)),
                np.rad2deg(max(s['q'][name] for s in samples))]
        candidates.append(row)
        print(json.dumps({k: v for k, v in row.items() if k not in ('samples', 'geometry')})+
              f" geometry_violations={geometry['violations']}", flush=True)
    return {'config_sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(),
            'scope': 'sampled push/lift/swing only; excludes dump roll and continuous trajectory; no payload/contact validation',
            'design_search': settings, 'selected_design': recipe['selected_design'], 'candidates': candidates}


def plot_comparison(result, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties

    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    selected = result['selected_design']
    rows = [r for r in result['candidates']
            if np.isclose(r['shaft_length_m'], selected['shaft_length_m']) and
            np.isclose(r['grip_spacing_m'], selected['grip_spacing_m'])]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    colors = ['#64748b', '#0891b2', '#e07a22']
    for row, color in zip(rows, colors):
        p = np.array([s['tool'] for s in row['samples']])[:, :3, 3]
        label = f"{row['azimuth_deg']:+g}°"
        axes[0].plot(p[:, 0], p[:, 1], 'o-', color=color, label=label, markersize=3)
        axes[1].plot(p[:, 0], p[:, 2], 'o-', color=color, label=label, markersize=3)
        margins = np.rad2deg([s['joint_margin_rad'] for s in row['samples']])
        axes[2].plot(margins, 'o-', color=color, label=label, markersize=3)
    axes[0].set(xlabel='X (m)', ylabel='Y (m)', aspect='equal')
    axes[1].set(xlabel='X (m)', ylabel='Z (m)', aspect='equal')
    axes[2].set(xlabel='Task pose index', ylabel='Minimum joint margin (deg)')
    for ax, title in zip(axes, ('铲头路径：俯视', '铲头路径：侧视', '距关节限位的最小余量')):
        ax.set_title(title, fontproperties=font)
        ax.grid(alpha=.25)
        ax.legend(title='Ready azimuth', loc='lower left')
    fig.suptitle(f"杆长 {selected['shaft_length_m']:.2f} m，握距 {selected['grip_spacing_m']:.2f} m；"
                 f"扬铲余量检查 {result['design_search']['lift_deg']}°，正式动作 {selected['lift_deg']}°",
                 fontproperties=font)
    fig.savefig(output/'workspace_comparison.png', dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=ROOT/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_shovel.json')
    parser.add_argument('--description', type=Path, default=ROOT/'src/hardware/bindu_description/urdf')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    destination = output/'workspace.json'
    if destination.exists():
        raise FileExistsError(destination)
    result = search(args.config.resolve(), args.description.resolve())
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    plot_comparison(result, output)


if __name__ == '__main__':
    main()
