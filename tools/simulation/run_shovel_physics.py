#!/usr/bin/env python3
"""Isaac Sim 6.2 / PhysX: dynamic Snowbot with a contact-grasped wooden shovel.

Replays verified references or runs the production QP/executor on measured joints.
Only the initial state is set; every subsequent state comes from physics.
No ROS/hardware transport is created. This is an empty-shovel load experiment,
using CAD jaw contacts by default. Jaw effort, friction and low-level drive
parameters are simulation assumptions, not measured hardware parameters.
"""
from tools import package_share
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from tools.simulation.shovel_physics import load_motion, wooden_parts, compound_mass, grip_wrench, internal_wrench, MeasuredJointVelocity, joint_drive_parameters

from tools import ROOT
sys.path.insert(0, str(ROOT/'src/capabilities/bindu_kinematics'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


from snowbot_bringup.simulation.scene import pose, quaternion, validate_grasp_config, build_scene








def contact_wrench(contacts, com, dt):
    """Sum independently measured PhysX normal and friction contact impulses."""
    def indices(counts, starts):
        groups = [np.arange(int(s), int(s+c)) for c,s in zip(np.ravel(counts),np.ravel(starts)) if c]
        return np.concatenate(groups) if groups else np.array([],dtype=int)
    magnitudes, points, normals, separations, counts, starts = contacts.get_contact_force_data(dt=dt)
    selected = indices(counts, starts)
    normal = np.asarray(magnitudes).reshape(-1,1)[selected]*np.asarray(normals).reshape(-1,3)[selected]
    normal_points = np.asarray(points).reshape(-1,3)[selected]
    friction, friction_points, counts, starts = contacts.get_friction_data(dt=dt)
    selected = indices(counts, starts)
    friction = np.asarray(friction).reshape(-1,3)[selected]
    friction_points = np.asarray(friction_points).reshape(-1,3)[selected]
    fn, ft = normal.sum(axis=0), friction.sum(axis=0)
    torque = (np.cross(normal_points-com, normal).sum(axis=0)+
              np.cross(friction_points-com, friction).sum(axis=0))
    return fn+ft, torque, fn, ft


def grasp_contact_wrenches(contacts, tool, offsets, com, dt):
    """Actual PhysX contact loads on the tool, expressed at the grasp points."""
    forces, torques, points, separation = [], [], [], []
    for side, offset in zip(('left', 'right'), offsets):
        point = tool[:3,3]+tool[:3,:3] @ offset
        force, moment, _, _ = contact_wrench(contacts[side], com, dt)
        forces.append(force); torques.append(moment-np.cross(point-com,force)); points.append(point)
        _, _, _, gaps, counts, starts = contacts[side].get_contact_force_data(dt=dt)
        for count, start in zip(np.ravel(counts), np.ravel(starts)):
            if count:
                separation.extend(np.ravel(gaps)[int(start):int(start+count)].tolist())
    return forces, torques, points, min(separation, default=0.)


def simulate(app, args, motion, cfg):
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.simulation_manager import SimulationManager
    output = args.output
    world, body, view, contacts, source = build_scene(motion, cfg, output)
    names = body.dof_names
    samples = motion['samples']
    times = np.array([s['t'] for s in samples])
    plan = np.array([[s['q'][n] for n in names] for s in samples])
    dense_path = args.motion.parent/'joint_plan.npz'
    if dense_path.is_file():
        dense = np.load(dense_path)
        dense_names = list(dense['names'])
        dense_q = dense['executed']
        dense_times = np.arange(len(dense_q))*float(dense['dt'])
        if (not set(dense_names).issubset(names) or not np.isfinite(dense_q).all() or
                abs(dense_times[-1]-times[-1]) > 1e-6):
            raise ValueError('DENSE_REFERENCE_LAYOUT')
        for j, name in enumerate(dense_names):
            if not np.allclose(np.interp(times, dense_times, dense_q[:, j]), plan[:, names.index(name)], atol=1e-10, rtol=0):
                raise ValueError('DENSE_REFERENCE_MISMATCH:'+name)
        reference = np.tile(plan[0], (len(dense_q), 1))
        for j, name in enumerate(dense_names):
            reference[:, names.index(name)] = dense_q[:, j]
        times, plan = dense_times, reference
        source['joint_reference_sha256'] = digest(dense_path)
    source['joint_reference_sample_period_s'] = float(times[1]-times[0])
    source_config = json.loads(Path(motion['config_path']).read_text())
    mounts = [np.array(source_config[side+'_to_tool']).reshape(4, 4)[:3, :3] for side in ('left', 'right')]
    recipe = motion['recipe']
    contact_grasp = cfg.get('grasp_model', 'proxy') == 'contact'
    left_axial=source_config.get('left_axial_grasp')
    if left_axial is not None and contact_grasp:
        raise ValueError('DUAL_AXIAL_PHYSICS_REQUIRES_PROXY_GRASP')
    left_axis=None if left_axial is None else np.asarray(left_axial['axis_in_tool'],dtype=float)
    cfg['shaft_axis_in_tool'] = np.array(recipe['shaft_axis_in_tool'])
    tool_offsets = [np.array(recipe[side+'_grip_in_tool_m']) for side in ('left', 'right')]
    hand_offsets = [np.array(recipe.get(side+'_grip_offset_m', recipe['grip_offset_m']))
                    for side in ('left', 'right')]
    # A known axial grip calibration error is carried with the right flange.
    hand_offsets[1] += mounts[1] @ cfg['shaft_axis_in_tool']*cfg.get('right_grip_axial_offset_m', 0.)
    body.set_joint_positions(plan[0])
    body.set_joint_velocities(np.zeros(len(names)))
    body.apply_action(ArticulationAction(joint_positions=plan[0]))
    home = np.array(motion['home'])
    view.set_world_poses(home[None, :3, 3], quaternion(home[:3, :3])[None, :], indices=[2])
    view.set_velocities(np.zeros((1, 6)), indices=[2])
    initial_xyz, initial_quat = view.get_world_poses()
    (output/'initial_state.json').write_text(json.dumps({
        'rigid_positions_m':initial_xyz.tolist(), 'rigid_quaternions_wxyz':initial_quat.tolist(),
        'tool_home':home.tolist(), 'q':body.get_joint_positions().tolist(),
        'source':source},indent=2)+'\n')
    # Propagate the single initial joint write to native link transforms.
    native_start = SimulationManager.get_num_physics_steps()
    world.step(render=False)
    if SimulationManager.get_num_physics_steps()-native_start != 1:
        raise ValueError('CALIBRATION_STEP_COUNT')
    freefall_acceleration = view.get_velocities()[2, :3]/cfg['physics_dt_s']
    if not contact_grasp and not np.allclose(freefall_acceleration, [0., 0., -9.81], atol=.01):
        raise ValueError('FREE_FALL_CALIBRATION:'+str(freefall_acceleration))
    source['initial_native_acceleration_m_s2' if contact_grasp else 'freefall_acceleration_m_s2'] = freefall_acceleration.tolist()
    dt, settle = cfg['physics_dt_s'], cfg['settle_s']
    warmup = cfg.get('control_warmup_s',0.) if args.closed_loop else 0.
    duration = times[-1]
    if args.duration is not None:
        duration = args.duration if args.impedance_binary else min(args.duration, times[-1])
    online = None
    responses = []
    mode = 'rigid'
    qref = plan[0].copy()
    impedance = None
    impedance_rows = []
    if args.impedance_binary:
        from tools.simulation.shovel_control import ImpedanceClient
        from bindu_contracts.teleoperation import joint_reference_limits
        impedance = ImpedanceClient(args,motion,source_config,qref,names,hand_offsets)
        arm_indices_imp = [names.index(n) for n in impedance.arm_names]
        joint_error_limits=np.array(joint_reference_limits(names,cfg['max_joint_error_rad'],
            cfg['joint_reference_error_limits']))
        kp,kd = body.get_articulation_controller().get_gains()
        kp,kd = np.array(kp).reshape(-1),np.array(kd).reshape(-1)
        kp[arm_indices_imp] = kd[arm_indices_imp] = 0.
        body.get_articulation_controller().set_gains(kp,kd)
        actual_gains = body.get_articulation_controller().get_gains()
        if any(np.any(np.asarray(g).reshape(-1)[arm_indices_imp] != 0.) for g in actual_gains):
            raise ValueError('IMPEDANCE_POSITION_DRIVE_NOT_ZERO')
        source['impedance_arm_drive_gains'] = {'kp':kp[arm_indices_imp].tolist(),'kd':kd[arm_indices_imp].tolist()}
        source['impedance_binary_sha256'] = digest(args.impedance_binary)
        source['impedance_config_sha256'] = digest(args.impedance_config)
        source['impedance_model_sha256'] = digest(source_config['urdf'])
        source['impedance_client_sha256'] = digest(ROOT/'tools/simulation/shovel_control.py')
        source['impedance_reference_policy_sha256'] = digest(args.impedance_reference_config)
        source['joint_reference_error_limits_rad'] = dict(zip(names,joint_error_limits.tolist()))
        source['max_target_lead_error_rad'] = cfg['max_target_lead_error_rad']
        (output/'impedance_setup.json').write_text(json.dumps(source,indent=2)+'\n')
    next_control = 0.
    stopped_since = None
    input_path = args.motion.parent/'tool_inputs.json'
    if args.closed_loop:
        from tools.simulation.shovel_control import ControlClient
        frames = json.loads(input_path.read_text())
        if not frames or any(b['stamp'] <= a['stamp'] for a,b in zip(frames,frames[1:])):
            raise ValueError('CONTROL_INPUT_ORDER')
        prefix_count = round(warmup/.01)
        if not np.isclose(prefix_count*.01,warmup):raise ValueError('CONTROL_WARMUP_GRID')
        # Known ready goal only, no future motion or future target derivatives.
        prefix = [dict(frames[0],stamp=round(i*.01,9),seq=i) for i in range(prefix_count)]
        frames = prefix+[dict(f,stamp=round(f['stamp']+warmup,9),seq=f['seq']+prefix_count) for f in frames]
        feedback = None
        if args.load_feedback:
            feedback = dict(cfg['load_feedback'],simulation_only=True,
                **{k:cfg[k] for k in ('grip_translation_stiffness_n_m','grip_translation_damping_ns_m',
                                     'grip_rotation_stiffness_nm_rad','grip_rotation_damping_nm_s_rad')})
        online = ControlClient(motion['config_path'],output,feedback)
        source['online_controller'] = online.hello
        source['tool_inputs_sha256'] = digest(input_path)
    input_cursor = -1
    sensor_readings = None
    encoder_velocity = MeasuredJointVelocity()
    source['feedback_velocity_source'] = cfg.get('feedback_velocity_source','native')
    count = round((settle+warmup+duration+(6. if online else 0.))/dt)
    com_local, _ = view.get_coms()
    com_local = np.asarray(com_local).reshape(-1, 3)
    rows = []
    previous_mode = 'rigid'
    last_drive_mode = None
    static_forces, static_torques = [], []
    started = time.monotonic()
    max_step_residual = 0.
    max_axial_couple = 0.
    minimum_contact_separation = 0.
    initial_tool = None
    max_projected_effort = np.zeros(len(names))
    npz_path = output/'samples.npz'
    recorder = None
    step_times, render_times, render_stamps = [], [], []
    idle_wall = 0.
    next_gui_render = started
    if args.record_video:
        from tools.simulation.shovel_video import ViewportRecorder
        recorder = ViewportRecorder(output, args.video_fps)
    try:
        for step in range(count+1):
            if not app.is_running():
                raise RuntimeError('SIMULATION_INTERRUPTED')
            controls = getattr(args, '_replay_panel', None)
            if controls is not None:
                started += controls.checkpoint(app, world)
            elapsed = step*dt
            control_t = round(max(0., elapsed-settle),9)
            t = round(control_t-warmup,9)
            if impedance:
                t = round(elapsed-settle,9)
            if recorder and 0. <= t <= duration and recorder.due(t):
                recorder.capture(world, t)
            q = body.get_joint_positions()
            qd = body.get_joint_velocities()
            if online and elapsed >= settle:
                response = online.poll(control_t)
                if response is not None:
                    responses.append(response)
                    qref = np.array([response['targets'][n] for n in names])
                    mode = response['grasp_mode']
                    if response['stopped']:
                        stopped_since = t if stopped_since is None else stopped_since
                    else:
                        stopped_since = None
            elif not online and not impedance:
                qref = np.array([np.interp(t, times, plan[:, j]) for j in range(len(names))])
                mode = next(k['mode'] for k in reversed(recipe['grasp_schedule']) if t+1e-9 >= k['t'])
            if getattr(args, 'grasp_test', None) == 'hold' or impedance:
                qref = plan[0].copy()
            if impedance:
                mode = 'rigid'
            if contact_grasp:
                settings = cfg['contact_grasp']
                released = getattr(args, 'grasp_test', None) == 'release' and t >= 1.
                positions = settings['open_positions_rad'] if released else settings['closed_positions_rad']
                for n, value in positions.items():
                    qref[names.index(n)] = value
                if last_drive_mode != mode:
                    limits = np.array(source['drive_effort_limits_nm'])
                    for n in settings['closed_positions_rad']:
                        limits[names.index(n)] = min(limits[names.index(n)], settings['max_effort_nm'])
                    if mode == 'right_axial':
                        limits[names.index('arm_r_joint_8')] = settings['axial_effort_nm']
                    body.get_articulation_controller().set_max_efforts(limits)
                    last_drive_mode = mode
            body.apply_action(ArticulationAction(joint_positions=qref))
            xyz, quat = view.get_world_poses()
            transforms = [pose(p, q) for p, q in zip(xyz, quat)]
            hands = [tf @ np.array(mount) for tf, mount in zip(transforms[:2], source['flange_from_physical_body'])]
            velocities = view.get_velocities()
            coms = np.array([tf[:3, 3]+tf[:3, :3] @ offset for tf, offset in zip(transforms, com_local)])
            if not online and not impedance:
                mode = next(k['mode'] for k in reversed(recipe['grasp_schedule']) if t+1e-9 >= k['t'])
            if mode != previous_mode and mode == 'rigid':
                # Re-lock the current physical relative angle, no pose write.
                mounts[1] = hands[1][:3, :3].T @ transforms[2][:3, :3]
            previous_mode = mode
            if initial_tool is None and t >= 0:
                initial_tool = transforms[2].copy()
            errors = [float(np.linalg.norm(hand[:3,3]+hand[:3,:3] @ ho-
                transforms[2][:3,3]-transforms[2][:3,:3] @ to))
                for hand,ho,to in zip(hands,hand_offsets,tool_offsets)]
            forces, torques, points = [], [], []
            if contact_grasp:
                forces, torques, points, _ = grasp_contact_wrenches(contacts, transforms[2], tool_offsets, coms[2], dt)
            else:
                for side in range(2):
                    f, tau, point, _ = grip_wrench(hands[side], transforms[2], velocities[side], velocities[2],
                        coms[side], coms[2], hand_offsets[side], tool_offsets[side], mounts[side], cfg,
                        axial=(side == 1 or left_axis is not None) and mode == 'right_axial',
                        released_axis_in_tool=left_axis if side == 0 else None)
                    forces.append(f); torques.append(tau); points.append(point)
            shaft_world = transforms[2][:3,:3] @ cfg['shaft_axis_in_tool']
            decomposition = internal_wrench(forces, torques, points, coms[2],
                released_axis=shaft_world if mode=='right_axial' and not contact_grasp else None,
                released_left_axis=transforms[2][:3,:3] @ left_axis
                    if left_axis is not None and mode=='right_axial' else None)
            if args.wrench_stop_at is None or t < args.wrench_stop_at:
                sensor_readings = {}
                for i,side in enumerate(('left','right')):
                    f = -forces[i]
                    moment = -torques[i]+np.cross(points[i]-hands[i][:3,3],f)
                    sensor_readings[side] = dict(stamp=t,frame_id=source_config[side+'_frame'],
                        wrench=np.r_[hands[i][:3,:3].T @ f,hands[i][:3,:3].T @ moment].tolist())
                    sensor_readings[side]['stamp'] = control_t
            if online and elapsed >= settle and control_t+1e-9 >= next_control and online.pending is None:
                cutoff = warmup+min(duration, args.input_stop_at if args.input_stop_at is not None else duration)
                while input_cursor+1 < len(frames) and frames[input_cursor+1]['stamp'] <= min(control_t,cutoff):
                    input_cursor += 1
                frame = frames[input_cursor] if input_cursor >= 0 else None
                measured_qd = (encoder_velocity.update(control_t,q,qd)
                    if source['feedback_velocity_source']=='position_difference' else qd)
                online.send(control_t,dict(zip(names,map(float,q))),dict(zip(names,map(float,measured_qd))),frame,
                    cancel=args.cancel_at is not None and t >= args.cancel_at,
                    freeze_feedback=args.feedback_stop_at is not None and t >= args.feedback_stop_at,wrenches=sensor_readings)
                next_control = control_t+.01
            if mode == 'right_axial':
                shaft_world = transforms[2][:3, :3] @ cfg['shaft_axis_in_tool']
                max_axial_couple = max(max_axial_couple, abs(float(torques[1] @ shaft_world)))
            # Accumulate both grips on the tool, one opposite wrench per hand.
            applied_forces = np.vstack((-np.array(forces), np.sum(forces, axis=0)))
            applied_torques = np.vstack(([-tau+np.cross(point-com, -f)
                for f, tau, point, com in zip(forces, torques, points, coms[:2])],
                decomposition['net_torque']))
            external_force = np.zeros(3)
            if impedance:
                measured_qd = encoder_velocity.update(elapsed,q,qd) if source['feedback_velocity_source']=='position_difference' else qd
                torque,dx,tcp = impedance.update(dt,q,measured_qd,sensor_readings)
                # Match session_v3's final PT gains in software, with no hidden PhysX drive.
                torque = np.array(torque)+1.*(qref[arm_indices_imp]-q[arm_indices_imp])-.2*measured_qd[arm_indices_imp]
                effort_limits = np.array(source['drive_effort_limits_nm'])[arm_indices_imp]
                clipped = np.clip(torque,-effort_limits,effort_limits)
                body.apply_action(ArticulationAction(joint_efforts=clipped,joint_indices=np.array(arm_indices_imp)))
                # Smooth fixed-world horizontal 10 N pulse; known external load at tool COM.
                ramp = min(max((t-1.)/.25,0.),1.,max((3.-t)/.25,0.))
                external_force[0] = 10.*ramp
                impedance_rows.append(np.r_[t,external_force,clipped,np.array(dx),np.array(tcp),float(np.any(clipped != torque))])
            if not contact_grasp or impedance:
                if contact_grasp:
                    applied_forces[:] = 0.; applied_torques[:] = 0.
                applied_forces[2] += external_force
                view.apply_forces_and_torques_at_pos(forces=applied_forces.astype(np.float32),
                    torques=applied_torques.astype(np.float32), positions=coms.astype(np.float32), is_global=True)
            joint_error = float(np.max(abs(q-qref)))
            if not np.isfinite(np.r_[q, velocities.ravel(), applied_forces.ravel(), applied_torques.ravel()]).all():
                raise ValueError('NONFINITE_PHYSICS')
            # Jaw error is expected while squeezing a blocked rigid rod. Arm
            # servo tracking and tool loss remain separate checks.
            if contact_grasp:
                arm_indices = [i for i,n in enumerate(names) if n not in cfg['contact_grasp']['closed_positions_rad']]
                joint_error = float(np.max(abs(q[arm_indices]-qref[arm_indices])))
            joint_violation=joint_error>cfg['max_joint_error_rad']
            if impedance:
                checked=arm_indices if contact_grasp else list(range(len(names)))
                joint_violation=bool(np.any(abs(q[checked]-qref[checked])>joint_error_limits[checked]))
            diagnostic_release = getattr(args, 'grasp_test', None) == 'release'
            if joint_violation or (not diagnostic_release and max(errors) > cfg['max_grip_error_m']):
                detail=''
                if impedance:
                    worst=max(checked,key=lambda j:abs(q[j]-qref[j])/joint_error_limits[j])
                    detail=f',worst_joint={names[worst]},error={abs(q[worst]-qref[worst])},limit={joint_error_limits[worst]}'
                raise ValueError(f'PHYSICS_TRACKING_LOST:t={t:.4f},elapsed={elapsed:.4f},grip={max(errors)},joint={joint_error}{detail}')
            native_before = SimulationManager.get_num_physics_steps()
            step_started = time.monotonic()
            world.step(render=False)
            step_times.append(time.monotonic()-step_started)
            now = time.monotonic()
            if args.gui and now >= next_gui_render:
                # World.step(render=True) advances rendering_dt/physics_dt
                # steps in 6.2. Render separately without advancing physics.
                render_stamps.append(time.monotonic())
                world.render()
                render_times.append(time.monotonic()-render_stamps[-1])
                # UI refresh follows wall time. A slow physics/diagnostic
                # loop must not reduce a 30 Hz viewport to a few frames/s.
                next_gui_render = render_stamps[-1]+1/cfg['gui_render_hz']
                idle_started = time.monotonic()
                time.sleep(max(0., elapsed-(idle_started-started)))
                idle_wall += time.monotonic()-idle_started
            if SimulationManager.get_num_physics_steps()-native_before != 1:
                raise ValueError('UNEXPECTED_NATIVE_STEP_COUNT')
            effort = body.get_measured_joint_efforts()
            if effort is None or not np.isfinite(effort).all():
                raise ValueError('MISSING_PHYSICS_JOINT_EFFORT')
            max_projected_effort = np.maximum(max_projected_effort, abs(effort))
            next_velocity = view.get_velocities()[2]
            cf, ct, cn, friction = contact_wrench(contacts['environment'], coms[2], dt)
            if contact_grasp:
                forces, torques, points, gap = grasp_contact_wrenches(contacts, transforms[2], tool_offsets, coms[2], dt)
                minimum_contact_separation = min(minimum_contact_separation, gap)
                decomposition = internal_wrench(forces, torques, points, coms[2])
            balance = decomposition['net_force']+cf+external_force+np.array([0., 0., -9.81*source['mass_kg']])
            residual = np.linalg.norm(source['mass_kg']*(next_velocity[:3]-velocities[2, :3])/dt-balance)
            max_step_residual = max(max_step_residual, float(residual))
            if elapsed >= settle-1 and elapsed < settle:
                static_forces.append(decomposition['net_force']+cf)
                static_torques.append(decomposition['net_torque']+ct)
            rows.append(np.r_[elapsed-settle-warmup, np.array(forces).ravel(), np.array(torques).ravel(),
                decomposition['wrenches'].ravel(), decomposition['axial_squeeze_n'], errors, joint_error,
                transforms[2][:3, 3], velocities[2], decomposition['net_force'], decomposition['net_torque'],
                residual, float(mode == 'right_axial'), q, qref, cf, ct, cn, friction, quat[2],qd])
            if step % max(1, round(5/dt)) == 0:
                print('PHYSICS', args.case, round(elapsed-settle-warmup, 3), 'force_N',
                      [round(float(np.linalg.norm(f)), 2) for f in forces], 'squeeze_N',
                      round(decomposition['axial_squeeze_n'], 2),
                      'control', {k:responses[-1][k] for k in ('accepted','code','stopped')} if responses else None, flush=True)
            if online and stopped_since is not None and t-stopped_since >= .5:
                break
    finally:
        if online:
            online.close()
        if impedance:
            impedance.close()
        if impedance_rows:
            np.savez_compressed(output/'impedance.npz',samples=np.array(impedance_rows),arm_names=impedance.arm_names)
        if rows:
            np.savez_compressed(npz_path, samples=np.array(rows), joint_names=names)
        if recorder:
            recorder.close()
    data = np.array(rows)
    active = data[:, 0] >= 0
    steady = np.mean(static_forces, axis=0)
    static_error = np.linalg.norm(steady+np.array([0., 0., -9.81*source['mass_kg']]))
    static_torque_error = np.linalg.norm(np.mean(static_torques, axis=0))
    status = ('passed' if static_error < cfg['static_force_balance_tolerance_n'] and
              static_torque_error < cfg['static_torque_balance_tolerance_nm'] and
              max_step_residual < cfg['max_discrete_newton_residual_n'] else 'failed_balance')
    def stats(values):
        return dict(mean=float(np.mean(values)), rms=float(np.sqrt(np.mean(values**2))),
                    p95=float(np.percentile(values, 95)), peak=float(np.max(values)))
    loop_wall = time.monotonic()-started
    performance = dict(loop_wall_s=loop_wall, simulated_s=len(rows)*dt,
        real_time_factor=len(rows)*dt/loop_wall,
        physics_step_wall_s=stats(np.array(step_times)),
        physics_wall_total_s=float(sum(step_times)),
        render_wall_total_s=float(sum(render_times)), idle_wall_s=idle_wall,
        python_and_io_wall_s=loop_wall-sum(step_times)-sum(render_times)-idle_wall,
        render_frames=len(render_times),
        render_interval_wall_s=stats(np.diff(render_stamps)) if len(render_stamps)>1 else None)
    result = dict(status=status, case=args.case, duration_s=duration,
        grasp_model=cfg.get('grasp_model','proxy'), minimum_contact_separation_m=minimum_contact_separation,
        controller_mode=('measured_qp_load' if args.load_feedback else 'measured_qp') if online else 'reference_replay',
        control_warmup_s=warmup,
        physics_steps=SimulationManager.get_num_physics_steps()-native_start,
        calibration_steps=1, recorded_grasp_steps=len(rows),
        wall_s=time.monotonic()-started, physics_dt_s=dt, source=source,
        performance=performance,
        static_force_balance_error_n=float(static_error), static_torque_balance_error_nm=float(static_torque_error),
        max_discrete_newton_residual_n=max_step_residual,
        grip_force_n={side:stats(np.linalg.norm(data[active, 1+3*i:4+3*i], axis=1)) for i, side in enumerate(('left', 'right'))},
        grip_moment_nm={side:stats(np.linalg.norm(data[active, 7+3*i:10+3*i], axis=1)) for i, side in enumerate(('left', 'right'))},
        internal_force_n={side:stats(np.linalg.norm(data[active, 13+6*i:16+6*i], axis=1)) for i, side in enumerate(('left', 'right'))},
        internal_moment_nm={side:stats(np.linalg.norm(data[active, 16+6*i:19+6*i], axis=1)) for i, side in enumerate(('left', 'right'))},
        axial_squeeze_abs_n=stats(abs(data[active, 25])), axial_squeeze_signed_range_n=[float(data[active,25].min()), float(data[active,25].max())],
        max_grip_error_m=float(data[active, 26:28].max()), max_joint_error_rad=float(data[active,28].max()),
        right_axial_max_direct_shaft_couple_nm=max_axial_couple,
        max_solver_projected_joint_effort_nm=dict(zip(names, map(float, max_projected_effort))),
        environment_contact_force_n=stats(np.linalg.norm(data[active,46+2*len(names):49+2*len(names)],axis=1)),
        columns={'time_s':0, 'grip_forces_world_n':[1,7], 'grip_couples_world_nm':[7,13],
            'internal_wrenches_world_n_nm':[13,25], 'axial_squeeze_n':25, 'grip_errors_m':[26,28],
            'max_joint_error_rad':28, 'tool_position_world_m':[29,32], 'tool_velocity_com_world':[32,38],
            'net_grip_force_world_n':[38,41], 'net_grip_torque_at_com_world_nm':[41,44],
            'discrete_newton_residual_n':44, 'right_axial':45, 'q':[46,46+len(names)], 'qref':[46+len(names),46+2*len(names)],
            'environment_contact_force_world_n':[46+2*len(names),49+2*len(names)],
            'environment_contact_moment_at_com_world_nm':[49+2*len(names),52+2*len(names)],
            'normal_contact_force_world_n':[52+2*len(names),55+2*len(names)],
            'friction_contact_force_world_n':[55+2*len(names),58+2*len(names)],
            'tool_quaternion_world_wxyz':[58+2*len(names),62+2*len(names)]},
        assumptions={'wood':'homogeneous rigid compound; no elastic bending or failure',
            'grips':'finite stiffness wrench proxies; no finger contact/friction limit or jaw squeeze force',
            'base':'fixed to world; not a mobile-base stability test', 'snow':'empty shovel, no snow or cutting resistance',
            'robot_collision':'self-collision off; tool-robot filtered; tool-ground/bin enabled',
            'control':('actual joint position/velocity feedback to production QP/session/executor; no tool ground truth feedback'
                       if online else 'force-limited joint reference replay, no online feedback to QP'),
            'internal_metric':'grasp-mode constrained nullspace, moments scaled by actual grip spacing; axial squeeze separately invariant',
            'sample_time':'pre-step state and wrench, discrete Newton residual uses same-step velocity increment'},
        provenance={'motion':str(args.motion), 'motion_sha256':digest(args.motion),
                    'config_sha256':args.source_hashes['config'], 'runner_sha256':args.source_hashes['runner'],
                    'math_sha256':args.source_hashes['math'],
                    'controller_sha256':online.hello['controller_sha256'] if online else None,
                    'samples_sha256':digest(npz_path), 'isaac_version':args.isaac_version})
    # Config contains a NumPy axis only for spring calculations.
    result['columns']['qd_rad_s'] = [62+2*len(names),62+3*len(names)]
    result['configuration'] = {k:v for k,v in cfg.items() if k != 'shaft_axis_in_tool'}
    if contact_grasp:
        result['assumptions'].update(grips='force-limited jaw joints; native mesh-decomposition contact and Coulomb friction; no grasp springs',
            robot_collision='self-collision off; all tool-robot contacts enabled; jaw CAD convex decomposition',
            internal_metric='full grasp-map nullspace of measured contact wrenches; actual friction couples retained')
        if args.grasp_test:
            pose_data = data[data[:,0]>=0,29:32]
            held = data[(data[:,0]>=0)&(data[:,0]<(1. if args.grasp_test=='release' else duration+dt))]
            held_error = float(np.linalg.norm(held[:,29:32]-home[:3,3],axis=1).max())
            drift = float(np.linalg.norm(held[:,29:32]-held[0,29:32],axis=1).max())
            drop = float(pose_data[0,2]-pose_data[-1,2])
            passed = (held_error < .006 and drift < .002 and minimum_contact_separation > -.001 and
                      (args.grasp_test != 'release' or drop > .05))
            result['grasp_test'] = dict(kind=args.grasp_test,
                held_position_error_m=held_error, held_position_drift_m=drift,
                final_vertical_drop_m=drop, passed=passed)
            if not passed:
                result['status'] = status = 'failed_grasp_test'
    if online:
        result['controller'] = dict(last=responses[-1] if responses else None,
            replies=len(responses), max_ipc_wall_s=max((r['ipc_wall_s'] for r in responses),default=None),
            max_worker_wall_s=max((r['worker_wall_s'] for r in responses),default=None),
            simulated_end_s=float(data[-1,0]), stop_observed_for_s=0. if stopped_since is None else t-stopped_since,
            fault_injection=dict(input_stop_at=args.input_stop_at,feedback_stop_at=args.feedback_stop_at,
                                 wrench_stop_at=args.wrench_stop_at,cancel_at=args.cancel_at))
        last = responses[-1] if responses else {}
        expected = ('CANCELED' if args.cancel_at is not None else 'FEEDBACK_STALE' if args.feedback_stop_at is not None
                    else 'GRIP_LOAD_STALE_OR_FRAME' if args.wrench_stop_at is not None else 'TOOL_INPUT_TIMEOUT')
        expected_unconfirmed = args.feedback_stop_at is not None and last.get('stop_fault') == 'STOP_FEEDBACK_TIMEOUT' and not last.get('stopped')
        control_ok = last.get('code') == expected and last.get('accepted',0) > 0 and (last.get('stopped') or expected_unconfirmed)
        result['controller']['expected_stop_code'] = expected
        result['controller']['stop_confirmed'] = bool(last.get('stopped'))
        result['controller']['outcome'] = ('expected_stop_unconfirmed' if expected_unconfirmed else 'completed') if control_ok else 'failed'
        if not control_ok:
            result['status'] = status = 'failed_control'
    if impedance:
        imp = np.array(impedance_rows)
        baseline = data[(data[:,0]>=.5)&(data[:,0]<1.),29:32].mean(axis=0)
        loaded = data[(data[:,0]>=2.5)&(data[:,0]<2.75),29:32].mean(axis=0)
        recovered = data[data[:,0]>=duration-.5,29:32].mean(axis=0)
        result['controller_mode'] = 'actual_cpp_impedance_torque_fixed_nominal'
        result['impedance'] = dict(stiffness_scale=args.impedance_stiffness_scale,
            joint_hard_limits_rad=source['joint_reference_error_limits_rad'],
            max_target_lead_error_rad=cfg['max_target_lead_error_rad'],
            force_pulse={'world_axis':'x','peak_n':10.,'ramp_s':.25,'start_s':1.,'end_s':3.},
            loaded_displacement_m=(loaded-baseline).tolist(), recovery_residual_m=(recovered-baseline).tolist(),
            peak_commanded_torque_nm=dict(zip(impedance.arm_names,np.max(abs(imp[:,4:18]),axis=0).tolist())),
            peak_tcp_translation_error_m=np.max(np.linalg.norm(imp[:,18:30].reshape(-1,2,6)[:,:,:3],axis=2),axis=0).tolist(),
            clipped_steps=int(imp[:,-1].sum()), samples_sha256=digest(output/'impedance.npz'),
            verdict_scope='existing physics balance/tracking checks only; recovery precision, robustness and hardware acceptance are not established')
        result['assumptions']['control'] = 'actual prepared C++ impedance controller; measured joints/virtual local FT at physics dt; fixed nominal, no upper session transport'
        result['assumptions']['impedance_calibration'] = 'same model gravity scaling 1; no LuGre or payload feedforward; sensor at FT frame, TCP at grasp; PT gains 1/.2; stiffness scaled, damping scaled by sqrt(scale)'
        if not np.isfinite(np.r_[loaded,recovered]).all():
            result['status'] = status = 'failed_impedance_window'
    (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    world.stage.GetRootLayer().Export(str(output/'scene.usda'))
    print('RESULT', json.dumps(result), flush=True)
    if status != 'passed':
        raise RuntimeError(status)
    return world


def capture_paused(app, world, output):
    """Capture the completed scene while physics is paused (no extra steps)."""
    import asyncio
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    world.pause()
    path = output/'viewport.png'
    capture = capture_viewport_to_file(get_active_viewport(), str(path))
    pending = asyncio.ensure_future(capture.wait_for_result())
    deadline = time.monotonic()+20.
    while not pending.done() and app.is_running() and time.monotonic() < deadline:
        app.update()
    import omni.kit.renderer_capture
    renderer = omni.kit.renderer_capture.acquire_renderer_capture_interface()
    while not path.is_file() and app.is_running() and time.monotonic() < deadline:
        renderer.wait_async_capture()
        app.update()
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError('VIEWPORT_CAPTURE_FAILED')


class ReplayInterrupt(Exception):
    """A local GUI replay/reset request, not a controller failure."""


class ReplayPanel:
    def __init__(self, args, motion):
        import omni.ui as ui
        self.motions = [('Flip' if 'flip' in str(args.motion) else 'Sweep', args.motion)]
        self.motions += [(name, Path(path).resolve()) for name, path in args.replay_motion]
        self.request = None
        self.active = False
        self.paused = True
        speeds = self.speeds = tuple(motion.get('presentation', {}).get('allowed_speeds', (1, 2, 4, 8)))
        initial = motion.get('presentation', {}).get('speed', 1)
        self.window = ui.Window('Shovel Replay', width=310, height=190)
        self.window.position_x = 1040
        self.window.position_y = 100
        with self.window.frame:
            with ui.VStack(spacing=8):
                with ui.HStack(height=28):
                    ui.Label('Motion', width=65)
                    self.motion = ui.ComboBox(0, *(name for name, _ in self.motions))
                with ui.HStack(height=28):
                    ui.Label('Speed', width=65)
                    self.speed = ui.ComboBox(speeds.index(initial) if initial in speeds else 0, *(f'{s}x' for s in speeds))
                with ui.HStack(height=32, spacing=6):
                    ui.Button('Play / Replay', clicked_fn=self.play)
                    ui.Button('Pause / Resume', clicked_fn=self.pause)
                ui.Button('Reset', height=28, clicked_fn=lambda: self.start(True))
                self.status = ui.Label('Ready')
        self.request = (*self.selection(), not args.replay_start)

    def selection(self):
        return (self.motion.model.get_item_value_model().as_int,
                self.speeds[self.speed.model.get_item_value_model().as_int])

    def start(self, paused=False):
        self.request = (*self.selection(), paused)

    def play(self):
        if self.active and self.paused and self.selection() == self.current:
            self.paused = False
            self.status.text = 'Playing'
        else:
            self.start()

    def pause(self):
        if self.active:
            self.paused = not self.paused
            self.status.text = 'Paused' if self.paused else 'Playing'

    def checkpoint(self, app, world):
        if self.request is not None:
            raise ReplayInterrupt()
        started = time.monotonic()
        if self.paused:
            world.pause()
            while app.is_running() and self.paused and self.request is None:
                app.update()
                time.sleep(.01)
            if self.request is not None:
                raise ReplayInterrupt()
            world.play()
        return time.monotonic()-started


def replay_controls(app, args, motion, cfg):
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import create_new_stage
    panel = ReplayPanel(args, motion)
    args._replay_panel = panel
    root = args.output
    sequence = 0
    world = None
    while app.is_running():
        if panel.request is None:
            app.update()
            time.sleep(.01)
            continue
        selected, speed, paused = panel.request
        panel.request = None
        panel.current = (selected, speed)
        panel.paused, panel.active = paused, True
        panel.status.text = 'Ready' if paused else 'Playing'
        try:
            if world is not None:
                world.stop()
                world.clear()
                World.clear_instance()
                create_new_stage()
                app.update()
            source = panel.motions[selected][1]
            original = load_motion(source)
            previous = original.get('presentation', {}).get('speed', 1)
            sequence += 1
            run = root / f'replay-{sequence:04d}'
            run.mkdir()
            for sample in original['samples']:
                sample['t'] *= previous/speed
            original['duration_s'] *= previous/speed
            dense_path = source.parent/'joint_plan.npz'
            if dense_path.is_file():
                with np.load(dense_path) as dense:
                    np.savez_compressed(run/'joint_plan.npz', names=dense['names'],
                        executed=dense['executed'], dt=float(dense['dt'])*previous/speed)
            original['presentation'] = dict(speed=speed, source_motion=str(source),
                source_sha256=digest(source), purpose='Isaac native GUI reference replay')
            args.motion = run/'motion.json'
            args.motion.write_text(json.dumps(original)+'\n')
            args.output = run/'output'
            args.output.mkdir()
            current_motion = load_motion(args.motion)
            world = simulate(app, args, current_motion, json.loads(json.dumps(cfg)))
            world.pause()
            panel.active = False
            panel.status.text = 'Completed'
            print('REPLAY_COMPLETED', panel.motions[selected][0], speed, flush=True)
        except ReplayInterrupt:
            world = World.instance()
            if world is not None:
                world.pause()
            print('REPLAY_RESET_REQUESTED', flush=True)
        except Exception as exc:
            world = World.instance()
            if world is not None:
                world.pause()
            panel.active = False
            panel.status.text = 'Replay error - see terminal'
            traceback.print_exc()
            (args.output/'failure.json').write_text(json.dumps({'error':str(exc)})+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('motion', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_shovel_physics.json')
    parser.add_argument('--case', default='baseline')
    parser.add_argument('--duration', type=float, help='Optional shorter simulated motion interval for diagnostics')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--closed-loop', action='store_true', help='Feed measured PhysX joints to production QP/executor')
    parser.add_argument('--load-feedback', action='store_true', help='Estimate grip deflection from calibrated virtual F/T; simulation only')
    parser.add_argument('--input-stop-at', type=float, help='Stop new operator packets at simulation time')
    parser.add_argument('--feedback-stop-at', type=float, help='Freeze controller feedback and its timestamp')
    parser.add_argument('--cancel-at', type=float, help='Request a latched shared-executor stop')
    parser.add_argument('--wrench-stop-at', type=float, help='Freeze virtual force sensor packets and timestamps')
    parser.add_argument('--keep-open', action='store_true', help='Keep the completed GUI scene paused for inspection')
    parser.add_argument('--replay-controls', action='store_true', help='Isaac built-in replay controls; GUI reference mode only')
    parser.add_argument('--replay-motion', action='append', nargs=2, default=[], metavar=('NAME','PATH'))
    parser.add_argument('--replay-start', action='store_true', help='Start the first native GUI replay immediately')
    parser.add_argument('--record-video', action='store_true', help='Embed no overlays: record the live viewport at simulation-time intervals')
    parser.add_argument('--video-fps', type=int, default=20, help='Recording rate; must divide the physics step grid (default 20)')
    parser.add_argument('--grasp-test', choices=('hold','release'), help='Static contact diagnostic; release opens both jaws at t=1 s')
    parser.add_argument('--grasp-model', choices=('contact','proxy'), help='Explicit contact model or legacy spring comparison')
    parser.add_argument('--impedance-binary',type=Path,help='Simulation-only C++ stdio torque bridge; fixed nominal force/release experiment')
    parser.add_argument('--impedance-prefix',type=Path,help='Numeric runtime prefix, isolated from Kit libraries')
    parser.add_argument('--impedance-config',type=Path,help='Selected original impedance YAML; hardware file is never modified')
    parser.add_argument('--impedance-reference-config',type=Path,
        default=package_share('snowbot_bringup')/'config/robots/snowbot/bimanual_snowbot_real.json',
        help='Reuse the production named impedance reference bounds; no ROS nodes are started')
    parser.add_argument('--impedance-stiffness-scale',type=float,default=1.)
    # Kit portable arguments are supplied by an isolated SDK launcher.
    args, kit_args = parser.parse_known_args()
    if args.impedance_binary:
        if args.closed_loop or args.replay_controls or args.grasp_test or not args.impedance_prefix or not args.impedance_config:
            parser.error('impedance requires prefix/config and fixed nominal mode without QP/replay/grasp diagnostics')
        if args.duration is None or args.duration < 5. or not np.isfinite(args.impedance_stiffness_scale) or args.impedance_stiffness_scale <= 0.:
            parser.error('impedance requires duration >=5 s and positive finite stiffness scale')
        for name in ('impedance_binary','impedance_prefix','impedance_config','impedance_reference_config'):
            setattr(args,name,getattr(args,name).resolve())
    if args.replay_start and not args.replay_controls:
        parser.error('--replay-start requires --replay-controls')
    if args.replay_controls and (not args.gui or args.closed_loop or args.record_video or args.grasp_test):
        parser.error('--replay-controls requires GUI reference mode without recording or grasp diagnostics')
    if args.record_video and not args.gui:
        parser.error('--record-video requires --gui')
    if not 1 <= args.video_fps <= 60:
        parser.error('--video-fps must be between 1 and 60')
    for name in ('motion', 'output', 'config'):
        setattr(args, name, getattr(args, name).resolve())
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be finite and positive')
    if (args.load_feedback or any(getattr(args,n) is not None for n in ('input_stop_at','feedback_stop_at','cancel_at','wrench_stop_at'))) and not args.closed_loop:
        parser.error('feedback options and fault injection require --closed-loop')
    if any(v is not None and (not np.isfinite(v) or v < 0) for v in
           (args.input_stop_at,args.feedback_stop_at,args.cancel_at,args.wrench_stop_at)):
        parser.error('fault times must be finite and nonnegative')
    if sum(v is not None for v in (args.input_stop_at,args.feedback_stop_at,args.cancel_at,args.wrench_stop_at)) > 1:
        parser.error('inject one fault per run')
    if args.wrench_stop_at is not None and not args.load_feedback:
        parser.error('wrench faults require --load-feedback')
    cfg = json.loads(args.config.read_text())
    if args.impedance_binary:
        # Separate the hanging tool from ground support. Load the actual tracking
        # policy below, after physics-case overrides, without changing gains.
        cfg['diagnostic_ground_drop_m'] = .3
    if args.grasp_model:
        cfg['grasp_model'] = args.grasp_model
    validate_grasp_config(cfg)
    if cfg.get('grasp_model') == 'contact' and args.load_feedback:
        parser.error('contact grasps cannot use the spring inverse load observer')
    if args.grasp_test and (args.closed_loop or cfg.get('grasp_model') != 'contact'):
        parser.error('--grasp-test requires contact grasps and reference mode')
    if args.grasp_test:
        # The ready blade is only 13 mm above the normal floor. Lower the
        # diagnostic floor so a release can demonstrate actual free fall.
        cfg['diagnostic_ground_drop_m'] = .3
    if cfg.get('feedback_velocity_source','native') not in ('native','position_difference'):
        raise ValueError('PHYSICS_VELOCITY_SOURCE')
    if cfg.get('simulation_only') is not True or args.case not in cfg['cases']:
        raise ValueError('SIMULATION_CASE_REQUIRED')
    override = cfg.pop('cases')[args.case]
    cfg.update({k:v for k,v in override.items() if k not in ('density_kg_m3', 'grip_stiffness_scale')})
    if args.impedance_binary:
        for package in ('shared/bindu_contracts','integration/bindu_runtime'):
            sys.path.insert(0,str(ROOT/'src'/package))
        from bindu_runtime.teleop_deployment import apply_execution_tracking
        policy=apply_execution_tracking(json.loads(args.impedance_reference_config.read_text()),
            'impedance',trajectory_reference=True)
        cfg['max_joint_error_rad']=policy['max_reference_error']
        cfg['joint_reference_error_limits']=policy.get('joint_reference_error_limits',{})
        cfg['max_target_lead_error_rad']=policy.get('max_target_lead_error',policy['max_reference_error'])
    frame_steps = 1/(args.video_fps*cfg['physics_dt_s'])
    if args.record_video and not np.isclose(frame_steps, round(frame_steps)):
        parser.error('video frame interval must be an integer number of physics steps')
    if 'density_kg_m3' in override:
        cfg['material']['density_kg_m3'] = override['density_kg_m3']
    for name in ('translation', 'rotation'):
        cfg[f'grip_{name}_stiffness_'+('n_m' if name == 'translation' else 'nm_rad')] *= override.get('grip_stiffness_scale', 1.)
    motion = load_motion(args.motion)
    if cfg.get('grasp_model') == 'contact' and not motion['recipe'].get('check_grasp_contacts'):
        raise ValueError('CONTACT_GRASP_REQUIRES_CHECKED_GRIP_GEOMETRY')
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        raise FileExistsError(args.output)
    args.isaac_version = os.environ.get('ISAAC_VERSION', 'unknown')
    args.source_hashes = {name:digest(path) for name,path in (
        ('config',args.config),('runner',__file__),('math',ROOT/'tools/simulation/shovel_physics.py'))}
    snapshots = args.output/'sources';snapshots.mkdir()
    for path in (Path(__file__),ROOT/'tools/simulation/shovel_physics.py',args.config):
        (snapshots/path.name).write_bytes(path.read_bytes())
    if args.impedance_binary:
        (snapshots/'impedance-reference.json').write_bytes(args.impedance_reference_config.read_bytes())
    if args.record_video:
        (snapshots/'shovel_video.py').write_bytes((ROOT/'tools/simulation/shovel_video.py').read_bytes())
    from isaacsim import SimulationApp
    sys.argv = [sys.argv[0]]+kit_args
    app = SimulationApp({'headless':not args.gui, 'fast_shutdown':True, 'width':1280, 'height':900})
    try:
        import isaacsim
        args.isaac_version = str(Path(isaacsim.__file__).resolve())
        # Record runtime's own version, never infer from the launcher's name.
        from isaacsim.core.version import get_version
        args.isaac_version = list(get_version())
        if args.replay_controls:
            replay_controls(app, args, motion, cfg)
            app.close()
            return
        world = simulate(app, args, motion, cfg)
        if args.gui:
            capture_paused(app, world, args.output)
        if args.gui and args.keep_open:
            print('GUI_PAUSED_AFTER_COMPLETED_EXPERIMENT', flush=True)
            while app.is_running():
                app.update()
                time.sleep(.01)
    except BaseException as exc:
        traceback.print_exc()
        (args.output/'failure.json').write_text(json.dumps({'error':str(exc), 'isaac_version':args.isaac_version})+'\n')
        sys.stdout.flush(); sys.stderr.flush()
        app.close(exit_code=1)
        os._exit(1)
    sys.stdout.flush(); sys.stderr.flush()
    app.close()


if __name__ == '__main__':
    main()
