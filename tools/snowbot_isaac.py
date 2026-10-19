#!/usr/bin/env python3
"""Live Snowbot PhysX device. ROS position targets in; measured joints out.

No VR mapping, QP, interpolation, grasp Boolean override, or hardware driver.
The operator's load confirmation places a tool once; subsequent motion/contact
comes from PhysX. Jaw positions are exactly the shared gripper contract.
"""
import argparse
import gc
import json
import math
from pathlib import Path
import signal
import time
import uuid


class RealtimeStatus:
    """Assess completion of a physical step, without redating delayed state."""
    def __init__(self, start, limit):
        self.start, self.limit = start, limit
        self.lag = self.maximum_lag = 0.
        self.fault = ''

    def update(self, wall, simulated, commanded):
        self.lag = wall-self.start-simulated
        self.maximum_lag = max(self.maximum_lag, self.lag)
        changed = not self.fault and commanded and self.lag > self.limit
        if changed:
            self.fault = 'PHYSICS_REALTIME_LAG'
        return changed

    @property
    def ready(self):
        return not self.fault and self.lag <= self.limit


def validate_live_config(cfg):
    if cfg.get('simulation_only') is not True or cfg.get('grasp_model')!='contact':
        raise ValueError('PHYSICS_CONTACT_CONFIG_REQUIRED')
    live=cfg['live']
    bounds={'physics_dt_s':(.001,.005),'feedback_hz':(20,200),
        'max_wall_lag_s':(.02,.2),'command_timeout_s':(.25,.4),
        'gripper_stiffness_nm_rad':(1.,100000.),'gripper_damping_nm_s_rad':(1.,10000.),
        'joint_limit_inset_rad':(.00001,.001)}
    for key,(low,high) in bounds.items():
        value=live[key]
        if type(value) not in (int,float) or not math.isfinite(value) or not low<=value<=high:
            raise ValueError('PHYSICS_LIVE_CONFIG:'+key)
    if type(live['headless']) is not bool:raise ValueError('PHYSICS_LIVE_CONFIG:headless')
    return live


def scene_request(view, tool_present):
    # An interrupted unload changes the coordinator's load to unknown. The
    # later explicit empty confirmation must still remove the existing prop.
    if view.get('load_source') != 'operator_load_confirmation':return None
    if view.get('load_state') == 'held' and not tool_present:return 'install'
    if view.get('load_state') == 'empty' and tool_present:return 'remove'
    return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args,kit_args=parser.parse_known_args()
    args.output=args.output.resolve()
    session=json.loads(args.session.read_text())
    if session['runtime_mode']!='simulation':raise ValueError('PHYSICS_SIMULATION_ONLY')
    cfg=json.loads(Path(session['physics_config']).read_text())
    tool=json.loads(Path(session['tool_config']).read_text())
    pico=json.loads(Path(session['pico_config']).read_text())
    live=validate_live_config(cfg);cfg['physics_dt_s']=float(live['physics_dt_s'])
    cfg['joint_limit_inset_rad']=live['joint_limit_inset_rad']
    cfg['contact_grasp']=dict(cfg['contact_grasp'],
        joint_stiffness_nm_rad=live['gripper_stiffness_nm_rad'],
        joint_damping_nm_s_rad=live['gripper_damping_nm_s_rad'])
    args.output.mkdir(parents=True,exist_ok=False)
    from isaacsim import SimulationApp
    app=SimulationApp(dict(headless=live['headless'],fast_shutdown=True,width=640,height=480))
    stopping=[False]
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stopping.__setitem__(0,True))
    node=None;publisher=None
    try:
        import numpy as np
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from bindu_interfaces.msg import SimulationCommand,SimulationFeedback
        from builtin_interfaces.msg import Time
        from bindu_contracts.profile import Profile
        from bindu_hardware.drivers.physics import PhysicsPositionState
        from run_shovel_physics import build_scene,quaternion,pose
        from shovel_physics import MeasuredJointVelocity
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import UsdGeom,UsdPhysics
        profile=Profile.load(session['profile'])
        if profile.digest!=session['profile_hash']:raise ValueError('PHYSICS_PROFILE_CHANGED')
        recipe=dict(tool['demo'])
        for side in ('left','right'):
            offset=recipe.get(side+'_grip_offset_m',recipe['grip_offset_m'])
            recipe[side+'_grip_in_tool_m']=(np.linalg.inv(np.array(tool[side+'_to_tool']).reshape(4,4)) @
                                           np.r_[offset,1.])[:3].tolist()
        motion=dict(display_urdf=session['display_urdf'],recipe=recipe,
                    left_frame=tool['left_frame'],right_frame=tool['right_frame'])
        world,body,view,contacts,source=build_scene(motion,cfg,args.output)
        names=body.dof_names
        if set(names)!=set(profile.limits):raise ValueError('PHYSICS_JOINT_LAYOUT')
        initial=pico['device_initial_positions_rad']
        if set(initial)!=set(names):raise ValueError('PHYSICS_INITIAL_LAYOUT')
        q0=np.array([initial[n] for n in names])
        body.set_joint_positions(q0);body.set_joint_velocities(np.zeros(len(names)))
        body.apply_action(ArticulationAction(joint_positions=q0))
        # The uninstalled tool is a scene fixture until explicit load confirmation.
        tool_prim=world.stage.GetPrimAtPath('/World/Shovel')
        rigid=UsdPhysics.RigidBodyAPI(tool_prim)
        rigid.CreateKinematicEnabledAttr(True)
        UsdGeom.Imageable(tool_prim).MakeInvisible()
        # Keep it outside the robot/ground while no tool is installed.
        view.set_world_poses(np.array([[0.,0.,-10.]]),np.array([[1.,0.,0.,0.]]),indices=[2])
        for _ in range(30):world.step(render=False)
        simulator_id=uuid.uuid4().hex
        core=PhysicsPositionState(profile,simulator_id,initial,live['command_timeout_s'])
        rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
        node=rclpy.create_node('snowbot_isaac',namespace='/'+session['namespace'].strip('/'))
        from snowbot_isaac_io import PhysicsPublisher
        publisher=PhysicsPublisher(node.get_namespace())
        seq=0;tool_present=False;requests=[]
        measured=dict(zip(names,map(float,body.get_joint_positions())))
        stats=dict(accepted=0,rejected=0,expired=0)
        rejection_codes={}
        def receive(command):
            try:
                if fault:raise ValueError(fault)
                core.accept(command,time.time(),measured);stats['accepted']+=1
            except ValueError as exc:
                stats['rejected']+=1
                code=str(exc)
                rejection_codes[code]=rejection_codes.get(code,0)+1
        node.create_subscription(SimulationCommand,'simulation/command',receive,8)
        def operator(msg):
            try:
                p=json.loads(msg.data)
                if (p.get('run_id')!=session['run_id'] or p.get('profile_hash')!=profile.digest or
                    p.get('runtime_mode')!='simulation' or not 0 <= time.time()-p['stamp'] <= .6):return
                request=scene_request(p,tool_present)
                if request:requests.append(request)
            except (ValueError,KeyError,TypeError):pass
        node.create_subscription(String,'teleop/state',operator,1)
        encoder=MeasuredJointVelocity()
        ordered=[n for group in profile.groups.values() for n in group]
        dt=cfg['physics_dt_s'];start=time.monotonic();sim_t=0.;next_pub=0.;fault=''
        mount=np.array(tool['left_to_tool']).reshape(4,4)
        left_fixed=np.array(source['flange_from_physical_body'][0])
        source.update(run_id=session['run_id'],profile_hash=profile.digest,simulator_id=simulator_id,
                      physics_dt_s=dt,feedback_hz=live['feedback_hz'],grasp_model='contact',
                      initial_positions_rad=initial,servo_and_friction_parameters='unvalidated simulation assumptions')
        (args.output/'source.json').write_text(json.dumps(source,indent=2))
        gc.collect();gc.freeze();gc.disable()
        def stamp(t):
            ns=round(t*1e9);return Time(sec=ns//1000000000,nanosec=ns%1000000000)
        from snowbot_diagnostics import DiagnosticWriter
        trace=DiagnosticWriter(args.output/'diagnostics.jsonl')
        native_start=SimulationManager.get_num_physics_steps()
        start=time.monotonic()
        realtime=RealtimeStatus(start,live['max_wall_lag_s'])
        ready_written=False;next_render=0.;previous_iteration=None;timing_max={}
        try:
            while not stopping[0] and app.is_running():
                iteration=time.monotonic()
                timings=dict(iteration_gap_s=0. if previous_iteration is None else iteration-previous_iteration)
                previous_iteration=iteration
                for _ in range(12):rclpy.spin_once(node,timeout_sec=0.)
                timings['ros_spin_s']=time.monotonic()-iteration
                stage=time.monotonic()
                while requests:
                    request=requests.pop(0)
                    if request=='install' and not tool_present:
                        xyz,quat=view.get_world_poses()
                        home=pose(xyz[0],quat[0]) @ left_fixed @ mount
                        view.set_world_poses(home[None,:3,3],quaternion(home[:3,:3])[None,:],indices=[2])
                        view.set_velocities(np.zeros((1,6)),indices=[2])
                        rigid.GetKinematicEnabledAttr().Set(False)
                        UsdGeom.Imageable(tool_prim).MakeVisible();tool_present=True
                    elif request=='remove' and tool_present:
                        # Explicit unloading parks the scene prop only after
                        # the shared operator flow has required external support.
                        rigid.GetKinematicEnabledAttr().Set(True)
                        view.set_world_poses(np.array([[0.,0.,-10.]]),np.array([[1.,0.,0.,0.]]),indices=[2])
                        UsdGeom.Imageable(tool_prim).MakeInvisible();tool_present=False
                timings['scene_update_s']=time.monotonic()-stage
                stats['expired']+=len(core.expire(time.time(),measured))
                if realtime.update(time.monotonic(),sim_t,bool(core.gate.sender)):
                    fault=realtime.fault
                    for group in profile.groups:core.hold(group,measured)
                stage=time.monotonic()
                body.apply_action(ArticulationAction(joint_positions=np.array([core.targets[n] for n in names])))
                timings['apply_action_s']=time.monotonic()-stage
                before=SimulationManager.get_num_physics_steps()
                stage=time.monotonic()
                world.step(render=False);sim_t+=dt
                timings['physics_step_s']=time.monotonic()-stage
                if SimulationManager.get_num_physics_steps()!=before+1:raise RuntimeError('PHYSICS_STEP_COUNT')
                core.stepped()
                # A slow step must invalidate this very sample, not the next
                # iteration after a freshly stamped ready=true publication.
                if realtime.update(time.monotonic(),sim_t,bool(core.gate.sender)):
                    fault=realtime.fault
                    measured=dict(zip(names,map(float,body.get_joint_positions())))
                    for group in profile.groups:core.hold(group,measured)
                for key,value in timings.items():timing_max[key]=max(timing_max.get(key,0.),value)
                if sim_t>=next_pub:
                    stage=time.monotonic()
                    sampled=time.time()
                    q=body.get_joint_positions();raw_qd=body.get_joint_velocities()
                    qd=encoder.update(sim_t,q,raw_qd)
                    if not np.isfinite(np.r_[q,qd]).all():raise RuntimeError('PHYSICS_NONFINITE')
                    measured=dict(zip(names,map(float,q)));velocity=dict(zip(names,map(float,qd)))
                    seq+=1
                    sample_ready=realtime.ready
                    msg=SimulationFeedback(stamp=stamp(sampled),simulation_time=sim_t,simulator_id=simulator_id,
                        sequence=seq,profile_hash=profile.digest,ready=sample_ready,code=fault or ('OK' if sample_ready else 'PHYSICS_WARMUP'))
                    msg.joints.header.stamp=msg.stamp;msg.joints.name=ordered
                    msg.joints.position=[measured[n] for n in ordered];msg.joints.velocity=[velocity[n] for n in ordered]
                    msg.base_pose.orientation.w=1.
                    timings['sample_s']=time.monotonic()-stage
                    for key,value in timings.items():timing_max[key]=max(timing_max.get(key,0.),value)
                    health=dict(run_id=session['run_id'],runtime_mode='simulation',profile_hash=profile.digest,
                        simulator_id=simulator_id,stamp=sampled,simulation_time=sim_t,sequence=seq,ready=sample_ready,
                        code=msg.code,rtf=sim_t/max(1e-6,time.monotonic()-start),wall_lag_s=realtime.lag,
                        max_wall_lag_s=realtime.maximum_lag,rejection_codes=dict(rejection_codes),
                        applied_sequences=dict(core.applied_sequences),hold_positions=dict(core.hold_positions),
                        tool_present=tool_present,grasp_model='contact',diagnostics=trace.status(),
                        timings=dict(timings),timing_max_s=dict(timing_max),publication=publisher.status(),**stats)
                    xyz,quat=view.get_world_poses()
                    health['tool_pose']=pose(xyz[2],quat[2]).tolist() if tool_present else None
                    stage=time.monotonic()
                    publisher.submit(msg,dict(health))
                    timings['publish_enqueue_s']=time.monotonic()-stage
                    health['timings']=timings
                    for key,value in timings.items():timing_max[key]=max(timing_max.get(key,0.),value)
                    health['timing_max_s']=dict(timing_max)
                    trace.record('physics-feedback',dict(health,joints=measured,velocities=velocity))
                    if not ready_written and realtime.ready and publisher.ready.is_set():
                        (args.output/'ready.json').write_text(json.dumps(dict(simulator_id=simulator_id,profile_hash=profile.digest)))
                        print('SNOWBOT_ISAAC_READY '+simulator_id,flush=True)
                        ready_written=True
                    next_pub+=1/live['feedback_hz']
                pause=sim_t-(time.monotonic()-start)
                if not live['headless'] and time.monotonic()>=next_render:
                    world.render();next_render=time.monotonic()+1/cfg['gui_render_hz']
                if pause>0:time.sleep(min(pause,dt))
        finally:
            trace.close()
            (args.output/'result.json').write_text(json.dumps(dict(stats,
                physics_steps=SimulationManager.get_num_physics_steps()-native_start,published_samples=seq,simulated_s=sim_t,
                wall_s=time.monotonic()-start,code=fault or 'OK',simulator_id=simulator_id,
                max_wall_lag_s=realtime.maximum_lag,rejection_codes=rejection_codes,timing_max_s=timing_max,
                publication=publisher.status(),
                diagnostics=trace.status()),indent=2))
    except BaseException as exc:
        import traceback
        import sys
        traceback.print_exc()
        (args.output/'failure.json').write_text(json.dumps(dict(error=str(exc),traceback=traceback.format_exc())))
        sys.stdout.flush();sys.stderr.flush()
        app.close(exit_code=1)
        import os
        os._exit(1)
    finally:
        if publisher is not None:
            publisher.close();publisher=None
        if node is not None:node.destroy_node()
        try:
            import rclpy
            rclpy.try_shutdown()
        finally:
            # IPC objects were allocated before gc.freeze(). Kit's fast exit
            # bypasses Python finalizers, so release frozen IPC cycles first.
            gc.unfreeze();gc.collect()
            app.close()


if __name__=='__main__':main()
