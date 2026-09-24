#!/usr/bin/env python3
"""Validate a running Isaac GUI/backend through Bindu's real execution services.

Start tools/run_g1_isaac.py first. This validator owns only its ROS launch.
It uses an isolated namespace, physics feedback and provisional tolerances.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from bindu_contracts.profile import Profile
from bindu_interfaces.msg import ExecutionState, SimulationFeedback, MotionCommand
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from validate_ros import until, wait, terminate


def dynamic_checks(node, args, profile, seen, samples, results, command, send,
                   finished, release, drain, submit, control):
    """Real IK worker + public execution port + measured PhysX joint feedback.

    Cartesian errors are encoder FK in base_link, not independent optical truth.
    No headset or Pi service is involved in these synthetic-input experiments.
    """
    import copy
    import numpy as np
    from bindu_contracts.teleoperation import IKRequest
    from bindu_kinematics.model import ArmModel
    from bindu_kinematics.worker import KinematicsWorker
    from bindu_teleoperation.config import load_config
    from bindu_runtime.common import EVENT_QOS

    root = Path(__file__).resolve().parents[1]
    accepted, states, inputs = [], [], []
    sub = node.create_subscription(MotionCommand, args.namespace+'/execution/accepted',
                                   lambda m: accepted.append(m.command_id), EVENT_QOS)
    def state_sample(m):
        states.append({'stamp':m.stamp.sec+m.stamp.nanosec/1e9,
            'feedback_stamp':m.feedback_stamp.sec+m.feedback_stamp.nanosec/1e9,
            'names':list(m.joints.name),'q':list(m.joints.position),
            'reference_names':list(m.reference.name),'reference':list(m.reference.position),
            'dq':list(m.reference.velocity),'ddq':list(m.reference_accelerations),
            'jerk':list(m.reference_jerks),'state':m.state,'code':m.code})
    state_sub = node.create_subscription(ExecutionState,args.namespace+'/execution/state',state_sample,100)
    publisher = node.create_publisher(MotionCommand,args.namespace+'/execution/targets',8)
    until(node,lambda:publisher.get_subscription_count()>0)
    workers = []
    def positions():
        m = seen['physics']
        assert node.get_clock().now().nanoseconds/1e9-(m.stamp.sec+m.stamp.nanosec/1e9)<.2
        return dict(zip(m.joints.name,m.joints.position))
    def pose_error(actual,target):
        return (float(np.linalg.norm(actual[:3,3]-target[:3,3])),
            float(np.arccos(np.clip((np.trace(target[:3,:3].T@actual[:3,:3])-1)/2,-1.,1.))))
    def stop(cmd, timeout=False):
        if not timeout:
            reply = wait(node,control.call_async(ControlExecution.Request(operation='stop',
                lease_id=cmd.lease_id,epoch=cmd.epoch)))
            assert reply.ok,reply.code
        expected = 'FAILED' if timeout else 'CANCELED'
        until(node,lambda:seen['state'].command_id==cmd.command_id and
              seen['state'].state==expected and not seen['state'].reference.name,seconds=6.)
        if timeout:assert seen['state'].code=='COMMAND_TIMEOUT',seen['state'].code
        first = len(samples);drain(.5)
        names = list(seen['physics'].joints.name)
        indices = [names.index(n) for n in cmd.joint_names]
        tail = samples[first:]
        drift = max(max(s['q'][i] for s in tail)-min(s['q'][i] for s in tail) for i in indices)
        speed = max(abs(tail[-1]['dq'][i]) for i in indices)
        assert drift<.003 and speed<.02,(drift,speed)
        return {'hold_drift_rad':drift,'last_measured_speed_rad_s':speed}
    def online(cmd, values, index, prefix):
        target = copy.deepcopy(cmd)
        target.command_id=prefix+'_'+str(index)
        target.stamp=node.get_clock().now().to_msg()
        target.positions=list(map(float,values))
        target.valid_for=.25
        return target
    try:
        for side in ('left','right'):
            cfg=load_config(root/f'src/integration/bindu_runtime/config/teleop_g1_{side}.json',
                profile,root/'src/hardware/bindu_description/urdf')['kinematics']
            arm=ArmModel(cfg);worker=KinematicsWorker(cfg);workers.append(worker)
            def ready():worker.poll();return worker.ready
            until(node,ready,seconds=30.)
            def solve(target, index):
                measured=positions()
                now=node.get_clock().now().nanoseconds/1e9
                req=IKRequest(side+str(index),1,now,now+.2,arm.names,
                    tuple(measured[n] for n in arm.names),tuple(target.ravel()),
                    tuple(measured[n] for n in arm.context_names))
                assert worker.submit(req)
                reply=[None]
                def got():
                    if reply[0] is None:reply[0]=worker.poll()
                    return reply[0] is not None
                until(node,got,seconds=1.)
                assert node.get_clock().now().nanoseconds/1e9<req.expires,'IK_RESULT_EXPIRED'
                return reply[0]
            measured=positions();q0=np.array([measured[n] for n in arm.names])
            context=tuple(measured[n] for n in arm.context_names)
            before=arm.fk(q0,context)
            guide=q0.copy();guide[0]+=.15;guide[3]+=-.10 if side=='left' else .10
            target=arm.fk(guide,context)
            solved=solve(target,0);assert solved.success,solved.code
            cmd=command(side+'_arm','finite_trajectory');cmd.joint_names=list(arm.names)
            cmd.points=[JointTrajectoryPoint(positions=list(solved.positions),time_from_start=Duration(sec=2))]
            send(cmd);finished(cmd);drain(.2)
            measured=positions()
            actual=arm.fk([measured[n] for n in arm.names],[measured[n] for n in arm.context_names])
            pe,re=pose_error(actual,target)
            movement=float(np.linalg.norm(actual[:3,3]-before[:3,3]))
            assert pe<cfg['position_tolerance'] and re<cfg['rotation_tolerance'] and movement>.01,(pe,re,movement)
            results.append({'case':side+'_ik_physical_arrival','passed':True,
                'position_error_m':pe,'rotation_error_rad':re,'tip_movement_m':movement,'solve_seconds':solved.elapsed})
            release();drain(.2)
            impossible=actual.copy();impossible[0,3]+=10.
            count=len(accepted);bad=solve(impossible,'unreachable')
            assert not bad.success and not bad.positions and len(accepted)==count,bad
            results.append({'case':side+'_unreachable_ik_no_command','passed':True,'code':bad.code})
            if side=='right':continue
            # Continuous reachable Cartesian targets, independently solved from
            # actual feedback in a worker; each result becomes an online target.
            measured=positions();center=np.array([measured[n] for n in arm.names])
            cmd=command('left_arm','joint_target');cmd.joint_names=list(arm.names)
            prefix=uuid.uuid4().hex;start=time.monotonic();errors=[];solve_times=[]
            for i in range(100):
                while time.monotonic()<start+i*.05:rclpy.spin_once(node,timeout_sec=.002)
                values=center.copy();values[0]+=.08*math.sin(i*.05*math.pi/2)
                values[3]+=.04*math.sin(i*.05*math.pi/2)
                measured=positions();ctx=[measured[n] for n in arm.context_names]
                target=arm.fk(values,ctx)
                actual=arm.fk([measured[n] for n in arm.names],ctx)
                errors.append(pose_error(actual,target))
                solved=solve(target,i+1);assert solved.success,(i,solved.code)
                cmd=online(cmd,solved.positions,i,prefix);publisher.publish(cmd)
                solve_times.append(solved.elapsed)
                inputs.append({'case':'cartesian_stream','stamp':cmd.stamp.sec+cmd.stamp.nanosec/1e9,
                    'target':target.ravel().tolist(),'positions':list(cmd.positions),'solve_seconds':solved.elapsed})
                rclpy.spin_once(node,timeout_sec=0.)
            until(node,lambda:sum(x.startswith(prefix) for x in accepted)==100)
            stream_seconds=time.monotonic()-start
            stopped=stop(cmd);release();drain(.2)
            assert max(e[0] for e in errors)<.04 and max(e[1] for e in errors)<.15,errors[-1]
            results.append({'case':'cartesian_ik_stream','passed':True,'sent':100,'accepted':100,
                'nominal_hz':20,'mean_hz':99/stream_seconds,
                'position_rms_m':float(np.sqrt(np.mean(np.array(errors)[:,0]**2))),
                'position_max_m':max(e[0] for e in errors),'rotation_max_rad':max(e[1] for e in errors),
                'solve_p95_seconds':float(np.percentile(solve_times,95)),**stopped})
        # 100 Hz changing/reversing targets; no IK cost in this producer.
        cmd=command('left_arm','joint_target');cmd.joint_names=profile.groups['left_arm']
        center=np.array([positions()[n] for n in cmd.joint_names])
        prefix=uuid.uuid4().hex;start=time.monotonic();start_state=len(states);sent_at=[]
        for i in range(600):
            while time.monotonic()<start+i*.01:rclpy.spin_once(node,timeout_sec=.001)
            q=center.copy();q[0]+=.12*math.sin(i*.01*math.pi/2);q[3]-=.08*math.sin(i*.01*math.pi/2)
            cmd=online(cmd,q,i,prefix);publisher.publish(cmd);sent_at.append(time.monotonic())
            inputs.append({'case':'joint_stream','stamp':cmd.stamp.sec+cmd.stamp.nanosec/1e9,'positions':list(cmd.positions)})
            rclpy.spin_once(node,timeout_sec=0.)
        until(node,lambda:sum(x.startswith(prefix) for x in accepted)==600)
        end_state=len(states)
        stopped=stop(cmd,timeout=True)
        recent=states[start_state:end_state]
        reference=[s for s in recent if s['reference_names']==cmd.joint_names]
        assert len(reference)>300,len(reference)
        for key,limit in [('dq',profile.speed),('ddq',profile.acceleration),('jerk',profile.jerk)]:
            assert all(abs(v)<=limit(n)+1e-5 for s in reference for n,v in zip(cmd.joint_names,s[key])),key
        # Align reference to source measurement time, not subscriber receipt time.
        ref_times=np.array([s['stamp'] for s in reference]);qref=np.array([s['reference'] for s in reference])
        tracking=[]
        for s in reference:
            if ref_times[0]<=s['feedback_stamp']<=ref_times[-1]:
                expected=np.array([np.interp(s['feedback_stamp'],ref_times,qref[:,i]) for i in range(7)])
                actual=np.array([s['q'][s['names'].index(n)] for n in cmd.joint_names])
                tracking.extend(actual-expected)
        assert tracking and max(map(abs,tracking))<.02,max(map(abs,tracking))
        assert np.ptp(qref[:,0])>.08
        results.append({'case':'joint_stream_100hz_and_timeout','passed':True,'sent':600,'accepted':600,
            'mean_hz':599/(sent_at[-1]-sent_at[0]),'interval_p95_ms':float(np.percentile(np.diff(sent_at),95)*1000),
            'reference_tracking_rms_rad':float(np.sqrt(np.mean(np.array(tracking)**2))),
            'reference_tracking_max_rad':max(map(abs,tracking)),
            'reference_speed_max':max(abs(v) for s in reference for v in s['dq']),
            'reference_acceleration_max':max(abs(v) for s in reference for v in s['ddq']),
            'reference_jerk_max':max(abs(v) for s in reference for v in s['jerk']),**stopped})
        release();drain(.2)
        old=online(cmd,center,601,prefix)
        reply=wait(node,submit.call_async(SubmitMotion.Request(command=old)))
        assert not reply.accepted and reply.code=='INVALID_LEASE',reply
        results.append({'case':'old_stream_lease_rejected','passed':True,'code':reply.code})
        cmd=command('left_arm','joint_target');cmd.joint_names=profile.groups['left_arm'];cmd.positions=list(center)
        for label,change,code in [('limit',lambda m:m.positions.__setitem__(0,100.),'JOINT_LIMIT'),
            ('layout',lambda m:setattr(m,'joint_names',list(reversed(m.joint_names))),'JOINT_LAYOUT_MISMATCH')]:
            bad=copy.deepcopy(cmd);change(bad)
            reply=wait(node,submit.call_async(SubmitMotion.Request(command=bad)))
            assert not reply.accepted and reply.code==code,(label,reply)
            results.append({'case':'stream_'+label+'_rejected','passed':True,'code':reply.code})
        release()
    finally:
        for worker in workers:worker.close()
        node.destroy_subscription(sub);node.destroy_subscription(state_sub)
        node.destroy_publisher(publisher)
        (args.output/'dynamic-inputs.json').write_text(json.dumps(inputs)+'\n')
        (args.output/'dynamic-execution.json').write_text(json.dumps(states)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace',default='/bindu_g1_physics')
    parser.add_argument('--model',type=Path,default=Path('artifacts/g1-physics-model'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--suite',choices=('basic','dynamics','soak','boundaries','collision','faults','recovery','vr'),default='basic')
    parser.add_argument('--keep-simulator',action='store_true',help='Keep the final recovery GUI open after validation')
    parser.add_argument('--side',choices=('left','right'),default='left')
    parser.add_argument('--vr-stress',action='store_true',help='Fine/noisy, wide, fast, and abrupt-target controller inputs')
    parser.add_argument('--vr-filter-off',action='store_true',help='Same VR test with input pose filter disabled')
    parser.add_argument('--vr-tls',action='store_true',help='Use a locally trusted ephemeral test certificate and WSS')
    parser.add_argument('--vr-network-delay',type=float,default=0.,help='Extra synthetic one-way network delay in seconds (0..1)')
    parser.add_argument('--soak-seconds',type=float,default=600.)
    parser.add_argument('--soak-amplitude',type=float,default=.45,help='Shoulder oscillation amplitude in rad; elbow uses 60 percent')
    parser.add_argument('--soak-period',type=float,default=12.,help='Oscillation period in seconds')
    args=parser.parse_args()
    if not math.isfinite(args.vr_network_delay) or not 0 <= args.vr_network_delay <= 1:
        parser.error('--vr-network-delay must be finite and between 0 and 1 seconds')
    args.output.mkdir(parents=True,exist_ok=False)
    profile=Profile.load(args.model/'g1_physics_sim.json')
    rclpy.init();node=rclpy.create_node('physics_validator');seen={};samples=[];results=[];process=None
    from collections import deque
    import gzip
    raw=gzip.open(args.output/'measurements.jsonl.gz','wt') if args.suite in ('soak','recovery') else None
    if raw:samples=deque(maxlen=1000)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    physical_namespace=args.namespace
    relay=None
    if args.suite in ('faults','recovery'):
        from physics_faults import FaultRelay
        args.namespace=physical_namespace+'/fault_executor'
        relay=FaultRelay(node,physical_namespace,args.namespace,args.output)
    def feedback(msg):
        seen['physics']=msg
        samples.append({'stamp':msg.stamp.sec+msg.stamp.nanosec/1e9,'sim_time':msg.simulation_time,
            'q':list(msg.joints.position),'dq':list(msg.joints.velocity),
            'xy':[msg.base_pose.position.x,msg.base_pose.position.y],
            'z':msg.base_pose.position.z,'v':msg.base_velocity.linear.x,'w':msg.base_velocity.angular.z,
            'code':msg.code})
        if raw:raw.write(json.dumps(samples[-1])+'\n')
    node.create_subscription(SimulationFeedback,physical_namespace+'/simulation/feedback',feedback,qos)
    node.create_subscription(ExecutionState,args.namespace+'/execution/state',lambda m:seen.update(state=m),10)
    lease_client=node.create_client(Lease,args.namespace+'/execution/lease')
    submit=node.create_client(SubmitMotion,args.namespace+'/execution/submit')
    control=node.create_client(ControlExecution,args.namespace+'/execution/control')
    current=[None]
    def renew():
        if current[0]:lease_client.call_async(Lease.Request(operation='renew',lease_id=current[0].lease_id,epoch=current[0].epoch))
    node.create_timer(.5,renew)
    def acquire(group):
        reply=wait(node,lease_client.call_async(Lease.Request(operation='acquire',owner='physics-test',resources=[group])))
        assert reply.ok,reply;current[0]=reply;return reply
    def release():
        lease=current[0]
        reply=wait(node,lease_client.call_async(Lease.Request(operation='release',lease_id=lease.lease_id,epoch=lease.epoch)))
        assert reply.ok,reply;current[0]=None
    def command(group,mode):
        lease=acquire(group)
        return MotionCommand(schema_version=1,command_id=uuid.uuid4().hex,lease_id=lease.lease_id,epoch=lease.epoch,
            profile_hash=profile.digest,task_id='physics-test',resource_group=group,mode=mode,
            stamp=node.get_clock().now().to_msg(),valid_for=5.)
    def send(cmd):
        reply=wait(node,submit.call_async(SubmitMotion.Request(command=cmd)))
        assert reply.accepted,reply
    def finished(cmd):
        def done():
            if seen['physics'].code=='SIM_EXECUTOR_CHANGED':
                raise AssertionError('SIM_EXECUTOR_CHANGED: restart Isaac before a new validator/executor')
            return seen['state'].command_id==cmd.command_id and seen['state'].state in ('SUCCEEDED','FAILED')
        until(node,done,
              seconds=max(6.,cmd.valid_for+profile.stop_timeout+1.))
        assert seen['state'].state=='SUCCEEDED',seen['state'].code
    def drain(seconds):
        end=time.monotonic()+seconds
        until(node,lambda:time.monotonic()>=end,seconds=seconds+1.)
    log=(args.output/'launch.log').open('w')
    simulator=None;scene_logs=[];scene_generation=0
    def restart_physics(crash=False):
        nonlocal simulator,scene_generation
        if simulator:
            if crash and simulator.poll() is None:
                import os,signal
                os.killpg(simulator.pid,signal.SIGKILL)
                simulator.wait(timeout=10.)
            else:terminate(simulator)
        if crash:return
        scene_generation+=1
        root=Path(__file__).resolve().parents[1]
        scene_log=(args.output/f'scene-{scene_generation}.log').open('w');scene_logs.append(scene_log)
        # The validator owns these processes; never find/kill arbitrary Isaac instances.
        simulator=subprocess.Popen([str(root/'tools/run_g1_isaac.sh'),
            '--namespace',physical_namespace,'--model',str(args.model.resolve()),
            '--navigation-scene',str(root/'src/integration/bindu_runtime/config/navigation_room.json'),
            '--navigation-robot',str(root/'src/integration/bindu_runtime/config/navigation_g1_fixture.json'),
            '--output',str((args.output/f'scene-{scene_generation}').resolve())],
            stdout=scene_log,stderr=subprocess.STDOUT,start_new_session=True)
    def restart_executor():
        nonlocal process
        current[0]=None
        if process:terminate(process)
        process=subprocess.Popen(launch,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        return process
    try:
        if args.suite=='recovery':restart_physics()
        def physics_ready():
            if simulator and simulator.poll() is not None:
                raise RuntimeError('OWNED_SIMULATOR_EXITED:'+str(simulator.returncode))
            return 'physics' in seen and seen['physics'].ready
        until(node,physics_ready,seconds=90. if args.suite=='recovery' else 20.)
        assert seen['physics'].profile_hash==profile.digest
        launch=['ros2','launch','bindu_runtime','g1_sim.launch.py','recording_mode:=normal','namespace:='+args.namespace,
            'profile:='+str((args.model/'g1_physics_sim.json').resolve()),'device_backend:=external_simulation',
            'navigation_enabled:=false','output:='+str((args.output/'episodes').resolve())]
        if args.suite=='vr':
            import socket
            with socket.socket() as sock:
                sock.bind(('127.0.0.1',0));args.vr_port=sock.getsockname()[1]
            launch+=['vr_enabled:=true','side:='+args.side,'port:='+str(args.vr_port)]
            root=Path(__file__).resolve().parents[1]
            cfg=json.loads((root/f'src/integration/bindu_runtime/config/teleop_g1_{args.side}.json').read_text())
            cfg['pose_filter']['enabled']=not args.vr_filter_off
            cfg['kinematics']['collision_model']=str(root/'src/integration/bindu_runtime/config/g1_ik_collision.json')
            config_path=(args.output/'teleop-config.json').resolve();config_path.write_text(json.dumps(cfg,indent=2))
            launch+=['teleop_config:='+str(config_path)]
            if args.vr_tls:
                cert=(args.output/'test-cert.pem').resolve();key=(args.output/'test-key.pem').resolve()
                subprocess.run(['openssl','req','-x509','-newkey','rsa:2048','-nodes','-days','1',
                    '-subj','/CN=localhost','-addext','subjectAltName=IP:127.0.0.1',
                    '-keyout',str(key),'-out',str(cert)],check=True,stdout=log,stderr=log)
                key.chmod(0o600)
                launch+=['cert_file:='+str(cert),'key_file:='+str(key)]
        process=subprocess.Popen(launch,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        assert lease_client.wait_for_service(timeout_sec=15.)
        until(node,lambda:'state' in seen and seen['state'].feedback_stamp.sec>0)
        results.append({'case':'physics_feedback_19_axes','passed':len(seen['physics'].joints.name)==19})
        if args.suite == 'vr':
            from physics_vr import vr_checks
            vr_checks(node,args,profile,seen,results,command,send,finished,release,drain,samples)
        if args.suite == 'faults':
            from physics_faults import fault_checks
            fault_checks(node,profile,seen,results,command,send,release,drain,submit,control,current,relay)
        if args.suite == 'recovery':
            from physics_faults import fault_checks,recovery_checks
            fault_checks(node,profile,seen,results,command,send,release,drain,submit,control,current,relay)
            recovery_checks(node,profile,seen,results,command,send,finished,release,drain,submit,
                current,relay,restart_executor,restart_physics)
        if args.suite == 'boundaries':
            from physics_boundaries import boundary_checks
            boundary_checks(node,args,profile,seen,results,command,send,finished,release,drain)
        if args.suite == 'collision':
            from physics_boundaries import collision_checks
            collision_checks(node,args,profile,seen,results,command,send,finished,release,drain)
        if args.suite == 'soak':
            from physics_stress import soak_checks
            # A transport pose can lie close to a joint limit. Move the two
            # exercised axes into a feasible oscillation range before streaming.
            cmd=command('left_arm','finite_trajectory');cmd.joint_names=profile.groups['left_arm']
            values=dict(zip(seen['physics'].joints.name,seen['physics'].joints.position))
            target=[values[n] for n in cmd.joint_names]
            for index,excursion in ((0,args.soak_amplitude),(3,args.soak_amplitude*.6)):
                low,high=profile.limits[cmd.joint_names[index]]
                assert high-low>2*(excursion+.03),'SOAK_EXCURSION_EXCEEDS_RANGE'
                target[index]=min(high-excursion-.03,max(low+excursion+.03,target[index]))
            cmd.valid_for=8.
            cmd.points=[JointTrajectoryPoint(positions=target,time_from_start=Duration(sec=6))]
            send(cmd);finished(cmd);release();drain(.3)
            results.append({'case':'soak_preparation_arrival','passed':True})
            soak_checks(node,args,profile,seen,results,command,release,drain,control,process)
        if args.suite == 'dynamics':
            dynamic_checks(node, args, profile, seen, samples, results, command, send, finished, release, drain, submit, control)
        if args.suite == 'basic':
            for group in profile.groups:
                before=dict(zip(seen['physics'].joints.name,seen['physics'].joints.position))
                cmd=command(group,'finite_trajectory');cmd.joint_names=profile.groups[group]
                targets=[.015 if profile.limits[n][1]>.015 else -.015 for n in cmd.joint_names]
                cmd.points=[JointTrajectoryPoint(positions=targets,time_from_start=Duration(sec=1))]
                send(cmd);finished(cmd);drain(.2)
                after=dict(zip(seen['physics'].joints.name,seen['physics'].joints.position))
                error=max(abs(after[n]-q) for n,q in zip(cmd.joint_names,targets))
                isolation=max(abs(after[n]-v) for n,v in before.items() if n not in cmd.joint_names)
                movement=max(abs(after[n]-before[n]) for n in cmd.joint_names)
                assert error<profile.feedback_tolerances['joint_position'] and isolation<.004 and movement>.005,(group,error,isolation,movement)
                release();drain(.15)
                results.append({'case':group+'_physical_tracking','passed':True,'max_error_rad':error,'inactive_drift_rad':isolation,'movement_rad':movement})
            cmd=command('left_arm','finite_trajectory');cmd.joint_names=profile.groups['left_arm']
            cmd.points=[JointTrajectoryPoint(positions=[.12 if profile.limits[n][1]>.12 else -.12 for n in cmd.joint_names],time_from_start=Duration(sec=2))]
            send(cmd);drain(.5)
            assert wait(node,control.call_async(ControlExecution.Request(operation='stop',lease_id=cmd.lease_id,epoch=cmd.epoch))).ok
            until(node,lambda:not seen['state'].reference.name and seen['state'].state!='STOPPING',seconds=6.)
            drain(.3);speed=max(abs(v) for v in seen['physics'].joints.velocity)
            assert speed<.02,speed;release()
            results.append({'case':'cancel_measured_joint_stop','passed':True,'max_velocity_rad_s':speed})
            def yaw():
                q=seen['physics'].base_pose.orientation
                return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            for name,v,w in [('forward',.1,0.),('arc',.08,.15)]:
                start=seen['physics'].base_pose.position;xy=(start.x,start.y);a=yaw()
                cmd=command('base','base_velocity');cmd.duration=1.;cmd.velocity.linear.x=v;cmd.velocity.angular.z=w
                send(cmd);finished(cmd);drain(.2)
                end=seen['physics'].base_pose.position
                distance=math.hypot(end.x-xy[0],end.y-xy[1]);turn=math.atan2(math.sin(yaw()-a),math.cos(yaw()-a))
                assert distance>.04,(name,distance)
                if w:assert turn>.05,turn
                assert abs(seen['physics'].base_velocity.linear.x)<.003
                assert abs(seen['physics'].base_velocity.angular.z)<.005
                release();results.append({'case':'differential_'+name,'passed':True,'distance_m':distance,'turn_rad':turn})
            # Freeze only the owned executor: simulator watchdog must stop the base.
            cmd=command('base','base_velocity');cmd.duration=4.;cmd.velocity.linear.x=.1;send(cmd)
            drain(.5)
            import os,signal
            os.killpg(process.pid,signal.SIGSTOP)
            try:
                until(node,lambda:seen['physics'].code=='SIM_COMMAND_TIMEOUT',seconds=2.)
                drain(.4)
                assert abs(seen['physics'].base_velocity.linear.x)<.005
                results.append({'case':'executor_loss_watchdog','passed':True,'linear_velocity':seen['physics'].base_velocity.linear.x})
            finally:os.killpg(process.pid,signal.SIGCONT)
            current[0]=None
    except BaseException as exc:
        import traceback
        status=seen.get('state')
        results.append({'case':'failure','passed':False,'error':str(exc),'traceback':traceback.format_exc(),
            'execution_state':status.state if status else None,'execution_code':status.code if status else None})
    finally:
        current[0]=None
        if process:terminate(process)
        if simulator:
            if args.keep_simulator and results and all(r['passed'] for r in results):
                (args.output/'simulator-process.json').write_text(json.dumps({'pid':simulator.pid,
                    'namespace':physical_namespace,'scene_generation':scene_generation})+'\n')
            else:terminate(simulator)
        for scene_log in scene_logs:scene_log.close()
        if relay:relay.close()
        log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        (args.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
        if raw:raw.close()
        else:(args.output/'measurements.json').write_text(json.dumps(samples)+'\n')
    print(json.dumps(results,indent=2))
    raise SystemExit(not results or not all(r['passed'] for r in results))


if __name__=='__main__':main()
