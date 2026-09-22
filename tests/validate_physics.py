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


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace',default='/bindu_g1_physics')
    parser.add_argument('--model',type=Path,default=Path('artifacts/g1-physics-model'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    profile=Profile.load(args.model/'g1_physics_sim.json')
    rclpy.init();node=rclpy.create_node('physics_validator');seen={};samples=[];results=[];process=None
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    def feedback(msg):
        seen['physics']=msg
        samples.append({'stamp':msg.stamp.sec+msg.stamp.nanosec/1e9,'sim_time':msg.simulation_time,
            'q':list(msg.joints.position),'dq':list(msg.joints.velocity),
            'xy':[msg.base_pose.position.x,msg.base_pose.position.y],
            'z':msg.base_pose.position.z,'v':msg.base_velocity.linear.x,'w':msg.base_velocity.angular.z,
            'code':msg.code})
    node.create_subscription(SimulationFeedback,args.namespace+'/simulation/feedback',feedback,qos)
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
        until(node,lambda:seen['state'].command_id==cmd.command_id and seen['state'].state in ('SUCCEEDED','FAILED'),seconds=6.)
        assert seen['state'].state=='SUCCEEDED',seen['state'].code
    def drain(seconds):
        end=time.monotonic()+seconds
        until(node,lambda:time.monotonic()>=end,seconds=seconds+1.)
    log=(args.output/'launch.log').open('w')
    try:
        until(node,lambda:'physics' in seen and seen['physics'].ready,seconds=20.)
        assert seen['physics'].profile_hash==profile.digest
        process=subprocess.Popen(['ros2','launch','bindu_runtime','g1_sim.launch.py','namespace:='+args.namespace,
            'profile:='+str((args.model/'g1_physics_sim.json').resolve()),'device_backend:=external_simulation',
            'navigation_enabled:=false','output:='+str((args.output/'episodes').resolve())],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        assert lease_client.wait_for_service(timeout_sec=15.)
        until(node,lambda:'state' in seen and seen['state'].feedback_stamp.sec>0)
        results.append({'case':'physics_feedback_19_axes','passed':len(seen['physics'].joints.name)==19})
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
        log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        (args.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
        (args.output/'measurements.json').write_text(json.dumps(samples)+'\n')
    print(json.dumps(results,indent=2))
    raise SystemExit(not results or not all(r['passed'] for r in results))


if __name__=='__main__':main()
