#!/usr/bin/env python3
"""Actual Nav2 + external physics fixture. Does not launch or command hardware.

The caller supplies embodiment/profile/namespace via navigation_physics.launch.
Raw scan, pose, commands and actions are retained alongside the runtime recorder.
"""
import argparse
import gzip
import json
import math
from pathlib import Path
import time
import traceback
import uuid
import os
import signal
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped
from bindu_interfaces.action import NavigateToSite
from bindu_interfaces.msg import ExecutionState, RuntimeEvent, MotionCommand, NavigationPose, SimulationFeedback
from validate_ros import until, wait


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--namespace',default='/bindu_navigation')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--sessions',type=Path,required=True)
    parser.add_argument('--cases',nargs='+',default=['roundtrip','cancel','detour','blocked','pose_loss','process_loss'])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('nav2_physics_validator_'+uuid.uuid4().hex[:8]);ns=args.namespace
    seen={};samples=[];events=[];accepted=[];results=[]
    raw=gzip.open(args.output/'observations.jsonl.gz','wt')
    def receive(topic,msg):
        seen[topic]=msg
        raw.write(json.dumps({'topic':topic,'received':time.time(),'data':message_to_ordereddict(msg)})+'\n')
        if topic=='simulation/feedback':
            p=msg.base_pose; q=p.orientation
            samples.append({'stamp':msg.stamp.sec+msg.stamp.nanosec/1e9,'x':p.position.x,'y':p.position.y,
                'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                'v':msg.base_velocity.linear.x,'w':msg.base_velocity.angular.z})
        elif topic=='events':events.append(msg)
        elif topic=='execution/accepted':accepted.append(msg)
    for topic,typ in [('simulation/feedback',SimulationFeedback),('navigation/odom',Odometry),('navigation/scan',LaserScan),('execution/state',ExecutionState),
                      ('navigation/backend/pose',NavigationPose),('events',RuntimeEvent),('execution/accepted',MotionCommand),('navigation/scene_status',String),('navigation/backend/ready',Bool)]:
        qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT) if topic in ('navigation/backend/pose','simulation/feedback') else 100
        node.create_subscription(typ,ns+'/'+topic,lambda m,t=topic:receive(t,m),qos)
    action=ActionClient(node,NavigateToSite,ns+'/navigation/navigate_to_site')
    scene=node.create_publisher(String,ns+'/navigation/scene_command',1)
    fault=node.create_publisher(String,ns+'/navigation/sensor_fault',1)
    def drain(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def setscene(value):
        seen.pop('navigation/scene_status',None);scene.publish(String(data=value))
        until(node,lambda:seen.get('navigation/scene_status') and seen['navigation/scene_status'].data==value)
        drain(.4)
    def navigate(site,case='normal'):
        until(node,lambda:seen.get('navigation/backend/ready') and seen['navigation/backend/ready'].data,seconds=30)
        start=len(samples);command_start=len(accepted);event_start=len(events);started=time.monotonic()
        goal=wait(node,action.send_goal_async(NavigateToSite.Goal(task_id=case+'_'+uuid.uuid4().hex[:8],site=site)))
        assert goal.accepted,'site action rejected'
        future=goal.get_result_async();injected=False;frozen=None;injection_pose=None;plan_file=None;initial_lane=None
        while not future.done() and time.monotonic()-started<130:
            rclpy.spin_once(node,timeout_sec=.02)
            moving=samples and (abs(samples[-1]['v'])>.02 or abs(samples[-1]['w'])>.08)
            if not injected and moving and case in ('cancel','pose_loss','process_loss','detour_mid'):
                injection_pose=samples[-1].copy();injected=True
                if case=='cancel':assert wait(node,goal.cancel_goal_async()).goals_canceling
                elif case=='pose_loss':fault.publish(String(data='pose_loss'))
                elif case=='detour_mid':
                    active=max(args.sessions.glob('*/session.json'),key=lambda p:p.stat().st_mtime)
                    plan_file=active.parent/'plans.jsonl'
                    initial=json.loads(plan_file.read_text().splitlines()[0])['points']
                    initial_lane=min(initial,key=lambda p:abs(p[0]-1.5))[1]
                    setscene('detour' if initial_lane>0 else 'detour_lower')
                else:
                    active=sorted(args.sessions.glob('*/session.json'),key=lambda p:p.stat().st_mtime)[-1]
                    frozen=json.loads(active.read_text())['pids'][0]
                    os.killpg(frozen,signal.SIGKILL)
        try:
            assert future.done(),'navigation did not terminate'
            response=future.result();drain(.5)
            final=samples[-1];assert time.time()-final['stamp']<.2,'stale physical stop evidence'
            target=(3.,0.,0.) if site=='pickup' else (0.,0.,0.)
            position=math.hypot(final['x']-target[0],final['y']-target[1]);yaw=abs(math.atan2(math.sin(final['yaw']),math.cos(final['yaw'])))
            metrics={'case':case,'site':site,'success':response.result.success,'status':response.status,'code':response.result.code,
                'seconds':time.monotonic()-started,'start_stamp':samples[start]['stamp'],'end_stamp':final['stamp'],'position_error_m':position,'yaw_error_rad':yaw,
                'final_v':final['v'],'final_w':final['w'],'samples':len(samples)-start,'commands':len(accepted)-command_start,
                'max_abs_y':max(abs(s['y']) for s in samples[start:]),'injected':injected,
                'navigation_errors':[e.code for e in events[event_start:] if e.state=='NAV_ERROR']}
            if injection_pose:
                metrics['injection_stamp']=injection_pose['stamp']
                metrics['distance_after_injection_m']=math.hypot(final['x']-injection_pose['x'],final['y']-injection_pose['y'])
            if case=='detour_mid':
                plans=[json.loads(line) for line in plan_file.read_text().splitlines()]
                lanes=[min(row['points'],key=lambda p:abs(p[0]-1.5))[1] for row in plans if row['points']]
                metrics.update(initial_lane_y=initial_lane,planned_lane_y=lanes)
                assert len(lanes)>1 and any(y*initial_lane<0 for y in lanes[1:]),('no forced replan',metrics)
            if case=='pose_loss':assert response.result.code=='NAV_POSE_STALE',metrics
            if case=='process_loss':assert response.result.code in ('NAV_ACTION_FAILED:900','NAV_INPUT_TIMEOUT'),metrics
            if case in ('normal','detour_mid'):
                assert response.result.success and position<.12 and yaw<.14,metrics
            elif case=='cancel':assert injected and response.status==5,metrics
            else:assert not response.result.success,metrics
            assert abs(final['v'])<.01 and abs(final['w'])<.01,metrics
            if case in ('pose_loss','process_loss'):assert injected,metrics
            metrics['passed']=True
            return metrics
        finally:
            fault.publish(String(data='clear'))
            if frozen:
                try:os.killpg(frozen,signal.SIGCONT)
                except ProcessLookupError:pass
            drain(1.)
    try:
        assert action.wait_for_server(timeout_sec=15)
        until(node,lambda:all(t in seen for t in ('simulation/feedback','navigation/odom','navigation/scan','navigation/backend/pose','execution/state')),seconds=20)
        drain(3.)
        scan=seen['navigation/scan'];assert len(scan.ranges)>=180 and sum(math.isfinite(x) for x in scan.ranges)>100,'missing physical laser returns'
        for case in args.cases:
            try:
                setscene('clear')
                if case=='home':result=navigate('home')
                elif case=='roundtrip':
                    rows=[navigate('pickup'),navigate('home')]
                    assert rows[0]['max_abs_y']>.7,rows
                    result={'case':case,'passed':True,'legs':rows}
                elif case=='detour':
                    rows=[navigate('pickup','detour_mid'),navigate('home')]
                    result={'case':case,'passed':True,'legs':rows}
                elif case=='blocked':
                    setscene('blocked');result=navigate('pickup','blocked');setscene('clear')
                else:
                    result=navigate('pickup',case)
                    navigate('home')
                results.append(result)
            except Exception as exc:
                results.append({'case':case,'passed':False,'error':str(exc),'traceback':traceback.format_exc()})
                print(json.dumps(results[-1]),flush=True)
                break
            print(json.dumps(results[-1]),flush=True)
            (args.output/'results.json').write_text(json.dumps(results,indent=2))
    finally:
        fault.publish(String(data='clear'));scene.publish(String(data='clear'));drain(.2)
        (args.output/'results.json').write_text(json.dumps(results,indent=2))
        (args.output/'trajectory.json').write_text(json.dumps(samples))
        raw.close();node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if len(results)==len(args.cases) and all(r['passed'] for r in results) else 1)

if __name__=='__main__':main()
