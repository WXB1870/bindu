#!/usr/bin/env python3
"""N1 ROS action/velocity fixture tests; not a real Nav2 or SLAM acceptance."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import uuid
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from bindu_interfaces.action import FetchDrink, NavigateToSite
from bindu_interfaces.msg import ExecutionState, RecorderHealth, RuntimeEvent, MotionCommand, NavigationPose
from bindu_interfaces.srv import InjectFault, Lease
from validate_ros import wait, until, terminate


CASES=('normal','differential_turn','false_yaw_success','late_old_goal','reject','cancel','abort','input_loss','pose_loss','source_change',
       'false_success','invalid_axis','wrong_map','timeout','backend_loss','feedback_loss','late_accept','lease_busy')


def run_case(root,output,case,g1=False):
    folder=output/(case+'_'+uuid.uuid4().hex[:8]);folder.mkdir(parents=True)
    ns='/bindu_nav_'+uuid.uuid4().hex[:8]
    run=folder.name
    node=rclpy.create_node('nav_verifier_'+uuid.uuid4().hex[:8])
    processes={};logs=[]
    seen={'state':None,'health':None,'pose':None,'events':[],'accepted':[]}
    def start(name,command):
        log=(folder/(name+'.log')).open('w');logs.append(log)
        processes[name]=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        config=json.loads((root/'src/integration/bindu_runtime/config/navigation_sim.json').read_text())
        if case=='timeout':config['timeout']=1.
        if case=='differential_turn':
            config['sites'][0].update(x=.1,y=.04,yaw=.35)
        if case=='false_yaw_success':
            config['sites'][0].update(x=0.,y=0.,yaw=.5)
        cfg=folder/'navigation.json';cfg.write_text(json.dumps(config))
        start('launch',['ros2','launch','bindu_runtime','g1_sim.launch.py' if g1 else 'skeleton.launch.py','recording_mode:=normal','namespace:='+ns,'run_id:='+run,
            'output:='+str(output/'episodes'),'navigation_config:='+str(cfg)]+
            ([] if g1 else ['navigation_provider:=bindu_runtime.navigation:Nav2Navigation']))
        start('peer',[sys.executable,str(root/'tests/navigation_peer.py'),'--ros-args','-r','__ns:='+ns,
                      '-p','case:='+case,'-p','run_id:='+run])
        node.create_subscription(ExecutionState,ns+'/execution/state',lambda m:seen.update(state=m),10)
        node.create_subscription(RecorderHealth,ns+'/recorder/health',lambda m:seen.update(health=m),5)
        node.create_subscription(NavigationPose,ns+'/navigation/backend/pose',lambda m:seen.update(pose=m),QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(RuntimeEvent,ns+'/events',lambda m:seen['events'].append(m),512)
        node.create_subscription(MotionCommand,ns+'/execution/accepted',lambda m:seen['accepted'].append(m),512)
        action=ActionClient(node,NavigateToSite if g1 else FetchDrink,ns+('/navigation/navigate_to_site' if g1 else '/tasks/fetch_drink'))
        assert action.wait_for_server(timeout_sec=15),'no task server'
        until(node,lambda:seen['state'] and seen['health'] and seen['health'].ready and seen['pose'],seconds=15)
        # Discovery of task clients and the source action happens in other nodes.
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        if case=='lease_busy':
            lease=node.create_client(Lease,ns+'/execution/lease')
            assert lease.wait_for_service(timeout_sec=5)
            assert wait(node,lease.call_async(Lease.Request(operation='acquire',owner='other',resources=['base']))).ok
        if g1:
            invalid=wait(node,action.send_goal_async(NavigateToSite.Goal(task_id=run+'_invalid',site='unknown_site')))
            assert not invalid.accepted,'unknown navigation site accepted'
        request=NavigateToSite.Goal(task_id=run,site='pickup') if g1 else FetchDrink.Goal(task_id=run,object_id='drink',strategy='planner')
        goal=wait(node,action.send_goal_async(request))
        assert goal.accepted,'task rejected'
        future=goal.get_result_async()
        if case in ('cancel','backend_loss','feedback_loss'):
            until(node,lambda:any(m.mode=='base_velocity' for m in seen['accepted']))
            if case=='cancel':assert wait(node,goal.cancel_goal_async()).goals_canceling
            elif case=='backend_loss':
                os.killpg(processes['peer'].pid,signal.SIGKILL);processes['peer'].wait(timeout=3)
            else:
                client=node.create_client(InjectFault,ns+'/execution/sim_fault')
                assert client.wait_for_service(timeout_sec=5)
                assert wait(node,client.call_async(InjectFault.Request(fault='base:feedback_loss'))).ok
        response=wait(node,future,seconds=25)
        if g1 and case in ('normal','differential_turn','late_old_goal') and response.result.success:
            returned=wait(node,action.send_goal_async(NavigateToSite.Goal(task_id=run+'_return',site='home')))
            assert returned.accepted
            response=wait(node,returned.get_result_async(),seconds=25)
        if g1:assert all(abs(q)<1e-8 for q in seen['state'].joints.position),'navigation moved articulation joints'
        code=response.result.code
        result={'case':case,'status':response.status,'success':response.result.success,'code':code,
                'nav_succeeded':sum(e.state=='NAV_SUCCEEDED' for e in seen['events']),
                'base_commands':sum(m.mode=='base_velocity' for m in seen['accepted'])}
        if case in ('normal','late_old_goal','differential_turn'):
            assert response.status==4 and response.result.success and result['nav_succeeded']==2,result
            if case=='differential_turn':
                assert any(abs(m.velocity.angular.z)>.1 for m in seen['accepted']),'no turning command'
                assert seen['state'].base_y != 0.,'y feedback never integrated'
                assert math.hypot(seen['state'].base_x,seen['state'].base_y)<=config['position_tolerance']
                assert abs(seen['state'].base_yaw)<=config['yaw_tolerance']
        elif case=='cancel':
            assert response.status==5 and code=='CANCELED',result
        else:
            expected={'reject':'NAV_GOAL_REJECTED','abort':'NAV_ACTION_FAILED','input_loss':'NAV_INPUT_TIMEOUT',
                'pose_loss':'NAV_POSE_STALE','source_change':'NAV_SOURCE_CHANGED','false_success':'NAV_GOAL_POSE_MISMATCH',
                'false_yaw_success':'NAV_GOAL_POSE_MISMATCH',
                'invalid_axis':'NAV_UNSUPPORTED_VELOCITY_AXIS','wrong_map':'NAV_INVALID_POSE','timeout':'NAV_TIMEOUT',
                'backend_loss':'NAV_CANCEL_UNCONFIRMED','feedback_loss':'NAV_STOP_UNCONFIRMED',
                'late_accept':'NAV_CANCEL_UNCONFIRMED','lease_busy':'BUSY'}[case]
            assert response.status==6 and not response.result.success and expected in code,result
        if case in ('reject','lease_busy','late_accept'):
            assert result['base_commands']==0,result
        if case!='feedback_loss':
            until(node,lambda:seen['state'].base_velocity.linear.x==0. and seen['state'].base_velocity.angular.z==0.)
        # No retired source velocity may pass the gate on the return trip.
        assert all(abs(m.velocity.linear.x)<=.15 for m in seen['accepted'] if m.mode=='base_velocity'),result
        before=len(seen['accepted']);end=time.monotonic()+.4
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert len(seen['accepted'])==before,'commands accepted after task termination'
        (folder/'result.json').write_text(json.dumps(result,indent=2))
        return result
    finally:
        (folder/'events.json').write_text(json.dumps([{'state':e.state,'code':e.code,'task_id':e.task_id,'command_id':e.command_id} for e in seen['events']],indent=2))
        for p in processes.values():terminate(p)
        for log in logs:log.close()
        node.destroy_node()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cases',nargs='+',choices=CASES,default=CASES)
    parser.add_argument('--g1',action='store_true')
    args=parser.parse_args();args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();results=[]
    try:
        for case in args.cases:
            try:
                result=run_case(Path(__file__).resolve().parents[1],args.output,case,args.g1);result['passed']=True
            except Exception as exc:
                result={'case':case,'passed':False,'error':str(exc),'traceback':traceback.format_exc()}
            results.append(result);print(json.dumps(result),flush=True)
            (args.output/'results.json').write_text(json.dumps(results,indent=2))
    finally:rclpy.shutdown()
    raise SystemExit(not all(r['passed'] for r in results))


if __name__=='__main__':main()
