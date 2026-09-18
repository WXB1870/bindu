#!/usr/bin/env python3
"""VR/IK/ROS simulation integration; owns and stops all processes it starts.

The websocket case uses the real Vuer server with synthetic controller events.
It is not a headset or physical-robot acceptance test.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import uuid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from bindu_interfaces.action import TeleopSession
from bindu_interfaces.msg import VRInput, ExecutionState, MotionCommand, RuntimeEvent
from bindu_interfaces.srv import Lease
from bindu_runtime.common import stamp
from validate_ros import until, wait, terminate


def websocket_peer(port, control, stop, errors):
    from websockets.sync.client import connect
    from msgpack import packb
    try:
        deadline=time.monotonic()+15
        ws=None
        while ws is None and time.monotonic()<deadline and not stop.is_set():
            try:
                ws=connect(f'ws://127.0.0.1:{port}/',open_timeout=1)
            except OSError:
                stop.wait(.1)
        if ws is None:
            raise RuntimeError('Vuer websocket not available')
        with ws:
            while not stop.is_set():
                pose=np.eye(4);pose[2,3]=control['delta']
                state={'squeezeValue':control['grip'],'triggerValue':0.,'aButton':False,'bButton':False}
                value={'left':pose.ravel(order='F').tolist(),'right':np.eye(4).ravel().tolist(),
                       'leftState':state,'rightState':dict(state,squeezeValue=0.)}
                ws.send(packb({'etype':'CONTROLLER_MOVE','key':'motionControllers','value':value},use_bin_type=True))
                stop.wait(.02)
    except Exception as exc:
        errors.append(str(exc))


def run_case(root,output,case):
    run=case+'_'+uuid.uuid4().hex[:8]
    namespace='/bindu_vr_'+uuid.uuid4().hex[:8]
    folder=output/run;folder.mkdir(parents=True)
    processes={};handles=[]
    node=rclpy.create_node('teleop_verifier_'+uuid.uuid4().hex[:8])
    seen={'state':None,'accepted':[],'events':[],'inputs':[]}
    control={'grip':0.,'delta':0.,'enabled':True,'valid':True,'source':'synthetic'}
    stopped=threading.Event();errors=[];peer=None
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    def start(key,cmd):
        log=(folder/(key+'.log')).open('w');handles.append(log)
        processes[key]=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        profile=root/'src/integration/bindu_runtime/config/huawei_v34_left_sim.json'
        if case=='normal':
            start('launch',['ros2','launch','bindu_runtime','teleop.launch.py','namespace:='+namespace,
                            'run_id:='+run,'output:='+str(output/'episodes')])
        for executable in (() if case=='normal' else ('execution','recorder','teleop')):
            start(executable,['ros2','run','bindu_runtime',executable,'--ros-args','-r','__ns:='+namespace,
                              '-p','profile:='+str(profile),'-p','run_id:='+run,
                              '-p','output:='+str(output/'episodes')])
        node.create_subscription(ExecutionState,namespace+'/execution/state',lambda m:seen.update(state=m),10)
        node.create_subscription(MotionCommand,namespace+'/execution/accepted',lambda m:seen['accepted'].append(m),512)
        node.create_subscription(RuntimeEvent,namespace+'/events',lambda m:seen['events'].append(m),512)
        node.create_subscription(VRInput,namespace+'/teleop/vr/input',lambda m:seen['inputs'].append(m),20)
        pub=node.create_publisher(VRInput,namespace+'/teleop/vr/input',1)
        counter=[0]
        def publish():
            if not control['enabled'] or case=='websocket':return
            counter[0]+=1
            pose=np.eye(4);pose[2,3]=control['delta']
            if control.get('rolled'):
                pose[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
            pub.publish(VRInput(stamp=node.get_clock().now().to_msg(),source_id=control['source'],
                seq=counter[0],side='left',pose=pose.ravel().tolist(),grip=control['grip'],valid=control['valid']))
        node.create_timer(.02,publish)
        if case=='websocket':
            start('vr_input',['ros2','run','bindu_runtime','vr_input','--ros-args','-r','__ns:='+namespace,
                             '-p','profile:='+str(profile),'-p','port:='+str(port)])
            peer=threading.Thread(target=websocket_peer,args=(port,control,stopped,errors),daemon=True)
            peer.start()
        client=ActionClient(node,TeleopSession,namespace+'/teleop/session')
        assert client.wait_for_server(timeout_sec=15),'no teleop action'
        ready=node.create_client(Trigger,namespace+'/teleop/ready')
        assert ready.wait_for_service(timeout_sec=5)
        deadline=time.monotonic()+30
        response=None
        readiness_timeouts=0
        while time.monotonic()<deadline:
            request=ready.call_async(Trigger.Request())
            try:
                response=wait(node,request,seconds=1.)
            except AssertionError:
                ready.remove_pending_request(request)
                readiness_timeouts+=1
                continue
            if response.success:break
            rclpy.spin_once(node,timeout_sec=.05)
        assert response and response.success,(response,errors)
        duration=5.5 if case=='clutch' else 3.5
        goal=wait(node,client.send_goal_async(TeleopSession.Goal(task_id=run,resource_group='left_arm',duration=duration)))
        assert goal.accepted,'goal rejected'
        future=goal.get_result_async()
        until(node,lambda:any(e.state=='TELEOP_STARTED' for e in seen['events']))
        # Let the action observe a release before the operator squeezes.
        end=time.monotonic()+.15
        until(node,lambda:time.monotonic()>=end)
        control['grip']=1.
        until(node,lambda:len(seen['accepted'])>=5,seconds=8.)
        lease=node.create_client(Lease,namespace+'/execution/lease')
        assert lease.wait_for_service(timeout_sec=2)
        busy=wait(node,lease.call_async(Lease.Request(operation='acquire',owner='pi-competitor',resources=['left_arm'])))
        assert not busy.ok and busy.code=='BUSY',busy
        control['delta']=-.004
        if case=='cancel':
            assert wait(node,goal.cancel_goal_async()).goals_canceling
        elif case=='disconnect':control['enabled']=False
        elif case=='invalid':control['valid']=False
        elif case=='reconnect':control['source']='new-connection'
        elif case=='unreachable':control['delta']=10.
        elif case=='recorder_loss':
            os.killpg(processes['recorder'].pid,signal.SIGKILL);processes['recorder'].wait(timeout=3)
        elif case=='clutch':
            control['grip']=0.
            until(node,lambda:seen['state'] and not seen['state'].reference.name)
            held=dict(zip(seen['state'].joints.name,seen['state'].joints.position))
            count=len(seen['accepted'])
            control['delta']=.2
            control['rolled']=True  # Change wrist orientation while disengaged.
            end=time.monotonic()+.2
            until(node,lambda:time.monotonic()>=end)
            assert len(seen['accepted'])==count,'commands issued while idle'
            control['grip']=1.
            until(node,lambda:len(seen['accepted'])>count,seconds=4.)
            anchor=seen['accepted'][count]
            assert max(abs(q-held[n]) for n,q in zip(anchor.joint_names,anchor.positions))<.005,'re-anchor jumped'
            target_start=len(seen['events'])
            def new_targets():
                return [json.loads(e.code)['target'] for e in seen['events'][target_start:]
                        if e.state=='TELEOP_IK_RESULT' and json.loads(e.code)['target']]
            until(node,lambda:bool(new_targets()))
            origin=np.array(new_targets()[-1]).reshape(4,4)
            control['delta']=.204
            expected=origin.copy();expected[0,3]-=.004  # OpenXR +Z -> robot -X.
            until(node,lambda:any(np.allclose(np.array(t).reshape(4,4),expected,atol=1e-7)
                                  for t in new_targets()))
        response=wait(node,future,10.)
        result=response.result
        if case in ('normal','websocket','clutch'):
            assert response.status==4 and result.success and result.code=='TELEOP_SESSION_COMPLETE',result
        elif case=='cancel':
            assert response.status==5 and result.code=='CANCELED',result
        else:
            assert response.status==6 and not result.success,result
            expected={'disconnect':('VR_INPUT_TIMEOUT','COMMAND_TIMEOUT'),
                      'invalid':('VR_TRACKING_INVALID',),'reconnect':('VR_CONNECTION_CHANGED',),
                      'unreachable':('IK_RESIDUAL','IK_DISCONTINUITY','IK_SOLVER_FAILED','IK_TIMEOUT'),
                      'recorder_loss':('RECORDER_UNAVAILABLE',)}[case]
            assert any(code in result.code for code in expected),result
        assert 'STOP_UNCONFIRMED' not in result.code,result
        until(node,lambda:len(seen['accepted'])>=result.accepted_commands)
        assert result.accepted_commands==len(seen['accepted'])
        until(node,lambda:not seen['state'].reference.name)
        if case in ('normal','websocket'):
            assert max(abs(q) for q in seen['state'].joints.position)>.0001,'arm did not move'
        new=wait(node,lease.call_async(Lease.Request(operation='acquire',owner='after-vr',resources=['left_arm'])))
        assert new.ok,new
        assert wait(node,lease.call_async(Lease.Request(operation='release',lease_id=new.lease_id,epoch=new.epoch))).ok
        assert seen['inputs'] and not errors,errors
        for command in seen['accepted']:
            assert command.resource_group=='left_arm' and command.joint_names==[f'l_joint_{i}' for i in range(1,8)]
            assert command.observation_id==command.command_id
        if case=='websocket':
            previous_source=seen['inputs'][-1].source_id
            stopped.set();peer.join(timeout=3)
            assert not peer.is_alive()
            stopped.clear()
            control['grip']=0.
            peer=threading.Thread(target=websocket_peer,args=(port,control,stopped,errors),daemon=True)
            peer.start()
            until(node,lambda:seen['inputs'][-1].source_id!=previous_source,seconds=5.)
            assert not errors,errors
        # Stop recorder cleanly before inspecting its durable output.
        if case!='recorder_loss':
            terminate(processes['launch' if case=='normal' else 'recorder'])
            summary=json.loads((output/'episodes'/run/'summary.json').read_text())
            assert summary['writer_complete'],summary
        return {'case':case,'passed':True,'code':result.code,'accepted':result.accepted_commands,
                'readiness_query_timeouts':readiness_timeouts,
                'simulated_devices':True,'input':'synthetic_vuer_websocket' if case=='websocket' else 'synthetic_ros'}
    finally:
        stopped.set()
        if peer:peer.join(timeout=3)
        for process in processes.values():terminate(process)
        node.destroy_node()
        for handle in handles:handle.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cases',nargs='+',default=['normal','websocket','clutch','cancel','disconnect','invalid','reconnect','unreachable','recorder_loss'])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[1]
    rclpy.init();results=[]
    try:
        for case in args.cases:
            try:
                result=run_case(root,args.output,case)
            except Exception as exc:
                import traceback
                traceback.print_exc()
                result={'case':case,'passed':False,'error':str(exc)}
            results.append(result);print(json.dumps(result),flush=True)
            (args.output/'results.json').write_text(json.dumps({'passed':all(r['passed'] for r in results),'cases':results},indent=2))
    finally:rclpy.shutdown()
    raise SystemExit(0 if all(r['passed'] for r in results) else 1)


if __name__=='__main__':main()
