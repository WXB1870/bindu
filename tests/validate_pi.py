#!/usr/bin/env python3
"""Loopback ZMQ + ROS 2 integration. Starts/stops only owned simulation processes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import socket
import subprocess
import sys
import time
import uuid
import traceback
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from bindu_interfaces.action import PiSession
from bindu_interfaces.msg import PiObservation, ExecutionState, RecorderHealth, MotionCommand, RuntimeEvent
from bindu_interfaces.srv import Lease
from validate_ros import wait, until, terminate


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0))
        return sock.getsockname()[1]


def run_case(root,output,name,mode='pubsub',fault='',correlate=False,launch=False):
    run=name+'_'+uuid.uuid4().hex[:8]; namespace='/bindu_pi_'+uuid.uuid4().hex[:8]
    folder=output/run;folder.mkdir(parents=True)
    cfg=json.loads((root/'src/integration/bindu_runtime/config/pi_loopback.json').read_text())
    cfg.update(mode=mode,state_bind=f'tcp://127.0.0.1:{free_port()}',command_connect=f'tcp://127.0.0.1:{free_port()}',require_correlation=correlate)
    config=folder/'config.json';config.write_text(json.dumps(cfg))
    handles=[]; processes={}; node=rclpy.create_node('pi_verifier_'+uuid.uuid4().hex[:8])
    seen={'state':None,'health':None,'accepted':[],'events':[],'observations':0}
    publish_enabled=[True]
    def start(key,command):
        log=(folder/(key+'.log')).open('w');handles.append(log)
        processes[key]=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        if launch:
            start('launch',['ros2','launch','bindu_runtime','skeleton.launch.py','namespace:='+namespace,
                  'run_id:='+run,'output:='+str(output/'episodes'),'pi_enabled:=true','pi_config:='+str(config)])
        else:
            for executable in ('execution','recorder','pi_client'):
                start(executable,['ros2','run','bindu_runtime',executable,'--ros-args','-r','__ns:='+namespace,
                    '-p','run_id:='+run,'-p','output:='+str(output/'episodes'),'-p','pi_config:='+str(config)])
        peerfault=fault if fault not in ('cancel','stale_observation','recorder_loss') else ''
        start('peer',[sys.executable,str(root/'tests/pi_peer.py'),'--config',str(config),
                      '--output',str(folder/'peer.json'),'--fault',peerfault]+(['--correlate'] if correlate else []))
        subs=[node.create_subscription(ExecutionState,namespace+'/execution/state',lambda m:seen.update(state=m),10),
              node.create_subscription(RecorderHealth,namespace+'/recorder/health',lambda m:seen.update(health=m),5),
              node.create_subscription(MotionCommand,namespace+'/execution/accepted',lambda m:seen['accepted'].append(m),512),
              node.create_subscription(RuntimeEvent,namespace+'/events',lambda m:seen['events'].append(m),512)]
        pub=node.create_publisher(PiObservation,namespace+'/vla/pi/observation',1)
        image=Image(height=224,width=224,encoding='rgb8',step=224*3,data=bytes(224*224*3))
        def observation():
            state=seen['state']
            if state is None or not publish_enabled[0]:return
            seen['observations']+=1
            image.header.stamp=state.feedback_stamp
            pub.publish(PiObservation(observation_id=run+'_obs_'+str(seen['observations']),joints=state.joints,
                  image_names=['left_wrist_rgb','right_wrist_rgb','front_rgb'],images=[image,image,image],simulated=True))
        timer=node.create_timer(.04,observation)
        client=ActionClient(node,PiSession,namespace+'/vla/pi/session')
        assert client.wait_for_server(timeout_sec=12),'no Pi action server'
        until(node,lambda:seen['state'] and seen['health'] and seen['health'].ready)
        until(node,lambda:seen['observations']>4)
        goal=wait(node,client.send_goal_async(PiSession.Goal(task_id=run,prompt='pick water',resource_group='arm',duration=2.)))
        assert goal.accepted,'Pi goal rejected'
        future=goal.get_result_async()
        if fault in ('cancel','stale_observation','recorder_loss'):
            until(node,lambda:len(seen['accepted'])>=5)
            if fault=='cancel':
                assert wait(node,goal.cancel_goal_async()).goals_canceling
            elif fault=='stale_observation':publish_enabled[0]=False
            else:
                os.killpg(processes['recorder'].pid,signal.SIGKILL);processes['recorder'].wait(timeout=3)
        elif not fault:
            until(node,lambda:len(seen['accepted'])>=5)
            lease=node.create_client(Lease,namespace+'/execution/lease')
            assert lease.wait_for_service(timeout_sec=3)
            busy=wait(node,lease.call_async(Lease.Request(operation='acquire',owner='competitor',resources=['arm'])))
            assert not busy.ok and busy.code=='BUSY'
        response=wait(node,future,8)
        result=response.result
        if not fault:
            assert result.success and response.status==4 and result.code=='PI_SESSION_COMPLETE',result
            assert result.accepted_commands>=5
        elif fault=='cancel':
            assert not result.success and response.status==5 and result.code=='CANCELED',result
        else:
            assert not result.success and response.status==6,result
            if fault in ('stale','missing','wrong_session','malformed'):
                assert result.code=='PI_COMMAND_TIMEOUT' and not seen['accepted'],result
                expected={'stale':'PI_STALE_OR_FUTURE_COMMAND','missing':'PI_MISSING_JOINT',
                          'wrong_session':'PI_SESSION_MISMATCH','malformed':'Expecting value'}[fault]
                assert any(expected in e.code for e in seen['events']),expected
            elif fault=='disconnect':assert 'COMMAND_TIMEOUT' in result.code,result
            elif fault=='stale_observation':assert 'PI_OBSERVATION_STALE_OR_UNSYNCED' in result.code,result
            elif fault=='recorder_loss':assert result.code=='RECORDER_UNAVAILABLE',result
        assert 'STOP_UNCONFIRMED' not in result.code,result
        until(node,lambda:len(seen['accepted'])>=result.accepted_commands)
        assert result.accepted_commands==len(seen['accepted']), (result.accepted_commands,len(seen['accepted']))
        until(node,lambda:seen['state'] and not seen['state'].reference.name)
        assert all(abs(q-.2)<.02 for q in seen['state'].joints.position[:2]) if not fault else True
        peer=json.loads((folder/'peer.json').read_text());assert peer['observations']>0
        for command in seen['accepted']:
            assert command.mode=='joint_target' and command.resource_group=='arm'
            assert command.joint_names==['arm_joint_1','arm_joint_2']
            assert bool(command.observation_id)==correlate
        # Confirm cancellation/release revoked the old lease and ownership is reusable.
        lease=node.create_client(Lease,namespace+'/execution/lease')
        assert lease.wait_for_service(timeout_sec=3)
        new=wait(node,lease.call_async(Lease.Request(operation='acquire',owner='after-pi',resources=['arm'])))
        assert new.ok,new
        assert wait(node,lease.call_async(Lease.Request(operation='release',lease_id=new.lease_id,epoch=new.epoch))).ok
        return {'case':name,'passed':True,'simulated':True,'code':result.code,'accepted':result.accepted_commands,
                'received_references':len(seen['accepted']),'observations':peer['observations'],'mode':mode,'correlated':correlate}
    finally:
        for process in processes.values():terminate(process)
        node.destroy_node()
        for handle in handles:handle.close()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='artifacts/pi-verification');parser.add_argument('--case',action='append')
    args=parser.parse_args();root=Path(__file__).resolve().parents[1];output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']='116';os.environ['ROS_LOCALHOST_ONLY']='1'
    import zmq,numpy
    inputs={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for folder in ('src','tests') for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    (output/'environment.json').write_text(json.dumps({'platform':platform.platform(),'python':sys.version,'ros_distro':os.environ.get('ROS_DISTRO'),
        'pyzmq':zmq.__version__,'numpy':numpy.__version__,'source_sha256':inputs},indent=2))
    cases=[('pubsub_legacy',{}),('pull_legacy',{'mode':'pull'}),('pubsub_correlated',{'correlate':True}),
           ('pull_correlated',{'mode':'pull','correlate':True}),('pi_launch',{'launch':True})]
    cases += [(fault,{'fault':fault,'correlate':fault=='wrong_session'}) for fault in
              ('cancel','disconnect','stale','missing','wrong_session','malformed','stale_observation','recorder_loss')]
    if args.case:cases=[item for item in cases if item[0] in args.case]
    assert cases
    rclpy.init();results=[]
    try:
        for name,kwargs in cases:
            try:result=run_case(root,output,name,**kwargs)
            except Exception as exc:result={'case':name,'passed':False,'error':str(exc),'traceback':traceback.format_exc()}
            results.append(result);print(json.dumps(result),flush=True)
            (output/'results.json').write_text(json.dumps({'passed':all(r['passed'] for r in results),'cases':results},indent=2))
    finally:rclpy.shutdown()
    raise SystemExit(0 if all(r['passed'] for r in results) else 1)


if __name__=='__main__':main()
