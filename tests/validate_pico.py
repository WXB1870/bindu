#!/usr/bin/env python3
"""Synthetic PICO -> operation coordinator -> QP/shared executor ROS validation.

Only launches isolated kinematic simulation processes. No hardware mode or IO.
"""
import argparse
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path
import numpy as np
import rclpy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from bindu_interfaces.msg import VRInput,ExecutionState,MotionCommand,RuntimeEvent
from bindu_interfaces.srv import PicoContext
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.vr.receiver import ButtonCounters
from bindu_runtime.common import stamp


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--crash-coordinator',action='store_true');parser.add_argument('--reject-virtual',action='store_true');parser.add_argument('--tool-config');args=parser.parse_args()
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[1];cfg=root/'src/integration/bindu_runtime/config'
    profile=cfg/'snowbot_shovel_sim.json';tool=Path(args.tool_config).resolve() if args.tool_config else cfg/'bimanual_snowbot_shovel.json'
    if args.reject_virtual and not json.loads(tool.read_text())['demo'].get('check_grasp_contacts'):
        raise ValueError('REJECT_VIRTUAL_TEST_REQUIRES_CONFIGURED_GRASP_CONTACT_CHECKS')
    model=root/'src/hardware/bindu_description/urdf'
    run=uuid.uuid4().hex;ns='/pico_'+run[:8]
    # Explicit scene fixture, not a startup preparation action. A must move from
    # this measured perturbation and pass the empty-robot CAD path check.
    initial=json.loads(tool.read_text())['demo']['initial_positions_rad'];initial['body_joint_1']+=.03
    procs=[];logs=[];expected_exits=set();result=dict(simulated=True,namespace=ns,checks=[])
    rclpy.init();node=rclpy.create_node('pico_verifier_'+run[:8])
    views=[];states=[];accepted=[];events=[]
    node.create_subscription(String,ns+'/teleop/pico/state',lambda m:views.append(json.loads(m.data)),100)
    node.create_subscription(ExecutionState,ns+'/execution/state',states.append,100)
    node.create_subscription(MotionCommand,ns+'/execution/accepted',accepted.append,100)
    node.create_subscription(RuntimeEvent,ns+'/events',lambda m:events.append(dict(component=m.component,state=m.state,code=m.code)),1000)
    pub=node.create_publisher(VRInput,ns+'/teleop/vr/raw',10)
    client=node.create_client(PicoContext,ns+'/teleop/pico/context')
    pause_client=node.create_client(SetBool,ns+'/teleop/tool_pause')
    source={'id':'synthetic_pico_'+run,'seq':0,'buttons':[0,0],'grip':[0.,0.],
            'trigger':[0.,0.],'delta':0.,'enabled':True}
    counters=[ButtonCounters(),ButtonCounters()]
    def publish():
        if not source['enabled']:return
        source['seq']+=1;now=node.get_clock().now().nanoseconds/1e9
        for i,side in enumerate(('left','right')):
            p=np.eye(4);p[0,3]=.34*i+source['delta']
            f=counters[i].update(VRFrame(source['id'],source['seq'],now,side,tuple(p.ravel()),
                source['grip'][i],source['trigger'][i],mapping='pico_snowbot_v1',buttons=source['buttons'][i]))
            pub.publish(VRInput(stamp=stamp(now),source_id=f.source_id,runtime_mode='simulation',seq=f.seq,
                side=side,pose=list(f.pose),grip=f.grip,trigger=f.trigger,valid=True,mapping=f.mapping,
                buttons=f.buttons,button_edges=list(f.button_edges)))
    node.create_timer(.02,publish)
    def pump(duration):
        until=time.monotonic()+duration
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
    def wait(predicate,timeout=20.,label='condition'):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return
            if any(p.poll() is not None and p.pid not in expected_exits for p in procs):raise RuntimeError('CHILD_EXITED:'+label)
        raise AssertionError(label+': '+json.dumps(views[-1] if views else {}))
    def phase(value,pending=None,timeout=20.):
        wait(lambda:views and views[-1]['phase']==value and views[-1]['pending']==pending,timeout,'phase '+value)
    def click(side,bit):
        source['buttons'][side]|=1<<bit;pump(.12);source['buttons'][side]&=~(1<<bit);pump(.12)
    def context(load,support):
        for _ in range(3):
            v=views[-1]
            future=client.call_async(PicoContext.Request(source_id=v['source_id'],expected_generation=v['generation'],
                load_state=load,external_support=support))
            wait(future.done,2.,'context reply')
            reply=future.result()
            if reply.success:pump(.15);return
            pump(.15)
        raise AssertionError(reply.code)
    def start(name,extra=(),suffix=''):
        log=(out/(name+suffix+'.log')).open('w');logs.append(log)
        cmd=['ros2','run','bindu_runtime',name,'--ros-args','-r','__ns:='+ns,
             '-p','profile:='+str(profile),'-p','run_id:='+run,'-p','runtime_mode:=simulation',*extra]
        procs.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    def check(name):result['checks'].append(name);print('PASS',name,flush=True)
    try:
        start('execution',['-p','kinematic_initial_positions:='+json.dumps(json.dumps(initial))])
        start('recorder',['-p','output:='+str(out/'recording')])
        common=['-p','tool_config:='+str(tool)]
        start('teleop_mode',common+['-p','supported_sides:=[left,right]','-p','input_source:=vr'])
        start('teleop_tool',common+['-p','model_root:='+str(model),'-p','input_source:=vr'])
        pico_config=cfg/'pico_snowbot_sim.json'
        if args.reject_virtual:
            settings=json.loads(pico_config.read_text());settings['grippers']['right']['virtual']=-.08
            pico_config=out/'invalid_virtual_config.json';pico_config.write_text(json.dumps(settings))
        start('pico_control',common+['-p','model_root:='+str(model),'-p','pico_config:='+str(pico_config)])
        phase('standby',timeout=40.);pump(.5)
        assert not accepted,'unsolicited startup motion'
        check('startup stationary, no unsolicited motion')
        context('empty',False)
        source['buttons'][1]=1
        wait(lambda:accepted,40.,'prepare first bounded segment')
        if args.crash_coordinator:
            coordinator=procs[-1];expected_exits.add(coordinator.pid)
            before=len(accepted);os.killpg(coordinator.pid,signal.SIGKILL);coordinator.wait()
            killed=node.get_clock().now().nanoseconds/1e9
            wait(lambda:states and states[-1].stamp.sec+states[-1].stamp.nanosec/1e9>killed and
                 states[-1].state not in ('RUNNING','STOPPING') and not states[-1].reference.name,3.,'coordinator crash stop')
            pump(.3);assert len(accepted)<=before+1
            check('coordinator crash stops on bounded execution segment while raw input stays live')
            result.update(success=True,accepted_commands=len(accepted),final=views[-1]);return
        source['buttons'][1]=0;phase('standby');pump(.3)
        assert states[-1].state not in ('RUNNING','STOPPING') and not states[-1].reference.name
        check('A release stops preparation with actual feedback')
        source['buttons'][1]=1
        wait(lambda:views[-1]['reason']=='PREPARED' and views[-1]['pending'] is None,30.,'prepare complete')
        source['buttons'][1]=0;pump(.15)
        q=dict(zip(states[-1].joints.name,states[-1].joints.position))
        target=json.loads(tool.read_text())['demo']['initial_positions_rad']
        assert abs(q['body_joint_1']-target['body_joint_1'])<.002
        check('A explicit preparation uses validated path and measured arrival')
        context('empty',True);click(0,2);phase('load_check');click(1,1);phase('loading')
        source['trigger'][0]=1.
        wait(lambda:views[-1]['pending'] is None and views[-1]['gripper_modes']==['open','closed'],10.,'left open')
        source['trigger'][0]=0.
        wait(lambda:views[-1]['pending'] is None and views[-1]['gripper_modes']==['closed','closed'],10.,'left closed')
        source['trigger'][1]=1.
        wait(lambda:views[-1]['pending'] is None and views[-1]['gripper_modes']==['closed','open'],10.,'right open')
        source['trigger'][1]=0.
        wait(lambda:views[-1]['pending'] is None and views[-1]['gripper_modes']==['closed','closed'],10.,'right closed')
        check('left/right Trigger open and release-close independently')
        context('held',True);click(1,1);phase('tool_standby',timeout=40.)
        pump(.15);source['grip']=[1.,1.];phase('following',timeout=15.)
        count=len(accepted);source['delta']=.003;pump(.7)
        assert len(accepted)>count
        check('Squeeze engages real QP and shared execution from current pose')
        source['buttons'][0]=1
        if args.reject_virtual:
            phase('locked',timeout=20.)
            assert 'GRASP_PENETRATION' in views[-1]['reason'],views[-1]
            assert views[-1]['gripper_modes']==['closed','closed'],views[-1]
            q=dict(zip(states[-1].joints.name,states[-1].joints.position))
            assert abs(q['arm_r_joint_8'])<.003
            check('invalid virtual path rejected before changing the right actuator')
            result.update(success=True,accepted_commands=len(accepted),final=views[-1]);return
        wait(lambda:views[-1]['phase']=='following' and views[-1]['grasp_requested']=='right_axial',20.,'virtual follow')
        assert views[-1]['gripper_modes']==['closed','virtual']
        assert views[-1]['physical_gripper_confirmed'] is None
        source['buttons'][0]=0
        wait(lambda:views[-1]['phase']=='following' and views[-1]['grasp_requested']=='rigid',20.,'rigid relock')
        assert views[-1]['gripper_modes']==['closed','closed']
        check('X virtual/relock waits for simulated actuator and QP recapture')
        source['grip']=[0.,1.];phase('tool_standby');pump(.2)
        before=len(accepted);source['delta']=.01;pump(.3);assert len(accepted)==before
        source['grip']=[0.,0.];pump(.15);source['grip']=[1.,1.];phase('following',timeout=12.)
        check('Squeeze release holds and reengagement reanchors without queued displacement')
        source['trigger'][1]=1.;phase('standby');pump(.2)
        assert views[-1]['gripper_modes']==['closed','closed']
        source['trigger'][1]=0.;source['grip']=[0.,0.];pump(.15)
        check('work-time Trigger stops, never fully opens a held tool')
        click(1,1);phase('tool_standby');pump(.1);source['grip']=[1.,1.];phase('following',timeout=12.)
        click(0,1);phase('locked');pump(.2)
        assert not states[-1].reference.name
        source['grip']=[0.,0.];pump(.15);click(1,1);phase('standby')
        pump(.3);before=len(accepted);pump(.3);assert len(accepted)==before
        check('Y locks and B only returns to standby')
        click(1,1);phase('tool_standby');pump(.1);source['grip']=[1.,1.];phase('following',timeout=12.)
        source['enabled']=False;phase('locked',timeout=8.);pump(.25)
        assert not states[-1].reference.name
        source['enabled']=True;pump(.3);before=len(accepted);pump(.3);assert len(accepted)==before
        check('input timeout physically stops; reconnect input does not auto-resume')
        wait(lambda:views[-1]['recording'] is not None,2.,'recording health')
        old_recording=views[-1]['recording']
        source['grip']=[0.,0.];pump(.15);click(1,1);phase('standby')
        click(1,2);wait(lambda:views[-1]['recording']!=old_recording,3.,'record toggle')
        check('right stick toggles actual recorder capture')
        click(1,1);phase('tool_standby');pump(.1);source['grip']=[1.,1.];phase('following',timeout=12.)
        paused=pause_client.call_async(SetBool.Request(data=True));wait(paused.done,2.,'external pause reply')
        assert paused.result().success
        wait(lambda:views[-1]['tool_phase'].startswith('paused:'),5.,'tool paused with lease retained')
        assert states[-1].lease_active
        old_id=views[-1]['coordinator_id'];old_source=views[-1]['source_id'];old=procs[-1];expected_exits.add(old.pid)
        os.killpg(old.pid,signal.SIGINT);wait(lambda:old.poll() is not None,8.,'old coordinator exit')
        source['buttons']=[0,0];source['grip']=[0.,0.]
        start('pico_control',common+['-p','model_root:='+str(model),'-p','pico_config:='+str(pico_config)],suffix='_restart')
        wait(lambda:views[-1]['coordinator_id']!=old_id and views[-1]['phase']=='standby' and views[-1]['handoff_complete'],20.,'held restart standby')
        assert views[-1]['load_state']=='unknown' and not views[-1]['external_support']
        stale=client.call_async(PicoContext.Request(source_id=old_source,expected_generation=views[-1]['generation'],
            load_state='empty',external_support=True));wait(stale.done,2.,'old coordinator context rejection')
        assert not stale.result().success
        before=len(accepted);source['buttons'][1]=1;pump(1.5);source['buttons'][1]=0;pump(.2)
        assert len(accepted)==before
        check('held coordinator restart requires fresh load declaration and never defaults to preparation')
        assert all(m.valid_for<=.4 for m in accepted)
        assert all(m.runtime_mode=='simulation' for m in accepted)
        result.update(success=True,accepted_commands=len(accepted),final=views[-1])
    except Exception as exc:
        result.update(success=False,error=type(exc).__name__+': '+str(exc),final=views[-1] if views else {})
        raise
    finally:
        (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
        (out/'events.json').write_text(json.dumps(events,indent=2)+'\n')
        for p in reversed(procs):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in procs:
            try:p.wait(timeout=8.)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
