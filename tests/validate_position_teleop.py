#!/usr/bin/env python3
"""Same PICO operation sequence on kinematic IO and synthetic vendor wire IO.

Requires an isolated localhost ROS domain. Never starts vendor drivers or SOEM.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
import rclpy
from std_msgs.msg import String, Float64MultiArray, UInt64MultiArray
from snowbot_ctl_msgs.msg import RetInformation, JointPositionCommand, CtlInformation, MidNodeCtlMsgSend
from bindu_interfaces.msg import VRInput, ExecutionState, MotionCommand, GripperState
from bindu_interfaces.srv import PicoContext, Lease, SubmitMotion
from bindu_runtime.common import stamp
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.vr.receiver import ButtonCounters
from bindu_hardware.snowbot import GROUPS
from validate_snowbot_adapter import until, call, terminate

ROOT=Path(__file__).resolve().parents[1]
CFG=ROOT/'src/integration/bindu_runtime/config/robots/snowbot'


def scenario(out, mode):
    token=uuid.uuid4().hex[:8];ns='/position_'+token;vendor='/fixture_'+token
    node=rclpy.create_node('position_fixture_'+token)
    states=[];views=[];accepted=[];grippers={};packets=[]
    node.create_subscription(ExecutionState,ns+'/execution/state',states.append,10)
    node.create_subscription(String,ns+'/teleop/state',lambda m:views.append(json.loads(m.data)),10)
    node.create_subscription(MotionCommand,ns+'/execution/accepted',accepted.append,100)
    node.create_subscription(GripperState,ns+'/gripper/state',lambda m:grippers.update({m.side:m}),10)
    source=dict(buttons=[0,0],trigger=[0.,0.],grip=0.,delta=0.,stream=True,seq=0)
    counters=[ButtonCounters(),ButtonCounters()]
    vr=node.create_publisher(VRInput,ns+'/teleop/vr/raw',10)
    fixture=json.loads((CFG/'pico_snowbot_position_sim.json').read_text())
    q=fixture['device_initial_positions_rad'].copy()
    session=0;sample=0;acks=[0,0];hold_mask=0;holds=[0.,0.]
    if mode=='real':
        measured=node.create_publisher(RetInformation,vendor+'/dualArmCtl_fdback',1)
        health=node.create_publisher(String,vendor+'/bindu/servo_health',1)
        control_source=node.create_publisher(String,vendor+'/bindu/control_source',1)
        bridges=[node.create_publisher(cls,vendor+topic,1) for cls,topic in
                 ((CtlInformation,'/ctl_data_pub'),(MidNodeCtlMsgSend,'/upperMonitor_msg_send'))]
        for cls,topic in ((CtlInformation,'/ctl_data_pub'),(MidNodeCtlMsgSend,'/upperMonitor_msg_send')):
            node.create_subscription(cls,vendor+topic,lambda _:None,1)
        def heartbeat(msg):
            nonlocal session
            if len(msg.data)==3:session=msg.data[2]
        node.create_subscription(UInt64MultiArray,vendor+'/bindu/heartbeat',heartbeat,1)
        def arms(msg):
            values=dict(zip(GROUPS['left_arm']+GROUPS['right_arm'],list(msg.l_arm_position)+list(msg.r_arm_position)))
            q.update(values);packets.append('arms')
        node.create_subscription(JointPositionCommand,vendor+'/joint_position_cmd',arms,10)
        node.create_subscription(Float64MultiArray,vendor+'/body/motor_cmd',lambda m:q.update(zip(GROUPS['waist'],m.data)),10)
        node.create_subscription(Float64MultiArray,vendor+'/head/motor_cmd',lambda _:None,1)
        def grip_target(msg):
            nonlocal hold_mask
            d=msg.data;i=int(d[4]);assert len(d)==11 and (int(d[2])<<32|int(d[3]))==session
            assert int(d[1])>acks[i];acks[i]=int(d[1]);packets.append('gripper')
            name=GROUPS[('left','right')[i]+'_tool_axis'][0]
            if d[5]==0:hold_mask|=1<<i;holds[i]=q[name]
            else:hold_mask&=~(1<<i);q[name]=d[6]
        node.create_subscription(Float64MultiArray,vendor+'/bindu/gripper_command',grip_target,10)
    def publish():
        nonlocal sample
        now=node.get_clock().now().nanoseconds*1e-9
        if mode=='real':
            sample+=1;msg=RetInformation();msg.header.stamp=stamp(now)
            for prefix,groups in (('l',('left_arm','left_tool_axis')),('r',('right_arm','right_tool_axis')),('torso',('waist',)),('head',('head',))):
                names=[n for g in groups for n in GROUPS[g]]
                setattr(msg,prefix+'_position',[q[n] for n in names]);setattr(msg,prefix+'_speed',[0.]*len(names))
            measured.publish(msg)
            control_source.publish(String(data=json.dumps(dict(schema=1,stamp=now,source='topic'))))
            health.publish(String(data=json.dumps(dict(schema=1,stamp=now,sample=sample,sample_age=.001,
                operational=True,ready=True,guard_enabled=True,guard_armed=bool(session),guard_latched=False,
                guard_session=session,wkc=40,expected_wkc=40,present_mask=0xfffff,fault_mask=0,
                enable_requested_mask=0xfffff,operation_enabled_mask=0xfffff,gripper_control_enabled=True,
                gripper_latched_mask=0,gripper_sequence=acks,gripper_hold_mask=hold_mask,gripper_hold_position=holds))))
        if source['stream']:
            source['seq']+=1
            for i,side in enumerate(('left','right')):
                pose=(1.,0.,0.,.34*i+source['delta'],0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.)
                f=counters[i].update(VRFrame(token,source['seq'],now,side,pose,source['grip'],source['trigger'][i],
                    mapping='pico_snowbot_v1',buttons=source['buttons'][i]))
                vr.publish(VRInput(stamp=stamp(now),runtime_mode=mode,source_id=f.source_id,seq=f.seq,side=side,
                    pose=list(f.pose),grip=f.grip,trigger=f.trigger,valid=True,mapping=f.mapping,
                    buttons=f.buttons,button_edges=list(f.button_edges)))
    node.create_timer(.01,publish)
    recipe=json.loads((ROOT/f'src/integration/bindu_runtime/config/deployments/snowbot_vr_{"sim" if mode=="simulation" else "real"}.json').read_text())
    recipe['arguments'].update(namespace=ns,vr_enabled=False,command_enabled=True)
    if mode=='real':
        # This is explicitly synthetic calibration, written only under test output.
        gc=json.loads((CFG/'snowbot_grippers_real.json').read_text());gc['calibrated']=True
        gp=out/'fixture-grippers.json';gp.write_text(json.dumps(gc))
        pc=json.loads((CFG/'pico_snowbot_position_real.json').read_text());pc['prepare_positions_rad']=fixture['prepare_positions_rad']
        pp=out/'fixture-pico.json';pp.write_text(json.dumps(pc))
        recipe['arguments'].update(gripper_config=str(gp),pico_config=str(pp),vendor_namespace=vendor,
            **{k:'/'+node.get_name() for k in ('arm_receiver_node','servo_receiver_node','mid_receiver_node')})
    path=out/(mode+'.json');path.write_text(json.dumps(recipe))
    log=(out/(mode+'.log')).open('w')
    process=subprocess.Popen(['ros2','launch','bindu_runtime','teleop_only.launch.py','config:='+str(path)],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    checks=[];observer=None;observer_log=None
    trace_dir=out/(mode+'-diagnostics')
    def pump(seconds):
        end=time.monotonic()+seconds;until(node,lambda:time.monotonic()>end,seconds+1.)
    def wait(predicate, seconds=15.):
        try:until(node,predicate,seconds)
        except AssertionError as exc:raise AssertionError(json.dumps(views[-1] if views else {},ensure_ascii=False)) from exc
    def phase(value):wait(lambda:views and views[-1]['phase']==value and views[-1]['pending'] is None)
    def click(side,bit):
        source['buttons'][side]|=1<<bit;pump(.12);source['buttons'][side]&=~(1<<bit);pump(.12)
    context_client=node.create_client(PicoContext,ns+'/teleop/pico/context')
    def context(load,support):
        v=views[-1];reply=call(node,context_client,PicoContext.Request(source_id=v['source_id'],
            expected_generation=v['generation'],load_state=load,external_support=support))
        assert reply.success,reply.code;pump(.15)
    try:
        wait(lambda:views and views[-1]['phase']=='standby' and len(grippers)==2,40.)
        assert not accepted and not packets,'startup actuation'
        assert states[-1].simulated==(mode=='simulation') and states[-1].runtime_mode==mode
        assert views[-1]['capabilities']['axial_grasp'] is False
        checks.append('same feedback contract, no startup commands, unavailable capability explicit')
        observer_log=(out/(mode+'-observer.log')).open('w')
        observer=subprocess.Popen([sys.executable,str(ROOT/'tools/snowbot_live.py'),'--observe',
            '--namespace',ns,'--run-id',views[-1]['run_id'],
            '--robot-config',str(CFG/'robot_snowbot_real.json'),
            '--runtime-mode',mode,'--device-backend','kinematic' if mode=='simulation' else 'snowbot_vendor',
            '--state-file',str(out/(mode+'-observer.json')),
            '--trace-file',str(trace_dir/'diagnostics.jsonl')],
            stdout=observer_log,stderr=subprocess.STDOUT,start_new_session=True)
        wait(lambda:(trace_dir/'diagnostics-health.json').exists())
        leases=node.create_client(Lease,ns+'/execution/lease')
        rejected=call(node,leases,Lease.Request(operation='acquire',owner='contract_test',resources=['left_arm']))
        assert not rejected.ok and 'COUPLED' in rejected.code,rejected.code
        submit=node.create_client(SubmitMotion,ns+'/execution/submit')
        rejected=call(node,submit,SubmitMotion.Request(command=MotionCommand(mode='joint_reference_segment',runtime_mode=mode)))
        assert not rejected.accepted and 'UNTIMED_JOINT_TARGET' in rejected.code,rejected.code
        checks.append('coupled arm ownership and unsupported timed trajectory rejected on both backends')
        assert views[-1]['load_state']==('empty' if mode=='simulation' else 'unknown')
        if mode!='simulation':context('empty',False)
        source['buttons'][1]=1
        wait(lambda:views[-1]['reason']=='PREPARED',40.)
        source['buttons'][1]=0;pump(.3)
        assert accepted and all(m.mode=='joint_target' and not m.points for m in accepted)
        assert all(m.runtime_mode==mode for m in accepted)
        checks.append('A preparation uses untimed joint_target and measured arrival')
        click(0,2);phase('load_check');context('empty',True);click(1,1);phase('loading')
        source['trigger']=[1.,1.]
        wait(lambda:views[-1]['gripper_modes']==['open','open'] and views[-1]['pending'] is None)
        assert all(g.grasp_state=='unknown' for g in grippers.values())
        source['trigger']=[0.,0.]
        wait(lambda:views[-1]['gripper_modes']==['closed','closed'] and views[-1]['pending'] is None)
        checks.append('Trigger open/close uses shared leased gripper services; grasp stays unknown')
        context('held',True);click(1,1);phase('tool_standby')
        before=len(accepted);source['grip']=1.;phase('following');source['delta']=.006
        wait(lambda:len(accepted)>before+5)
        source['grip']=0.;phase('tool_standby')
        wait(lambda:states and not states[-1].lease_active and states[-1].state not in ('RUNNING','STOPPING'))
        assert not states[-1].stop_failures
        checks.append('B tool check, Squeeze follow, release and fresh stop confirmation')
        source['grip']=1.;phase('following');source['stream']=False;phase('locked')
        wait(lambda:not states[-1].lease_active and states[-1].state not in ('RUNNING','STOPPING'))
        assert not states[-1].stop_failures
        checks.append('input loss cancels follow and releases ownership after stop')
        source.update(stream=True,grip=0.);pump(.4);click(1,1);phase('standby')
        context('empty',True);click(0,2);phase('load_check');click(1,1);phase('loading')
        source['trigger']=[1.,0.]
        wait(lambda:grippers['left'].state=='MOVING')
        source['stream']=False;phase('locked')
        wait(lambda:all(not g.lease_active and g.state not in ('MOVING','STOPPING') for g in grippers.values()))
        assert all(g.state!='FAULT' and g.grasp_state=='unknown' for g in grippers.values())
        checks.append('input loss during gripper operation releases lease and confirms hold without closing')
        terminate(observer,node);observer=None
        diagnostic_health=json.loads((trace_dir/'diagnostics-health.json').read_text())
        assert not diagnostic_health['active'] and not diagnostic_health['error'] and diagnostic_health['dropped']==0,diagnostic_health
        for stream in ('vr-raw','vr-projected','execution-state','gripper-state','mode-state','teleop-state','accepted-commands'):
            assert (trace_dir/(stream+'.jsonl')).stat().st_size>0,stream
        events=[json.loads(line)['data'] for line in (trace_dir/'runtime-events.jsonl').read_text().splitlines()]
        attempts=[json.loads(e['code']) for e in events if e['state']=='PICO_PREPARE_CHECK']
        assert attempts and attempts[0]['accepted'] and all(attempts[0]['checks'].values()),attempts
        checks.append('raw/projected input, commands, feedback and preparation checks persisted without logger queue loss')
        return dict(mode=mode,checks=checks,accepted=len(accepted),vendor_packets=len(packets))
    finally:
        (out/(mode+'-final.json')).write_text(json.dumps(views[-1] if views else {},indent=2))
        if observer:terminate(observer,node)
        if observer_log:observer_log.close()
        terminate(process,node);log.close();node.destroy_node()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--mode',choices=('simulation','real','both'),default='both');args=p.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Isolated localhost ROS domain required')
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    try:
        result=[scenario(args.output,m) for m in (('simulation','real') if args.mode=='both' else (args.mode,))]
        (args.output/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    finally:rclpy.shutdown()

if __name__=='__main__':main()
