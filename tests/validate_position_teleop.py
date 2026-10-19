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
import socket
import threading
import time
import uuid
import rclpy
from std_msgs.msg import String, Float64MultiArray, UInt64MultiArray
from snowbot_ctl_msgs.msg import RetInformation, JointPositionCommand, CtlInformation, MidNodeCtlMsgSend
from bindu_interfaces.msg import VRInput, ExecutionState, MotionCommand, GripperState
from bindu_interfaces.srv import Lease, SubmitMotion
from bindu_runtime.common import stamp
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.vr.receiver import ButtonCounters
from bindu_hardware.snowbot import GROUPS
from validate_snowbot_adapter import until, call, terminate

ROOT=Path(__file__).resolve().parents[1]
CFG=ROOT/'src/integration/bindu_runtime/config/robots/snowbot'


def websocket_input(port, source, stop, errors):
    """Real Vuer wire input for this isolated fixture, including downlink drain."""
    from websockets.sync.client import connect
    from msgpack import packb
    end=time.monotonic()+35.;peer=None
    try:
        while peer is None and not stop.is_set() and time.monotonic()<end:
            try:peer=connect(f'ws://127.0.0.1:{port}/',open_timeout=1.)
            except OSError:stop.wait(.1)
        if peer is None:raise RuntimeError('VUER_CONNECT_TIMEOUT')
        with peer:
            while not stop.is_set():
                if source['stream']:
                    value={}
                    for i,side in enumerate(('left','right')):
                        value[side]=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,
                                     .34*i+source['delta']+source['deltas'][i],0.,0.,1.]
                        buttons=source['buttons'][i]
                        value[side+'State']=dict(squeezeValue=source['grips'][i] if source['grips'] is not None else source['grip'],triggerValue=source['trigger'][i],
                            aButton=bool(buttons&1),bButton=bool(buttons&2),thumbstick=bool(buttons&4))
                    peer.send(packb(dict(etype='CONTROLLER_MOVE',key='motionControllers',value=value),use_bin_type=True))
                for _ in range(4):
                    try:peer.recv(timeout=.001)
                    except TimeoutError:break
                stop.wait(.01)
    except Exception as exc:
        if not stop.is_set():errors.append(type(exc).__name__+':'+str(exc))


def scenario(out, mode, websocket=False, ordinary=False):
    token=uuid.uuid4().hex[:8];ns='/position_'+token;vendor='/fixture_'+token
    node=rclpy.create_node('position_fixture_'+token)
    states=[];views=[];accepted=[];grippers={};packets=[]
    node.create_subscription(ExecutionState,ns+'/execution/state',states.append,10)
    node.create_subscription(String,ns+'/teleop/state',lambda m:views.append(json.loads(m.data)),10)
    node.create_subscription(MotionCommand,ns+'/execution/accepted',accepted.append,100)
    node.create_subscription(GripperState,ns+'/gripper/state',lambda m:grippers.update({m.side:m}),10)
    source=dict(buttons=[0,0],trigger=[0.,0.],grip=0.,grips=None,delta=0.,deltas=[0.,0.],stream=True,seq=0)
    counters=[ButtonCounters(),ButtonCounters()]
    vr=node.create_publisher(VRInput,ns+'/teleop/vr/raw',10)
    fixture=json.loads((CFG/'pico_snowbot_position_sim.json').read_text())
    # Exercise an invalid startup posture even if the deployment example is
    # later calibrated to a valid grasp. Keep this override in the fixture only.
    fixture['device_initial_positions_rad']={n:0. for n in fixture['device_initial_positions_rad']}
    fixture['initial_teleop_mode']='independent_arms' if ordinary else 'bimanual_tool'
    q=fixture['device_initial_positions_rad'].copy()
    if mode=='real' and not ordinary:
        # Synthetic measured posture, deliberately independent of an A command.
        # The actual real deployment continues to use only vendor feedback.
        q.update(fixture['prepare_positions_rad'])
    session=0;sample=0;acks=[0,0];hold_mask=0;holds=[0.,0.]
    if mode=='real':
        vendor_feedback=node.create_publisher(RetInformation,vendor+'/dualArmCtl_fdback',1)
        vendor_health=node.create_publisher(String,vendor+'/bindu/servo_health',1)
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
            vendor_feedback.publish(msg)
            control_source.publish(String(data=json.dumps(dict(schema=1,stamp=now,source='topic'))))
            vendor_health.publish(String(data=json.dumps(dict(schema=1,stamp=now,sample=sample,sample_age=.001,
                operational=True,ready=True,guard_enabled=True,guard_armed=bool(session),guard_latched=False,
                guard_session=session,wkc=40,expected_wkc=40,present_mask=0xfffff,fault_mask=0,
                enable_requested_mask=0xfffff,operation_enabled_mask=0xfffff,gripper_control_enabled=True,
                gripper_latched_mask=0,gripper_sequence=acks,gripper_hold_mask=hold_mask,gripper_hold_position=holds))))
        if source['stream'] and not websocket:
            source['seq']+=1
            for i,side in enumerate(('left','right')):
                pose=(1.,0.,0.,.34*i+source['delta']+source['deltas'][i],0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.)
                f=counters[i].update(VRFrame(token,source['seq'],now,side,pose,
                    source['grips'][i] if source['grips'] is not None else source['grip'],source['trigger'][i],
                    mapping='pico_snowbot_v1',buttons=source['buttons'][i]))
                vr.publish(VRInput(stamp=stamp(now),runtime_mode=mode,source_id=f.source_id,seq=f.seq,side=side,
                    pose=list(f.pose),grip=f.grip,trigger=f.trigger,valid=True,mapping=f.mapping,
                    buttons=f.buttons,button_edges=list(f.button_edges)))
    node.create_timer(.01,publish)
    recipe=json.loads((ROOT/f'src/integration/bindu_runtime/config/deployments/snowbot_vr_{"sim" if mode=="simulation" else "real"}.json').read_text())
    recipe['arguments'].update(namespace=ns,vr_enabled=websocket,command_enabled=True)
    if websocket:
        with socket.socket() as probe:
            probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
        recipe['arguments'].update(host='127.0.0.1',port=port)
    if mode=='real':
        # This is explicitly synthetic calibration, written only under test output.
        gc=json.loads((CFG/'snowbot_grippers_real.json').read_text());gc['calibrated']=True
        gp=out/'fixture-grippers.json';gp.write_text(json.dumps(gc))
        pc=json.loads((CFG/'pico_snowbot_position_real.json').read_text());pc['prepare_positions_rad']=fixture['prepare_positions_rad']
        pc['initial_teleop_mode']=fixture['initial_teleop_mode']
        pp=out/'fixture-pico.json';pp.write_text(json.dumps(pc))
        recipe['arguments'].update(gripper_config=str(gp),pico_config=str(pp),vendor_namespace=vendor,
            **{k:'/'+node.get_name() for k in ('arm_receiver_node','servo_receiver_node','mid_receiver_node')})
    else:
        pp=out/'fixture-pico-simulation.json';pp.write_text(json.dumps(fixture))
        recipe['arguments']['pico_config']=str(pp)
    path=out/(mode+'.json');path.write_text(json.dumps(recipe))
    log=(out/(mode+'.log')).open('w')
    process=subprocess.Popen(['ros2','launch','bindu_runtime','teleop_only.launch.py','config:='+str(path)],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    peer_stop=threading.Event();peer_errors=[];peer=None
    if websocket:
        peer=threading.Thread(target=websocket_input,args=(port,source,peer_stop,peer_errors),daemon=True)
        peer.start()
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
    try:
        wait(lambda:views and views[-1]['phase']=='standby' and len(grippers)==2,40.)
        assert not accepted and not packets,'startup actuation'
        initial=dict(zip(states[-1].joints.name,states[-1].joints.position))
        assert all(n in initial and abs(initial[n]-v)<1e-9
                   for n,v in (q if mode=='real' else fixture['device_initial_positions_rad']).items()),'startup feedback differs from configured fixture'
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
        wait(lambda:(trace_dir/'teleop-state.jsonl').exists() and
                    (trace_dir/'teleop-state.jsonl').stat().st_size>0,30.)
        pump(.3)
        leases=node.create_client(Lease,ns+'/execution/lease')
        rejected=call(node,leases,Lease.Request(operation='acquire',owner='contract_test',resources=['left_arm']))
        assert not rejected.ok and 'COUPLED' in rejected.code,rejected.code
        submit=node.create_client(SubmitMotion,ns+'/execution/submit')
        rejected=call(node,submit,SubmitMotion.Request(command=MotionCommand(mode='joint_reference_segment',runtime_mode=mode)))
        assert not rejected.accepted and 'UNTIMED_JOINT_TARGET' in rejected.code,rejected.code
        checks.append('coupled arm ownership and unsupported timed trajectory rejected on both backends')
        if ordinary:
            assert views[-1]['teleop_mode']=='independent_arms'
            baseline=dict(zip(states[-1].joints.name,states[-1].joints.position))
            source['grips']=[1.,0.];phase('following')
            assert views[-1]['load_state']==('empty' if mode=='simulation' else 'unknown')
            source['deltas'][0]=.004
            wait(lambda:len(accepted)>5 and max(abs(v-baseline[n]) for n,v in
                zip(states[-1].joints.name,states[-1].joints.position) if n in GROUPS['left_arm'])>1e-5)
            measured=dict(zip(states[-1].joints.name,states[-1].joints.position))
            assert max(abs(measured[n]-baseline[n]) for n in GROUPS['right_arm']+GROUPS['waist'])<1e-8
            checks.append('zero posture with unknown or empty load accepts left-only clutch; right arm and waist remain fixed')
            source['grips']=[1.,1.]
            wait(lambda:views[-1]['tool_phase']=='follow:independent:3')
            source['deltas'][1]=-.003;pump(.4)
            source['grips']=[0.,1.]
            wait(lambda:views[-1]['tool_phase']=='follow:independent:2')
            held=dict(zip(states[-1].joints.name,states[-1].joints.position))
            source['deltas'][0]=.08;source['deltas'][1]=-.006;pump(.5)
            measured=dict(zip(states[-1].joints.name,states[-1].joints.position))
            assert max(abs(measured[n]-held[n]) for n in GROUPS['left_arm']+GROUPS['waist'])<.002
            assert max(abs(measured[n]-held[n]) for n in GROUPS['right_arm'])>1e-5
            checks.append('releasing one clutch holds that arm while the other follows, with no waist motion')
            source['grips']=[1.,1.]
            wait(lambda:views[-1]['tool_phase']=='follow:independent:3')
            recaptured=dict(zip(states[-1].joints.name,states[-1].joints.position));pump(.2)
            measured=dict(zip(states[-1].joints.name,states[-1].joints.position))
            assert max(abs(measured[n]-recaptured[n]) for n in GROUPS['left_arm'])<.002
            checks.append('paused-hand displacement is discarded when only that hand recaptures its baseline')
            source['grips']=[0.,0.];phase('tool_standby');pump(.3)
            count=len(accepted);click(1,1)
            wait(lambda:views[-1]['phase']=='standby' and views[-1]['pending'] is None and
                 'BIMANUAL_GRASP_POSE' in views[-1]['reason'])
            assert views[-1]['teleop_mode']=='independent_arms' and len(accepted)==count
            checks.append('B requests tool mode at rest; geometric refusal preserves normal mode and sends no motion')
            source['grips']=[1.,0.];phase('following');source['buttons'][0]=2;phase('locked')
            source.update(grips=[0.,0.],buttons=[0,0]);pump(.3);click(1,1);phase('standby')
            count=len(accepted);pump(.3);assert len(accepted)==count
            checks.append('Y stops both arms; B explicitly recovers standby without switching or resuming')
            if mode=='simulation':
                source['buttons'][1]=1
                wait(lambda:views[-1]['reason']=='PREPARED' and views[-1]['pending'] is None,40.)
                source['buttons'][1]=0;pump(.2)
                assert views[-1]['teleop_mode']=='independent_arms'
                count=len(accepted);click(1,1);phase('standby')
                assert views[-1]['teleop_mode']=='bimanual_tool' and len(accepted)==count
                click(1,1);phase('standby')
                assert views[-1]['teleop_mode']=='independent_arms' and len(accepted)==count
                checks.append('A explicitly prepares; B changes mode in either direction while retaining pose and standby')
            source['grips']=[1.,0.];phase('following');source['stream']=False;phase('locked')
            wait(lambda:not states[-1].lease_active and states[-1].state not in ('RUNNING','STOPPING'))
            assert not states[-1].stop_failures
            checks.append('ordinary-mode input loss cancels and confirms hold with released execution ownership')
            terminate(observer,node);observer=None
            health=json.loads((trace_dir/'diagnostics-health.json').read_text())
            assert not health['active'] and health['dropped']==0 and not health['error']
            events=[json.loads(line)['data'] for line in (trace_dir/'runtime-events.jsonl').read_text().splitlines()]
            assert any(e['state']=='PICO_MODE_CHECK' for e in events)
            assert any(e['state']=='BIMANUAL_CAPTURED' and 'independent:' in e['code'] for e in events)
            return dict(mode=mode,teleop_mode='independent_arms',checks=checks,accepted=len(accepted),vendor_packets=len(packets))
        assert views[-1]['load_state']==('empty' if mode=='simulation' else 'unknown')
        source['grip']=1.
        if mode=='simulation':
            wait(lambda:views[-1]['phase']=='standby' and views[-1]['pending'] is None and
                 'BIMANUAL_GRASP_POSE' in views[-1]['reason'])
            pump(.3);assert not accepted and not packets
            checks.append('direct engagement from invalid zero posture is refused with geometric reason and no actuation')
        else:
            phase('following')
            assert views[-1]['load_state']=='unknown'
            checks.append('unknown load at a valid measured posture engages directly without A, B or an empty declaration')
        source['grip']=0.;phase('standby' if mode=='simulation' else 'tool_standby')
        before=len(accepted)
        source['buttons'][1]=1
        wait(lambda:views[-1]['pending'] is None and views[-1]['reason']=='PREPARED',40.)
        source['buttons'][1]=0;pump(.3)
        assert views[-1]['phase']=='standby'
        assert accepted and all(m.mode=='joint_target' and not m.points for m in accepted)
        assert all(m.runtime_mode==mode for m in accepted)
        if mode=='simulation':assert len(accepted)>before
        if mode!='simulation':
            assert views[-1]['load_state']=='unknown'
            assert len(accepted)==before
        checks.append('optional A validates the path and measured arrival, returning standby without entering loading')
        click(0,2);phase('load_check')
        assert views[-1]['operator_guide']['step']=='support'
        source['stream']=False;pump(.4)
        assert views[-1]['phase']=='load_check' and views[-1]['idle_input_lost']
        source['stream']=True;pump(.3);click(1,1);phase('loading')
        source['trigger']=[1.,1.]
        wait(lambda:views[-1]['gripper_modes']==['open','open'] and views[-1]['pending'] is None)
        assert all(g.grasp_state=='unknown' for g in grippers.values())
        source['trigger']=[0.,0.]
        wait(lambda:views[-1]['gripper_modes']==['closed','closed'] and views[-1]['pending'] is None)
        checks.append('Trigger open/close uses shared leased gripper services; grasp stays unknown')
        click(1,1);phase('standby')
        assert views[-1]['load_source']=='operator_load_confirmation'
        pump(.3);assert views[-1]['phase']=='standby' and views[-1]['pending'] is None
        checks.append('separate loading completes to standby without automatic follow; idle input gaps keep the step')
        before=len(accepted)
        source['buttons'][1]=1
        wait(lambda:views[-1]['reason']=='PREPARED' and views[-1]['pending'] is None)
        source['buttons'][1]=0;pump(.2)
        assert views[-1]['load_state']=='held' and len(accepted)==before
        checks.append('held-tool A validates coupling and tool CAD; already-reached pose submits no motion or gripper commands')
        before=len(accepted);source['grip']=1.;phase('following');source['delta']=.006
        wait(lambda:len(accepted)>before+5)
        source['grip']=0.;phase('tool_standby')
        wait(lambda:states and not states[-1].lease_active and states[-1].state not in ('RUNNING','STOPPING'))
        assert not states[-1].stop_failures
        checks.append('Squeeze performs tool preflight, captures the current baseline and follows; release confirms fresh stop')
        paused=len(accepted);source['delta']=.025;pump(.3)
        assert len(accepted)==paused
        source['grip']=1.;phase('following')
        reengaged=states[-1]
        pump(.2)
        assert max(abs(a-b) for a,b in zip(reengaged.joints.position,states[-1].joints.position))<.003
        source['buttons'][0]=2;phase('locked');source['buttons'][0]=0;source['grip']=0.
        wait(lambda:not states[-1].lease_active and states[-1].state not in ('RUNNING','STOPPING'))
        pump(.3);click(1,1);phase('standby');paused=len(accepted);pump(.3)
        assert len(accepted)==paused
        checks.append('paused controller displacement is discarded on recapture; Y/B recovery never resumes automatically')
        source['grip']=1.;phase('following');source['stream']=False;phase('locked')
        wait(lambda:not states[-1].lease_active and states[-1].state not in ('RUNNING','STOPPING'))
        assert not states[-1].stop_failures
        checks.append('input loss cancels follow and releases ownership after stop')
        source.update(stream=True,grip=0.);pump(.4);click(1,1);phase('standby')
        click(0,2);phase('load_check');click(1,1);phase('loading')
        source['trigger']=[1.,0.]
        wait(lambda:grippers['left'].state=='MOVING')
        source['stream']=False;phase('locked')
        wait(lambda:all(not g.lease_active and g.state not in ('MOVING','STOPPING') for g in grippers.values()))
        assert all(g.state!='FAULT' and g.grasp_state=='unknown' for g in grippers.values())
        checks.append('input loss during gripper operation releases lease and confirms hold without closing')
        source.update(stream=True,grip=0.,trigger=[0.,0.]);pump(.4)
        click(1,1);phase('standby')
        assert views[-1]['load_state']=='unknown'
        assert views[-1]['pending'] is None
        click(0,2);phase('load_check')
        assert views[-1]['loading_goal']=='empty'
        click(1,1);phase('loading')
        source['trigger']=[1.,1.]
        wait(lambda:views[-1]['gripper_modes']==['open','open'] and views[-1]['pending'] is None)
        source['trigger']=[0.,0.]
        wait(lambda:views[-1]['gripper_modes']==['closed','closed'] and views[-1]['pending'] is None)
        click(1,1);phase('standby')
        assert views[-1]['load_state']=='empty'
        checks.append('interrupted unloading recovers to standby; a new request preserves intent but requires fresh support')
        terminate(observer,node);observer=None
        diagnostic_health=json.loads((trace_dir/'diagnostics-health.json').read_text())
        assert not diagnostic_health['active'] and not diagnostic_health['error'] and diagnostic_health['dropped']==0,diagnostic_health
        for stream in ('vr-raw','vr-projected','execution-state','gripper-state','mode-state','teleop-state','accepted-commands'):
            assert (trace_dir/(stream+'.jsonl')).stat().st_size>0,stream
        events=[json.loads(line)['data'] for line in (trace_dir/'runtime-events.jsonl').read_text().splitlines()]
        attempts=[json.loads(e['code']) for e in events if e['state']=='PICO_PREPARE_CHECK']
        assert attempts and attempts[0]['accepted'] and all(attempts[0]['checks'].values()),attempts
        validations=[json.loads(e['code']) for e in events if e['state']=='PICO_PREPARE_VALIDATION']
        assert any(v['operation']=='loaded_prepare_path' and v['tool_present'] and v['coupling_checked'] for v in validations)
        assert any(e['state']=='PICO_ENGAGE_CHECK' for e in events)
        if mode=='simulation':
            assert any(json.loads(e['code']).get('read_only_rejection') for e in events if e['state']=='PICO_OPERATION_RESULT')
        checks.append('raw/projected input, commands, feedback and preparation checks persisted without logger queue loss')
        if websocket:
            assert not peer_errors,peer_errors
            reports=[json.loads(line)['data'] for line in (trace_dir/'vr-health.jsonl').read_text().splitlines()]
            assert any(r['stage']=='websocket_handler' for r in reports),reports
            assert any((r.get('event_interval_s') or 0)>.25 for r in reports),reports[-3:]
            checks.append('real Vuer WebSocket input and event-loop diagnostics cover induced controller gaps')
        return dict(mode=mode,checks=checks,accepted=len(accepted),vendor_packets=len(packets))
    finally:
        peer_stop.set()
        if peer:peer.join(timeout=2.)
        (out/(mode+'-final.json')).write_text(json.dumps(views[-1] if views else {},indent=2))
        if observer:terminate(observer,node)
        if observer_log:observer_log.close()
        terminate(process,node);log.close();node.destroy_node()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--mode',choices=('simulation','real','both'),default='both')
    p.add_argument('--websocket-input',action='store_true');args=p.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Isolated localhost ROS domain required')
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    try:
        result=[]
        for m in (('simulation','real') if args.mode=='both' else (args.mode,)):
            for ordinary in (True,False):
                out=args.output/(('ordinary-' if ordinary else 'tool-')+m);out.mkdir()
                case=scenario(out,m,args.websocket_input,ordinary);result.append(case)
                (out/'verification.json').write_text(json.dumps(case,indent=2))
                print(out.name+': '+str(len(case['checks']))+' checks passed',flush=True)
        (args.output/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    finally:rclpy.shutdown()

if __name__=='__main__':main()
