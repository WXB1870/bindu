#!/usr/bin/env python3
"""Same PICO operation sequence on kinematic IO and synthetic vendor wire IO.

Requires an isolated localhost ROS domain. Never starts vendor drivers or SOEM.
"""
from bindu_contracts.resources import package_share
from tests.support.paths import ROOT
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
import math
import rclpy
from std_msgs.msg import String, Float64MultiArray, UInt64MultiArray
from snowbot_ctl_msgs.msg import RetInformation, JointPositionCommand, CtlInformation, MidNodeCtlMsgSend
from bindu_interfaces.msg import VRInput, ExecutionState, MotionCommand, GripperState
from bindu_interfaces.srv import Lease, SubmitMotion, PicoContext
from bindu_runtime.common import stamp, seconds
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.vr.receiver import ButtonCounters
from snowbot_hardware.protocol import GROUPS
from tests.support.snowbot import until, call, terminate

CFG=package_share('snowbot_bringup')/'config/robots/snowbot'


def websocket_input(port, source, stop, errors):
    """Real Vuer wire input for this isolated fixture, including downlink drain."""
    from websockets.sync.client import connect
    from msgpack import packb
    end=time.monotonic()+35.;peer=None;last_raw=None
    try:
        while peer is None and not stop.is_set() and time.monotonic()<end:
            try:peer=connect(f'ws://127.0.0.1:{port}/',open_timeout=1.)
            except OSError:stop.wait(.1)
        if peer is None:raise RuntimeError('VUER_CONNECT_TIMEOUT')
        with peer:
            while not stop.is_set():
                if source['stream'] and (not source.get('raw_replay') or source['raw_index']!=last_raw):
                    value={}
                    for i,side in enumerate(('left','right')):
                        if source.get('raw_pair') is not None:
                            import numpy as np
                            d=source['raw_pair'][i]
                            value[side]=np.array(d['pose']).reshape(4,4).ravel(order='F').tolist() if d['valid'] else None
                            buttons=d['buttons'] if source['raw_held'] else 0
                            value[side+'State']=dict(squeezeValue=d['grip'] if source['raw_held'] else 0.,
                                triggerValue=d['trigger'] if source['raw_held'] else 0.,
                                aButton=bool(buttons&1),bButton=bool(buttons&2),thumbstick=bool(buttons&4))
                            continue
                        angle=source.get('rotation',0.);c,s=math.cos(angle),math.sin(angle)
                        line=source.get('line_rotation',0.);dx=.34*i-.17
                        value[side]=[c,s,0.,0.,-s,c,0.,0.,0.,0.,1.,0.,
                                     .17+dx*math.cos(line)+source['delta']+source['deltas'][i],dx*math.sin(line),0.,1.]
                        buttons=source['buttons'][i]
                        value[side+'State']=dict(squeezeValue=source['grips'][i] if source['grips'] is not None else source['grip'],triggerValue=source['trigger'][i],
                            aButton=bool(buttons&1),bButton=bool(buttons&2),thumbstick=bool(buttons&4))
                    source['seq']+=1
                    timing=dict(version=1,seq=source['seq'],perf_ms=time.monotonic()*1000,
                        wall_ms=time.time()*1000,buffered_bytes=0,loop_max_ms=0,raf_max_ms=0,
                        xr_max_ms=0,long_task_max_ms=0,long_task_count=0,page_visibility='synthetic')
                    head={} if source.get('raw_pair') is not None else dict(version=1,samples=[dict(
                        seq=source['seq'],age_ms=0.,frame_time_ms=timing['perf_ms'],
                        frame_id='fixture-local',tracking_id=source['head_tracking_id'],mode='immersive-vr',
                        pose=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.],
                        controllers={side:value[side] for side in ('left','right')})])
                    peer.send(packb(dict(etype='CONTROLLER_MOVE',key='motionControllers',value=value,
                        bindu_timing=timing,bindu_head=head),use_bin_type=True))
                    last_raw=source.get('raw_index')
                for _ in range(4):
                    try:peer.recv(timeout=.001)
                    except TimeoutError:break
                stop.wait(.01)
    except Exception as exc:
        if not stop.is_set():errors.append(type(exc).__name__+':'+str(exc))


def scenario(out, mode, websocket=False, ordinary=False, physics=False, replay=None, collision_recovery=False, prepare_return=None, stop_boundary=False):
    token=uuid.uuid4().hex[:8];ns='/position_'+token;vendor='/fixture_'+token
    node=rclpy.create_node('position_fixture_'+token)
    states=[];views=[];accepted=[];grippers={};packets=[]
    node.create_subscription(ExecutionState,ns+'/execution/state',states.append,10)
    node.create_subscription(String,ns+'/teleop/state',lambda m:views.append(json.loads(m.data)),10)
    node.create_subscription(MotionCommand,ns+'/execution/accepted',accepted.append,100)
    node.create_subscription(GripperState,ns+'/gripper/state',lambda m:grippers.update({m.side:m}),10)
    source=dict(buttons=[0,0],trigger=[0.,0.],grip=0.,grips=None,delta=0.,deltas=[0.,0.],stream=False,seq=0,head_tracking_id=token)
    counters=[ButtonCounters(),ButtonCounters()]
    vr=node.create_publisher(VRInput,ns+'/teleop/vr/raw',10)
    fixture=json.loads((CFG/'pico_snowbot_position_sim.json').read_text())
    # Exercise an invalid startup posture even if the deployment example is
    # later calibrated to a valid grasp. Keep this override in the fixture only.
    fixture['device_initial_positions_rad']={n:0. for n in fixture['device_initial_positions_rad']}
    if replay:
        fixture['device_initial_positions_rad']=dict(replay['segment']['capture']['feedback'])
        source.update(raw_pair=replay['segment']['pairs'][0],raw_held=False,raw_replay=False,raw_index=0)
    if collision_recovery:
        fixture['device_initial_positions_rad']=json.loads((ROOT/'tests/config/teleoperation/snowbot_prepare_collision.json').read_text())['feedback']
    if prepare_return:
        fixture['device_initial_positions_rad']=dict(prepare_return)
        fixture['initial_load_state']='unknown'
    # Unknown load is intentional: the current position contract has no load-state gate.
    fixture['initial_load_state']='unknown'
    fixture['initial_teleop_mode']='independent_arms' if ordinary else 'bimanual_tool'
    q=fixture['device_initial_positions_rad'].copy()
    if mode=='real' and not ordinary and not prepare_return:
        # Synthetic measured posture, deliberately independent of an A command.
        # The actual real deployment continues to use only vendor feedback.
        q.update(fixture['prepare_positions_rad'])
    session=0;sample=0
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
        def grip_target(side,msg):
            assert len(msg.data)==5 and list(msg.data[1:3])==[0.,0.]
            q[GROUPS[side+'_tool_axis'][0]]=msg.data[0];packets.append('gripper')
        for side in ('left','right'):
            node.create_subscription(Float64MultiArray,vendor+'/gripper/'+side+'_cmd',
                                     lambda msg,s=side:grip_target(s,msg),10)
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
                enable_requested_mask=0xfffff,operation_enabled_mask=0xfffff))))
        if source['stream'] and not websocket:
            source['seq']+=1
            for i,side in enumerate(('left','right')):
                pose=(1.,0.,0.,.34*i+source['delta']+source['deltas'][i],0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.)
                f=counters[i].update(VRFrame(token,source['seq'],now,side,pose,
                    source['grips'][i] if source['grips'] is not None else source['grip'],source['trigger'][i],
                    mapping='pico_snowbot_v1',buttons=source['buttons'][i]))
                vr.publish(VRInput(stamp=stamp(now),runtime_mode=mode,source_id=f.source_id,seq=f.seq,side=side,
                    pose=list(f.pose),grip=f.grip,trigger=f.trigger,valid=True,mapping=f.mapping,
                    buttons=f.buttons,button_edges=list(f.button_edges),
                    head_pose=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.],
                    head_stamp=stamp(now),head_age_s=0.,head_source_id=token,head_seq=source['seq']))
    node.create_timer(.01,publish)
    recipe=json.loads((package_share('snowbot_bringup')/f'config/deployments/snowbot_vr_{"sim" if mode=="simulation" else "real"}.json').read_text())
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
    command=['ros2','launch','bindu_runtime','teleop_only.launch.py','config:='+str(path)]
    if physics:
        with socket.socket() as probe:
            probe.bind(('127.0.0.1',0));web_port=probe.getsockname()[1]
        command=[sys.executable, '-m', 'tools.teleop.run_vr','--config',str(path),'--physics',
            '--domain-id',os.environ['ROS_DOMAIN_ID'],'--namespace',ns,'--port',str(web_port),
            '--vr-port',str(port),'--output',str(out/'unified-session')]
    process=subprocess.Popen(command,
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    peer_stop=threading.Event();peer_errors=[];peer=None
    if websocket:
        peer=threading.Thread(target=websocket_input,args=(port,source,peer_stop,peer_errors),daemon=True)
        peer.start()
    checks=[];observer=None;observer_log=None
    trace_dir=out/(mode+'-diagnostics')
    def pump(seconds):
        end=time.monotonic()+seconds;until(node,lambda:time.monotonic()>end,seconds+1.)
    def set_delta(value):
        if physics:
            start=source['delta']
            for i in range(30):
                source['delta']=start+(value-start)*(i+1)/30;pump(.02)
        else:source['delta']=value
    def wait(predicate, seconds=15.):
        try:until(node,predicate,seconds)
        except AssertionError as exc:raise AssertionError(json.dumps(views[-1] if views else {},ensure_ascii=False)) from exc
    def phase(value):wait(lambda:views and views[-1]['phase']==value and views[-1]['pending'] is None)
    def error_sample(grasp):
        def ready():
            path=out/(mode+'-observer.json')
            if not path.exists():return False
            value=json.loads(path.read_text()).get('end_effector_errors',{})
            return (value.get('valid') and value.get('grasp_mode')==grasp and
                    all(isinstance(value.get(k),list) and len(value[k])==2 and
                        all(math.isfinite(v) for v in value[k]) for k in ('left','right','coupled')))
        wait(ready,5.)
        value=json.loads((out/(mode+'-observer.json')).read_text())['end_effector_errors']
        assert value['runtime_mode']==mode and value['source']=='关节反馈 FK'
        tool_cfg=json.loads((CFG/'bimanual_snowbot_real.json').read_text())
        limit_mm=tool_cfg['grasp_position_tolerance_m']*1000
        assert value['position_limit_mm']==limit_mm and value['coupled'][0]<limit_mm,value
        checks.append(grasp+' measured FK tracking/coupling errors are available to the display observer')
    def click(side,bit):
        source['buttons'][side]|=1<<bit;pump(.12);source['buttons'][side]&=~(1<<bit);pump(.12)
    def start_observer():
        nonlocal observer,observer_log
        observer_log=(out/(mode+'-observer.log')).open('w')
        from bindu_runtime.teleop_deployment import load_deployment
        observer_robot=load_deployment(path,package_share('bindu_runtime'))[1]['robot_config']
        observer=subprocess.Popen([sys.executable, '-m', 'tools.teleop.snowbot_live','--observe',
            '--namespace',ns,'--run-id',json.loads((out/'unified-session/session.json').read_text())['run_id'] if physics else views[-1]['run_id'],
            '--robot-config',observer_robot,
            '--runtime-mode',mode,'--device-backend','external_simulation' if physics else 'kinematic' if mode=='simulation' else 'snowbot_vendor',
            '--state-file',str(out/(mode+'-observer.json')),
            '--trace-file',str(trace_dir/'diagnostics.jsonl')],
            stdout=observer_log,stderr=subprocess.STDOUT,start_new_session=True)
        wait(lambda:(trace_dir/'teleop-state.jsonl').exists() and
                    (trace_dir/'teleop-state.jsonl').stat().st_size>0,30.)
    try:
        wait(lambda:states and views and time.time()-seconds(states[-1].feedback_stamp)<.2 and
             len(grippers)==2 and all(g.feedback_valid for g in grippers.values()),60. if physics else 40.)
        start_observer()
        pump(2.)  # Discover every reader before connecting the test hands, on either backend.
        source['stream']=True
        wait(lambda:states and states[-1].feedback_stamp.sec>0 and views and
             views[-1]['phase']=='standby' and len(grippers)==2,60. if physics else 40.)
        assert not accepted and not packets,'startup actuation'
        initial=dict(zip(states[-1].joints.name,states[-1].joints.position))
        assert all(n in initial and abs(initial[n]-v)<(.003 if physics else 1e-9)
                   for n,v in (q if mode=='real' else fixture['device_initial_positions_rad']).items()),'startup feedback differs from configured fixture'
        assert states[-1].simulated==(mode=='simulation') and states[-1].runtime_mode==mode
        assert views[-1]['capabilities']['axial_grasp'] is True
        checks.append('same feedback contract, no startup commands, configured axial position capability explicit')
        pump(.3)
        if prepare_return:
            expected_mode='independent_arms' if ordinary else 'bimanual_tool'
            assert views[-1]['load_state']=='unknown' and views[-1]['teleop_mode']==expected_mode
            if stop_boundary:
                from bindu_contracts.profile import Profile
                profile=Profile.load(CFG/'snowbot_real_position.json')
                assert initial['arm_r_joint_3']<profile.limits['arm_r_joint_3'][0]
                source['buttons'][1]=1
                wait(lambda:len(accepted)>0,35.)
                source['buttons'][1]=0
                wait(lambda:views[-1]['pending'] is None and views[-1]['stop_confirmed'] and
                     not states[-1].lease_active and states[-1].state not in ('RUNNING','STOPPING'))
                assert not states[-1].stop_failures
                actual=dict(zip(states[-1].joints.name,states[-1].joints.position))
                assert actual['arm_r_joint_3']<profile.limits['arm_r_joint_3'][0]
                assert all(profile.limits[n][0]<=v<=profile.limits[n][1]
                           for m in accepted for n,v in zip(m.joint_names,m.positions))
                checks.append('measured boundary overrun remains visible; legal A starts, immediate release confirms actual hold')
                accepted.clear();pump(.3)
            source['trigger']=[1.,0.]
            wait(lambda:views[-1]['pending']=='manual_grippers')
            source['trigger']=[0.,0.];phase('standby')
            checks.append('rapid Trigger release replaces opening and finishes closed hold')
            source['trigger']=[1.,1.];phase('gripper_control')
            source['buttons'][1]=1;source['trigger']=[0.,0.]
            wait(lambda:views[-1]['reason']=='PREPARE_WAITING_FOR_GRIPPER_STOP',5.)
            assert not accepted
            checks.append('held A reports gripper-stop wait and sends no arm command before confirmed hold')
            wait(lambda:len(accepted)>0,35.)
            pump(.12);source['buttons'][1]=0
            wait(lambda:views[-1]['pending'] is None and not states[-1].lease_active and
                 states[-1].state not in ('RUNNING','STOPPING'))
            assert not states[-1].stop_failures
            stopped_at=len(accepted);pump(.3);assert len(accepted)==stopped_at
            checks.append('unknown-load A starts from recorded offset; releasing A cancels and confirms hold')
            context=node.create_client(PicoContext,ns+'/teleop/pico/context')
            response=call(node,context,PicoContext.Request(source_id=views[-1]['source_id'],
                expected_generation=views[-1]['generation'],load_state='held',external_support=False))
            assert response.success,response.code
            wait(lambda:views[-1]['load_state']=='held');pump(.2)
            source['buttons'][1]=1
            wait(lambda:views[-1]['reason']=='PREPARED' and views[-1]['pending'] is None,45.)
            source['buttons'][1]=0;pump(.3)
            measured=dict(zip(states[-1].joints.name,states[-1].joints.position))
            names=[n for g in ('waist','left_arm','right_arm') for n in GROUPS[g]]
            assert max(abs(measured[n]-fixture['prepare_positions_rad'][n]) for n in names)<.002
            assert views[-1]['load_state']=='held' and views[-1]['teleop_mode']==expected_mode
            assert views[-1]['stop_confirmed'] and views[-1]['phase']=='standby'
            assert views[-1]['gripper_modes']==['closed','closed'] and not states[-1].stop_failures
            checks.append('held-load A reaches preset despite changed two-hand relationship; mode/load/grippers retained')
            before=len(accepted);last_check=views[-1]['last_prepare_check']['stamp'];source['buttons'][1]=1
            wait(lambda:views[-1]['pending'] is None and views[-1]['reason']=='PREPARED' and
                 views[-1]['last_prepare_check']['stamp']>last_check,5.)
            source['buttons'][1]=0;pump(.3)
            assert len(accepted)==before and views[-1]['reason']=='PREPARED'
            checks.append('repeated A at measured preset sends no redundant motion and never starts following')
            terminate(observer,node);observer=None
            events=[json.loads(line)['data'] for line in (trace_dir/'runtime-events.jsonl').read_text().splitlines()]
            canceled=[json.loads(e['code']) for e in events if e['state']=='PICO_OPERATION_CANCELED']
            assert canceled and all(c['outcome']=='superseded' for c in canceled)
            assert not any('PICO_OPERATION_SUPERSEDED' in e['code'] for e in events if e['state']=='PICO_OPERATION_RESULT')
            checks.append('superseded operations are cancellation events; new intents and actual stop errors stay distinct')
            validations=[json.loads(e['code']) for e in events if e['state']=='PICO_PREPARE_VALIDATION']
            assert {v['load_state'] for v in validations}>={'unknown','held'}
            assert all(v['operation'] in ('prepare_path','prepare_recovery_path') and
                       not v['load_state_checked'] and not v['tool_present'] and not v['coupling_checked'] for v in validations)
            health=json.loads((trace_dir/'diagnostics-health.json').read_text())
            assert not health['active'] and not health['error'] and not health['dropped'] and not peer_errors
            checks.append('fresh body path validation persists for both loads; no load/coupling gate or diagnostic loss')
            return dict(mode=mode,teleop_mode=expected_mode,checks=checks,accepted=len(accepted),vendor_packets=len(packets))
        if not collision_recovery and not replay:
            before=len(accepted);load=views[-1]['load_state']
            def grippers_settled(modes):
                return (views[-1]['gripper_modes']==modes and
                        all(g.feedback_valid and abs(g.measured_velocity)<.01 and
                            g.state not in ('MOVING','STOPPING','FAULT') for g in grippers.values()))
            source['trigger']=[1.,0.]
            wait(lambda:grippers_settled(['open','closed']))
            assert views[-1]['phase']=='gripper_control' and not views[-1]['independent_grippers']
            assert len(accepted)==before and views[-1]['load_state']==load and not views[-1]['external_support']
            assert all(g.grasp_state=='unknown' for g in grippers.values())
            checks.append('direct left Trigger opens only left without loading, support or arm actuation')
            source['trigger']=[1.,1.];wait(lambda:grippers_settled(['open','open']))
            source['trigger']=[0.,1.];wait(lambda:grippers_settled(['closed','open']))
            source['trigger']=[0.,0.];wait(lambda:grippers_settled(['closed','closed']))
            checks.append('two Trigger targets update independently; releasing one closes only that gripper')
            source['trigger']=[1.,0.];pump(.10);source['trigger']=[0.,0.]
            wait(lambda:grippers_settled(['closed','closed']))
            checks.append('early Trigger release supersedes opening and completes measured closed hold')
            source['trigger']=[1.,0.]
            wait(lambda:grippers['left'].state=='MOVING')
            source['stream']=False
            wait(lambda:all(not g.lease_active and g.state not in ('MOVING','STOPPING') for g in grippers.values()))
            held=grippers['left'].measured_position;pump(.3)
            assert abs(grippers['left'].measured_position-held)<.003
            assert all(g.state!='FAULT' and g.grasp_state=='unknown' for g in grippers.values())
            source.update(stream=True,trigger=[0.,0.]);pump(.4)
            assert abs(grippers['left'].measured_position-held)<.003
            assert views[-1]['phase']=='locked'
            click(1,1);phase('standby')
            source['trigger']=[1.,0.];wait(lambda:grippers_settled(['open','closed']))
            source['trigger']=[0.,0.];wait(lambda:grippers_settled(['closed','closed']))
            phase('standby')
            wait(lambda:all(not g.lease_active for g in grippers.values()))
            assert len(accepted)==before
            checks.append('direct gripper input loss confirms fixed hold; restored input needs a fresh Trigger intent')
        if collision_recovery:
            source['grip']=1.
            wait(lambda:views[-1]['pending'] is None and 'SELF_COLLISION' in views[-1]['reason'])
            assert not accepted and views[-1]['teleop_mode']=='bimanual_tool'
            source['grip']=0.;pump(.3)
            click(1,1)
            wait(lambda:views[-1]['pending'] is None)
            assert views[-1]['teleop_mode']=='bimanual_tool' and not accepted
            checks.append('recorded initial self-collision still rejects follow and mode selection without actuation')
            source['buttons'][1]=1
            wait(lambda:len(accepted)>0,35.)
            pump(.12);source['buttons'][1]=0
            wait(lambda:views[-1]['pending'] is None and not states[-1].lease_active and
                 states[-1].state not in ('RUNNING','STOPPING'))
            assert not states[-1].stop_failures
            stopped_at=len(accepted);pump(.3);assert len(accepted)==stopped_at
            checks.append('releasing A cancels the bounded escape and confirms measured stop without continuing preparation')
            source['buttons'][1]=1
            wait(lambda:views[-1]['pending'] is None and views[-1]['reason']=='PREPARED',45.)
            source['buttons'][1]=0;pump(.3)
            measured=dict(zip(states[-1].joints.name,states[-1].joints.position))
            names=[n for g in ('waist','left_arm','right_arm') for n in GROUPS[g]]
            assert max(abs(measured[n]-fixture['prepare_positions_rad'][n]) for n in names)<.002
            assert views[-1]['stop_confirmed'] and not states[-1].stop_failures
            checks.append('holding A retreats from the original overlap then validates and reaches the preparation pose')
            source['grip']=1.;phase('following')
            set_delta(.002);pump(.3);assert views[-1]['phase']=='following'
            source['grip']=0.;phase('tool_standby')
            wait(lambda:not states[-1].lease_active and states[-1].state not in ('RUNNING','STOPPING'))
            assert not states[-1].stop_failures
            checks.append('fresh two-hand clutch follows after recovery; release confirms stop')
            terminate(observer,node);observer=None
            events=[json.loads(line)['data'] for line in (trace_dir/'runtime-events.jsonl').read_text().splitlines()]
            recoveries=[json.loads(e['code']) for e in events if e['state']=='PICO_PREPARE_RECOVERY']
            assert recoveries and all(max(r['initial_contacts'].values())<=.0005 and
                r['cleared_at_joint_travel_rad']<=.02 for r in recoveries)
            assert all(m.mode=='joint_target' and m.runtime_mode=='simulation' for m in accepted)
            health=json.loads((trace_dir/'diagnostics-health.json').read_text())
            assert not health['active'] and not health['error'] and not health['dropped'] and not peer_errors
            checks.append('recovery depth/travel evidence and commands persisted with no diagnostic loss')
            return dict(mode=mode,teleop_mode='bimanual_tool',checks=checks,accepted=len(accepted),
                        recoveries=recoveries,vendor_packets=0)
        if replay:
            # The only synthetic inputs are a stationary released/pressed
            # clutch setup. Motion packets retain the recorded raw poses,
            # validity/buttons and selected clock, without pose smoothing.
            source['raw_held']=True;phase('following')
            initial_count=len(accepted);motion_start=len(states)
            segment=replay['segment'];clocks=replay['clocks'];first=segment['pairs'][0][0]
            def packet_time(pair):
                d=pair[0]
                return clocks[(d['source_id'],d['seq'])] if replay['clock']=='client' else seconds(d['stamp'])
            origin=packet_time(segment['pairs'][0]);start=time.monotonic();source['raw_replay']=True
            for index,pair in enumerate(segment['pairs'][1:],1):
                delay=packet_time(pair)-origin-(time.monotonic()-start)
                if delay>0:pump(delay)
                assert views[-1]['phase']=='following',views[-1]
                source.update(raw_pair=pair,raw_index=index)
            pump(.04)
            assert views[-1]['phase']=='following',views[-1]
            assert len(accepted)>initial_count+5
            motion=list(states[motion_start:]);source.update(raw_held=False,raw_replay=False)
            phase('tool_standby')
            wait(lambda:not states[-1].lease_active and states[-1].state not in ('RUNNING','STOPPING'))
            assert not states[-1].stop_failures
            sys.path.insert(0,str(ROOT))
            from tools.teleop.snowbot_errors import pinocchio_fk
            from bindu_kinematics.tool_geometry import coupling_metrics
            tool_cfg=json.loads((CFG/'bimanual_snowbot_real.json').read_text())
            robot_meta=json.loads((out/'unified-session/session.json').read_text())
            from bindu_runtime.robot_setup import RobotSetup
            robot=RobotSetup.load(robot_meta['configs']['robot_config']['path']);fk=pinocchio_fk(robot.urdf,tool_cfg)
            samples=[]
            for state in motion:
                q=dict(zip(state.joints.name,state.joints.position));left,right=fk(q)
                samples.append(dict(t=seconds(state.feedback_stamp),flanges={tool_cfg['left_frame']:left,tool_cfg['right_frame']:right}))
            coupled=coupling_metrics(samples,tool_cfg,[dict(t=samples[0]['t'],mode='rigid')])
            assert coupled['passed'],coupled
            checks.extend(['recorded raw hand packets follow through Vuer, ROS, native QP and measured PhysX feedback',
                           'recorded-motion release confirms explicit hold with no stop fault',
                           'measured coupling stays inside configured position and rotation limits'])
            terminate(observer,node);observer=None
            health=json.loads((trace_dir/'diagnostics-health.json').read_text())
            assert not health['active'] and not health['dropped'] and not health['error']
            return dict(mode=mode,teleop_mode='bimanual_tool',checks=checks,accepted=len(accepted),vendor_packets=0,
                        source_log=str(replay['log']),input_seq=first['seq'],clock=replay['clock'],
                        source_packets=len(segment['pairs']),dataset_sha256=replay['sha256'],coupling=coupled)
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
            assert views[-1]['load_state']=='unknown'
            source['deltas'][0]=.004
            wait(lambda:len(accepted)>5 and max(abs(v-baseline[n]) for n,v in
                zip(states[-1].joints.name,states[-1].joints.position) if n in GROUPS['left_arm'])>1e-5)
            measured=dict(zip(states[-1].joints.name,states[-1].joints.position))
            assert max(abs(measured[n]-baseline[n]) for n in GROUPS['right_arm']+GROUPS['waist'])<(.002 if physics else 1e-8)
            checks.append('zero posture with unknown or empty load accepts left-only clutch; right arm and waist remain fixed')
            count=len(accepted);wait(lambda:len(accepted)>count)
            source['stream']=False;pump(.205);count=len(accepted);source['stream']=True
            wait(lambda:len(accepted)>count)
            assert views[-1]['phase']=='following',views[-1]
            checks.append('205ms controller gap continues with bounded steps under the 250ms sample limit')
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
            terminate(observer,node);observer=None
            if websocket:
                health=json.loads((trace_dir/'diagnostics-health.json').read_text())
                assert not health['error'] and health['dropped']==0,health
                reports=[json.loads(line)['data'] for line in (trace_dir/'vr-health.jsonl').read_text().splitlines()]
                assert any(r['stage']=='controller_timing' and r['client_timing_available'] for r in reports)
                events=[json.loads(line)['data'] for line in (trace_dir/'runtime-events.jsonl').read_text().splitlines()]
                gaps=[json.loads(e['code']) for e in events if e['state']=='BIMANUAL_SAMPLE_GAP']
                assert any(.2<g['sample_gap_s']<=.25 and not g['will_reject'] for g in gaps),gaps
                results=[json.loads(e['code']) for e in events if e['state']=='BIMANUAL_QP_RESULT']
                assert any(r['sample_gap_s']>.1 and r['integration_dt']<=.1 for r in results),results[-5:]
                checks.append('browser timing, ROS receipt and solver gap/step evidence persisted without logger loss')
            checks.append('ordinary-mode input loss cancels and confirms hold with released execution ownership')
            health=json.loads((trace_dir/'diagnostics-health.json').read_text())
            assert not health['active'] and health['dropped']==0 and not health['error']
            events=[json.loads(line)['data'] for line in (trace_dir/'runtime-events.jsonl').read_text().splitlines()]
            assert any(e['state']=='PICO_MODE_CHECK' for e in events)
            assert any(e['state']=='BIMANUAL_CAPTURED' and 'independent:' in e['code'] for e in events)
            return dict(mode=mode,teleop_mode='independent_arms',checks=checks,accepted=len(accepted),vendor_packets=len(packets))
        assert views[-1]['load_state']=='unknown'
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
        before=len(accepted)
        for side,index in (('left',0),('right',1)):
            other='right' if side=='left' else 'left'
            held=grippers[other].measured_position
            source['trigger'][index]=1.
            wait(lambda:views[-1]['gripper_modes'][index]=='open' and views[-1]['pending'] is None)
            assert abs(grippers[other].measured_position-held)<(.003 if physics else 1e-9)
            source['trigger'][index]=0.
            phase('standby')
        assert len(accepted)==before and all(g.grasp_state=='unknown' for g in grippers.values())
        checks.append('independent Trigger open/close uses only the requested gripper; no arm command or grasp claim')
        assert views[-1]['load_state']=='unknown'
        source['buttons'][1]=1
        wait(lambda:views[-1]['reason']=='PREPARED' and views[-1]['pending'] is None,40.)
        source['buttons'][1]=0;pump(.2)
        assert len(accepted)==before
        checks.append('A accepts unknown load at the reached preparation pose without motion or gripper commands')
        before=len(accepted);source['grip']=1.;phase('following');set_delta(.006)
        wait(lambda:len(accepted)>before+5)
        error_sample('rigid')
        left_position=grippers['left'].measured_position
        if physics:
            # X must brake a moving pair, not only confirm an already-resting
            # pose. This exposed the old set-target/expiry hold mismatch.
            for _ in range(8):
                source['delta']+=.002;pump(.02)
        source['buttons'][0]=1
        wait(lambda:views[-1]['phase']=='following' and views[-1]['grasp_applied']=='right_axial' and
             views[-1]['tool_phase']=='follow:right_axial',20.)
        expected_virtual=json.loads((CFG/'snowbot_grippers_sim.json').read_text())['grippers']['right']['virtual']
        assert abs(grippers['right'].measured_position-expected_virtual)<.003
        assert abs(grippers['left'].measured_position-left_position)<(.003 if physics else 1e-9)
        assert all(g.grasp_state=='unknown' for g in grippers.values())
        before_virtual=len(accepted);set_delta(source['delta']+.003)
        wait(lambda:len(accepted)>before_virtual+5)
        error_sample('right_axial')
        source['buttons'][0]=0
        wait(lambda:views[-1]['phase']=='following' and views[-1]['grasp_applied']=='rigid' and
             views[-1]['tool_phase']=='follow:rigid',20.)
        assert abs(grippers['right'].measured_position)<.003
        assert abs(grippers['left'].measured_position-left_position)<(.003 if physics else 1e-9)
        checks.append('X changes only right gripper position, waits for acknowledgement/hold, captures axial QP and returns to measured rigid grasp')
        source['buttons'][0]=1
        wait(lambda:views[-1]['phase']=='following' and views[-1]['tool_phase']=='follow:right_axial',20.)
        source['grip']=0.;phase('tool_standby')
        source['buttons'][0]=0;paused=len(accepted);pump(.3)
        assert len(accepted)==paused and abs(grippers['right'].measured_position-expected_virtual)<.003
        source['grip']=1.
        wait(lambda:views[-1]['phase']=='following' and views[-1]['tool_phase']=='follow:rigid',20.)
        assert abs(grippers['right'].measured_position)<.003
        checks.append('paused X release retains virtual opening; a fresh two-hand clutch confirms rigid position before resuming')
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
        source['trigger']=[1.,0.]
        wait(lambda:grippers['left'].state=='MOVING')
        source['stream']=False;phase('locked')
        wait(lambda:all(not g.lease_active and g.state not in ('MOVING','STOPPING') for g in grippers.values()))
        assert all(g.state!='FAULT' and g.grasp_state=='unknown' for g in grippers.values())
        checks.append('input loss during direct gripper operation releases lease and confirms hold without closing')
        source.update(stream=True,grip=0.,trigger=[0.,0.]);pump(.4)
        click(1,1);phase('standby')
        # A fresh Trigger press/release explicitly closes the interrupted side.
        source['trigger']=[1.,0.]
        wait(lambda:views[-1]['gripper_modes'][0]=='open' and views[-1]['pending'] is None)
        source['trigger']=[0.,0.];phase('standby')
        assert views[-1]['load_state']=='unknown'
        checks.append('gripper recovery requires a fresh intent; closing does not declare an empty or held tool')
        if websocket:
            baseline_delta=source['delta'];source['grip']=1.;phase('following')
            line_start=len(states)
            for i in range(120):
                source['delta']=baseline_delta+.012*math.sin(i*.15)
                source['line_rotation']=.4*math.sin(i*.08)
                pump(.1)
                assert views[-1]['phase']=='following',views[-1]
            assert views[-1]['control_point']=='grip_midpoint' and views[-1]['orientation_source']=='hand_line'
            motion=states[line_start:]
            assert max(max(s.joints.position[j] for s in motion)-min(s.joints.position[j] for s in motion)
                       for j,n in enumerate(motion[0].joints.name) if n.startswith('arm_'))>.01
            source['line_rotation']=0.;source['delta']=baseline_delta;pump(.5)
            source['grip']=0.;phase('tool_standby')
            wait(lambda:not states[-1].lease_active and states[-1].state not in ('RUNNING','STOPPING'))
            assert not states[-1].stop_failures
            checks.append('twelve seconds of hand-line rotation with fixed wrists follows about the grip midpoint without ground or grip alarms')
        terminate(observer,node);observer=None
        diagnostic_health=json.loads((trace_dir/'diagnostics-health.json').read_text())
        assert not diagnostic_health['active'] and not diagnostic_health['error'] and diagnostic_health['dropped']==0,diagnostic_health
        for stream in ('vr-raw','vr-projected','execution-state','gripper-state','mode-state','teleop-state','accepted-commands'):
            assert (trace_dir/(stream+'.jsonl')).stat().st_size>0,stream
        events=[json.loads(line)['data'] for line in (trace_dir/'runtime-events.jsonl').read_text().splitlines()]
        attempts=[json.loads(e['code']) for e in events if e['state']=='PICO_PREPARE_CHECK']
        assert attempts and attempts[0]['accepted'] and all(attempts[0]['checks'].values()),attempts
        validations=[json.loads(e['code']) for e in events if e['state']=='PICO_PREPARE_VALIDATION']
        assert any(v['load_state']=='unknown' and v['operation'] in ('prepare_path','prepare_recovery_path') and
                   not v['load_state_checked'] and not v['tool_present'] and not v['coupling_checked'] for v in validations)
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
    p.add_argument('--physics',action='store_true',help='Use the actual Isaac backend and unified launcher; requires simulation and WebSocket input')
    p.add_argument('--teleop-mode',choices=('ordinary','tool','both'),default='both')
    p.add_argument('--websocket-input',action='store_true')
    p.add_argument('--replay-log',type=Path,help='Replay one captured raw VR segment through the isolated physics stack')
    p.add_argument('--replay-seq',type=int)
    p.add_argument('--collision-recovery',action='store_true',help='Recorded shallow self-collision: reject follow, release A, then hold A to recover')
    p.add_argument('--prepare-return',type=Path,nargs='?',const=ROOT/'tests/config/teleoperation/snowbot_prepare_offset.json',
                   help='Recorded measured joint map: test unknown/held A return, cancel and repeat in either mode')
    p.add_argument('--stop-boundary',action='store_true',help='With physics and a boundary --prepare-return fixture: cancel A immediately and verify the out-of-limit measured hold')
    p.add_argument('--until',type=float,help='Freeze recorded inputs/events at this UNIX timestamp')
    p.add_argument('--clock',choices=('client','received'),default='client')
    p.add_argument('--runs',type=int,default=1)
    args=p.parse_args()
    prepare_return=json.loads(args.prepare_return.read_text()) if args.prepare_return else None
    if prepare_return:prepare_return=prepare_return.get('feedback',prepare_return)
    if args.prepare_return and (args.collision_recovery or args.replay_log):p.error('--prepare-return cannot combine with recovery/replay')
    if args.physics and (args.mode!='simulation' or not args.websocket_input):p.error('--physics requires --mode simulation --websocket-input')
    if args.stop_boundary and (not args.physics or not args.prepare_return):p.error('--stop-boundary requires --physics and --prepare-return')
    if args.collision_recovery and (args.mode!='simulation' or args.teleop_mode!='tool' or not args.websocket_input or args.replay_log):
        p.error('--collision-recovery requires simulation, tool, WebSocket and no raw replay')
    if not 1<=args.runs<=10:p.error('--runs must be 1..10')
    replay=None
    if args.replay_log:
        if not args.physics or args.teleop_mode!='tool' or args.replay_seq is None:
            p.error('--replay-log requires --physics --teleop-mode tool --replay-seq')
        from tests.replay.replay_vr import dataset
        segments,clocks=dataset(args.replay_log,args.until if args.until is not None else time.time())
        segment=next(s for s in segments if s['capture']['input_seq']==args.replay_seq)
        if segment['capture']['grasp_mode']!='rigid':p.error('recorded physics replay requires a rigid capture')
        import hashlib
        replay=dict(log=args.replay_log,segment=segment,clocks=clocks,clock=args.clock,
                    sha256=hashlib.sha256(json.dumps(segment['pairs'],sort_keys=True).encode()).hexdigest())
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Isolated localhost ROS domain required')
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    try:
        result=[]
        for run in range(args.runs):
            for m in (('simulation','real') if args.mode=='both' else (args.mode,)):
                for ordinary in ((True,False) if args.teleop_mode=='both' else (args.teleop_mode=='ordinary',)):
                    out=args.output/(('ordinary-' if ordinary else 'tool-')+m+('' if args.runs==1 else f'-{run:02}'));out.mkdir()
                    case=scenario(out,m,args.websocket_input,ordinary,physics=args.physics,replay=replay,
                                  collision_recovery=args.collision_recovery,prepare_return=prepare_return,stop_boundary=args.stop_boundary);result.append(case)
                    (out/'verification.json').write_text(json.dumps(case,indent=2))
                    print(out.name+': '+str(len(case['checks']))+' checks passed',flush=True)
        (args.output/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    finally:rclpy.shutdown()

if __name__=='__main__':main()
