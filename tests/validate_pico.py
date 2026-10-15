#!/usr/bin/env python3
"""Synthetic PICO -> operation coordinator -> QP/shared executor ROS validation.

Only launches isolated kinematic simulation processes. No hardware mode or IO.
"""
import argparse
import gc
import json
import os
import signal
import subprocess
import socket
import threading
import time
import uuid
from pathlib import Path
import numpy as np
import rclpy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from bindu_interfaces.msg import VRInput,ToolPoseInput,ExecutionState,MotionCommand,RuntimeEvent
from bindu_interfaces.srv import PicoContext
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.vr.receiver import ButtonCounters
from bindu_runtime.common import stamp


def websocket_source(port, source, stop, errors, records, period):
    """Send the actual Vuer wire format; decoding/counters stay in the receiver."""
    try:
        from websockets.sync.client import connect
        from msgpack import packb
        deadline=time.monotonic()+30.;ws=None
        while ws is None and not stop.is_set() and time.monotonic()<deadline:
            try:ws=connect(f'ws://127.0.0.1:{port}/',open_timeout=1.)
            except OSError:stop.wait(.1)
        if ws is None:raise RuntimeError('VUER_CONNECT_TIMEOUT')
        with ws:
            next_send=time.monotonic()
            while not stop.is_set():
                if source['enabled']:
                    generator=source.get('pose_generator')
                    if generator is not None:
                        poses,recipe_time,target=generator(time.monotonic())
                        source['poses']=poses;source['recipe_time']=recipe_time
                        source['cycle_targets'].append(dict(stamp=time.time(),recipe_time=recipe_time,
                                                             pose=target.ravel().tolist()))
                    poses=source['poses']
                    value={}
                    for i,side in enumerate(('left','right')):
                        buttons=source['buttons'][i]
                        pose=np.asarray(poses[i]).copy();pose[0,3]+=source['delta']
                        value[side]=pose.ravel(order='F').tolist()
                        value[side+'State']=dict(squeezeValue=source['grip'][i],
                            triggerValue=source['trigger'][i],aButton=bool(buttons&1),
                            bButton=bool(buttons&2),thumbstick=bool(buttons&4))
                    packet=dict(etype='CONTROLLER_MOVE',key='motionControllers',value=value)
                    ws.send(packb(packet,use_bin_type=True))
                    records.append(dict(sent_at=time.time(),**packet))
                for _ in range(4):
                    try:ws.recv(timeout=.001)
                    except TimeoutError:break
                next_send=max(next_send+period,time.monotonic())
                stop.wait(max(0.,next_send-time.monotonic()))
    except Exception as exc:errors.append(type(exc).__name__+': '+str(exc))


def run_shovel_cycle(node,source,views,states,events,trace,raw,projected,tool_inputs,
                     tool_path,model_root,out,pump,wait,phase,click,result,wire,operator_speed):
    """A synthetic operator: inverse-map a tool recipe into raw headset poses.

    All actuation goes through WebSocket -> VR receiver -> PICO -> tool Action.
    Grasp changes wait for the simulated actuator and actual QP capture. No
    target message or joint command is published by this operator.
    """
    import sys
    from bisect import bisect_right
    root=Path(__file__).resolve().parents[1]
    sys.path.insert(0,str(root/'tools'))
    from plan_shovel_motion import target_at
    from validate_bimanual import check_tool_trace
    from bindu_teleoperation.vr.mapping import OPENXR_TO_ROBOT,robot_pose,mapping_rotation,relative_target
    cfg=json.loads(tool_path.read_text());cfg['urdf']=str(model_root/cfg['urdf'])
    recipe=cfg['demo'];basis=mapping_rotation('robot_base',cfg['operator_yaw_rad'])
    axis=np.asarray(recipe['shaft_axis_in_tool']);pivot=np.asarray(recipe['pivot_in_tool_m'])
    def captures(mode,start=0):
        return [(e,json.loads(e['code'])) for e in events[start:] if e['state']=='BIMANUAL_CAPTURED'
                and json.loads(e['code'])['grasp_mode']==mode]
    def pair_pose(poses):
        left,right=map(robot_pose,poses);p=left.copy();p[:3,3]=(left[:3,3]+right[:3,3])/2
        return p
    anchors=[]
    def anchor(mode,start=0):
        wait(lambda:bool(captures(mode,start)),5.,'QP capture '+mode)
        event,data=captures(mode,start)[-1]
        poses=[p.copy() for p in source['poses']]
        item=dict(stamp=event['stamp'],tool=np.asarray(data['tool_pose']),input=pair_pose(poses),
                  hands=[robot_pose(p) for p in poses],mode=mode,continuation=data.get('grasp_continuation',False))
        anchors.append(item)
        return item
    current=anchor('rigid');home=current['tool'].copy()
    start_index=len(trace);wire_start=len(wire);began=time.monotonic();waits=[];desired=[]
    source['cycle_targets']=desired;source['cycle_anchors']=anchors
    result['shovel_cycle']=dict(last_recipe_time=0.,trace_start=start_index)
    schedule=recipe['grasp_schedule'][1:];next_switch=0;t=0.
    def raw_poses(target,a):
        center=a['input'][:3,3]+basis.T @ (target[:3,3]-a['tool'][:3,3])/cfg['position_scale']
        rotation=basis.T @ target[:3,:3] @ a['tool'][:3,:3].T @ basis @ a['input'][:3,:3]
        poses=[]
        for i,hand in enumerate(a['hands']):
            p=hand.copy();p[:3,3]+=center-a['input'][:3,3]
            if i==0:p[:3,:3]=rotation
            poses.append(OPENXR_TO_ROBOT.T @ p @ OPENXR_TO_ROBOT)
        return poses
    while t<recipe['keyframes'][-1]['t']-1e-9:
        boundary=schedule[next_switch]['t'] if next_switch<len(schedule) else recipe['keyframes'][-1]['t']
        # Capture an immutable segment/anchor. The wire sender samples the
        # continuous operator motion at transmission time, avoiding two
        # unsynchronized clocks that duplicate poses then send a double step.
        def generate(now,a=current,first=t,started=time.monotonic(),last=boundary):
            seconds=min(first+(now-started)*operator_speed,last)
            target=target_at(recipe['keyframes'],home,pivot,seconds,axis)
            return raw_poses(target,a),seconds,target
        source['recipe_time']=t;source['pose_generator']=generate
        while source['recipe_time']<boundary:
            pump(.005)
            result['shovel_cycle']['last_recipe_time']=source['recipe_time']
            if views[-1]['phase']!='following':raise AssertionError('SHOVEL_INTERRUPTED:'+json.dumps(views[-1]))
            if time.monotonic()-began>recipe['keyframes'][-1]['t']/operator_speed+60.:raise AssertionError('SHOVEL_WALL_TIMEOUT')
        source['pose_generator']=None;t=boundary
        if next_switch<len(schedule) and t>=boundary-1e-9:
            mode=schedule[next_switch]['mode'];event_start=len(events);start=time.monotonic()
            source['buttons'][0]=1 if mode=='right_axial' else 0
            wait(lambda:views[-1]['phase']=='following' and views[-1]['pending'] is None and
                 views[-1]['grasp_requested']==mode and views[-1]['tool_phase']=='follow:'+mode and
                 bool(captures(mode,event_start)),20.,'X transition '+mode)
            current=anchor(mode,event_start)
            waits.append(dict(recipe_time=t,mode=mode,wall_s=time.monotonic()-start))
            next_switch+=1
    pump(.8)
    motion_wall=time.monotonic()-began
    end_index=len(trace)
    source['grip']=[0.,0.];phase('tool_standby');pump(.3)
    assert states[-1].state not in ('RUNNING','STOPPING') and not states[-1].reference.name
    click(0,1);phase('locked');pump(.2);source['enabled']=False
    assert not tool_inputs,'Unexpected ToolPoseInput publisher in raw-VR scenario'
    assert wire[wire_start:]
    pairs={}
    for f in raw:
        if f['valid']:pairs.setdefault((f['source_id'],f['seq']),{})[f['side']]=f
    stamps=[a['stamp'] for a in anchors];requests=[]
    for pair in pairs.values():
        if set(pair)!= {'left','right'}:continue
        left,right=pair['left'],pair['right'];stamp_s=left['stamp'];i=bisect_right(stamps,stamp_s)-1
        if i<0:continue
        a=anchors[i];pose=pair_pose([np.array(left['pose']).reshape(4,4),np.array(right['pose']).reshape(4,4)])
        mapped=relative_target(a['input'],pose,a['tool'],cfg['position_scale'],
            mapping_mode='robot_base',operator_yaw_rad=cfg['operator_yaw_rad'])
        requests.append(dict(stamp=stamp_s,pose=list(mapped),seq=left['seq']))
    requests.sort(key=lambda p:p['stamp'])
    (out/'mapped_requests.json').write_text(json.dumps(requests)+'\n')
    (out/'operator_targets.json').write_text(json.dumps(desired)+'\n')
    (out/'mapping_anchors.json').write_text(json.dumps([dict(a,tool=a['tool'].tolist(),input=a['input'].tolist(),
        hands=[p.tolist() for p in a['hands']]) for a in anchors],indent=2)+'\n')
    tracking=check_tool_trace(trace[start_index:end_index],requests,cfg,model_root,out,
                             captures=[dict(t=a['stamp'],mode=a['mode']) for a in anchors])
    assert tracking['max_lift_m']>=recipe['acceptance']['min_lift_m'],'Shovel must actually lift, not only preserve a static grasp'
    assert [a['mode'] for a in anchors]==['rigid','right_axial','rigid']
    assert [a['continuation'] for a in anchors]==[False,True,True], 'Only continuous X handoffs may use the short hold'
    assert all(f['runtime_mode']=='simulation' for f in raw+projected)
    result['shovel_cycle']=dict(recipe_duration_s=recipe['keyframes'][-1]['t'],operator_speed=operator_speed,wall_s=motion_wall,
        feedback_start_stamp=trace[start_index]['stamp'],feedback_end_stamp=trace[end_index-1]['stamp'],
        grasp_waits=waits,tracking=tracking,websocket_events=len(wire)-wire_start,
        raw_frames=len(raw),projected_frames=len(projected),tool_pose_messages=len(tool_inputs),
        input='Vuer WebSocket CONTROLLER_MOVE, column-major OpenXR; PICO mapping',
        backend='ROS kinematic simulated joints',physical_grasp_validated=False)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--crash-coordinator',action='store_true');parser.add_argument('--reject-virtual',action='store_true');parser.add_argument('--tool-config')
    parser.add_argument('--websocket-input',action='store_true',help='Send raw CONTROLLER_MOVE events through the real Vuer receiver')
    parser.add_argument('--shovel-cycle',action='store_true',help='Complete the shovel recipe using paired poses, Squeeze and X')
    parser.add_argument('--operator-speed',type=float,default=1.,help='Scale synthetic controller movement speed, not robot limits')
    parser.add_argument('--input-period',type=float,default=.01,help='Raw VR event period in seconds')
    args=parser.parse_args()
    if not np.isfinite(args.operator_speed) or not 0 < args.operator_speed <= 1.:
        parser.error('--operator-speed must be in (0, 1]')
    if not np.isfinite(args.input_period) or not .005 <= args.input_period <= .05:
        parser.error('--input-period must be in [0.005, 0.05]')
    if args.shovel_cycle and (not args.websocket_input or args.crash_coordinator or args.reject_virtual):
        parser.error('--shovel-cycle requires --websocket-input and the ordinary scenario')
    if args.websocket_input and (args.crash_coordinator or args.reject_virtual):
        parser.error('WebSocket validation uses the ordinary scenario')
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[1];cfg=root/'src/integration/bindu_runtime/config/robots/snowbot'
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
    views=[];states=[];accepted=[];events=[];trace=[];raw=[];projected=[];tool_inputs=[]
    peer=None;peer_stop=threading.Event();peer_errors=[];wire=[]
    gc_was_enabled=gc.isenabled()
    node.create_subscription(String,ns+'/teleop/pico/state',lambda m:views.append(json.loads(m.data)),100)
    def on_state(m):
        states.append(m)
        trace.append(dict(stamp=m.feedback_stamp.sec+m.feedback_stamp.nanosec/1e9,
            positions=dict(zip(m.joints.name,m.joints.position)),
            velocities=dict(zip(m.joints.name,m.joints.velocity)),state=m.state,code=m.code))
    node.create_subscription(ExecutionState,ns+'/execution/state',on_state,100)
    node.create_subscription(MotionCommand,ns+'/execution/accepted',accepted.append,100)
    node.create_subscription(RuntimeEvent,ns+'/events',lambda m:events.append(dict(component=m.component,state=m.state,code=m.code,
        stamp=m.stamp.sec+m.stamp.nanosec/1e9)),1000)
    def frame_record(m):
        return dict(stamp=m.stamp.sec+m.stamp.nanosec/1e9,source_id=m.source_id,seq=m.seq,
            runtime_mode=m.runtime_mode,side=m.side,pose=list(m.pose),grip=m.grip,trigger=m.trigger,
            buttons=int(m.buttons),button_edges=[int(v) for v in m.button_edges],clutch_seq=int(m.clutch_seq),
            virtual_grip=m.virtual_grip,valid=m.valid)
    node.create_subscription(VRInput,ns+'/teleop/vr/raw',lambda m:raw.append(frame_record(m)),100)
    node.create_subscription(VRInput,ns+'/teleop/vr/input',lambda m:projected.append(frame_record(m)),100)
    node.create_subscription(ToolPoseInput,ns+'/teleop/tool/input',tool_inputs.append,10)
    pub=None if args.websocket_input else node.create_publisher(VRInput,ns+'/teleop/vr/raw',10)
    client=node.create_client(PicoContext,ns+'/teleop/pico/context')
    pause_client=node.create_client(SetBool,ns+'/teleop/tool_pause')
    source={'id':'synthetic_pico_'+run,'seq':0,'buttons':[0,0],'grip':[0.,0.],
            'trigger':[0.,0.],'delta':0.,'enabled':True}
    source['poses']=[np.eye(4),np.eye(4)]
    for i,p in enumerate(source['poses']):p[:3,3]=[-.17+.34*i,1.1,-.5]
    counters=[ButtonCounters(),ButtonCounters()]
    def publish():
        if not source['enabled'] or args.websocket_input:return
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
            if peer_errors:raise RuntimeError('WEBSOCKET_INPUT:'+str(peer_errors))
            if predicate():return
            if peer_errors:raise RuntimeError('WEBSOCKET_INPUT:'+str(peer_errors))
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
        coordinator_process=procs[-1]
        if args.websocket_input:
            with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
            start('vr_input',['-p','side:=both','-p','mapping:=pico_snowbot_v1','-p','port:='+str(port),
                '-r','teleop/vr/input:=teleop/vr/raw'])
            # This bounded verifier keeps full packet/feedback evidence in
            # memory. A cyclic collection holds the GIL and starves its sender
            # thread. Runtime nodes retain their normal collection policy.
            gc.collect();gc.disable()
            peer=threading.Thread(target=websocket_source,args=(port,source,peer_stop,peer_errors,wire,args.input_period),daemon=True)
            peer.start()
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
        if args.shovel_cycle:
            run_shovel_cycle(node,source,views,states,events,trace,raw,projected,tool_inputs,
                tool,model,out,pump,wait,phase,click,result,wire,args.operator_speed)
            check('full shovel cycle controlled only by raw WebXR poses, Squeeze and X')
            result.update(success=True,accepted_commands=len(accepted),final=views[-1]);return
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
        resumed=[json.loads(e['code']) for e in events if e['state']=='BIMANUAL_CAPTURED']
        assert resumed and not resumed[-1]['grasp_continuation'], 'Physical release must discard the short handoff'
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
        old_id=views[-1]['coordinator_id'];old_source=views[-1]['source_id'];old=coordinator_process;expected_exits.add(old.pid)
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
        peer_stop.set()
        if peer:peer.join(timeout=3.)
        # Stop children before writing diagnostics: a serialization or disk
        # failure must never leave a simulated controller running unattended.
        for p in reversed(procs):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in procs:
            try:p.wait(timeout=8.)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
        if gc_was_enabled:gc.enable()
        (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
        (out/'events.json').write_text(json.dumps(events,indent=2)+'\n')
        (out/'trace.json').write_text(json.dumps(trace)+'\n')
        if args.websocket_input:
            for name,records in (('websocket_events',wire),('raw_frames',raw),('projected_frames',projected),('websocket_errors',peer_errors)):
                (out/(name+'.json')).write_text(json.dumps(records)+'\n')
            for key in ('cycle_targets','cycle_anchors'):
                if key in source:
                    (out/(key+'.json')).write_text(json.dumps(source[key],default=lambda a:a.tolist())+'\n')


if __name__=='__main__':main()
