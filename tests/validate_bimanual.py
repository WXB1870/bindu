#!/usr/bin/env python3
"""Synthetic two-arm ROS gate: manual mode, QP, shared execution, cancel."""
import argparse
import json
import os
import shutil
import signal
import socket
import subprocess
import threading
import time
import uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from bindu_interfaces.action import SelectTeleopMode, TeleopSession
from bindu_interfaces.msg import VRInput, ToolPoseInput, ExecutionState, RecorderHealth, MotionCommand, TeleopModeState, RuntimeEvent, SimulationFeedback
from rclpy.qos import QoSProfile, ReliabilityPolicy
from bindu_runtime.common import stamp


def wait(node,future,timeout=10.):
    until=time.monotonic()+timeout
    while not future.done() and time.monotonic()<until:
        rclpy.spin_once(node,timeout_sec=.02)
    if not future.done(): raise RuntimeError('ROS_RESPONSE_TIMEOUT')
    return future.result()


def until(node,predicate,timeout=10.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.02)
        if predicate(): return
    raise RuntimeError('ROS_CONDITION_TIMEOUT')


def websocket_pair(port,control,stop,errors):
    """Use the real Vuer decoder/receiver, with explicitly synthetic controllers."""
    from websockets.sync.client import connect
    from msgpack import packb
    try:
        deadline=time.monotonic()+20.
        ws=None
        while ws is None and time.monotonic()<deadline and not stop.is_set():
            try: ws=connect(f'ws://127.0.0.1:{port}/',open_timeout=1.)
            except OSError: stop.wait(.1)
        if ws is None: raise RuntimeError('VUER_NOT_AVAILABLE')
        with ws:
            while not stop.is_set():
                if control['enabled']:
                    value={}
                    for side,x in (('left',0.),('right',.3)):
                        pose=np.eye(4);pose[0,3]=x+control['delta']
                        value[side]=pose.ravel(order='F').tolist()
                        value[side+'State']={'squeezeValue':control['grip'],
                            'triggerValue':control['trigger'] if side=='right' else 0.,
                            'aButton':False,'bButton':False}
                    ws.send(packb({'etype':'CONTROLLER_MOVE','key':'motionControllers',
                                   'value':value},use_bin_type=True))
                    control['seq']+=1
                for _ in range(4):
                    try: ws.recv(timeout=.001)
                    except TimeoutError: break
                stop.wait(.02)
    except Exception as exc: errors.append(str(exc))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=Path('/tmp/bindu-bimanual-check'))
    parser.add_argument('--g1-model',type=Path,help='Use a copied G1 PhysX fixture, initialized at its dual-arm test pose')
    parser.add_argument('--snowbot',action='store_true',help='Use the supplied Snowbot CAD-derived 20-axis kinematic fixture')
    parser.add_argument('--expect-geometry-rejection',action='store_true',
                        help='Require mode switch to fail closed on an incompatible tool grasp')
    parser.add_argument('--switch-while-active',action='store_true',
                        help='Exercise active single-arm to bimanual mode preemption')
    parser.add_argument('--coalesced-release',action='store_true',
                        help='Drop the released frame but preserve its clutch generation')
    parser.add_argument('--websocket-input',action='store_true',
                        help='Send paired synthetic controllers through real Vuer WebSocket')
    parser.add_argument('--axial-grasp',action='store_true',
                        help='Snowbot shovel fixture: right trigger support rotation and re-lock')
    parser.add_argument('--drop-tool-input',action='store_true',
                        help='Require tool input loss to stop all controlled joints and release ownership')
    parser.add_argument('--tool-pose-input',action='store_true',help='Exercise absolute mapped tool pose and tight state')
    parser.add_argument('--shovel-cycle',action='store_true',help='Complete shovel input stream over the production ROS path')
    args=parser.parse_args()
    if args.tool_pose_input and not args.axial_grasp: parser.error('tool input requires --axial-grasp')
    if args.tool_pose_input and (args.websocket_input or args.coalesced_release):
        parser.error('mapped tool input does not use VR WebSocket/clutch fixtures')
    if args.shovel_cycle and not args.tool_pose_input: parser.error('shovel cycle requires --tool-pose-input')
    if args.websocket_input and args.coalesced_release: parser.error('coalesced fixture injects ROS counters')
    if args.snowbot and args.g1_model: parser.error('choose one robot fixture')
    if args.axial_grasp and not args.snowbot: parser.error('axial grasp requires --snowbot')
    if args.expect_geometry_rejection and not args.snowbot:
        parser.error('geometry rejection fixture requires --snowbot')
    if args.switch_while_active and not args.snowbot:
        parser.error('active switch fixture requires --snowbot')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[1]
    fixture=root/'src/capabilities/bindu_bimanual_qp/test/two_arms.urdf'
    left=['left_joint_'+str(k) for k in range(1,7)]
    right=['right_joint_'+str(k) for k in range(1,7)]
    physics_model=None
    if args.snowbot:
        profile=root/'src/integration/bindu_runtime/config/snowbot_sim.json'
        config=root/'src/integration/bindu_runtime/config/bimanual_snowbot_sim.json'
        if args.axial_grasp:
            config=root/'src/integration/bindu_runtime/config/bimanual_snowbot_shovel.json'
            profile=config.parent/json.loads(config.read_text())['demo']['profile']
        if args.expect_geometry_rejection:
            invalid=json.loads(config.read_text())
            invalid['flange_distance_range']=[.1,.2]
            config=output/'invalid_tool.json'
            config.write_text(json.dumps(invalid))
        command_groups=['waist','left_arm','right_arm']
        command_count=16
    elif args.g1_model:
        physics_model=output/'g1_model'
        shutil.copytree(args.g1_model.resolve(),physics_model)
        physical=json.loads((physics_model/'physics.json').read_text())
        prepared=json.loads((root/'src/integration/bindu_runtime/config/bimanual_g1_pose.json').read_text())
        physical['initial_joint_positions'].update(prepared['joint_positions_rad'])
        (physics_model/'physics.json').write_text(json.dumps(physical,indent=2)+'\n')
        profile=physics_model/'g1_physics_sim.json'
        config=root/'src/integration/bindu_runtime/config/bimanual_g1_sim.json'
        command_groups=['waist','left_arm','right_arm']
        command_count=16
    else:
        profile=output/'profile.json'
        profile.write_text(json.dumps({'name':'two_arm_kinematic_gate',
            'groups':{'waist':['waist_joint'],'left_arm':left,'right_arm':right},
            'limits':{'waist_joint':[-1.2,1.2],**{n:[-2.5,2.5] for n in left+right}},
            'max_speed':.5,'capabilities':['joint_position']}))
        config=output/'tool.json'
        config.write_text(json.dumps({'simulation_only':True,'collision_scope':'empty_scene_only',
            'urdf':str(fixture),'resource_groups':['left_arm','right_arm'],
            'left_frame':'left_ee','right_frame':'right_ee',
            'left_to_tool':[1,0,0,.12,0,1,0,-.35,0,0,1,0,0,0,0,1],
            'flange_distance_range':[.5,1.5],'input_max_age':.25,'feedback_max_age':.2,
            'command_max_age':.4,'engage_seconds':.2,'position_scale':1.,
            'operator_yaw_rad':0.,'sample_period':.01,'relative_gain':2.,
            'max_relative_error':.05,'max_linear_speed':.04,
            'max_angular_speed':.12,'solve_timeout_us':30000}))
        command_groups=['left_arm','right_arm']
        command_count=12
    ns='/bindu_bimanual_'+uuid.uuid4().hex[:8]
    profile_groups=json.loads(profile.read_text())['groups']
    controlled={n for g in command_groups for n in profile_groups[g]}
    run=uuid.uuid4().hex
    processes=[];logs=[]
    def start(executable,extra=()):
        log=(output/(executable+'.log')).open('w');logs.append(log)
        cmd=['ros2','run','bindu_runtime',executable,'--ros-args','-r','__ns:='+ns,
             '-p','profile:='+str(profile),'-p','run_id:='+run,*extra]
        processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,
                                          start_new_session=True))
    rclpy.init()
    node=rclpy.create_node('bimanual_verifier_'+uuid.uuid4().hex[:8])
    source=rclpy.create_node('bimanual_source_'+uuid.uuid4().hex[:8])
    source_executor=SingleThreadedExecutor()
    source_executor.add_node(source)
    source_thread=None
    peer=None; peer_stop=threading.Event(); peer_errors=[]
    state={'execution':None,'mode':None,'accepted':[],'events':[],'physics':None,'trace':[],
           'tool_feedback':None}
    control={'grip':0.,'trigger':0.,'delta':0.,'enabled':True,'seq':0,'left_clutch_seq':0}
    source_timing={'last':None,'max_gap_s':0.}
    try:
        if physics_model:
            log=(output/'isaac.log').open('w');logs.append(log)
            processes.append(subprocess.Popen([str(root/'tools/run_g1_isaac.sh'),'--headless',
                '--model',str(physics_model),'--namespace',ns,'--output',str(output/'isaac_run'),
                '--duration','180'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        initial = json.loads(config.read_text())['demo']['initial_positions_rad'] if args.axial_grasp else {}
        start('execution',['-p','device_backend:=external_simulation'] if physics_model else
              ['-p','kinematic_initial_positions:='+json.dumps(json.dumps(initial))] if initial else [])
        start('teleop_mode',['-p','require_single_worker:=true','-p','tool_config:='+str(config),
                             '-p','input_source:='+('tool_pose' if args.tool_pose_input else 'vr')] if args.snowbot else [])
        start('teleop_tool',['-p','tool_config:='+str(config),
                            '-p','input_source:='+('tool_pose' if args.tool_pose_input else 'vr'),
                            '-p','model_root:='+str((root/'src/hardware/bindu_description/urdf')
                                                    if physics_model or args.snowbot else fixture.parent)])
        if args.snowbot:
            start('teleop',['-p','teleop_config:='+str(root/'src/integration/bindu_runtime/config/teleop_snowbot_left.json'),
                            '-p','model_root:='+str(root/'src/hardware/bindu_description/urdf'),
                            '-p','require_mode_selection:=true'])
        if physics_model:
            node.create_subscription(SimulationFeedback,ns+'/simulation/feedback',
                lambda m:state.update(physics=m),
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        def on_execution(m):
            state['execution']=m
            actual=dict(zip(m.joints.name,m.joints.position))
            tracked={n:abs(actual[n]-q) for n,q in zip(m.reference.name,m.reference.position)
                     if n in actual and n in controlled}
            velocities={n:abs(v) for n,v in zip(m.joints.name,m.joints.velocity) if n in controlled}
            state['trace'].append({'stamp':m.feedback_stamp.sec+m.feedback_stamp.nanosec*1e-9,
                'positions':actual,'velocities':dict(zip(m.joints.name,m.joints.velocity)),
                'state':m.state,'code':m.code,'lease':m.lease_active,
                'reference_count':len(m.reference.name),'max_tracking_error':max(tracked.values(),default=0.),
                'worst_tracking_joint':max(tracked,key=tracked.get,default=''),
                'max_velocity':max(velocities.values(),default=0.),
                'fastest_joint':max(velocities,key=velocities.get,default='')})
        node.create_subscription(ExecutionState,ns+'/execution/state',on_execution,10)
        node.create_subscription(TeleopModeState,ns+'/teleop/mode_state',
            lambda m:state.update(mode=m),10)
        node.create_subscription(MotionCommand,ns+'/execution/accepted',
            lambda m:state['accepted'].append(m),100)
        node.create_subscription(RuntimeEvent,ns+'/events',
            lambda m:state['events'].append((m.component,m.state,m.code)),100)
        if args.tool_pose_input:
            from bindu_contracts.profile import Profile
            from bindu_kinematics.bimanual import BimanualQP
            import sys
            sys.path.insert(0,str(root/'tools'))
            from plan_shovel_motion import target_at,grasp_mode_at
            tool_cfg=json.loads(config.read_text());recipe=tool_cfg['demo']
            tool_cfg['urdf']=str(root/'src/hardware/bindu_description/urdf'/tool_cfg['urdf'])
            solver=BimanualQP(tool_cfg,Profile.load(profile))
            try:home=solver.capture(recipe['initial_positions_rad'])
            finally:solver.close()
            control['cycle_started']=None
            tool_input=source.create_publisher(ToolPoseInput,ns+'/teleop/tool/input',10)
        vr=source.create_publisher(VRInput,ns+'/teleop/vr/input',10)
        health=source.create_publisher(RecorderHealth,ns+'/recorder/health',10)
        def publish():
            tick=time.monotonic()
            if source_timing['last'] is not None:
                source_timing['max_gap_s']=max(source_timing['max_gap_s'],tick-source_timing['last'])
            source_timing['last']=tick
            now=source.get_clock().now().to_msg()
            health.publish(RecorderHealth(stamp=now,run_id=run,ready=True))
            if not control['enabled'] or args.websocket_input: return
            control['seq']+=1
            if args.tool_pose_input:
                pose=home.copy();pose[0,3]+=control['delta']
                tight=control['trigger']<.7
                if control['cycle_started'] is not None:
                    elapsed=min(time.monotonic()-control['cycle_started'],recipe['keyframes'][-1]['t'])
                    pose=target_at(recipe['keyframes'],home,np.array(recipe['pivot_in_tool_m']),elapsed,recipe['shaft_axis_in_tool'])
                    tight=grasp_mode_at(recipe,elapsed)=='rigid'
                tool_input.publish(ToolPoseInput(stamp=now,source_id='synthetic_tool',seq=control['seq'],
                    frame_id=tool_cfg['input_frame'],pose=pose.ravel().tolist(),tight=tight,valid=True))
            if args.shovel_cycle: return  # No paired VR/grip/clutch inputs in this acceptance.
            for side,x in (('left',0.),('right',.3)):
                pose=np.eye(4);pose[0,3]=x+control['delta']
                vr.publish(VRInput(stamp=now,source_id='synthetic_pair',seq=control['seq'],
                    side=side,pose=pose.ravel().tolist(),grip=control['grip'],
                    trigger=control['trigger'] if side=='right' else 0.,valid=True,
                    clutch_seq=control['left_clutch_seq'] if side=='left' else 0))
        source.create_timer(.02,publish)
        source_thread=threading.Thread(target=source_executor.spin,daemon=True)
        source_thread.start()
        if args.websocket_input:
            with socket.socket() as sock:
                sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
            start('vr_input',['-p','side:=both','-p','port:='+str(port)])
            peer=threading.Thread(target=websocket_pair,args=(port,control,peer_stop,peer_errors),daemon=True)
            peer.start()
        selector=ActionClient(node,SelectTeleopMode,ns+'/teleop/select_mode')
        tool=ActionClient(node,TeleopSession,ns+'/teleop/tool_session')
        assert selector.wait_for_server(timeout_sec=20.)
        assert tool.wait_for_server(timeout_sec=20.)
        if physics_model:
            until(node,lambda:state['physics'] is not None and state['physics'].ready,120.)
        until(node,lambda:state['execution'] is not None and state['mode'] is not None,10.)
        selected=wait(node,selector.send_goal_async(SelectTeleopMode.Goal(
            mode='bimanual_tool',side='',expected_generation=1,timeout=6.)))
        assert selected.accepted
        selected_result=wait(node,selected.get_result_async(),8.).result
        if args.expect_geometry_rejection:
            assert (not selected_result.success and
                    'BIMANUAL_GRASP_GEOMETRY' in selected_result.code),selected_result
            until(node,lambda:state['mode'] and state['mode'].phase=='SAFE')
            assert not state['accepted']
            result={'robot':'snowbot','mode_phase':'SAFE','code':selected_result.code,
                    'accepted_commands':0,'simulation_only':True}
            assert not peer_errors,peer_errors
            (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result))
            return
        assert selected_result.success and selected_result.generation==2,selected_result
        until(node,lambda:state['mode'] and state['mode'].mode=='bimanual_tool' and
              state['mode'].phase=='READY')
        until(node,lambda:state['execution'] and state['execution'].feedback_stamp.sec>0)
        # A fresh frame after the completed switch is required by the session.
        prior_seq=control['seq']
        until(node,lambda:control['seq']>=prior_seq+3)
        goal=None
        for _ in range(15):
            goal=wait(node,tool.send_goal_async(TeleopSession.Goal(task_id=run,
                resource_group='bimanual_tool',duration=70. if args.shovel_cycle else 30. if args.axial_grasp else (16. if args.coalesced_release else 8.)),
                feedback_callback=lambda msg:state.update(tool_feedback=msg.feedback.mode)))
            if goal.accepted: break
            prior_seq=control['seq']
            until(node,lambda:control['seq']>=prior_seq+2)
        assert goal.accepted,('tool goal rejected',state['events'][-10:],
            (state['execution'].state,state['execution'].code) if state['execution'] else None)
        goal_result=goal.get_result_async()
        # Goal acceptance precedes execute/lease acquisition. Keep released
        # input until the actual session has ingested it and reports idle.
        if not args.tool_pose_input:
            until(node,lambda:state['tool_feedback'] in ('idle','idle:rigid'))
        control['grip']=1.
        try: until(node,lambda:len(state['accepted'])>=3,6.)
        except RuntimeError:
            print('DEBUG_ACTION',goal_result.result() if goal_result.done() else None,state['events'][-10:],
                  (state['execution'].state,state['execution'].code) if state['execution'] else None)
            raise
        if args.shovel_cycle:
            begin_event=len(state['events'])
            control['cycle_started']=time.monotonic()
            until(node,lambda:time.monotonic()-control['cycle_started']>=recipe['keyframes'][-1]['t']+.5
                  or goal_result.done(),recipe['keyframes'][-1]['t']+5.)
            if goal_result.done():
                raise RuntimeError('SHOVEL_ROS_EARLY_END:'+goal_result.result().result.code)
            transitions=[json.loads(e[2])['grasp_mode'] for e in state['events'][begin_event:]
                         if e[1]=='BIMANUAL_CAPTURED']
            assert transitions==['right_axial','rigid'],transitions
            control['enabled']=False
            ended=wait(node,goal_result,8.).result
            assert ended.code=='TOOL_INPUT_TIMEOUT',ended
            until(node,lambda:not state['execution'].lease_active and not state['execution'].reference.name,6.)
            result={'status':'passed','input':'ToolPoseInput','scope':'production ROS, QP, executor, ideal simulated joints',
                    'code':ended.code,'accepted_commands':ended.accepted_commands,'grasp_transitions':transitions,
                    'source_timing':source_timing,'dynamic_contact_validated':False,'paired_vr_frames_published':False}
            (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result));return
        count=len(state['accepted'])
        control['delta']=.01
        until(node,lambda:len(state['accepted'])>=count+3,6.)
        commands=list(state['accepted'])
        assert all(c.schema_version==2 and list(c.resource_groups)==command_groups
                   and len(c.joint_names)==command_count for c in commands),commands
        assert any(max(abs(a-b) for a,b in zip(c.positions,commands[count-1].positions))>1e-5
                   for c in commands[count:]),'no arm response'
        axial_checked=False
        if args.axial_grasp:
            transitions=[(1.,'right_axial'),(0.,'rigid')]
            if args.drop_tool_input or args.coalesced_release:
                transitions.append((1.,'right_axial'))
            for trigger,mode in transitions:
                event_start=len(state['events'])
                control['trigger']=trigger
                until(node,lambda:any(e[1]=='BIMANUAL_CAPTURED' and
                    json.loads(e[2]).get('grasp_mode')==mode
                    for e in state['events'][event_start:]),6.)
                count_after=len(state['accepted'])
                until(node,lambda:len(state['accepted'])>=count_after+2,4.)
                assert any(e[1]=='BIMANUAL_GRASP_REQUESTED' and
                    json.loads(e[2])['mode']==mode for e in state['events'][event_start:])
            axial_checked=True
        reclutch_checked=False
        if args.coalesced_release:
            event_start=len(state['events'])
            control.update(delta=control['delta']+.2, left_clutch_seq=control['left_clutch_seq']+1)
            # Both current grips remain pressed; no released frame is sent.
            until(node,lambda:any(e[1:] == ('BIMANUAL_CLUTCH','idle')
                                 for e in state['events'][event_start:]),3.)
            until(node,lambda:state['execution'] and not state['execution'].reference.name,3.)
            until(node,lambda:any(e[1]=='BIMANUAL_CAPTURED'
                                 for e in state['events'][event_start:]),4.)
            count_after_capture=len(state['accepted'])
            until(node,lambda:len(state['accepted'])>=count_after_capture+2,4.)
            reclutch_checked=True
        if args.drop_tool_input:
            control['enabled']=False
        else:
            assert wait(node,goal.cancel_goal_async()).goals_canceling
        ended=wait(node,goal_result,8.).result
        expected_end=('TOOL_INPUT_TIMEOUT' if args.tool_pose_input else 'VR_PAIR_TIMEOUT') if args.drop_tool_input else 'CANCELED'
        assert ended.code==expected_end and ended.accepted_commands>=3,(
            ended,state['events'][-10:],state['trace'][-10:],
            (state['execution'].state,state['execution'].code) if state['execution'] else None)
        until(node,lambda:state['execution'] and not state['execution'].lease_active and
              not state['execution'].reference.name,6.)
        control['grip']=0.
        control['enabled']=True
        switched=wait(node,selector.send_goal_async(SelectTeleopMode.Goal(
            mode='single_arm',side='left',expected_generation=2,timeout=6.)))
        assert switched.accepted
        switch_result=wait(node,switched.get_result_async(),8.).result
        assert switch_result.success and switch_result.generation==3,switch_result
        single_accepted=0
        active_switch_generation=None
        if args.snowbot:
            single=ActionClient(node,TeleopSession,ns+'/teleop/session')
            assert single.wait_for_server(timeout_sec=15.)
            until(node,lambda:state['mode'] and state['mode'].mode=='single_arm' and
                  state['mode'].phase=='READY')
            prior_seq=control['seq']
            until(node,lambda:control['seq']>=prior_seq+3)
            single_ready=node.create_client(Trigger,ns+'/teleop/ready')
            assert single_ready.wait_for_service(timeout_sec=10.)
            readiness=''
            for _ in range(20):
                check=wait(node,single_ready.call_async(Trigger.Request()),1.)
                if check.success: break
                readiness=check.message
                prior_seq=control['seq']
                until(node,lambda:control['seq']>=prior_seq+2)
            else:
                raise RuntimeError('SNOWBOT_SINGLE_NOT_READY:'+readiness)
            single_goal=wait(node,single.send_goal_async(TeleopSession.Goal(
                task_id=run+'_single',resource_group='left_arm',duration=8.)))
            assert single_goal.accepted,('snowbot single-arm rejected',state['events'][-10:])
            single_result=single_goal.get_result_async()
            count=len(state['accepted'])
            control['grip']=1.
            until(node,lambda:len(state['accepted'])>=count+3,6.)
            # Preserve the re-clutched hand origin; absolute .02 would be a
            # 19 cm jump after the coalesced-release fixture's 20 cm relocation.
            control['delta']+=.01
            until(node,lambda:len(state['accepted'])>=count+5,6.)
            assert wait(node,single_goal.cancel_goal_async()).goals_canceling
            single_end=wait(node,single_result,8.).result
            assert single_end.code=='CANCELED',single_end
            single_accepted=len(state['accepted'])-count
            until(node,lambda:state['execution'] and not state['execution'].lease_active and
                  not state['execution'].reference.name,6.)
            if args.switch_while_active:
                control['grip']=0.
                prior_seq=control['seq']
                until(node,lambda:control['seq']>=prior_seq+3)
                active_goal=wait(node,single.send_goal_async(TeleopSession.Goal(
                    task_id=run+'_active',resource_group='left_arm',duration=8.)))
                assert active_goal.accepted
                active_result=active_goal.get_result_async()
                count=len(state['accepted'])
                control['grip']=1.
                until(node,lambda:len(state['accepted'])>=count+3,6.)
                request=wait(node,selector.send_goal_async(SelectTeleopMode.Goal(
                    mode='bimanual_tool',side='',expected_generation=3,timeout=6.)))
                assert request.accepted
                control['grip']=0.
                switch=wait(node,request.get_result_async(),8.).result
                active_end=wait(node,active_result,8.).result
                assert switch.success and switch.generation==4,switch
                assert (active_end.code=='TELEOP_MODE_NOT_SELECTED' and
                        'STOP_UNCONFIRMED' not in active_end.code),active_end
                until(node,lambda:state['execution'] and not state['execution'].lease_active and
                      not state['execution'].reference.name,6.)
                active_switch_generation=switch.generation
        result={'mode_generation':switch_result.generation,'accepted_commands':ended.accepted_commands,
                'cancel_code':ended.code,'last_execution':state['execution'].state,
                'profile':str(profile),'tool':str(config),'simulation_only':True,
                'input':'synthetic_vuer_websocket' if args.websocket_input else 'synthetic_ros',
                'backend':'PhysX' if physics_model else 'kinematic',
                'robot':'snowbot' if args.snowbot else ('g1' if physics_model else 'two_arm_fixture'),
                'single_arm_accepted':single_accepted,
                'coalesced_release_checked':reclutch_checked,
                'axial_grasp_checked':axial_checked,
                'tool_input_loss_checked':args.drop_tool_input,
                'active_switch_generation':active_switch_generation}
        assert not peer_errors,peer_errors
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result))
    finally:
        peer_stop.set()
        if peer: peer.join(timeout=3.)
        (output/'websocket_errors.json').write_text(json.dumps(peer_errors)+'\n')
        (output/'trace.json').write_text(json.dumps(state['trace'])+'\n')
        (output/'events.json').write_text(json.dumps(state['events'])+'\n')
        (output/'source_stats.json').write_text(json.dumps({
            'published_pairs':control['seq'],
            'max_publish_gap_s':source_timing['max_gap_s']})+'\n')
        source_executor.shutdown(timeout_sec=3.)
        if source_thread: source_thread.join(timeout=3.)
        source.destroy_node()
        node.destroy_node();rclpy.shutdown()
        for process in processes:
            if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3.)
        for log in logs: log.close()


if __name__=='__main__': main()
