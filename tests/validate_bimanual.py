#!/usr/bin/env python3
"""Synthetic two-arm ROS gate: manual mode, QP, shared execution, cancel."""
import argparse
import json
import os
import shutil
import signal
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
from bindu_interfaces.msg import VRInput, ExecutionState, RecorderHealth, MotionCommand, TeleopModeState, RuntimeEvent, SimulationFeedback
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


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=Path('/tmp/bindu-bimanual-check'))
    parser.add_argument('--g1-model',type=Path,help='Use a copied G1 PhysX fixture, initialized at its dual-arm test pose')
    parser.add_argument('--snowbot',action='store_true',help='Use the supplied Snowbot CAD-derived 20-axis kinematic fixture')
    parser.add_argument('--expect-geometry-rejection',action='store_true',
                        help='Require mode switch to fail closed on an incompatible tool grasp')
    parser.add_argument('--switch-while-active',action='store_true',
                        help='Exercise active single-arm to bimanual mode preemption')
    args=parser.parse_args()
    if args.snowbot and args.g1_model: parser.error('choose one robot fixture')
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
    state={'execution':None,'mode':None,'accepted':[],'events':[],'physics':None,'trace':[]}
    control={'grip':0.,'delta':0.,'enabled':True,'seq':0}
    source_timing={'last':None,'max_gap_s':0.}
    try:
        if physics_model:
            log=(output/'isaac.log').open('w');logs.append(log)
            processes.append(subprocess.Popen([str(root/'tools/run_g1_isaac.sh'),'--headless',
                '--model',str(physics_model),'--namespace',ns,'--output',str(output/'isaac_run'),
                '--duration','180'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        start('execution',['-p','device_backend:=external_simulation'] if physics_model else [])
        start('teleop_mode',['-p','require_single_worker:=true'] if args.snowbot else [])
        start('teleop_tool',['-p','tool_config:='+str(config),
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
            state['trace'].append({'state':m.state,'code':m.code,'lease':m.lease_active,
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
        vr=source.create_publisher(VRInput,ns+'/teleop/vr/input',10)
        health=source.create_publisher(RecorderHealth,ns+'/recorder/health',10)
        def publish():
            tick=time.monotonic()
            if source_timing['last'] is not None:
                source_timing['max_gap_s']=max(source_timing['max_gap_s'],tick-source_timing['last'])
            source_timing['last']=tick
            now=source.get_clock().now().to_msg()
            health.publish(RecorderHealth(stamp=now,run_id=run,ready=True))
            if not control['enabled']: return
            control['seq']+=1
            for side,x in (('left',0.),('right',.3)):
                pose=np.eye(4);pose[0,3]=x+control['delta']
                vr.publish(VRInput(stamp=now,source_id='synthetic_pair',seq=control['seq'],
                    side=side,pose=pose.ravel().tolist(),grip=control['grip'],valid=True))
        source.create_timer(.02,publish)
        source_thread=threading.Thread(target=source_executor.spin,daemon=True)
        source_thread.start()
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
                resource_group='bimanual_tool',duration=8.)))
            if goal.accepted: break
            prior_seq=control['seq']
            until(node,lambda:control['seq']>=prior_seq+2)
        assert goal.accepted,('tool goal rejected',state['events'][-10:],
            (state['execution'].state,state['execution'].code) if state['execution'] else None)
        goal_result=goal.get_result_async()
        control['grip']=1.
        try: until(node,lambda:len(state['accepted'])>=3,6.)
        except RuntimeError:
            print('DEBUG_ACTION',goal_result.result() if goal_result.done() else None,state['events'][-10:],
                  (state['execution'].state,state['execution'].code) if state['execution'] else None)
            raise
        count=len(state['accepted'])
        control['delta']=.01
        until(node,lambda:len(state['accepted'])>=count+3,6.)
        commands=list(state['accepted'])
        assert all(c.schema_version==2 and list(c.resource_groups)==command_groups
                   and len(c.joint_names)==command_count for c in commands),commands
        assert any(max(abs(a-b) for a,b in zip(c.positions,commands[count-1].positions))>1e-5
                   for c in commands[count:]),'no arm response'
        assert wait(node,goal.cancel_goal_async()).goals_canceling
        ended=wait(node,goal_result,8.).result
        assert ended.code=='CANCELED' and ended.accepted_commands>=3,(
            ended,state['events'][-10:],state['trace'][-10:],
            (state['execution'].state,state['execution'].code) if state['execution'] else None)
        until(node,lambda:state['execution'] and not state['execution'].lease_active and
              not state['execution'].reference.name,6.)
        control['grip']=0.
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
            control['delta']=.02
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
        result={'mode_generation':switch_result.generation,'accepted_commands':len(commands),
                'cancel_code':ended.code,'last_execution':state['execution'].state,
                'profile':str(profile),'tool':str(config),'simulation_only':True,
                'backend':'PhysX' if physics_model else 'kinematic',
                'robot':'snowbot' if args.snowbot else ('g1' if physics_model else 'two_arm_fixture'),
                'single_arm_accepted':single_accepted,
                'active_switch_generation':active_switch_generation}
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result))
    finally:
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
