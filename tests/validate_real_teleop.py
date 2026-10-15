#!/usr/bin/env python3
"""Real-mode teleop wire checks with synthetic controller feedback, no robot IO."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import socket
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray, String, UInt64MultiArray
from snowbot_ctl_msgs.msg import RetInformation, JointPositionCommand, CtlInformation, MidNodeCtlMsgSend
from bindu_interfaces.msg import ExecutionState, VRInput, TeleopModeState, RuntimeEvent
from bindu_interfaces.action import SelectTeleopMode, TeleopSession
from bindu_interfaces.srv import Lease
from bindu_runtime.common import stamp
from bindu_hardware.snowbot import GROUPS
from validate_snowbot_adapter import until, call, terminate


def scenario(root, out, enabled):
    identity=uuid.uuid4().hex[:8];ns='/real_teleop_'+identity;vendor='/controller_fixture_'+identity
    node=rclpy.create_node('real_teleop_verifier_'+identity)
    q=json.loads((root/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_shovel.json').read_text())['demo']['initial_positions_rad'].copy()
    source=node.create_publisher(RetInformation,vendor+'/dualArmCtl_fdback',1)
    servo=node.create_publisher(String,vendor+'/bindu/servo_health',1)
    upper=node.create_publisher(String,vendor+'/bindu/control_source',1)
    bridge=[node.create_publisher(MidNodeCtlMsgSend,vendor+'/upperMonitor_msg_send',1),
            node.create_publisher(CtlInformation,vendor+'/ctl_data_pub',1)]
    node.create_subscription(MidNodeCtlMsgSend,vendor+'/upperMonitor_msg_send',lambda _:None,1)
    node.create_subscription(CtlInformation,vendor+'/ctl_data_pub',lambda _:None,1)
    events=[];state=[];modes=[];packets=[];heartbeats=[];clutch=0.;offset=0.;stop=False;stream=True;seq=0;sample=0;session=0
    node.create_subscription(RuntimeEvent,ns+'/events',lambda m:events.append(dict(component=m.component,state=m.state,code=m.code)),100)
    node.create_subscription(ExecutionState,ns+'/execution/state',lambda m:state.append(m),10)
    node.create_subscription(TeleopModeState,ns+'/teleop/mode_state',lambda m:modes.append(m),10)
    vr=node.create_publisher(VRInput,ns+'/teleop/vr/input',10)
    def heartbeat(msg):
        nonlocal session
        heartbeats.append(msg)
        if len(msg.data)==3:session=msg.data[2]
    node.create_subscription(UInt64MultiArray,vendor+'/bindu/heartbeat',heartbeat,1)
    def arms(msg):
        values=dict(zip(GROUPS['left_arm']+GROUPS['right_arm'],list(msg.l_arm_position)+list(msg.r_arm_position)))
        packets.append(values);q.update(values)
    def waist(msg):
        values=dict(zip(GROUPS['waist'],msg.data));packets.append(values);q.update(values)
    node.create_subscription(JointPositionCommand,vendor+'/joint_position_cmd',arms,10)
    node.create_subscription(Float64MultiArray,vendor+'/body/motor_cmd',waist,10)
    def publish():
        nonlocal seq,sample
        now=node.get_clock().now().nanoseconds*1e-9;sample+=1
        msg=RetInformation();msg.header.stamp=stamp(now)
        for prefix,groups in (('l',('left_arm','left_tool_axis')),('r',('right_arm','right_tool_axis')),('torso',('waist',)),('head',('head',))):
            names=[n for g in groups for n in GROUPS[g]]
            setattr(msg,prefix+'_position',[float(q[n]) for n in names]);setattr(msg,prefix+'_speed',[0.]*len(names))
        source.publish(msg)
        upper.publish(String(data=json.dumps(dict(schema=1,stamp=now,source='topic'))))
        servo.publish(String(data=json.dumps(dict(schema=1,stamp=now,sample=sample,sample_age=.001,
            operational=True,ready=True,guard_enabled=True,guard_armed=bool(session),guard_latched=False,
            wkc=40,expected_wkc=40,fault_mask=0,present_mask=0xfffff,enable_requested_mask=0,
            operation_enabled_mask=0,guard_session=session))))
        if stream:
            seq+=1
            for side,x in (('left',-.15),('right',.15)):
                pose=[1.,0.,0.,x+offset,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.]
                vr.publish(VRInput(stamp=stamp(now),runtime_mode='real',source_id=identity,seq=seq,
                    side=side,pose=pose,grip=clutch,mapping='pico_snowbot_v1',valid=True,stop=stop))
    node.create_timer(.01,publish)
    recipe=json.loads((root/'src/integration/bindu_runtime/config/deployments/snowbot_real_teleop.json').read_text())
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    recipe['arguments'].update(namespace=ns,vendor_namespace=vendor,command_enabled=enabled,vr_enabled=not enabled,port=port,
        arm_receiver_node='/'+node.get_name(),servo_receiver_node='/'+node.get_name(),mid_receiver_node='/'+node.get_name())
    file=out/('enabled.json' if enabled else 'readonly.json');file.write_text(json.dumps(recipe))
    handle=(out/('enabled-launch.log' if enabled else 'readonly-launch.log')).open('w')
    process=subprocess.Popen(['ros2','launch','bindu_runtime','teleop_only.launch.py','config:='+str(file)],
        stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
    checks=[]
    try:
        until(node,lambda:state and state[-1].feedback_stamp.sec and modes,40.)
        assert not state[-1].simulated and state[-1].device_backend=='snowbot_vendor'
        expected={'execution','teleop_mode','teleop_tool'} | ({'vr_input'} if not enabled else set())
        until(node,lambda:{name for name,namespace in node.get_node_names_and_namespaces() if namespace==ns}==expected,15.)
        assert node.count_publishers(ns+'/recorder/health')==0
        assert node.count_subscribers(ns+'/recorder/health')==0
        checks.append('only configured teleop nodes; no recorder or fake recorder health')
        assert not packets,'startup movement'
        if not enabled:
            lease=node.create_client(Lease,ns+'/execution/lease')
            reply=call(node,lease,Lease.Request(operation='acquire',owner='readonly',resources=['waist','left_arm','right_arm']))
            assert not reply.ok and 'COMMANDS_DISABLED' in reply.code,reply.code
            assert not heartbeats and node.count_publishers(vendor+'/joint_position_cmd')==0
            checks.append('readonly startup creates no command or heartbeat writer')
        else:
            until(node,lambda:bool(heartbeats),6.)
            mode=ActionClient(node,SelectTeleopMode,ns+'/teleop/select_mode')
            until(node,mode.server_is_ready)
            future=mode.send_goal_async(SelectTeleopMode.Goal(mode='bimanual_tool',side='',expected_generation=modes[-1].generation,timeout=15.))
            until(node,future.done,18.);goal=future.result();assert goal.accepted
            result=goal.get_result_async();until(node,result.done,18.);assert result.result().result.success,result.result().result.code
            checks.append('dual mode selection succeeds without recorder health')
            end=time.monotonic()+.2;until(node,lambda:time.monotonic()>end)
            client=ActionClient(node,TeleopSession,ns+'/teleop/tool_session');until(node,client.server_is_ready)
            phases=[]
            future=client.send_goal_async(TeleopSession.Goal(task_id='real-wire',resource_group='bimanual_tool',duration=30.),
                feedback_callback=lambda f:phases.append(f.feedback.mode))
            until(node,future.done);goal=future.result();assert goal.accepted,events[-5:]
            end=time.monotonic()+.2;until(node,lambda:time.monotonic()>end)
            clutch=1.;until(node,lambda:any(p.startswith('follow:') for p in phases),8.)
            before=q.copy();offset=.012
            until(node,lambda:any(abs(q[n]-before[n])>.0005 for n in GROUPS['left_arm']+GROUPS['right_arm']),8.)
            assert packets and state[-1].runtime_mode=='real'
            checks.append('relative VR pose drives native QP and real position wire adapter')
            stop=True;result=goal.get_result_async();until(node,result.done,8.)
            until(node,lambda:not state[-1].lease_active and state[-1].state not in ('RUNNING','STOPPING'),8.)
            assert not state[-1].stop_failures
            assert not result.result().result.simulated
            checks.append('VR stop confirms stopped feedback and releases ownership')
        return {'passed':True,'command_enabled':enabled,'checks':checks,'packets':len(packets)}
    finally:
        (out/('enabled-events.json' if enabled else 'readonly-events.json')).write_text(json.dumps(events,indent=2)+'\n')
        terminate(process,node);handle.close();node.destroy_node()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('An isolated localhost ROS domain is required')
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[1];rclpy.init();results=[]
    try:
        for enabled in (False,True):
            results.append(scenario(root,out,enabled));print(json.dumps(results[-1]),flush=True)
            (out/'results.json').write_text(json.dumps({'scope':'synthetic controller and input only; no hardware',
                'cases':results,'passed':all(r['passed'] for r in results)},indent=2)+'\n')
    finally:rclpy.shutdown()


if __name__=='__main__':main()
