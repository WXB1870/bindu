#!/usr/bin/env python3
"""Protected gripper ROS loop with synthetic feedback only; never starts a driver."""
from tests.support.paths import ROOT
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import rclpy
from std_msgs.msg import Float64MultiArray, String, UInt64MultiArray
from snowbot_ctl_msgs.msg import RetInformation, JointPositionCommand
from bindu_interfaces.msg import GripperState, VRInput
from bindu_interfaces.srv import Lease, GripperControl
from bindu_runtime.common import stamp
from bindu_hardware.snowbot import GROUPS
from tests.support.snowbot import until, call, terminate


def scenario(output, name, enabled, calibrated, *, freeze=False, shutdown=False):
    ident=uuid.uuid4().hex[:8];vendor='/gripper_vendor_'+ident;ns='/gripper_upper_'+ident
    node=rclpy.create_node('gripper_fixture_'+ident)
    identity='/'+node.get_name()
    health_pub=node.create_publisher(String,vendor+'/bindu/servo_health',1)
    source_pub=node.create_publisher(String,vendor+'/bindu/control_source',1)
    feedback_pub=node.create_publisher(RetInformation,vendor+'/dualArmCtl_fdback',1)
    vr_pub=node.create_publisher(VRInput,ns+'/teleop/vr/input',10)
    state={};packets=[];session=0;sample=0;vr_seq=0
    q={n:0. for names in GROUPS.values() for n in names}
    q['arm_l_joint_2'],q['arm_r_joint_2']=.2,-.2
    control=dict(frozen=False,valid=True,triggers=[0.,0.],edges=[[0,0],[0,0]],vr=False)
    def heartbeat(msg):
        nonlocal session
        if len(msg.data)==3:session=msg.data[2]
    node.create_subscription(UInt64MultiArray,vendor+'/bindu/heartbeat',heartbeat,1)
    node.create_subscription(JointPositionCommand,vendor+'/joint_position_cmd',lambda _:None,1)
    node.create_subscription(Float64MultiArray,vendor+'/body/motor_cmd',lambda _:None,1)
    node.create_subscription(Float64MultiArray,vendor+'/head/motor_cmd',lambda _:None,1)
    def target(side, msg):
        assert len(msg.data)==10 and list(msg.data[1:3])==[0.,0.]
        assert int(msg.data[7])*2**32+int(msg.data[8])==session
        assert msg.data[9] in (0.,1.)
        packets.append(dict(side=side, data=list(msg.data)))
        q[GROUPS[side+'_tool_axis'][0]]=msg.data[0]
    for side in ('left','right'):
        node.create_subscription(Float64MultiArray,vendor+'/gripper/'+side+'_cmd',
                                 lambda msg,s=side:target(s,msg),1)
    node.create_subscription(GripperState,ns+'/gripper/state',lambda m:state.update({m.side:m}),10)
    def publish():
        nonlocal sample,vr_seq
        now=node.get_clock().now().nanoseconds*1e-9
        if not control['frozen']:sample+=1
        feedback=RetInformation();feedback.header.stamp=stamp(now)
        for prefix,groups in (('l',('left_arm','left_tool_axis')),('r',('right_arm','right_tool_axis')),('torso',('waist',)),('head',('head',))):
            if not hasattr(feedback,prefix+'_position'):continue
            names=[n for g in groups for n in GROUPS[g]]
            setattr(feedback,prefix+'_position',[q[n] for n in names]);setattr(feedback,prefix+'_speed',[0.]*len(names))
        feedback.pdo_sample=sample
        feedback.pdo_age=.001
        feedback.pdo_wkc=feedback.pdo_expected_wkc=40
        feedback.feedback_valid_mask=0xfffff
        feedback.feedback_status=[39]*20
        feedback_pub.publish(feedback)
        source_pub.publish(String(data=json.dumps(dict(schema=1,stamp=now,source='topic'))))
        health_pub.publish(String(data=json.dumps(dict(schema=1,stamp=now,sample=sample,sample_age=.001,
            operational=True,ready=True,guard_enabled=True,guard_armed=bool(session),guard_latched=False,
            native_position_guard_version=1,
            guard_session=session,wkc=40,expected_wkc=40,present_mask=0xfffff,fault_mask=0,
            enable_requested_mask=0xfffff,operation_enabled_mask=0xfffff))))
        if control['vr']:
            vr_seq+=1
            for i,side in enumerate(('left','right')):
                vr_pub.publish(VRInput(stamp=stamp(now),runtime_mode='real',source_id='fixture_vr',side=side,
                    seq=vr_seq,valid=control['valid'],mapping='pico_snowbot_v1',trigger=control['triggers'][i],
                    pose=[1.,0.,0.,float(i),0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.],
                    button_edges=[0,0,0,0,0,0,*control['edges'][i]]))
    node.create_timer(.01,publish)
    config=json.loads((ROOT/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_grippers_real.json').read_text())
    config['calibrated']=calibrated
    path=output/(name+'.json');path.write_text(json.dumps(config))
    log=(output/(name+'.log')).open('w')
    cmd=['ros2','run','bindu_runtime','snowbot_adapter','--ros-args','-r','__ns:='+ns,
        '-p','runtime_mode:=real','-p','profile:='+str(ROOT/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_real_position.json'),
        '-p','vendor_namespace:='+vendor,'-p','command_enabled:='+str(enabled).lower(),'-p','gripper_config:='+str(path)]
    for key in ('arm_receiver_node','servo_receiver_node','mid_receiver_node'):cmd+=['-p',key+':='+identity]
    process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    checks=[]
    try:
        lease_client=node.create_client(Lease,ns+'/gripper/lease')
        api=node.create_client(GripperControl,ns+'/gripper/control')
        until(node,lambda:state and state['left'].feedback_valid,15.)
        assert not packets,'startup gripper motion'
        checks.append('startup publishes no gripper target; measured feedback available')
        if enabled:until(node,lambda:session!=0,5.)
        lease=call(node,lease_client,Lease.Request(operation='acquire',owner='wire_test',resources=['left_tool_axis','right_tool_axis']))
        if not enabled or not calibrated:
            assert not lease.ok and ('DISABLED' if not enabled else 'CALIBRATION') in lease.code,lease.code
            assert not packets
            checks.append('readonly or uncalibrated acquisition rejected')
            return checks
        assert lease.ok,lease.code
        sequence=0
        def command(operation,side='',load='empty',support=False):
            nonlocal sequence
            sequence+=1
            return call(node,api,GripperControl.Request(operation=operation,side=side,
                lease_id=lease.lease_id,epoch=lease.epoch,sequence=sequence,
                stamp=node.get_clock().now().to_msg(),valid_for=.3,load_state=load,external_support=support))
        assert command('authorize_loading',load='held').accepted
        assert not command('open','left').accepted
        assert not packets
        checks.append('unsupported loaded opening rejected before publish')
        assert command('authorize_loading',load='held',support=True).accepted
        assert command('open','left').accepted
        if freeze:
            control['frozen']=True
            until(node,lambda:state['left'].state=='FAULT',5.)
            assert 'STOP_UNCONFIRMED' in state['left'].code
            again=call(node,lease_client,Lease.Request(operation='acquire',owner='late',resources=['left_tool_axis']))
            assert not again.ok
            checks.append('frozen PDO with advancing ROS headers faults and blocks reacquisition')
            return checks
        if shutdown:
            before=len(packets)
            os.killpg(process.pid,signal.SIGINT)
            until(node,lambda:process.poll() is not None,8.)
            assert process.returncode==0,process.returncode
            assert len(packets)>before
            assert packets[-1]['data'][0]==q['arm_l_joint_8']
            checks.append('SIGINT sends hold and waits for measured confirmation')
            return checks
        until(node,lambda:state['left'].state=='SUCCEEDED',5.)
        assert state['left'].position_reached and state['left'].grasp_state=='unknown'
        assert all(p['side']=='left' for p in packets)
        checks.append('position completion is measured, independent, and never physical grasp confirmation')
        body=node.create_client(Lease,ns+'/execution/lease')
        blocked=call(node,body,Lease.Request(operation='acquire',owner='body',resources=['left_arm','right_arm']))
        assert not blocked.ok and 'GRIPPERS_BUSY' in blocked.code
        checks.append('arm acquisition cannot race a gripper owner')
        assert call(node,lease_client,Lease.Request(operation='release',lease_id=lease.lease_id,epoch=lease.epoch)).ok
        until(node,lambda:not state['left'].lease_active and state['left'].state!='STOPPING',4.)
        lease=call(node,lease_client,Lease.Request(operation='acquire',owner='fixture_vr',resources=['left_tool_axis','right_tool_axis']))
        assert lease.ok,lease.code
        sequence=0;assert command('authorize_loading').accepted
        control['vr']=True
        until(node,lambda:state['right'].trigger_armed and state['left'].trigger_armed,3.)
        control['triggers'][1]=1.;control['edges'][1]=[1,0]
        until(node,lambda:state['right'].state=='SUCCEEDED',5.)
        before=len(packets);held_q=q['arm_r_joint_8']
        control['valid']=False
        until(node,lambda:not state['right'].lease_active and state['right'].state=='CANCELED',4.)
        assert all(p['data'][0]==held_q for p in packets[before:] if p['side']=='right')
        assert q['arm_r_joint_8']==held_q
        checks.append('explicitly leased Trigger control works; invalid input holds without synthetic close')
        recipe=json.loads((ROOT/'src/integration/bindu_runtime/config/deployments/snowbot_real_teleop.json').read_text())
        recipe['arguments'].update(namespace=ns,vendor_namespace=vendor,command_enabled=True,gripper_config=str(path.resolve()))
        site=output/(name+'-site.json');site.write_text(json.dumps(recipe))
        with (output/(name+'-cli.log')).open('w') as cli_log:
            client=subprocess.Popen(['python3', '-m', 'tools.deployment.snowbot_teleop','gripper',
                '--config',str(site.resolve()),'--side','right','--operation','close','--load-state','empty'],
                stdout=cli_log,stderr=subprocess.STDOUT)
            try:
                until(node,lambda:client.poll() is not None,20.)
                assert client.returncode==0,client.returncode
            finally:
                if client.poll() is None:client.kill();client.wait(timeout=2.)
        checks.append('deployment CLI acquires, commands, renews and confirms release hold')
        return checks
    finally:
        (output/(name+'-snapshot.json')).write_text(json.dumps(dict(packets=packets,
            states={s:dict(state=m.state,code=m.code,owner=m.owner,epoch=m.epoch,trigger_armed=m.trigger_armed)
                    for s,m in state.items()},control=control),indent=2)+'\n')
        terminate(process,node);log.close();node.destroy_node()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('isolated localhost ROS domain required')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    try:
        results={}
        for name,enabled,calibrated,kw in (('readonly',False,True,{}),('uncalibrated',True,False,{}),
            ('position-trigger',True,True,{}),('pdo-freeze',True,True,dict(freeze=True)),
            ('active-shutdown',True,True,dict(shutdown=True))):
            results[name]=scenario(args.output,name,enabled,calibrated,**kw)
        (args.output/'results.json').write_text(json.dumps(dict(scope='synthetic ROS only; no driver or hardware',results=results),indent=2)+'\n')
        print('PASS:',len(results),'gripper ROS scenarios;',sum(map(len,results.values())),'checks')
    finally:rclpy.shutdown()


if __name__=='__main__':main()
