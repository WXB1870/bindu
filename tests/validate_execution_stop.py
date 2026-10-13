#!/usr/bin/env python3
"""Exercise the production execution node against a synthetic ROS device peer."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from ament_index_python.packages import get_package_prefix
from rosgraph_msgs.msg import Clock
from bindu_contracts.profile import Profile
from bindu_interfaces.msg import ExecutionState, MotionCommand, SimulationCommand, SimulationFeedback
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution
from bindu_runtime.common import seconds, stamp


def until(node, predicate, timeout=6.):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.005)
        if predicate():return
    raise AssertionError('EXECUTION_STOP_CHECK_TIMEOUT')


def call(node,client,request):
    until(node,client.service_is_ready)
    future=client.call_async(request);until(node,future.done)
    return future.result()


class Peer:
    def __init__(self,node,namespace,profile,sim_clock=False):
        self.node,self.profile,self.sim_clock=node,profile,sim_clock
        self.enabled=False;self.ack=False;self.frozen=False;self.frame_stamp=None
        self.v=0.;self.seq=0;self.clock_time=10.;self.freeze_clock=False
        self.states=[];self.commands=[];self.identity=uuid.uuid4().hex
        self.pub=node.create_publisher(SimulationFeedback,namespace+'/simulation/feedback',5)
        self.clock_pub=node.create_publisher(Clock,'/clock',1) if sim_clock else None
        self.state_sub=node.create_subscription(ExecutionState,namespace+'/execution/state',self.states.append,20)
        self.command_sub=node.create_subscription(SimulationCommand,namespace+'/simulation/command',self.command,20)
        self.timer=node.create_timer(.01,self.publish)

    def command(self,msg):
        self.commands.append(msg)
        if msg.resource_group=='base':
            if msg.operation=='set':self.v=msg.velocity.linear.x
            elif self.ack:self.v=0.

    def publish(self):
        if self.sim_clock:
            if not self.freeze_clock:self.clock_time+=.01
            self.clock_pub.publish(Clock(clock=stamp(self.clock_time)))
        if not self.enabled:return
        now=self.clock_time-.02 if self.sim_clock else self.node.get_clock().now().nanoseconds*1e-9
        if not self.frozen or self.frame_stamp is None:self.frame_stamp=now
        self.seq+=1
        msg=SimulationFeedback(stamp=stamp(self.frame_stamp),simulator_id=self.identity,
            profile_hash=self.profile.digest,sequence=self.seq,ready=True)
        msg.joints.name=['j'];msg.joints.position=[.25];msg.joints.velocity=[0.]
        msg.base_pose.orientation.w=1.;msg.base_velocity.linear.x=self.v
        self.pub.publish(msg)

    def close(self):
        self.node.destroy_timer(self.timer)
        self.node.destroy_publisher(self.pub)
        self.node.destroy_subscription(self.state_sub)
        self.node.destroy_subscription(self.command_sub)
        if self.clock_pub:self.node.destroy_publisher(self.clock_pub)


def scenario(node,output,profile,path,label,signum=None,ack=True,sim_clock=False):
    namespace='/stop_'+uuid.uuid4().hex[:8]
    peer=Peer(node,namespace,profile,sim_clock)
    clients=[node.create_client(kind,namespace+'/execution/'+name) for kind,name in
             ((Lease,'lease'),(SubmitMotion,'submit'),(ControlExecution,'control'))]
    lease_client,submit_client,control_client=clients
    executable=[str(Path(get_package_prefix('bindu_runtime'))/'lib/bindu_runtime/execution'),
        '--ros-args','-r','__ns:='+namespace,
        '-p','profile:='+str(path),'-p','runtime_mode:=simulation','-p','device_backend:=external_simulation']
    if sim_clock:executable+=['-p','use_sim_time:=true']
    with (output/(label+'.log')).open('w') as log:
        process=subprocess.Popen(executable,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            until(node,lambda:peer.states)
            assert not peer.states[-1].joints.velocity and not peer.states[-1].reference.name
            cold=call(node,lease_client,Lease.Request(operation='acquire',owner='probe',resources=['base']))
            assert not cold.ok,'Cold placeholder feedback cannot grant control'
            peer.enabled=True
            until(node,lambda:peer.states and list(peer.states[-1].joints.position)==[.25] and
                  list(peer.states[-1].joints.velocity)==[0.])
            assert not peer.states[-1].reference.name
            lease=call(node,lease_client,Lease.Request(operation='acquire',owner='probe',resources=['base']))
            assert lease.ok,lease.code
            now=seconds(peer.states[-1].stamp)
            command=MotionCommand(schema_version=1,runtime_mode='simulation',command_id='move',task_id='stop_probe',
                lease_id=lease.lease_id,epoch=lease.epoch,profile_hash=profile.digest,mode='base_velocity',
                resource_group='base',stamp=stamp(now),valid_for=3.,duration=2.)
            command.velocity.linear.x=.15
            reply=call(node,submit_client,SubmitMotion.Request(command=command));assert reply.accepted,reply.code
            until(node,lambda:peer.states[-1].state=='RUNNING' and peer.states[-1].base_velocity.linear.x>.03)
            if signum is None:
                reply=call(node,control_client,ControlExecution.Request(operation='stop',lease_id=lease.lease_id,epoch=lease.epoch))
                assert reply.ok,reply.code
                until(node,lambda:peer.states[-1].state=='STOPPING' and any(c.operation=='hold' for c in peer.commands))
                reply=call(node,lease_client,Lease.Request(operation='release',lease_id=lease.lease_id,epoch=lease.epoch))
                assert reply.ok
                reply=call(node,lease_client,Lease.Request(operation='acquire',owner='new',resources=['base']))
                assert not reply.ok and reply.code=='BUSY'
                command.command_id='late'
                reply=call(node,submit_client,SubmitMotion.Request(command=command))
                assert not reply.accepted and reply.code=='INVALID_LEASE'
                # A fresh DDS packet with an old source timestamp is no stop acknowledgement.
                peer.frozen=True;peer.v=0.
                held_until=peer.seq+5
                until(node,lambda:peer.seq>=held_until,timeout=.15)
                assert peer.states[-1].state=='STOPPING'
                peer.frozen=False
                until(node,lambda:peer.states[-1].state=='CANCELED')
                assert peer.states[-1].base_velocity.linear.x==0. and not peer.states[-1].stop_failures
                reply=call(node,lease_client,Lease.Request(operation='acquire',owner='new',resources=['base']))
                assert reply.ok,reply.code
            else:
                peer.ack=ack
                peer.freeze_clock=sim_clock
                os.killpg(process.pid,signum)
                until(node,lambda:process.poll() is not None,timeout=3.)
                assert process.returncode==(0 if ack and not sim_clock else 1),process.returncode
                assert any(c.operation=='hold' and c.resource_group=='base' for c in peer.commands)
                if not ack or sim_clock:
                    assert any(s.code=='STOP_FEEDBACK_TIMEOUT' and s.stop_failures for s in peer.states)
            return {'case':label,'passed':True,'packets':len(peer.commands),'returncode':process.poll(),
                    'cold_control_rejected':True,'initial_reference_empty':True,'simulation_only':True}
        finally:
            if process.poll() is None:
                peer.ack=True;peer.frozen=False
                os.killpg(process.pid,signal.SIGINT)
                try:until(node,lambda:process.poll() is not None,timeout=3.)
                except AssertionError:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2.)
            for client in clients:node.destroy_client(client)
            peer.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('ISOLATED_LOCALHOST_DOMAIN_REQUIRED')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    path=output/'profile.json'
    path.write_text(json.dumps({'name':'stop_probe','groups':{'arm':['j']},'limits':{'j':[-1.,1.]},
        'max_speed':1.,'capabilities':['joint_position','base_velocity'],'execution':{'stop_timeout':.8}}))
    profile=Profile.load(path)
    rclpy.init();node=rclpy.create_node('execution_stop_verifier_'+uuid.uuid4().hex[:8])
    results=[]
    try:
        for label,signum,ack,sim_clock in (
                ('asynchronous_cancel_and_old_owner',None,True,False),
                ('sigint_waits_for_rest',signal.SIGINT,True,False),
                ('sigterm_waits_for_rest',signal.SIGTERM,True,False),
                ('unconfirmed_shutdown_fails',signal.SIGTERM,False,False),
                ('frozen_ros_clock_has_monotonic_shutdown_budget',signal.SIGTERM,False,True)):
            result=scenario(node,output,profile,path,label,signum,ack,sim_clock)
            results.append(result);print(json.dumps(result),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()
        (output/'results.json').write_text(json.dumps({'cases':results,'hardware_started':False,
            'passed':len(results)==5 and all(r['passed'] for r in results)},indent=2)+'\n')


if __name__=='__main__':main()
