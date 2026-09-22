"""Bounded ROS fault injection between an owned executor and local PhysX."""
from copy import deepcopy
import json
import uuid
from bindu_interfaces.msg import SimulationFeedback, SimulationCommand
from bindu_interfaces.srv import SubmitMotion, ControlExecution
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from validate_ros import until, wait


class FaultRelay:
    def __init__(self,node,physical,front,output):
        self.node=node;self.mode='normal';self.frozen=None;self.last_command=None
        self.drop_commands=False;self.events=(output/'fault-injection.jsonl').open('w')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.feedback_pub=node.create_publisher(SimulationFeedback,front+'/simulation/feedback',qos)
        self.command_pub=node.create_publisher(SimulationCommand,physical+'/simulation/command',8)
        self.subs=[node.create_subscription(SimulationFeedback,physical+'/simulation/feedback',self.feedback,qos),
            node.create_subscription(SimulationCommand,front+'/simulation/command',self.command,8)]

    def mark(self,mode):
        self.mode=mode;self.frozen=None
        self.events.write(json.dumps({'stamp':self.node.get_clock().now().nanoseconds/1e9,'mode':mode,
                                     'drop_commands':self.drop_commands})+'\n');self.events.flush()

    def feedback(self,msg):
        if self.mode=='drop':return
        if self.mode=='freeze':
            if self.frozen is None:self.frozen=deepcopy(msg)
            msg=self.frozen  # Never refresh the original measurement stamp.
        elif self.mode=='changed_identity':
            msg=deepcopy(msg);msg.simulator_id='injected-new-instance'
        self.feedback_pub.publish(msg)

    def command(self,msg):
        self.last_command=deepcopy(msg)
        if not self.drop_commands:self.command_pub.publish(msg)

    def close(self):
        self.events.close()
        for sub in self.subs:self.node.destroy_subscription(sub)
        self.node.destroy_publisher(self.feedback_pub);self.node.destroy_publisher(self.command_pub)


def fault_checks(node,profile,seen,results,command,send,release,drain,submit,control,current,relay):
    def moving_command():
        cmd=command('left_arm','finite_trajectory');cmd.joint_names=profile.groups['left_arm']
        values=dict(zip(seen['physics'].joints.name,seen['physics'].joints.position))
        target=[values[n] for n in cmd.joint_names];target[0]+=.4
        cmd.points=[JointTrajectoryPoint(positions=target,time_from_start=Duration(sec=3))]
        send(cmd);drain(.6);return cmd

    def hold_sample():
        drain(.6);before=list(seen['physics'].joints.position);drain(.4)
        drift=max(abs(a-b) for a,b in zip(before,seen['physics'].joints.position))
        speed=max(map(abs,seen['physics'].joints.velocity))
        assert drift<.003 and speed<.02,(drift,speed)
        return drift,speed

    for mode in ('drop','freeze'):
        cmd=moving_command();relay.mark(mode)
        until(node,lambda:seen['state'].command_id==cmd.command_id and seen['state'].state=='FAILED',seconds=3.)
        code=seen['state'].code;current[0]=None
        drift,speed=hold_sample()
        relay.mark('normal');drain(.4)
        cmd.command_id=uuid.uuid4().hex;cmd.stamp=node.get_clock().now().to_msg()
        rejected=wait(node,submit.call_async(SubmitMotion.Request(command=cmd)))
        assert not rejected.accepted and rejected.code=='INVALID_LEASE',rejected
        results.append({'case':mode+'_feedback_fail_closed','passed':True,'failure_code':code,
            'hold_drift_rad':drift,'max_speed_rad_s':speed,'old_lease_code':rejected.code})

    cmd=moving_command();release()
    until(node,lambda:seen['state'].command_id==cmd.command_id and seen['state'].state=='CANCELED',
          seconds=profile.stop_timeout+1.)
    drift,speed=hold_sample()
    results.append({'case':'release_after_previous_stop_failure','passed':True,
        'hold_drift_rad':drift,'max_speed_rad_s':speed})

    cmd=moving_command();relay.drop_commands=True;relay.mark('normal')
    until(node,lambda:seen['physics'].code=='SIM_COMMAND_TIMEOUT',seconds=2.)
    delayed=deepcopy(relay.last_command)
    reply=wait(node,control.call_async(ControlExecution.Request(operation='stop',lease_id=cmd.lease_id,epoch=cmd.epoch)))
    assert reply.ok,reply.code
    until(node,lambda:seen['state'].state!='STOPPING' and not seen['state'].reference.name,
          seconds=profile.stop_timeout+1.)
    assert seen['state'].state=='FAILED' and seen['state'].code=='STOP_FEEDBACK_TIMEOUT',seen['state'].code
    stop_code=seen['state'].code
    current[0]=None
    drift,speed=hold_sample()
    relay.drop_commands=False;relay.mark('normal')
    # Retain the source timestamp but use a newer sequence so TTL, rather
    # than sequence rejection, is exercised by a delayed command.
    delayed.sequence=relay.last_command.sequence+1
    relay.command_pub.publish(delayed)
    until(node,lambda:seen['physics'].code=='SIM_STALE_COMMAND',seconds=2.)
    drain(.2)
    results.append({'case':'command_loss_watchdog_delayed_packet_rejected','passed':True,
        'hold_drift_rad':drift,'max_speed_rad_s':speed,'delayed_code':'SIM_STALE_COMMAND','stop_code':stop_code})

    # Identity fault injection tests the ROS transport latch; it does not
    # claim that the Isaac process was actually restarted.
    relay.mark('changed_identity');drain(.4)
    relay.mark('normal');drain(.4)
    cmd=command('left_arm','joint_target');cmd.joint_names=profile.groups['left_arm']
    values=dict(zip(seen['physics'].joints.name,seen['physics'].joints.position))
    cmd.positions=[values[n] for n in cmd.joint_names]
    reply=wait(node,submit.call_async(SubmitMotion.Request(command=cmd)))
    assert not reply.accepted and reply.code=='INVALID_LEASE',reply
    results.append({'case':'changed_simulator_identity_latched','passed':True,'rejection_code':reply.code,
        'injection':'ROS simulator_id substitution; no process restart'})
    current[0]=None
