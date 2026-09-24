"""Bounded ROS fault injection between an owned executor and local PhysX."""
from copy import deepcopy
import json
import os
import signal
import time
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
        drain(.6);before=list(seen['physics'].joints.position);sequence=seen['physics'].sequence;drain(.4)
        measurement=seen['physics']
        age=node.get_clock().now().nanoseconds/1e9-measurement.stamp.sec-measurement.stamp.nanosec/1e9
        assert measurement.sequence>sequence and 0<=age<.2,('stale_physical_evidence',age)
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


def recovery_checks(node,profile,seen,results,command,send,finished,release,drain,submit,
                    current,relay,restart_executor,restart_physics):
    """Actually kill owned processes. Recovery is explicit, never auto-replay.

    A new executor is deliberately unable to take over an already bound PhysX
    bridge. Reset both sides to rebind; a real hardware re-arm protocol is not
    implemented by this simulation acceptance test.
    """
    from physics_stress import process_memory

    def fresh_physics(previous_id=None):
        def ready():
            msg=seen['physics']
            age=node.get_clock().now().nanoseconds/1e9-msg.stamp.sec-msg.stamp.nanosec/1e9
            return msg.ready and 0<=age<.2 and (previous_id is None or msg.simulator_id!=previous_id)
        until(node,ready,seconds=90.)
        drain(.5)

    def stopped():
        fresh_physics()
        drain(.7)
        start=deepcopy(seen['physics']);drain(.5);end=seen['physics']
        age=node.get_clock().now().nanoseconds/1e9-end.stamp.sec-end.stamp.nanosec/1e9
        drift=max(abs(a-b) for a,b in zip(start.joints.position,end.joints.position))
        speed=max(map(abs,end.joints.velocity))
        assert end.simulator_id==start.simulator_id and end.sequence>start.sequence and 0<=age<.2
        assert drift<.003 and speed<.02,(drift,speed)
        return {'hold_drift_rad':drift,'max_speed_rad_s':speed}

    def restart_control():
        previous=seen['state'].instance_id
        process=restart_executor()
        until(node,lambda:seen['state'].instance_id!=previous and seen['state'].feedback_stamp.sec>0,seconds=20.)
        drain(.4)
        return process

    def moving():
        cmd=command('left_arm','finite_trajectory');cmd.joint_names=profile.groups['left_arm']
        msg=seen['physics'];values=dict(zip(msg.joints.name,msg.joints.position))
        target=[values[n] for n in cmd.joint_names]
        low,high=profile.limits[cmd.joint_names[0]]
        target[0]+=.25 if target[0]<(low+high)/2 else -.25
        cmd.points=[JointTrajectoryPoint(positions=target,time_from_start=Duration(sec=3))]
        send(cmd);drain(.6)
        assert seen['state'].state=='RUNNING',seen['state'].code
        return cmd

    def old_lease_rejected(cmd):
        replay=deepcopy(cmd);replay.command_id=uuid.uuid4().hex;replay.stamp=node.get_clock().now().to_msg()
        reply=wait(node,submit.call_async(SubmitMotion.Request(command=replay)))
        assert not reply.accepted and reply.code=='INVALID_LEASE',reply
        return reply.code

    # The preceding identity-injection test latched the old transport. A new
    # physics process and executor provide a clean explicitly re-armed baseline.
    previous=seen['physics'].simulator_id
    restart_physics();fresh_physics(previous)
    process=restart_control()
    for cycle in range(2):
        cmd=moving();identity=seen['physics'].simulator_id
        memory=process_memory(process.pid)
        assert 'execution' in memory,('owned_executor_not_found',memory)
        killed_pid=memory['execution']['pid'];started=time.monotonic()
        os.kill(killed_pid,signal.SIGKILL)
        current[0]=None
        until(node,lambda:seen['physics'].code=='SIM_COMMAND_TIMEOUT',seconds=2.)
        watchdog=time.monotonic()-started
        hold=stopped()
        old_instance=seen['state'].instance_id
        process=restart_control()
        assert seen['state'].instance_id!=old_instance
        old_code=old_lease_rejected(cmd)
        hold_after_restart=stopped()
        # New upper-layer commands cannot silently rebind the physical bridge.
        probe=command('left_arm','joint_target');probe.joint_names=cmd.joint_names
        values=dict(zip(seen['physics'].joints.name,seen['physics'].joints.position))
        probe.positions=[values[n] for n in probe.joint_names];probe.positions[0]+=.1
        probe.valid_for=.25;send(probe)
        until(node,lambda:seen['physics'].code=='SIM_EXECUTOR_CHANGED',seconds=2.)
        blocked=stopped();current[0]=None
        results.append({'case':'executor_crash_restart_no_auto_resume','cycle':cycle,'passed':True,
            'killed_pid':killed_pid,'watchdog_observed_s':watchdog,'old_lease_code':old_code,
            'rebind_code':'SIM_EXECUTOR_CHANGED','simulator_id':identity,
            'after_loss':hold,'after_restart':hold_after_restart,'blocked_new_sender':blocked})

        previous=seen['physics'].simulator_id
        restart_physics();fresh_physics(previous)
        process=restart_control()
        recovery=moving();finished(recovery);release();drain(.3)
        results.append({'case':'explicit_full_restart_new_command_succeeds','cycle':cycle,'passed':True,
                        'command_id':recovery.command_id,**stopped()})

        cmd=moving();packet=deepcopy(relay.last_command)
        identity=seen['physics'].simulator_id;started=time.monotonic()
        restart_physics(crash=True);current[0]=None
        until(node,lambda:seen['state'].command_id==cmd.command_id and seen['state'].state=='FAILED',seconds=3.)
        loss_code=seen['state'].code;loss_time=time.monotonic()-started
        restart_physics();fresh_physics(identity)
        hold=stopped()
        old_code=old_lease_rejected(cmd)
        # Old simulator-targeted packets must remain invalid after a real reboot.
        relay.command_pub.publish(packet)
        until(node,lambda:seen['physics'].code=='SIM_ID_OR_PROFILE_MISMATCH',seconds=2.)
        blocked=stopped()
        results.append({'case':'simulator_crash_restart_old_commands_rejected','cycle':cycle,'passed':True,
            'failure_code':loss_code,'failure_observed_s':loss_time,
            'old_simulator_id':identity,'new_simulator_id':seen['physics'].simulator_id,
            'old_lease_code':old_code,'packet_code':'SIM_ID_OR_PROFILE_MISMATCH',
            'after_restart':hold,'after_replay':blocked,
            'limitation':'Simulator resets scene on restart; no physical coast-down exists while process is absent'})
        process=restart_control()
        recovery=moving();finished(recovery);release();drain(.3)
        results.append({'case':'simulator_restart_explicit_control_recovery','cycle':cycle,'passed':True,
                        'command_id':recovery.command_id,**stopped()})
