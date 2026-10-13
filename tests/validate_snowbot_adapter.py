#!/usr/bin/env python3
"""Isolated ROS wire test with original vendor messages and synthetic feedback.

Never starts the vendor controller, SOEM, or any robot device. Call with
ROS_LOCALHOST_ONLY=1 and a dedicated ROS_DOMAIN_ID after loading both installs.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from std_msgs.msg import Float64MultiArray
from snowbot_ctl_msgs.msg import JointPositionCommand, RetInformation
from bindu_interfaces.msg import MotionCommand, ExecutionState
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution
from bindu_runtime.common import stamp
from bindu_hardware.snowbot import GROUPS
from bindu_contracts.profile import Profile


def until(node, predicate, timeout=6.):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.01)
        if predicate(): return
    raise AssertionError('ROS condition timed out')


def call(node, client, request):
    until(node, client.service_is_ready)
    future = client.call_async(request)
    until(node, future.done)
    return future.result()


def terminate(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=4.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=2.)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('An isolated localhost ROS domain is required')
    root = Path(__file__).resolve().parents[1]
    profile_path = root/'src/integration/bindu_runtime/config/snowbot_vendor_position.json'
    profile = Profile.load(profile_path)
    output = Path(args.output).resolve(); output.mkdir(parents=True, exist_ok=False)
    token = uuid.uuid4().hex[:8]
    vendor, ns = '/snowbot_fixture_'+token, '/bindu_adapter_'+token
    rclpy.init(); node = rclpy.create_node('snowbot_verifier_'+token)
    source = node.create_publisher(RetInformation, vendor+'/dualArmCtl_fdback', 1)
    q = dict.fromkeys(profile.limits, 0.)
    q['arm_l_joint_2'], q['arm_r_joint_2'] = .2, -.2
    config = {'feedback': False, 'velocity': 0.}
    packets, states, outcomes, processes, handles = [], [], [], [], []

    def feedback():
        if not config['feedback']: return
        msg = RetInformation(); msg.header.stamp = stamp(node.get_clock().now().nanoseconds*1e-9)
        for prefix, groups in (('l', ('left_arm', 'left_tool_axis')),
                               ('r', ('right_arm', 'right_tool_axis')),
                               ('torso', ('waist',)), ('head', ('head',))):
            names = [n for g in groups for n in GROUPS[g]]
            setattr(msg, prefix+'_position', [float(q[n]) for n in names])
            setattr(msg, prefix+'_speed', [float(config['velocity'])]*len(names))
        source.publish(msg)

    def arm(msg):
        values = dict(zip(GROUPS['left_arm']+GROUPS['right_arm'],
                          list(msg.l_arm_position)+list(msg.r_arm_position)))
        packets.append(('arms', values)); q.update(values)

    def waist(msg):
        values = dict(zip(GROUPS['waist'], msg.data)); packets.append(('waist', values)); q.update(values)

    node.create_timer(.01, feedback)
    node.create_subscription(JointPositionCommand, vendor+'/joint_position_cmd', arm, 10)
    node.create_subscription(Float64MultiArray, vendor+'/body/motor_cmd', waist, 10)
    node.create_subscription(ExecutionState, ns+'/execution/state', lambda m: states.append(m), 10)
    lease = node.create_client(Lease, ns+'/execution/lease')
    submit = node.create_client(SubmitMotion, ns+'/execution/submit')
    control = node.create_client(ControlExecution, ns+'/execution/control')

    def start(enabled, simulated):
        handle = (output/('enabled.log' if enabled else 'readonly.log')).open('w'); handles.append(handle)
        p = subprocess.Popen(['ros2', 'run', 'bindu_runtime', 'snowbot_adapter', '--ros-args',
            '-r', '__ns:='+ns, '-p', 'profile:='+str(profile_path),
            '-p', 'vendor_namespace:='+vendor, '-p', 'command_enabled:='+str(enabled).lower(),
            '-p', 'simulation:='+str(simulated).lower()], stdout=handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        processes.append(p); until(node, lease.service_is_ready); return p

    def acquire(groups=('left_arm', 'right_arm', 'waist')):
        return call(node, lease, Lease.Request(operation='acquire', owner='wire_test', resources=list(groups)))

    def command(owner, identity):
        groups = ['waist', 'left_arm', 'right_arm']
        names = [n for g in groups for n in GROUPS[g]]
        positions = [q[n]+(.03 if n in ('body_joint_1', 'arm_l_joint_1') else 0.) for n in names]
        return MotionCommand(command_id=identity, task_id='wire', lease_id=owner.lease_id,
            epoch=owner.epoch, profile_hash=profile.digest, schema_version=2,
            mode='joint_target', resource_groups=groups, joint_names=names, positions=positions,
            stamp=stamp(node.get_clock().now().nanoseconds*1e-9), valid_for=.3)

    try:
        p = start(False, False)
        reply = acquire(); assert not reply.ok and 'COMMANDS_DISABLED' in reply.code
        config['feedback'] = True
        until(node, lambda: states and states[-1].feedback_stamp.sec > 0)
        assert not states[-1].simulated and not packets
        assert node.count_publishers(vendor+'/joint_position_cmd') == 0
        outcomes.append('read_only_no_commands_and_real_feedback_label')
        terminate(p)
        until(node, lambda: not lease.service_is_ready())
        states.clear(); config['feedback'] = False
        p = start(True, True)
        reply = acquire(); assert not reply.ok and 'FEEDBACK_UNAVAILABLE' in reply.code
        outcomes.append('cold_feedback_rejected')
        config['feedback'] = True
        until(node, lambda: states and states[-1].feedback_stamp.sec > 0)
        reply = acquire(('left_arm',)); assert not reply.ok and 'COUPLED_OWNERSHIP' in reply.code
        owner = acquire(); assert owner.ok, owner.code
        assert not packets  # Acquisition is also motionless.
        msg = command(owner, 'position')
        reply = call(node, submit, SubmitMotion.Request(command=msg)); assert reply.accepted, reply.code
        until(node, lambda: {kind for kind, _ in packets} == {'arms', 'waist'})
        assert next(values for kind, values in packets if kind == 'arms')['arm_r_joint_2'] == -.2
        outcomes.append('original_vendor_messages_and_synchronized_group_mapping')
        config['velocity'] = .2
        reply = call(node, control, ControlExecution.Request(operation='hold', lease_id=owner.lease_id, epoch=owner.epoch))
        assert reply.ok
        until(node, lambda: states[-1].state == 'STOPPING')
        count = len(states)
        until(node, lambda: len(states) > count+4)
        assert states[-1].state == 'STOPPING'
        config['velocity'] = 0.
        until(node, lambda: states[-1].state == 'CANCELED')
        outcomes.append('hold_waits_for_new_measured_velocity_zero')
        bad = command(owner, 'old'); bad.stamp = stamp(node.get_clock().now().nanoseconds*1e-9-1.)
        reply = call(node, submit, SubmitMotion.Request(command=bad)); assert not reply.accepted
        bad = command(owner, 'timed'); bad.mode = 'finite_trajectory'
        reply = call(node, submit, SubmitMotion.Request(command=bad)); assert not reply.accepted
        outcomes.append('old_and_timed_commands_rejected')
        msg = command(owner, 'timeout')
        reply = call(node, submit, SubmitMotion.Request(command=msg)); assert reply.accepted, reply.code
        until(node, lambda: states[-1].state == 'FAILED' and states[-1].code == 'COMMAND_TIMEOUT')
        outcomes.append('input_timeout_hold_and_failed_result')
        msg = command(owner, 'feedback_loss')
        reply = call(node, submit, SubmitMotion.Request(command=msg)); assert reply.accepted, reply.code
        until(node, lambda: states[-1].command_id == 'feedback_loss' and states[-1].state == 'RUNNING')
        config['feedback'] = False
        until(node, lambda: states[-1].state == 'FAILED' and 'STOP_UNCONFIRMED' in states[-1].code)
        assert not states[-1].lease_active
        outcomes.append('feedback_loss_stop_unconfirmed_and_lease_revoked')
        result = {'scope':'synthetic ROS only; original vendor message package; no SOEM or hardware',
                  'checks':outcomes, 'packets':len(packets), 'profile_sha256':profile.digest}
        (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        for p in processes: terminate(p)
        for handle in handles: handle.close()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
