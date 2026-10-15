#!/usr/bin/env python3
"""Isolated ROS wire test with original controller messages and synthetic feedback.

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
from std_msgs.msg import Float64MultiArray, String, UInt64MultiArray
from snowbot_ctl_msgs.msg import JointPositionCommand, RetInformation, CtlInformation, MidNodeCtlMsgSend
from bindu_interfaces.msg import MotionCommand, ExecutionState
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution
from bindu_runtime.common import stamp
from bindu_hardware.snowbot import GROUPS
from bindu_contracts.profile import Profile
from ament_index_python.packages import get_package_prefix


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


def terminate(process, node=None):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        end = time.monotonic()+6.5
        while process.poll() is None and time.monotonic() < end:
            if node is not None:
                rclpy.spin_once(node, timeout_sec=.01)
            else:
                time.sleep(.01)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=2.)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--shutdown-check', action='store_true',
                        help='Also verify active SIGINT/SIGTERM hold and feedback confirmation')
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
    health_pub = node.create_publisher(String, vendor+'/bindu/servo_health', 1)
    source_pub = node.create_publisher(String, vendor+'/bindu/control_source', 1)
    # Graph fixtures for the two internal control hops; no control is executed.
    bridge_pubs = [node.create_publisher(MidNodeCtlMsgSend, vendor+'/upperMonitor_msg_send', 1),
                   node.create_publisher(CtlInformation, vendor+'/ctl_data_pub', 1)]
    node.create_subscription(MidNodeCtlMsgSend, vendor+'/upperMonitor_msg_send', lambda m: None, 1)
    node.create_subscription(CtlInformation, vendor+'/ctl_data_pub', lambda m: None, 1)
    q = dict.fromkeys(profile.limits, 0.)
    q['arm_l_joint_2'], q['arm_r_joint_2'] = .2, -.2
    config = {'feedback': False, 'velocity': 0., 'tracking': True,
              'health': True, 'source': 'topic', 'pdo': 0, 'pdo_progress': True, 'guard_session': 0}
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
        if config['health']:
            if config['pdo_progress']: config['pdo'] += 1
            now = node.get_clock().now().nanoseconds*1e-9
            source_pub.publish(String(data=json.dumps(dict(schema=1, stamp=now, source=config['source']))))
            health_pub.publish(String(data=json.dumps(dict(schema=1, stamp=now,
                sample=config['pdo'], sample_age=.001, operational=True, ready=True,
                guard_enabled=True, guard_armed=bool(config['guard_session']), guard_latched=False,
                wkc=40, expected_wkc=40, fault_mask=0, present_mask=0xfffff,
                enable_requested_mask=0, operation_enabled_mask=0, guard_session=config['guard_session']))))

    def arm(msg):
        values = dict(zip(GROUPS['left_arm']+GROUPS['right_arm'],
                          list(msg.l_arm_position)+list(msg.r_arm_position)))
        packets.append(('arms', values))
        if config['tracking']: q.update(values)

    def waist(msg):
        values = dict(zip(GROUPS['waist'], msg.data)); packets.append(('waist', values))
        if config['tracking']: q.update(values)

    node.create_timer(.01, feedback)
    node.create_subscription(JointPositionCommand, vendor+'/joint_position_cmd', arm, 10)
    node.create_subscription(Float64MultiArray, vendor+'/body/motor_cmd', waist, 10)
    heartbeats = []
    def heartbeat(m):
        heartbeats.append(m)
        if len(m.data) == 3: config['guard_session'] = m.data[2]
    node.create_subscription(UInt64MultiArray, vendor+'/bindu/heartbeat', heartbeat, 1)
    node.create_subscription(ExecutionState, ns+'/execution/state', lambda m: states.append(m), 10)
    lease = node.create_client(Lease, ns+'/execution/lease')
    submit = node.create_client(SubmitMotion, ns+'/execution/submit')
    control = node.create_client(ControlExecution, ns+'/execution/control')

    def start(enabled):
        # Each phase uses a newly booted fake guard. A real guard deliberately
        # forbids a second session without an operator's driver recovery.
        config['guard_session'] = 0
        handle = (output/(('enabled' if enabled else 'readonly')+f'-{len(processes)}.log')).open('w')
        handles.append(handle)
        # Signal the actual entry process, not a ros2 run wrapper. Both call
        # the same installed entry point; a wrapper can exit before its child.
        executable = Path(get_package_prefix('bindu_runtime'))/'lib/bindu_runtime/snowbot_adapter'
        p = subprocess.Popen([str(executable), '--ros-args',
            '-r', '__ns:='+ns, '-p', 'profile:='+str(profile_path),
            '-p', 'vendor_namespace:='+vendor, '-p', 'command_enabled:='+str(enabled).lower(),
            '-p', 'arm_receiver_node:=/'+node.get_name(),
            '-p', 'servo_receiver_node:=/'+node.get_name(),
            '-p', 'mid_receiver_node:=/'+node.get_name(),
            '-p', 'runtime_mode:=real'], stdout=handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        processes.append(p); until(node, lease.service_is_ready); return p

    def acquire(groups=('left_arm', 'right_arm', 'waist')):
        return call(node, lease, Lease.Request(operation='acquire', owner='wire_test', resources=list(groups)))

    def retired():
        # A dead process can still have DDS endpoints in discovery caches.
        # Wait for its command writers to disappear before reusing the namespace.
        return not lease.service_is_ready() and all(node.count_publishers(vendor+'/'+topic) == 0
            for topic in ('joint_position_cmd', 'body/motor_cmd', 'head/motor_cmd'))

    def acquire_ready():
        deadline = time.monotonic()+6.
        while time.monotonic() < deadline:
            reply = acquire()
            if reply.ok:
                return reply
            assert any(code in reply.code for code in ('FEEDBACK_UNAVAILABLE', 'RECEIVER_UNAVAILABLE',
                                                       'HEALTH_UNAVAILABLE', 'SERVO_NOT_READY',
                                                       'HEARTBEAT_UNAVAILABLE',
                                                       'PUBLISHER_CONFLICT')), reply.code
            rclpy.spin_once(node, timeout_sec=.01)
        raise AssertionError('Vendor ROS route not ready: '+reply.code)

    def command(owner, identity):
        groups = ['waist', 'left_arm', 'right_arm']
        names = [n for g in groups for n in GROUPS[g]]
        positions = [q[n]+(.03 if n in ('body_joint_1', 'arm_l_joint_1') else 0.) for n in names]
        return MotionCommand(command_id=identity, task_id='wire', runtime_mode='real', lease_id=owner.lease_id,
            epoch=owner.epoch, profile_hash=profile.digest, schema_version=2,
            mode='joint_target', resource_groups=groups, joint_names=names, positions=positions,
            stamp=stamp(node.get_clock().now().nanoseconds*1e-9), valid_for=.3)

    try:
        p = start(False)
        reply = acquire(); assert not reply.ok and 'COMMANDS_DISABLED' in reply.code
        config['feedback'] = True
        until(node, lambda: states and states[-1].feedback_stamp.sec > 0)
        assert not states[-1].simulated and not packets
        assert node.count_publishers(vendor+'/joint_position_cmd') == 0
        outcomes.append('read_only_no_commands_and_real_feedback_label')
        terminate(p, node)
        until(node, retired)
        states.clear(); config['feedback'] = False
        config['health'] = False
        p = start(True)
        reply = acquire(); assert not reply.ok and 'FEEDBACK_UNAVAILABLE' in reply.code
        outcomes.append('cold_feedback_rejected')
        config['feedback'] = True
        until(node, lambda: states and states[-1].feedback_stamp.sec > 0)
        reply = acquire(); assert not reply.ok and 'HEALTH_UNAVAILABLE' in reply.code, reply.code
        assert not packets and not heartbeats
        outcomes.append('unpatched_vendor_health_missing_rejects_takeover')
        config['health'], config['source'] = True, 'demo'
        def rejects(code):
            reply = acquire()
            assert not reply.ok, 'Unsafe acquisition while expecting '+code
            return code in reply.code
        until(node, lambda: rejects('CONTROL_SOURCE_NOT_TOPIC'))
        assert not packets and not heartbeats
        outcomes.append('actual_demo_control_source_rejects_takeover_and_heartbeat')
        config['source'] = 'topic'
        until(node, lambda: bool(heartbeats))
        assert len(heartbeats[-1].data) == 3 and heartbeats[-1].data[1] > 0
        config['pdo_progress'] = False
        freeze_deadline = time.monotonic()+.09
        until(node, lambda: time.monotonic() >= freeze_deadline)
        until(node, lambda: rejects('SERVO_NOT_READY'))
        assert not packets
        outcomes.append('fresh_ros_messages_with_frozen_pdo_reject_takeover')
        config['pdo_progress'] = True
        reply = acquire(('left_arm',)); assert not reply.ok and 'COUPLED_OWNERSHIP' in reply.code
        owner = acquire_ready()
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
        reply = call(node, lease, Lease.Request(operation='release', lease_id=owner.lease_id, epoch=owner.epoch))
        assert reply.ok, reply.code
        until(node, lambda: not states[-1].lease_active)
        late = command(owner, 'released_owner')
        reply = call(node, submit, SubmitMotion.Request(command=late))
        assert not reply.accepted and reply.code == 'INVALID_LEASE', reply.code
        outcomes.append('release_during_stop_fences_old_owner')
        config['velocity'] = 0.
        until(node, lambda: states[-1].state == 'CANCELED')
        outcomes.append('hold_waits_for_new_measured_velocity_zero')
        owner = acquire_ready()
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
        if args.shutdown_check:
            terminate(p, node)
            assert p.returncode == 1, 'Unconfirmed physical stop must produce a nonzero exit status'
            outcomes.append('unconfirmed_stop_nonzero_exit')
            for signum in (signal.SIGINT, signal.SIGTERM):
                terminate(p, node)
                until(node, retired)
                states.clear()
                config.update(feedback=True, velocity=0., tracking=True)
                p = start(True)
                until(node, lambda: states and states[-1].feedback_stamp.sec > 0)
                owner = acquire_ready()
                identity = 'shutdown_'+str(signum)
                msg = command(owner, identity)
                reply = call(node, submit, SubmitMotion.Request(command=msg)); assert reply.accepted, reply.code
                until(node, lambda: states[-1].command_id == identity and states[-1].state == 'RUNNING')
                # Measured position differs from the previous command. Keep
                # reporting motion until the shutdown hold has really arrived.
                config.update(tracking=False, velocity=.2)
                q['arm_l_joint_1'] += .02
                expected = q['arm_l_joint_1']
                until(node, lambda: abs(dict(zip(states[-1].joints.name, states[-1].joints.position))[
                    'arm_l_joint_1']-expected) < 1e-9)
                before = len(packets)
                got_hold = lambda: any(kind == 'arms' and abs(values['arm_l_joint_1']-expected) < 1e-9
                                      for kind, values in packets[before:])
                os.killpg(p.pid, signum)
                until(node, lambda: got_hold() or p.poll() is not None)
                assert got_hold(), 'Shutdown exited without delivering the measured hold'
                assert p.poll() is None, 'Shutdown exited before measured rest was confirmed'
                config['velocity'] = 0.
                until(node, lambda: p.poll() is not None)
                assert p.returncode == 0, p.returncode
                outcomes.append(signal.Signals(signum).name.lower()+'_hold_before_context_shutdown')
        result = {'scope':'synthetic ROS only; original vendor message package; no SOEM or hardware',
                  'checks':outcomes, 'packets':len(packets), 'profile_sha256':profile.digest}
        (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        (output/'state-tail.json').write_text(json.dumps([{
            'instance': m.instance_id, 'command': m.command_id, 'state': m.state,
            'code': m.code, 'stop_failures': list(m.stop_failures)} for m in states[-16:]], indent=2)+'\n')
        for p in processes: terminate(p, node)
        for handle in handles: handle.close()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
