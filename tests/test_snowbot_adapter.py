"""Vendor boundary and delegated stop semantics; never touches hardware."""
import copy
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

from bindu_contracts.contracts import Motion
from bindu_contracts.profile import Profile
from bindu_execution.delegated_position import DelegatedPositionExecutor
from bindu_execution.executor import Rejected
from bindu_hardware.snowbot import GROUPS, VendorHealth, SnowbotIO, decode_feedback

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'src/integration/bindu_runtime/config'


class SnowbotAdapterTests(unittest.TestCase):
    def setUp(self):
        self.profile = Profile.load(CONFIG/'snowbot_vendor_position.json')
        self.now = 10.
        self.sent = []
        self.io = SnowbotIO(self.profile, lambda g, q: self.sent.append((set(g), dict(q))),
                            lambda: self.now, command_enabled=True)
        self.q = dict.fromkeys(self.profile.limits, 0.)
        self.q['arm_l_joint_2'], self.q['arm_r_joint_2'] = .2, -.2
        self.v = dict.fromkeys(self.q, 0.)
        self.observe()
        self.engine = DelegatedPositionExecutor(self.profile, self.io)

    def observe(self, **changes):
        self.io.observe(changes.get('positions', self.q), changes.get('velocities', self.v),
                        changes.get('stamp', self.now), self.now)

    def acquire(self, groups=('left_arm', 'right_arm')):
        return self.engine.acquire('test', groups, 2., self.now)

    def command(self, **changes):
        lease, epoch = self.acquire()
        names = tuple(GROUPS['left_arm'])
        return replace(Motion('cmd', lease, epoch, self.profile.digest, 'test', '',
                              'joint_target', 'left_arm', self.now, .3, names=names,
                              positions=(.3, .2, 0., 0., 0., 0., 0.), runtime_mode='real'), **changes)

    def tick(self, dt=.02, *, observe=True):
        self.now += dt
        if observe:
            self.observe()
        self.engine.tick(self.now)

    def test_vendor_limits_and_isolated_profile(self):
        self.assertEqual(self.profile.limits['body_joint_2'], [-.79, .55])
        self.assertEqual(self.profile.limits['arm_l_joint_3'], [-2.61, .58])
        self.assertEqual(self.profile.limits['arm_r_joint_3'], [-.58, 2.61])
        self.assertEqual(self.profile.limits['head_joint_1'], [-.791, .791])
        old = Profile.load(CONFIG/'snowbot_shovel_sim.json')
        self.assertEqual(old.limits['body_joint_2'], [-.79, 1.83])
        for filename in ('snowbot_shovel_sim.json', 'wheel_sim.json'):
            with self.assertRaisesRegex(ValueError, 'PROFILE_REQUIRED'):
                SnowbotIO(Profile.load(CONFIG/filename), None, lambda: self.now)

    def test_read_only_and_idle_shutdown_publish_nothing(self):
        self.io.command_enabled = False
        with self.assertRaisesRegex(Rejected, 'COMMANDS_DISABLED'):
            self.acquire()
        self.engine.halt(self.now, 'SHUTDOWN', revoke=True, emergency=True)
        self.assertFalse(self.sent)
        self.assertFalse(self.io.last_feedback.simulated)

    def test_arms_require_pair_ownership_and_gripper_is_context_only(self):
        for groups in (('left_arm',), ('right_arm',), ('left_tool_axis',), ('base',)):
            with self.assertRaises(Rejected): self.acquire(groups)
        self.assertFalse(self.sent)

    def test_unsmoothed_target_and_other_arm_captured_from_feedback(self):
        motion = self.command()
        self.engine.submit(motion, self.now)
        self.tick()
        groups, packet = self.sent[-1]
        self.assertEqual(groups, {'left_arm', 'right_arm'})
        self.assertEqual(packet['arm_l_joint_1'], .3)
        self.assertEqual(packet['arm_r_joint_2'], -.2)
        self.assertEqual(self.engine.reference.q, motion.positions)
        self.assertFalse({'waist', 'head'} & groups)

    def test_partial_groups_and_out_of_limit_targets_rejected_before_send(self):
        for target in ({'arm_l_joint_1': .1}, dict.fromkeys(GROUPS['waist'], 2.),
                       {'arm_l_joint_8': .1}):
            with self.assertRaises(RuntimeError): self.io.write_joints(target)
        self.assertFalse(self.sent)

    def test_bad_captured_other_arm_cannot_bypass_limit_check(self):
        self.q['arm_r_joint_2'] = 1.
        self.now += .01
        self.observe()
        with self.assertRaisesRegex(RuntimeError, 'JOINT_LIMIT'):
            self.io.write_joints(dict.fromkeys(GROUPS['left_arm'], 0.))
        self.assertFalse(self.sent)

    def test_source_stamp_not_refreshed_on_read_and_velocity_is_preserved(self):
        self.v['arm_l_joint_1'] = .2
        self.now += .01
        self.observe()
        fb = self.io.read_feedback(20., .01)
        self.assertEqual(fb.stamp, self.now)
        self.assertEqual(fb.joint_velocities['arm_l_joint_1'], .2)
        self.now += .21
        with self.assertRaisesRegex(Rejected, 'FEEDBACK_UNAVAILABLE'): self.acquire()

    def test_invalid_old_future_and_nonfinite_feedback_invalidates_capture(self):
        for delta in (-1., 1., 0.):
            with self.assertRaises(ValueError): self.observe(stamp=self.now+delta)
            with self.assertRaises(Rejected): self.acquire()
        bad = dict(self.q); bad['body_joint_1'] = float('nan')
        with self.assertRaises(ValueError): self.observe(positions=bad)
        with self.assertRaises(Rejected): self.acquire()
        self.now += .02; self.observe()
        self.acquire()

    def test_joint_target_time_modes_and_ttl_fail_closed(self):
        for changes in ({'mode':'finite_trajectory'}, {'stamp':10.01}, {'valid_for':.5},
                        {'stamp':9.}, {'profile_hash':'wrong'}, {'epoch':999}):
            with self.subTest(changes=changes):
                self.setUp()
                motion = self.command(**changes)
                with self.assertRaises(Rejected): self.engine.submit(motion, self.now)
                self.assertFalse(self.sent)

    def test_stop_waits_for_new_measured_rest_and_fences_commands(self):
        motion = self.command(); self.engine.submit(motion, self.now); self.tick()
        self.engine.halt(self.now)
        self.assertEqual(self.sent[-1][1]['arm_l_joint_1'], self.q['arm_l_joint_1'])
        self.assertIsNotNone(self.engine.stopping)
        with self.assertRaisesRegex(Rejected, 'STOPPING'):
            self.engine.submit(replace(motion, command_id='late'), self.now)
        self.engine.tick(self.now)
        self.assertIsNotNone(self.engine.stopping)  # Same feedback is no confirmation.
        self.v['arm_l_joint_1'] = .2
        self.tick(); self.assertIsNotNone(self.engine.stopping)
        self.v['arm_l_joint_1'] = 0.
        self.tick(); self.assertIsNone(self.engine.stopping)
        self.assertEqual(self.engine.results['cmd'].state, 'CANCELED')

    def test_timeout_requests_hold_and_never_returns_success(self):
        motion = self.command(); self.engine.submit(motion, self.now); self.tick()
        for _ in range(16): self.tick()
        result = self.engine.results['cmd']
        self.assertEqual(result.state, 'FAILED')
        self.assertEqual(result.code, 'COMMAND_TIMEOUT')
        self.assertEqual(self.sent[-1][1]['arm_l_joint_1'], self.q['arm_l_joint_1'])

    def test_feedback_loss_marks_stop_unconfirmed_and_blocks_recapture(self):
        motion = self.command(); self.engine.submit(motion, self.now); self.tick()
        for _ in range(12): self.tick(observe=False)
        self.assertIn('STOP_UNCONFIRMED', self.engine.results['cmd'].code)
        self.assertFalse(self.engine.lease_id)
        self.now += .02; self.observe()
        with self.assertRaisesRegex(Rejected, 'STOP_UNCONFIRMED'): self.acquire()

    def test_partial_delivery_failure_still_attempts_hold_for_all_affected_axes(self):
        calls = []
        def partial(groups, target):
            calls.append((groups, target.copy()))
            raise RuntimeError('second topic failed')
        self.io.send = partial
        self.acquire(('waist', 'left_arm', 'right_arm'))
        target = dict.fromkeys(GROUPS['waist']+GROUPS['left_arm']+GROUPS['right_arm'], 0.)
        with self.assertRaises(RuntimeError): self.io.write_joints(target)
        self.io.stop()
        self.assertEqual(len(calls), 3)  # Failed batch, then independent arm and waist holds.
        self.assertEqual(set().union(*(set(values) for _, values in calls[1:])), set(target))
        self.assertTrue(self.io.stop_failures)

    def test_feedback_loss_during_pending_hold_is_still_unconfirmed(self):
        motion = self.command(); self.engine.submit(motion, self.now); self.tick()
        self.engine.halt(self.now)
        self.assertTrue(self.io.stop_targets)
        for _ in range(12): self.tick(observe=False)
        self.assertIn('STOP_UNCONFIRMED', self.engine.results['cmd'].code)
        self.assertFalse(self.engine.lease_id)

    def test_second_fault_cannot_erase_a_pending_physical_stop(self):
        motion = self.command(); self.engine.submit(motion, self.now); self.tick()
        self.engine.halt(self.now)
        held = self.io.stop_targets.copy()
        self.v['arm_l_joint_1'] = .2
        self.tick(.15)  # New feedback but execution overrun during stop.
        self.assertIsNotNone(self.engine.stopping)
        self.assertEqual(self.io.stop_targets, held)
        self.v['arm_l_joint_1'] = 0.; self.tick()
        self.assertEqual(self.engine.results['cmd'].state, 'FAILED')
        self.assertEqual(self.engine.results['cmd'].code, 'EXECUTION_OVERRUN')
        self.assertFalse(self.io.stop_targets)

    def test_continuous_overruns_cannot_extend_stop_timeout(self):
        motion = self.command(); self.engine.submit(motion, self.now); self.tick()
        self.engine.halt(self.now)
        original_deadline = self.engine.stop_deadline
        self.v['arm_l_joint_1'] = .2
        for _ in range(40):
            self.tick(.15)
        self.assertGreater(self.now, original_deadline)
        self.assertIsNone(self.engine.stopping)
        self.assertIn('STOP_UNCONFIRMED', self.engine.results['cmd'].code)
        self.assertTrue(self.io.stop_failures)

    def test_release_during_pending_stop_immediately_revokes_old_lease(self):
        motion = self.command(); self.engine.submit(motion, self.now); self.tick()
        self.engine.halt(self.now)
        self.engine.halt(self.now, revoke=True)
        self.assertFalse(self.engine.lease_id)
        self.assertGreater(self.engine.epoch, motion.epoch)
        self.assertIsNotNone(self.engine.stopping)
        with self.assertRaisesRegex(Rejected, 'INVALID_LEASE'):
            self.engine.check_lease(motion.lease_id, motion.epoch, self.now)
        with self.assertRaisesRegex(Rejected, 'BUSY'):
            self.acquire()

    def test_unready_vendor_is_rejected_before_acquiring_control(self):
        def unready(groups):
            raise RuntimeError('SNOWBOT_VENDOR_RECEIVER_UNAVAILABLE:arms')
        self.io.check_ready = unready
        with self.assertRaisesRegex(Rejected, 'RECEIVER_UNAVAILABLE'):
            self.acquire()
        self.assertFalse(self.engine.lease_id)
        self.assertFalse(self.sent)
        self.io.check_ready = lambda groups: None
        self.acquire()

    def test_failed_waist_hold_does_not_prevent_arm_and_head_holds(self):
        self.acquire(('waist', 'head', 'left_arm', 'right_arm'))
        target = {n: self.q[n] for g in ('waist', 'head', 'left_arm', 'right_arm') for n in GROUPS[g]}
        self.io.write_joints(target)
        attempts = []
        def send(groups, values):
            attempts.append((groups.copy(), values.copy()))
            if 'waist' in groups:
                raise RuntimeError('waist receiver unavailable')
        self.io.send = send
        self.io.stop()
        self.assertEqual([g for g, _ in attempts], [{'left_arm', 'right_arm'}, {'waist'}, {'head'}])
        self.assertEqual(self.io.stop_failures, ('waist',))
        self.assertEqual(set(self.io.stop_targets), set(GROUPS['left_arm']+GROUPS['right_arm']+GROUPS['head']))

    def test_passive_command_observers_are_allowed(self):
        from bindu_runtime.snowbot_adapter import SnowbotDevices
        transport = SnowbotDevices.__new__(SnowbotDevices)
        transport.node = Mock()
        transport.check_health = Mock()
        transport.node.count_publishers.return_value = 1
        transport.arm_type = S
        transport.array_type = lambda data: S(data=data)
        transport.command_topics = {g: '/test/'+g for g in ('arms', 'waist', 'head')}
        transport.heartbeat_topic = '/test/heartbeat'
        transport.receivers = dict.fromkeys(transport.command_topics, '/test/vendor')
        transport.bridge_routes = (('/test/ctl', '/test/vendor', '/test/vendor'),)
        transport.node.get_publishers_info_by_topic.return_value = [S(node_namespace='/test', node_name='vendor')]
        transport.node.get_subscriptions_info_by_topic.return_value = [
            S(node_namespace='/test', node_name='vendor'), S(node_namespace='/', node_name='observer')]
        transport.publishers = {g: Mock() for g in transport.command_topics}
        for publisher in transport.publishers.values():
            publisher.get_subscription_count.return_value = 2  # Vendor + passive observer.
        target = {n: self.q[n] for g in ('waist', 'left_arm', 'right_arm') for n in GROUPS[g]}
        transport.send({'waist', 'left_arm', 'right_arm'}, target)
        transport.publishers['arms'].publish.assert_called_once()
        transport.publishers['waist'].publish.assert_called_once()
        for publisher in transport.publishers.values(): publisher.reset_mock()
        transport.node.get_subscriptions_info_by_topic.return_value = [S(node_namespace='/', node_name='observer')]
        with self.assertRaisesRegex(RuntimeError, 'UNAVAILABLE'):
            transport.send({'left_arm', 'right_arm'}, target)
        transport.publishers['arms'].publish.assert_not_called()

        transport.node.get_subscriptions_info_by_topic.return_value = [S(node_namespace='/test', node_name='vendor')]
        transport.node.count_publishers.side_effect = lambda topic: 2 if topic in transport.command_topics.values() else 1
        with self.assertRaisesRegex(RuntimeError, 'PUBLISHER_CONFLICT'):
            transport.send({'left_arm', 'right_arm'}, target)
        transport.publishers['arms'].publish.assert_not_called()
        transport.node.count_publishers.side_effect = None
        transport.node.count_publishers.return_value = 1
        transport.node.get_publishers_info_by_topic.return_value = []
        with self.assertRaisesRegex(RuntimeError, 'RECEIVER_UNAVAILABLE'):
            transport.send({'left_arm', 'right_arm'}, target)
        transport.publishers['arms'].publish.assert_not_called()

    def test_health_requires_real_source_mode_and_successful_pdo_progress(self):
        steady = [1.]
        health = VendorHealth(lambda: self.now, lambda: steady[0])
        servo = dict(schema=1, stamp=self.now, sample=5, sample_age=.001,
                     operational=True, ready=True, guard_enabled=True, guard_armed=True,
                     guard_latched=False, wkc=40, expected_wkc=40, fault_mask=0,
                     present_mask=0xfffff, enable_requested_mask=0, operation_enabled_mask=0, guard_session=123)
        def publish(**changes):
            self.now += .01
            health.receive('source', json.dumps(dict(schema=1, stamp=self.now, source='topic')))
            health.receive('servo', json.dumps(dict(servo, stamp=self.now, **changes)))
        with self.assertRaisesRegex(RuntimeError, 'HEALTH_UNAVAILABLE'):
            health.require({'waist'})
        publish()
        health.require({'left_arm', 'right_arm', 'waist'})
        steady[0] += .06
        publish()  # New ROS timestamp, same successful PDO sample.
        with self.assertRaisesRegex(RuntimeError, 'SERVO_NOT_READY'):
            health.require({'waist'})
        publish(sample=6)
        health.require({'waist'})
        health.expected_session = 456
        with self.assertRaisesRegex(RuntimeError, 'GUARD_SESSION_CONFLICT'): health.require({'waist'})
        health.expected_session = 123
        publish(sample=7, guard_latched=True)
        with self.assertRaisesRegex(RuntimeError, 'SERVO_NOT_READY'):
            health.require({'waist'})
        publish(sample=8)
        self.now += .01
        health.receive('source', json.dumps(dict(schema=1, stamp=self.now, source='demo')))
        with self.assertRaisesRegex(RuntimeError, 'CONTROL_SOURCE_NOT_TOPIC'):
            health.require({'waist'})

    def test_health_rejects_fault_missing_drive_and_malformed_feedback(self):
        health = VendorHealth(lambda: self.now, lambda: 1.)
        health.receive('source', json.dumps(dict(schema=1, stamp=self.now, source='topic')))
        servo = dict(schema=1, stamp=self.now, sample=5, sample_age=.001,
                     operational=True, ready=True, guard_enabled=True, guard_armed=True,
                     guard_latched=False, wkc=40, expected_wkc=40, fault_mask=0,
                     present_mask=0xfffff, enable_requested_mask=0, operation_enabled_mask=0, guard_session=123)
        for field, value, code in [('wkc', 39, 'SERVO_NOT_READY'), ('ready', False, 'SERVO_NOT_READY'),
                                   ('guard_enabled', False, 'SERVO_NOT_READY'),
                                   ('fault_mask', 1 << 18, 'DRIVE_UNAVAILABLE'),
                                   ('present_mask', 0, 'DRIVE_UNAVAILABLE')]:
            self.now += .01
            health.receive('servo', json.dumps(dict(servo, stamp=self.now, **{field: value})))
            with self.assertRaisesRegex(RuntimeError, code): health.require({'waist'})
        for value in ('{}', '{bad', json.dumps(dict(servo, stamp=self.now+1.)),
                      json.dumps(dict(servo, stamp=self.now, sample=True)),
                      json.dumps(dict(servo, stamp=self.now, operational='true'))):
            with self.assertRaisesRegex(ValueError, 'HEALTH_INVALID'): health.receive('servo', value)
        with self.assertRaisesRegex(RuntimeError, 'HEALTH_UNAVAILABLE'): health.require({'waist'})

    def test_health_loss_during_hold_cannot_confirm_arrival(self):
        self.io.write_joints(dict.fromkeys(GROUPS['waist'], .1))
        self.engine.halt(self.now, 'CANCELED')
        self.io.check_health = Mock(side_effect=RuntimeError('SNOWBOT_SERVO_NOT_READY'))
        self.now += .02
        self.observe()
        self.engine.tick(self.now)
        self.assertTrue(self.io.stop_failures)

    def test_feedback_wire_layout_and_j8_are_not_arm_axis_7(self):
        msg = S(header=S(stamp=S(sec=10, nanosec=20000000)),
                l_position=[.1]*7+[.8], r_position=[-.1]*7+[-.8],
                l_speed=[.01]*8, r_speed=[-.01]*8,
                torso_position=[.2, .3], torso_speed=[.02, .03],
                head_position=[.4, .5], head_speed=[.04, .05])
        q, v, stamp = decode_feedback(msg)
        self.assertEqual(q['arm_l_joint_8'], .8)
        self.assertEqual(q['arm_l_joint_7'], .1)
        self.assertEqual(q['body_joint_2'], .3)
        self.assertEqual(v['head_joint_2'], .05)
        self.assertEqual(stamp, 10.02)
        bad = copy.deepcopy(msg); bad.r_position = [0.]*7
        with self.assertRaises(ValueError): decode_feedback(bad)

    def test_feedback_from_an_observer_cannot_confirm_a_physical_hold(self):
        from bindu_runtime.snowbot_adapter import SnowbotDevices
        transport = SnowbotDevices.__new__(SnowbotDevices)
        transport.node, transport.io = Mock(), Mock()
        transport.feedback_topic = '/test/feedback'
        transport.health_receivers = {'servo': '/test/servo'}
        transport.node.get_publishers_info_by_topic.return_value = [S(node_namespace='/test', node_name='observer')]
        transport.receive(S())
        transport.io.observe.assert_not_called()
        self.assertEqual(transport.io.invalid, 'SNOWBOT_FEEDBACK_PUBLISHER_IDENTITY')


if __name__ == '__main__': unittest.main()
