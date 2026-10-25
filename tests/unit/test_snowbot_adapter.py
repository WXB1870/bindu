"""Position interface and delegated stop semantics; never touches hardware."""
from bindu_contracts.resources import package_share
from tests.support.paths import ROOT
import copy
import importlib.util
import json
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

from bindu_contracts.contracts import Motion
from bindu_contracts.profile import Profile
from bindu_execution.delegated_position import DelegatedPositionExecutor
from bindu_execution.executor import Rejected
from snowbot_hardware.protocol import GROUPS, VendorHealth, SnowbotIO, decode_feedback, feedback_schema, feedback_sample

CONFIG = package_share('snowbot_bringup')/'config'


class SnowbotAdapterTests(unittest.TestCase):
    def setUp(self):
        self.profile = Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_vendor_position.json')
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
        old = Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_shovel_sim.json')
        self.assertEqual(old.limits['body_joint_2'], [-.79, 1.83])
        for filename in ('robots/snowbot/snowbot_shovel_sim.json', 'simulation/wheel_sim.json'):
            with self.assertRaisesRegex(ValueError, 'PROFILE_REQUIRED'):
                SnowbotIO(Profile.load(CONFIG/filename), None, lambda: self.now)

    def test_session_source_binding_changes_only_after_an_accepted_motion(self):
        self.io.source_motion = None
        motion = self.command()
        self.engine.submit(motion, self.now)
        self.assertIs(self.io.source_motion, motion)
        with self.assertRaises(Rejected):
            self.engine.submit(replace(motion, command_id='invalid', positions=(100.,)*7), self.now)
        self.assertIs(self.io.source_motion, motion)

    def test_arm_control_does_not_require_missing_head_waist_velocity(self):
        self.io.last_feedback = replace(self.io.last_feedback, joint_velocities={
            n:v for n,v in self.v.items() if n.startswith('arm_')})
        motion = self.command()
        self.engine.submit(motion, self.now)
        self.tick(observe=False)
        self.assertTrue(self.sent)

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

    def test_stop_marks_explicit_hold_without_marking_normal_targets(self):
        kinds = []
        original = self.io.send
        def send(groups, target):
            kinds.append(self.io.sending_hold)
            original(groups, target)
        self.io.send = send
        self.io.write_joints({n:self.q[n] for n in GROUPS['left_arm']})
        self.io.stop()
        self.assertEqual(kinds, [False, True])
        self.assertFalse(self.io.sending_hold)

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

    def test_invalid_device_sample_reports_detail_and_revokes_without_exiting(self):
        motion=self.command()
        self.engine.submit(motion,self.now)
        self.tick()
        original=self.io.read_feedback
        self.io.read_feedback=Mock(side_effect=ValueError('SNOWBOT_FEEDBACK_STALE_OR_UNORDERED'))
        self.now+=.02
        self.engine.tick(self.now)
        self.assertFalse(self.engine.lease_id)
        self.assertIsNone(self.engine.active)
        self.assertTrue(any(e.state=='DEVICE_FEEDBACK_DETAIL' and
                            e.code=='SNOWBOT_FEEDBACK_STALE_OR_UNORDERED' for e in self.engine.events))
        # Fresh feedback cannot revive the revoked command or ownership.
        self.io.read_feedback=original
        self.tick()
        self.assertFalse(self.engine.lease_id)
        self.assertIsNone(self.engine.active)

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

    def test_frozen_clock_cannot_extend_vendor_hold_budget(self):
        steady = [1.]
        self.engine.steady = lambda: steady[0]
        motion = self.command(); self.engine.submit(motion, self.now); self.tick()
        self.engine.halt(self.now)
        sent = len(self.sent)
        steady[0] += self.profile.stop_timeout + .01
        self.engine.tick(self.now)  # No new ROS time or measured stop evidence.
        self.assertIsNone(self.engine.stopping)
        self.assertIn('STOP_UNCONFIRMED', self.engine.results['cmd'].code)
        self.assertFalse(self.engine.lease_id)
        self.assertTrue(self.io.stop_failures)
        self.assertEqual(len(self.sent), sent)

    def test_late_rest_after_vendor_hold_deadline_cannot_report_cancel(self):
        steady = [1.]
        self.engine.steady = lambda: steady[0]
        motion = self.command(); self.engine.submit(motion, self.now); self.tick()
        self.engine.halt(self.now)
        steady[0] += self.profile.stop_timeout + .01
        self.tick()  # Fresh stationary feedback arrives after the steady budget.
        self.assertIn('STOP_UNCONFIRMED', self.engine.results['cmd'].code)
        self.assertEqual(self.engine.results['cmd'].state, 'FAILED')
        with self.assertRaisesRegex(Rejected, 'STOP_UNCONFIRMED'): self.acquire()

    def test_repeated_vendor_faults_preserve_both_stop_deadlines(self):
        steady = [1.]
        self.engine.steady = lambda: steady[0]
        motion = self.command(); self.engine.submit(motion, self.now); self.tick()
        self.engine.halt(self.now)
        deadlines = (self.engine.stop_deadline, self.engine.stop_steady_deadline)
        self.assertGreater(deadlines[1], steady[0])
        self.v['arm_l_joint_1'] = .2
        for _ in range(3):
            steady[0] += .15
            self.tick(.15)
            self.assertEqual((self.engine.stop_deadline, self.engine.stop_steady_deadline), deadlines)

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
        from snowbot_hardware.adapter import SnowbotDevices
        transport = SnowbotDevices.__new__(SnowbotDevices)
        transport.node = Mock()
        transport.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
        transport.command_sequence, transport.session = 0, 123
        transport.io = S(sending_hold=False)
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
            publisher.get_subscription_count.return_value = 2  # Controller + passive observer.
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

    def sampled_feedback(self, **changes):
        fields = dict(header=S(stamp=S(sec=10, nanosec=20000000)),
                      l_position=[.1]*8, r_position=[-.1]*8,
                      l_speed=[0.]*8, r_speed=[0.]*8,
                      head_position=[.2, .3], head_speed=[0.]*2,
                      torso_position=[.4, .5], torso_speed=[0.]*2,
                      pdo_sample=1, pdo_age=.001, pdo_wkc=42, pdo_expected_wkc=42,
                      feedback_valid_mask=0xfffff, feedback_status=[39]*20)
        return S(**dict(fields, **changes))

    def test_sampled_feedback_dates_measurement_and_omits_invalid_optional_groups(self):
        msg = self.sampled_feedback()
        q, v, stamp = decode_feedback(msg)
        self.assertAlmostEqual(stamp, 10.019)
        self.assertEqual(q['head_joint_1'], .2)
        self.assertEqual(q['body_joint_2'], .5)
        self.assertEqual(feedback_sample(msg), 1)
        msg.feedback_valid_mask &= ~(1 << 16)
        msg.head_position[0] = float('nan')
        q, v, stamp = decode_feedback(msg)
        self.assertNotIn('head_joint_1', q)
        self.assertNotIn('head_joint_2', q)
        self.assertIn('body_joint_1', q)
        msg.feedback_valid_mask &= ~(1 << 18)
        q, _, _ = decode_feedback(msg)
        self.assertNotIn('body_joint_1', q)

    def test_invalid_pdo_never_becomes_feedback_despite_fresh_ros_header(self):
        for fields in (dict(pdo_sample=0), dict(pdo_sample=True), dict(pdo_age=.021),
                       dict(pdo_age=-1.), dict(pdo_age=float('nan')), dict(pdo_wkc=41),
                       dict(pdo_expected_wkc=0), dict(feedback_valid_mask=0xffffe),
                       dict(feedback_valid_mask=0x100ffff), dict(feedback_status=[])):
            with self.subTest(fields=fields), self.assertRaisesRegex(ValueError, 'SNOWBOT_PDO'):
                decode_feedback(self.sampled_feedback(**fields))
        msg = self.sampled_feedback()
        del msg.pdo_age
        with self.assertRaisesRegex(ValueError, 'METADATA_INCOMPLETE'):
            decode_feedback(msg)

    def test_republication_cannot_refresh_or_confirm_feedback_and_rollback_fails(self):
        msg = self.sampled_feedback()
        q, v, stamp = decode_feedback(msg)
        self.io.observe(q, v, stamp, 10.02, sample=feedback_sample(msg))
        self.assertTrue(self.io.last_pdo_sample)
        before = (self.io.received_at, self.io.last_source_stamp)
        self.assertFalse(self.io.observe(q, v, stamp+.01, 10.03, sample=1))
        self.assertEqual((self.io.received_at, self.io.last_source_stamp), before)
        self.now = 10.23
        with self.assertRaisesRegex(RuntimeError, 'UNAVAILABLE'):
            self.io.require_feedback()
        # A stationary sample is new when the successful PDO sequence advances.
        self.assertTrue(self.io.observe(q, v, 10.229, 10.23, sample=2))
        self.io.require_feedback(('head', 'waist'))
        for sample in (1, None):
            with self.assertRaisesRegex(ValueError, 'PDO_SEQUENCE'):
                self.io.observe(q, v, 10.239, 10.24, sample=sample)

    def test_pdo_schema_requires_complete_typed_metadata(self):
        fields = {'header': 'std_msgs/Header', **{prefix+'_'+kind: 'double[8]'
                  for prefix in ('l', 'r') for kind in ('position', 'speed', 'iq')}}
        fields.update(pdo_sample='uint64')
        self.assertFalse(feedback_schema(fields)['compatible'])
        fields.update(pdo_age='double', pdo_wkc='int32', pdo_expected_wkc='int32',
                      feedback_valid_mask='uint32', feedback_status='uint16[20]')
        self.assertTrue(feedback_schema(fields)['pdo_sampling_metadata'])
        self.assertTrue(feedback_schema(fields)['compatible'])

    def test_native_16_axis_feedback_is_kept_without_zero_filling_head_or_waist(self):
        msg = S(header=S(stamp=S(sec=10, nanosec=20000000)),
                l_position=[.1]*7+[.3], r_position=[-.1]*7+[-.3],
                l_speed=[.01]*8, r_speed=[-.01]*8)
        q, v, when = decode_feedback(msg)
        self.io.observe(q, v, when, when)
        self.assertEqual(len(self.io.last_feedback.positions), 16)
        self.assertEqual(self.io.missing_feedback_groups, ['waist', 'head'])
        self.assertNotIn('body_joint_1', self.io.last_feedback.positions)
        self.assertNotIn('head_joint_1', self.io.last_feedback.joint_velocities)
        self.assertNotIn('waist', self.io.source_stamps)
        self.assertEqual(self.io.last_feedback.positions['arm_l_joint_8'], .3)
        self.assertEqual(self.io.last_feedback.joint_velocities['arm_r_joint_8'], -.01)
        self.now = when
        self.io.require_feedback()
        with self.assertRaisesRegex(RuntimeError, 'FEEDBACK_INCOMPLETE:waist'):
            self.io.require_feedback(('waist',))
        with self.assertRaises(Rejected):
            self.engine.acquire('tool', ('left_arm', 'right_arm', 'waist'), 1., when)
        self.assertFalse(self.sent)
        # Missing axes in the next sample must not retain old measured values.
        self.assertEqual(set(self.io.read_feedback(when, 0.).positions), set(q))
        broken = copy.deepcopy(msg); broken.head_position = [0., 0.]
        with self.assertRaisesRegex(ValueError, 'LAYOUT'):
            decode_feedback(broken)

    def test_no_initial_measurements_and_schema_reports_missing_axes(self):
        io = SnowbotIO(self.profile, None, lambda: self.now)
        self.assertEqual(io.last_feedback.positions, {})
        self.assertEqual(io.source_stamps, {})
        fields = {'header': 'std_msgs/Header', **{prefix+'_'+kind: 'double[8]'
            for prefix in ('l', 'r') for kind in ('position', 'speed', 'iq')}}
        report = feedback_schema(fields)
        self.assertTrue(report['compatible'])
        self.assertEqual(report['missing_groups'], ['waist', 'head'])
        fields['l_position'] = 'double[7]'
        self.assertFalse(feedback_schema(fields)['compatible'])

    def test_missing_waist_feedback_during_stop_does_not_suppress_measured_arm_hold(self):
        self.acquire(('left_arm', 'right_arm', 'waist'))
        target = {n: self.q[n] for g in ('left_arm', 'right_arm', 'waist') for n in GROUPS[g]}
        self.io.write_joints(target)
        self.now += .02
        q = {n:q for n,q in self.q.items() if n.startswith('arm_')}
        self.io.observe(q, {n:0. for n in q}, self.now, self.now)
        self.sent.clear()
        self.io.stop()
        self.assertEqual([groups for groups, _ in self.sent], [{'left_arm', 'right_arm'}])
        self.assertEqual(self.io.stop_failures, ('waist',))
        self.assertNotIn('body_joint_1', self.io.stop_targets)

    def test_feedback_from_an_observer_cannot_confirm_a_physical_hold(self):
        from snowbot_hardware.adapter import SnowbotDevices
        transport = SnowbotDevices.__new__(SnowbotDevices)
        transport.node, transport.io = Mock(), Mock()
        transport.feedback_topic = '/test/feedback'
        transport.health_receivers = {'servo': '/test/servo'}
        transport.node.get_publishers_info_by_topic.return_value = [S(node_namespace='/test', node_name='observer')]
        transport.receive(S())
        transport.io.observe.assert_not_called()
        self.assertEqual(transport.io.invalid, 'SNOWBOT_FEEDBACK_PUBLISHER_IDENTITY')


class SnowbotStopDiagnosticTests(unittest.TestCase):
    def test_interrupted_device_session_cannot_ack_pause_or_recover_as_stream_loss(self):
        self.obs.command_sequence=10;self.obs.session_reason=7
        self.obs.reference_valid=self.obs.output_valid=False
        self.obs.hold_valid=False
        self.assertIsNone(self.health.confirmed_hold(self.io.pause_request))
        result=self.diagnose(self.health,self.engine,self.now)
        self.assertEqual(result['ack_reason'],'SESSION_TERMINATED:7')
        self.assertEqual(result['recovery_required'],'device_inspection')
        self.assertEqual(result['sequence'],12)
        self.assertEqual(result['observed']['command_sequence'],10)
        self.assertFalse(result['observed']['reference_valid'])

    def setUp(self):
        from snowbot_hardware.session import CONTROL_NAMES, SessionHealth, stop_diagnostic
        self.diagnose=stop_diagnostic
        self.names=CONTROL_NAMES
        self.now,self.steady=10.,1.
        self.health=SessionHealth(lambda:self.now,lambda:self.steady,reference_mode=2)
        self.obs=S(boot_id=1,pdo_sample=0,pdo_age=.001,wkc=67,expected_wkc=67,valid_mask=0xfffff,
            position=[0.]*20,speed=[0.]*20,current=[0.]*20,status=[0x27]*20,
            session=7,command_sequence=12,session_selected=True,session_reason=1,
            hold_valid=True,hold_sample=1,hold_position=[0.]*16,reference_mode=2,
            segment_id=12,trajectory_phase=4,hold_basis=0,reference_valid=True,output_valid=True)
        self.feed();self.feed()
        self.io=S(pause_request=(7,12,1,1),pause_targets=None,stop_requested_at=10.,
                  stop_confirmation_window=.1,stop_failures=())
        self.engine=S(io=self.io,stopping=(S(task_id='task',command_id='motion'),'CANCELED'),stop_fault='',
            feedback=S(stamp=self.now,positions=dict.fromkeys(self.names,0.),
                       joint_velocities=dict.fromkeys(self.names,0.)),reference_names=self.names,
            profile=S(limits=dict.fromkeys(self.names,(-1.,1.)),
                      feedback_tolerances={'joint_position':.002,'joint_velocity':.01}),
            stop_deadline=15.,stop_steady_deadline=6.,steady=lambda:self.steady,stop_rest_window=None)

    def feed(self):
        self.now=round(self.now+.01,6);self.steady=round(self.steady+.01,6)
        header=S(stamp=S(sec=int(self.now),nanosec=round((self.now-int(self.now))*1e9)))
        for layer in ('upper','middle','servo'):
            self.health.state(layer,S(header=header,layer=layer,startup_complete=True,feedback_ready=True,
                instance_id=1,control_source='topic' if layer=='upper' else 'position',
                stop_policy='disable_no_load',wire_version=5,reference_mode=2))
        self.obs.header=header;self.obs.pdo_sample+=1
        self.health.observe(self.obs)

    def test_hold_rejections_use_the_same_predicates_as_acceptance(self):
        request=self.io.pause_request
        self.assertEqual(self.health.hold_failure(request),'')
        self.assertEqual(self.health.confirmed_hold(request),dict.fromkeys(self.names,0.))
        for field,value,reason in (('boot_id',9,'BOOT_MISMATCH'),
                ('session_selected',False,'SESSION_NOT_SELECTED'),('session',8,'SESSION_MISMATCH'),
                ('command_sequence',13,'PAUSE_SEQUENCE_MISMATCH'),('hold_valid',False,'HOLD_NOT_VALID'),
                ('hold_sample',2,'HOLD_SAMPLE_NOT_ADVANCED'),('reference_mode',0,'REFERENCE_MODE_MISMATCH'),
                ('segment_id',13,'SEGMENT_ID_MISMATCH'),('session_reason',2,'PAUSE_EXPIRED_OR_REVOKED'),
                ('trajectory_phase',3,'TRAJECTORY_NOT_PAUSED'),('hold_basis',1,'HOLD_BASIS_MISMATCH'),
                ('reference_valid',False,'REFERENCE_NOT_VALID'),('output_valid',False,'OUTPUT_NOT_VALID'),
                ('hold_position',[float('nan')]*16,'HOLD_POSITION_INVALID')):
            original=getattr(self.obs,field)
            with self.subTest(field=field):
                setattr(self.obs,field,value)
                self.assertEqual(self.health.hold_failure(request),reason)
                self.assertIsNone(self.health.confirmed_hold(request))
            setattr(self.obs,field,original)
        self.assertEqual(self.health.sample,2)
        self.assertEqual(self.health.hold_failure(None),'PAUSE_REQUEST_MISSING')

    def test_ack_gate_error_is_not_misreported_as_measured_position_error(self):
        self.engine.feedback.positions[self.names[0]]=.0452061978
        self.obs.command_sequence=13
        result=self.diagnose(self.health,self.engine,self.now)
        self.assertEqual(result['reason'],'PAUSE_SEQUENCE_MISMATCH')
        self.assertAlmostEqual(result['max_position_error_rad'],.0452061978)
        self.assertEqual(result['observed']['pdo_sample'],2)
        self.obs.command_sequence=12
        self.assertEqual(self.diagnose(self.health,self.engine,self.now)['reason'],'STOP_POSITION_ERROR')
        self.engine.feedback.positions[self.names[0]]=0.
        self.engine.feedback.joint_velocities[self.names[1]]=.02
        self.assertEqual(self.diagnose(self.health,self.engine,self.now)['reason'],'STOP_SPEED_NOT_ZERO')
        self.engine.feedback.joint_velocities[self.names[1]]=0.
        self.assertEqual(self.diagnose(self.health,self.engine,self.now)['reason'],'WAIT_REST_WINDOW')
        self.engine.feedback.stamp=9.
        self.assertEqual(self.diagnose(self.health,self.engine,self.now)['reason'],'STOP_FEEDBACK_NOT_FRESH')
        self.engine.stop_steady_deadline=self.steady
        self.assertEqual(self.diagnose(self.health,self.engine,self.now)['reason'],'STOP_BUDGET_EXPIRED')

    def test_pause_reports_tracking_error_without_treating_it_as_motion(self):
        self.io.stop_requires_arrival=False
        self.engine.feedback.positions[self.names[0]]=.038
        result=self.diagnose(self.health,self.engine,self.now)
        self.assertEqual(result['reason'],'WAIT_REST_WINDOW')
        self.assertFalse(result['reference_arrived'])
        self.assertFalse(result['arrival_required'])
        self.assertEqual(result['max_position_error_rad'],.038)

    def test_missing_request_fault_and_lost_progress_stay_visible_without_authorizing_a_stop(self):
        request=self.io.pause_request
        self.io.pause_request=None
        result=self.diagnose(self.health,self.engine,self.now)
        self.assertEqual(result['reason'],'PAUSE_REQUEST_MISSING')
        self.assertIsNone(result['sequence'])
        self.io.pause_request=request
        self.obs.status[0]=0x08
        self.assertEqual(self.diagnose(self.health,self.engine,self.now)['ack_reason'],'SNOWBOT_DRIVE_FAULT')
        self.obs.status[0]=0x27
        self.health.progress=self.steady-.041
        self.assertEqual(self.diagnose(self.health,self.engine,self.now)['ack_reason'],'SNOWBOT_PDO_NOT_ADVANCING')
        self.assertIsNone(self.io.pause_targets)
        self.assertTrue(self.engine.stopping)

    def test_adapter_emits_changed_reasons_immediately_and_repeated_checks_at_most_twice_per_second(self):
        class Base:pass
        spec=importlib.util.spec_from_file_location('bindu_runtime._offline_stop_adapter',
            package_share('snowbot_hardware')/'snowbot_hardware/adapter.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'rcl_interfaces':S(),'rcl_interfaces.msg':S(ParameterDescriptor=object),
                'bindu_runtime.execution_node':S(ExecutionNode=Base,run_execution=lambda _:None)}):
            spec.loader.exec_module(module)
        node=module.SnowbotAdapter.__new__(module.SnowbotAdapter)
        node.engine=self.engine;node.snowbot_devices=S(health=self.health)
        node.now=lambda:self.now;node.event=Mock()
        node.record_stop_check()
        for _ in range(51):
            self.feed();self.engine.feedback.stamp=self.now
            node.record_stop_check()
        self.assertEqual(node.event.call_count,2)
        self.obs.hold_valid=False
        node.record_stop_check()
        self.assertEqual(node.event.call_count,3)
        result=json.loads(node.event.call_args.args[1])
        self.assertEqual(result['ack_reason'],'HOLD_NOT_VALID')
        self.assertEqual((result['session'],result['sequence'],result['boot_id']),(7,12,1))
        self.assertEqual(self.io.pause_targets,None)
        self.assertEqual(self.engine.stop_deadline,15.)
        self.assertEqual(self.engine.stopping[1],'CANCELED')
        self.engine.stopping=None;self.engine.stop_fault='STOP_UNCONFIRMED:vendor_hold'
        node.record_stop_check()
        self.assertEqual(json.loads(node.event.call_args.args[1])['reason'],self.engine.stop_fault)
        node.record_stop_check()
        self.assertEqual(node.event.call_count,4)
        node.get_logger=Mock(return_value=Mock())
        node._stop_check_signature=None
        self.health.messages['servo']['operation_enabled_mask']=object()
        node.record_stop_check()
        node.get_logger().warn.assert_called_once()
        self.assertEqual(node.event.call_count,4)
        self.assertEqual(self.engine.stop_fault,'STOP_UNCONFIRMED:vendor_hold')
        self.assertIsNone(node._stop_check_signature)
        self.health.messages['servo']['operation_enabled_mask']=0xc7f7f
        node.record_stop_check()
        self.assertEqual(node.event.call_count,5)


class SnowbotRosBindingTests(unittest.TestCase):
    def test_stop_diagnostic_serializes_actual_ros_numeric_array_comparisons(self):
        from snowbot_ctl_msgs.msg import PositionObservation
        case=SnowbotStopDiagnosticTests();case.setUp()
        # Fixed ROS float64 arrays produce NumPy scalars; list-only fixtures
        # miss the numpy.bool result that previously killed record_stop_check.
        case.obs.hold_position=PositionObservation().hold_position
        for offset in (0., .038):
            case.engine.feedback.positions[case.names[0]]=offset
            result=case.diagnose(case.health,case.engine,case.now)
            encoded=json.loads(json.dumps(result,allow_nan=False))
            self.assertIs(type(result['reference_arrived']),bool)
            self.assertEqual(encoded['reference_arrived'],offset==0.)
            self.assertEqual(encoded['max_position_error_rad'],offset)

    def test_existing_adapter_tick_advances_session_before_engine_including_shutdown(self):
        # Load the production adapter with only its ROS base replaced; no node
        # or generated ROS messages are needed to verify the existing tick order.
        calls = []
        class Base:
            def tick(self): calls.append('engine')
        spec = importlib.util.spec_from_file_location('bindu_runtime._offline_tick_adapter',
            package_share('snowbot_hardware')/'snowbot_hardware/adapter.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'rcl_interfaces': S(), 'rcl_interfaces.msg': S(ParameterDescriptor=object),
                'bindu_runtime.execution_node': S(ExecutionNode=Base, run_execution=lambda _:None)}):
            spec.loader.exec_module(module)
        node = module.SnowbotAdapter.__new__(module.SnowbotAdapter)
        node.engine = object()
        node.snowbot_devices = S(advance_session=lambda: calls.append('advance'),
            advance_startup_hold=lambda engine: calls.append('startup'),
            brake_before_source_expiry=lambda engine: calls.append('source_expiry'))
        node.grippers = S(core=True, tick=lambda: calls.append('gripper'))
        for closing in (False, True):
            node.closing = closing
            calls.clear(); node.tick()
            self.assertEqual(calls, ['advance', *([] if closing else ['startup','source_expiry']), 'engine', 'gripper'])
        node.snowbot_devices = S()  # legacy_guard retains its original tick.
        calls.clear(); node.tick()
        self.assertEqual(calls, ['engine', 'gripper'])

    def test_session_capture_changes_generation_and_pause_does_not_renew_motion(self):
        from snowbot_hardware.position_session import PositionSessionDevices, PositionSessionIO
        from snowbot_hardware.session import SourceEnvelope
        from snowbot_hardware.session import CONTROL_GROUPS
        profile=Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_vendor_position.json')
        devices=PositionSessionDevices.__new__(PositionSessionDevices)
        devices.node=S(now=lambda:10.)
        devices.health=S(reference_mode=0)
        devices.envelopes=SourceEnvelope(1,devices.node.now)
        io=PositionSessionIO(profile,Mock(),devices.node.now,command_enabled=True)
        io.after_capture=devices.new_session
        devices.io=io
        q=dict.fromkeys(profile.limits,0.)
        q['arm_l_joint_2'],q['arm_r_joint_2']=.2,-.2
        io.observe(q,dict.fromkeys(q,0.),10.,10.)
        io.capture(CONTROL_GROUPS)
        self.assertNotEqual(devices.envelopes.session,1)
        self.assertTrue(devices.envelopes.takeover)
        io.write_joints({n:q[n] for n in GROUPS['left_arm']})
        flags=[]
        io.check_health=lambda groups:flags.append(io.requesting_pause)
        io.stop()
        self.assertTrue(flags)
        self.assertTrue(all(flags))
        self.assertFalse(io.requesting_pause)

    """Use installed message classes with mocked publishers; never create a node."""
    def test_native_gripper_send_preserves_prefix_and_guards_only_selected_hand(self):
        from snowbot_hardware.gripper_node import SnowbotGrippers
        from snowbot_hardware.adapter import SnowbotDevices
        binding = SnowbotGrippers.__new__(SnowbotGrippers)
        binding.health = Mock()
        binding.session_protocol = False
        devices = SnowbotDevices.__new__(SnowbotDevices)
        devices.node = Mock()
        devices.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
        devices.session, devices.command_sequence = 123, 0
        devices.array_type = lambda data:S(data=data)
        binding.devices = devices
        binding.core = S(cfg=json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_grippers_real.json').read_text()))
        binding.publishers = {side:Mock() for side in ('left', 'right')}
        binding.send('right', .02, False, 999)
        self.assertEqual(binding.publishers['right'].publish.call_args.args[0].data,
                         [.02, 0., 0., 100., 30., 10., 1., 0., 123., 0.])
        binding.publishers['left'].publish.assert_not_called()
        binding.send('right', .01, True, 1000)
        self.assertEqual(binding.publishers['right'].publish.call_args.args[0].data,
                         [.01, 0., 0., 100., 30., 10., 2., 0., 123., 1.])

    def measured_devices(self):
        from snowbot_hardware.adapter import SnowbotDevices
        from snowbot_hardware.gripper_node import SnowbotGrippers
        from snowbot_hardware.gripper import GripperController
        profile = Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_real_position.json')
        devices = SnowbotDevices.__new__(SnowbotDevices)
        devices.node=Mock();devices.node.now.return_value=10.
        devices.feedback_topic='/dualArmCtl_fdback';devices.health_receivers={'servo':'/servo_node'}
        devices.node.get_publishers_info_by_topic.return_value=[S(node_namespace='/',node_name='servo_node')]
        devices.io=SnowbotIO(profile, Mock(), lambda:10.)
        devices.grippers=SnowbotGrippers.__new__(SnowbotGrippers)
        cfg=json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_grippers_real.json').read_text())
        devices.grippers.core=GripperController(profile,cfg,Mock(),Mock(),lambda:10.,lambda:True,
                                               require_driver_ack=False)
        native=S(header=S(stamp=S(sec=10,nanosec=0)),
            l_position=[0.]*8,r_position=[0.]*8,l_speed=[0.]*8,r_speed=[0.]*8,
            l_iq=[0.]*7+[.11],r_iq=[0.]*7+[.22])
        devices.receive(native)
        return devices,profile

    def test_native_feedback_populates_gripper_measurements_without_head_or_waist(self):
        devices,_=self.measured_devices()
        core=devices.grippers.core
        self.assertEqual(core.current, {'left':.11,'right':.22})
        self.assertTrue(core.snapshot('left')['feedback_valid'])
        self.assertEqual(len(devices.io.last_feedback.positions),16)
        self.assertEqual(devices.io.missing_feedback_groups,['waist','head'])

    def test_execution_state_explicitly_reports_sparse_measured_feedback(self):
        from bindu_runtime.execution_node import ExecutionNode
        devices,profile=self.measured_devices()
        node=S(now=lambda:10.,engine=DelegatedPositionExecutor(profile,devices.io),
               robot_io=devices.io,profile=profile,last_event=None,event=Mock(),state_pub=Mock(),
               instance_id='fixture',runtime_mode='real',device_backend='snowbot_vendor')
        ExecutionNode.tick(node)
        state=node.state_pub.publish.call_args.args[0]
        self.assertEqual(len(state.joints.name),16)
        self.assertEqual(state.missing_feedback_groups,['waist','head'])
        self.assertEqual(state.code,'FEEDBACK_INCOMPLETE:waist,head')
        self.assertNotIn('body_joint_1',state.joints.name)


if __name__ == '__main__': unittest.main()
