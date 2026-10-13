"""Vendor boundary and delegated stop semantics; never touches hardware."""
import copy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as S
import unittest

from bindu_contracts.contracts import Motion
from bindu_contracts.profile import Profile
from bindu_execution.delegated_position import DelegatedPositionExecutor
from bindu_execution.executor import Rejected
from bindu_hardware.snowbot import GROUPS, SnowbotIO, decode_feedback

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
                              positions=(.3, .2, 0., 0., 0., 0., 0.)), **changes)

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
        self.assertEqual(len(calls), 2)
        self.assertEqual(set(calls[-1][1]), set(target))
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


if __name__ == '__main__': unittest.main()
