"""Device-boundary safety semantics; no Isaac dependency for these tests."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from dataclasses import replace
import tempfile
from unittest.mock import patch
from bindu_contracts.contracts import Motion
from bindu_contracts.devices import JointReading, BaseReading
from bindu_contracts.profile import Profile
from bindu_execution.executor import Executor, Rejected
from bindu_hardware.drivers.physics import PhysicsCommandGate
from bindu_hardware.robot_io.system import RobotIO

ROOT = Path(__file__).resolve().parents[1]


class PhysicsGateTests(unittest.TestCase):
    def setUp(self):
        self.profile = Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/g1/g1_provisional_sim.json')
        self.gate = PhysicsCommandGate(self.profile, 'sim-boot')

    def command(self, **changes):
        cmd = S(stamp=S(sec=10, nanosec=0), simulator_id='sim-boot', sender_id='executor',
                sequence=1, profile_hash=self.profile.digest, valid_for=.25, operation='set',
                resource_group='left_arm', joint_names=self.profile.groups['left_arm'], positions=[0.]*7,
                velocity=S(linear=S(x=0., y=0., z=0.), angular=S(x=0., y=0., z=0.)))
        for key, value in changes.items():setattr(cmd, key, value)
        return cmd

    def test_expiration_uses_source_stamp_and_is_once(self):
        self.gate.accept(self.command(), 10.1)
        self.assertEqual(self.gate.expired(10.249), [])
        self.assertEqual(self.gate.expired(10.25), ['left_arm'])
        self.assertEqual(self.gate.expired(11.), [])

    def test_wrong_boot_profile_and_replay_rejected(self):
        for changes in ({'simulator_id':'old'}, {'profile_hash':'wrong'}, {'sequence':0}):
            with self.assertRaises(ValueError):self.gate.accept(self.command(**changes), 10.)
        self.gate.accept(self.command(), 10.)
        with self.assertRaisesRegex(ValueError, 'REPLAY'):self.gate.accept(self.command(), 10.01)
        with self.assertRaisesRegex(ValueError, 'EXECUTOR_CHANGED'):
            self.gate.accept(self.command(sender_id='new', sequence=2), 10.01)

    def test_expired_future_and_bad_ttl_rejected(self):
        for now, ttl in ((10.25,.25), (9.,.25), (10.,float('nan')), (10.,0.), (10.,1.)):
            with self.assertRaises(ValueError):self.gate.accept(self.command(valid_for=ttl), now)
        self.assertEqual(self.gate.sender, '')

    def test_joint_layout_limit_and_nonfinite_rejected(self):
        for changes in ({'joint_names':list(reversed(self.profile.groups['left_arm']))},
                        {'positions':[0.]}, {'positions':[100.]*7}, {'positions':[float('nan')]*7}):
            with self.assertRaises(ValueError):self.gate.accept(self.command(**changes), 10.)
        self.assertEqual(self.gate.sequence, 0)

    def test_base_limit_and_cross_resource_rejected(self):
        cmd=self.command(resource_group='base', joint_names=[], positions=[])
        cmd.velocity.linear.x=.2
        self.assertEqual(self.gate.accept(cmd, 10.), 'base')
        bad=copy.deepcopy(cmd);bad.sequence=2;bad.velocity.linear.x=1.
        with self.assertRaises(ValueError):self.gate.accept(bad, 10.01)
        bad=self.command(sequence=2);bad.velocity.angular.z=.1
        with self.assertRaises(ValueError):self.gate.accept(bad, 10.01)

    def test_hold_has_no_position_payload(self):
        with self.assertRaises(ValueError):self.gate.accept(self.command(operation='hold'), 10.)
        self.assertEqual(self.gate.accept(self.command(operation='hold',joint_names=[],positions=[]),10.),'left_arm')

    def test_old_profile_preserves_strict_tolerances(self):
        self.assertEqual(self.profile.feedback_tolerances,{})

    def test_hold_rejects_base_motion(self):
        cmd = self.command(resource_group='base', operation='hold', joint_names=[], positions=[])
        cmd.velocity.linear.x = .1
        with self.assertRaisesRegex(ValueError, 'HOLD_HAS_TARGET'):self.gate.accept(cmd, 10.)

    def test_invalid_feedback_tolerance_rejected(self):
        raw = json.loads((ROOT/'src/integration/bindu_runtime/config/robots/g1/g1_provisional_sim.json').read_text())
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'profile.json'
            for bad in ({'joint_position':0}, {'joint_velocity':float('nan')}, {'unknown':1}, {'joint_velocity':True}):
                raw['execution']['feedback_tolerances'] = bad
                path.write_text(json.dumps(raw))
                with self.assertRaisesRegex(ValueError, 'feedback tolerances'):Profile.load(path)


class MeasuredCompletionTests(unittest.TestCase):
    """Command/reference arrival cannot substitute for measured rest."""
    def setUp(self):
        self.q, self.dq, self.v = 0., 0., 0.
        self.sample_stamp = None
        self.velocity_known = True
        self.profile = Profile('physical', {'arm':['j']}, {'j':[-1.,1.]}, 1.,
            ('joint_position','base_velocity'), 'hash', feedback_tolerances={
                'joint_position':.002, 'joint_velocity':.01,
                'base_linear_velocity':.003, 'base_angular_velocity':.005})
        self.joints = joints = S(read=lambda now, dt:JointReading(
            now if self.sample_stamp is None else self.sample_stamp, {'j':self.q}, True,
            {'j':self.dq} if self.velocity_known else {}),
            write_positions=lambda q:None, request_stop=lambda:None)
        self.base = base = S(read=lambda now, dt:BaseReading(
            now if self.sample_stamp is None else self.sample_stamp, 0., 0., (self.v,0.), True),
            write_velocity=lambda v:None, request_stop=lambda:None)
        self.executor = Executor(self.profile, RobotIO(self.profile, {'arm':joints}, base))
        lease, epoch = self.executor.acquire('test', ['arm','base'], 5., 10.)
        self.motion = Motion('test', lease, epoch, 'hash', 'test', '', 'finite_trajectory',
            'arm', 10., 4., names=('j',), points=((.1,),), offsets=(1.,))
        self.dq,self.v = .1,.04

    def advance(self, end):
        while self.executor.last_tick < end-1e-8:
            self.executor.tick(min(end,self.executor.last_tick+.01))

    def test_joint_goal_waits_for_measured_position_and_velocity(self):
        self.executor.submit(self.motion, 10.)
        self.q = .1
        self.advance(11.1)
        self.assertIsNotNone(self.executor.active)
        self.dq, self.q = 0., .11
        self.advance(11.2)
        self.assertIsNotNone(self.executor.active)
        self.q = .1005
        self.advance(11.3)
        self.assertIsNone(self.executor.active)
        self.assertEqual(self.executor.results['test'].state,'SUCCEEDED')

    def test_base_zero_reference_waits_for_measured_speed(self):
        cmd = replace(self.motion, mode='base_velocity', group='base', names=(), points=(), offsets=(),
                      velocity=(.1,0.), duration=.2)
        self.executor.submit(cmd,10.)
        self.advance(10.5)
        self.assertEqual(self.executor.base_reference,(0.,0.))
        self.assertIsNotNone(self.executor.active)
        self.v = .002
        self.advance(10.6)
        self.assertIsNone(self.executor.active)

    def base_motion(self):
        return replace(self.motion, mode='base_velocity', group='base', names=(), points=(), offsets=(),
                       velocity=(.1,0.), duration=1.)

    def test_cancel_waits_for_base_and_joint_rest_before_new_lease(self):
        self.executor.submit(self.base_motion(),10.)
        self.executor.halt(10.,revoke=True)
        self.assertIsNotNone(self.executor.stopping)
        self.assertEqual(self.executor.results['test'].state,'STOPPING')
        with self.assertRaisesRegex(Rejected,'BUSY'):
            self.executor.acquire('new',['base'],2.,10.01)
        self.v = 0.  # Other stopped endpoints still need confirmation.
        self.executor.tick(10.02)
        self.assertIsNotNone(self.executor.stopping)
        self.dq = 0.
        self.executor.tick(10.03)
        self.assertEqual(self.executor.results['test'].state,'CANCELED')
        self.executor.acquire('new',['base'],2.,10.04)

    def test_stop_cannot_use_a_sample_from_before_request(self):
        self.dq,self.v = 0.,0.
        self.executor.submit(self.base_motion(),10.)
        self.executor.halt(10.,revoke=True)
        self.sample_stamp=10.
        self.executor.tick(10.01)
        self.assertIsNotNone(self.executor.stopping)
        self.sample_stamp=10.02
        self.executor.tick(10.02)
        self.assertIsNone(self.executor.stopping)

    def test_unknown_velocity_cannot_complete_finite_goal_or_stop(self):
        self.executor.submit(self.motion,10.)
        self.velocity_known=False
        self.q=.1
        self.advance(11.1)
        self.assertIsNone(self.executor.active)
        self.assertIsNotNone(self.executor.stopping)
        self.assertEqual(self.executor.results['test'].code,'FEEDBACK_VELOCITY_UNAVAILABLE')
        self.executor.halt(11.1,revoke=True,emergency=True)
        self.v=0.
        self.executor.tick(11.11)
        self.assertIsNotNone(self.executor.stopping)
        self.velocity_known=True;self.dq=0.
        self.executor.tick(11.12)
        self.assertIsNone(self.executor.stopping)

    def test_stop_failure_latches_even_after_a_successful_retry(self):
        with patch.object(self.joints,'request_stop',side_effect=RuntimeError('offline')):
            self.executor.halt(10.,revoke=True,emergency=True)
        self.executor.io.stop()
        self.dq,self.v = 0.,0.
        with self.assertRaisesRegex(Rejected,'STOP_UNCONFIRMED'):
            self.executor.acquire('new',['base'],2.,10.01)

    def test_repeated_faults_do_not_resend_or_extend_the_stop_deadline(self):
        self.executor.profile=replace(self.profile,stop_timeout=.3)
        self.executor.submit(self.base_motion(),10.)
        with patch.object(self.executor.io,'stop',wraps=self.executor.io.stop) as stop:
            self.executor.halt(10.,revoke=True,emergency=True)
            deadline=self.executor.stop_deadline
            epoch=self.executor.epoch
            for now in (10.15,10.3,10.45):self.executor.tick(now)
            self.assertEqual(self.executor.stop_deadline,deadline)
            self.assertEqual(self.executor.epoch,epoch)
            stop.assert_called_once()
        self.assertIsNone(self.executor.stopping)
        self.assertEqual(self.executor.results['test'].state,'FAILED')
        with self.assertRaisesRegex(Rejected,'STOP_UNCONFIRMED'):
            self.executor.acquire('new',['base'],2.,10.46)

    def test_monotonic_stop_budget_expires_with_frozen_ros_time(self):
        clock=[0.]
        self.executor.steady=lambda:clock[0]
        self.executor.profile=replace(self.profile,stop_timeout=.3)
        self.executor.submit(self.base_motion(),10.)
        self.executor.halt(10.,revoke=True,emergency=True)
        clock[0]=.31
        self.executor.tick(10.)
        self.assertIsNone(self.executor.stopping)
        self.assertEqual(self.executor.results['test'].code,'STOP_FEEDBACK_TIMEOUT')

    def test_idle_acquisition_requires_fresh_feedback(self):
        self.executor.halt(10.,revoke=True,emergency=True)
        self.dq,self.v=0.,0.
        self.executor.tick(10.01)
        self.sample_stamp=10.01
        with self.assertRaisesRegex(Rejected,'FEEDBACK_STALE'):
            self.executor.acquire('new',['base'],2.,10.3)

    def test_finite_goal_requires_feedback_at_or_after_the_end(self):
        self.executor.submit(self.motion,10.)
        self.q,self.dq= .1,0.
        self.advance(10.99)
        self.sample_stamp=10.99
        self.advance(11.1)
        self.assertIsNotNone(self.executor.active)
        self.sample_stamp=11.11
        self.executor.tick(11.11)
        self.assertEqual(self.executor.results['test'].state,'SUCCEEDED')

    def test_new_owner_requires_stationary_measured_feedback(self):
        self.executor.halt(10.,revoke=True,emergency=True)
        self.dq,self.v=0.,0.
        self.executor.tick(10.01)
        self.v=.02
        with self.assertRaisesRegex(Rejected,'FEEDBACK_NOT_STATIONARY'):
            self.executor.acquire('new',['base'],2.,10.02)


if __name__ == '__main__':unittest.main()
