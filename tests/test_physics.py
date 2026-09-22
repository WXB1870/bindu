"""Device-boundary safety semantics; no Isaac dependency for these tests."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from dataclasses import replace
import tempfile
from bindu_contracts.contracts import Motion
from bindu_contracts.devices import JointReading, BaseReading
from bindu_contracts.profile import Profile
from bindu_execution.executor import Executor
from bindu_hardware.drivers.physics import PhysicsCommandGate
from bindu_hardware.robot_io.system import RobotIO

ROOT = Path(__file__).resolve().parents[1]


class PhysicsGateTests(unittest.TestCase):
    def setUp(self):
        self.profile = Profile.load(ROOT/'src/integration/bindu_runtime/config/g1_provisional_sim.json')
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
        raw = json.loads((ROOT/'src/integration/bindu_runtime/config/g1_provisional_sim.json').read_text())
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'profile.json'
            for bad in ({'joint_position':0}, {'joint_velocity':float('nan')}, {'unknown':1}, {'joint_velocity':True}):
                raw['execution']['feedback_tolerances'] = bad
                path.write_text(json.dumps(raw))
                with self.assertRaisesRegex(ValueError, 'feedback tolerances'):Profile.load(path)


class MeasuredCompletionTests(unittest.TestCase):
    """Command/reference arrival cannot substitute for measured rest."""
    def setUp(self):
        self.q, self.dq, self.v = 0., .1, .04
        self.profile = Profile('physical', {'arm':['j']}, {'j':[-1.,1.]}, 1.,
            ('joint_position','base_velocity'), 'hash', feedback_tolerances={
                'joint_position':.002, 'joint_velocity':.01,
                'base_linear_velocity':.003, 'base_angular_velocity':.005})
        joints = S(read=lambda now, dt:JointReading(now, {'j':self.q}, True, {'j':self.dq}),
            write_positions=lambda q:None, request_stop=lambda:None)
        base = S(read=lambda now, dt:BaseReading(now, 0., 0., (self.v,0.), True),
            write_velocity=lambda v:None, request_stop=lambda:None)
        self.executor = Executor(self.profile, RobotIO(self.profile, {'arm':joints}, base))
        lease, epoch = self.executor.acquire('test', ['arm','base'], 5., 10.)
        self.motion = Motion('test', lease, epoch, 'hash', 'test', '', 'finite_trajectory',
            'arm', 10., 4., names=('j',), points=((.1,),), offsets=(1.,))

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


if __name__ == '__main__':unittest.main()
