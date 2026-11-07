"""Position interface and delegated stop semantics; never touches hardware."""
from snowbot_hardware.parameters import load_mode_config
from bindu_contracts.resources import package_share
from tests.support.paths import ROOT
import copy
import importlib.util
import json
import math
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
from snowbot_hardware.protocol import GROUPS, SnowbotIO

CONFIG = package_share('snowbot_bringup')/'config'


class SnowbotAdapterTests(unittest.TestCase):
    def prepared(self, *, held=False, mode='position'):
        self.engine.steady = lambda: self.now
        self.io.supports_prepared_replay = True
        self.io.source_motion = None
        self.io.execution_mode = mode
        self.io.held_reference = lambda: ('instance', 1, 2, 3, ())
        from snowbot_hardware.session import SourceEnvelope
        self.io.validate_motion = SourceEnvelope.validate_motion
        self.io.prepare_replay = lambda prepared: None
        self.io.check_health = lambda *a, **k: None
        self.io.check_ready = lambda *a, **k: None
        self.io.stop_confirmation_max_gap = .04
        self.io.stop_requires_arrival = False
        self.io.stop_confirmation_window = .1
        self.io.replay_timing = dict(period_s=.02, command_ttl_s=.25, renewal_period_s=.5,
            start_lead_s=.12, completion_timeout_s=6., settle_time_s=.1, velocity_tolerance_rad_s=.01)
        self.io.replay_position_tolerances = dict.fromkeys(self.q, math.radians(2))
        names = tuple(n for group in ('left_arm', 'right_arm') for n in GROUPS[group])
        first = tuple(self.q[n] for n in names)
        final = (first[0]+.01, *first[1:])
        zero = (0.,)*len(names)
        return Motion('curve', '', 0, self.profile.digest, 'test', 'instance' if held else '',
            'prepared_trajectory'+('_impedance' if mode == 'impedance' else ''), '', self.now, .25, names=names, offsets=(0., 1.),
            points=(first, final), velocities=(zero,zero), accelerations=(zero,zero),
            expected_revision=self.engine.revision, schema_version=2,
            resource_groups=('left_arm','right_arm'), runtime_mode='real')

    def take_replay(self, motion):
        with self.assertRaisesRegex(Rejected, 'WAIT_REST'):
            self.engine.acquire_replay('test', motion.resource_groups, self.now)
        for _ in range(6):
            self.now += .02
            self.observe()
            try:
                return self.engine.acquire_replay('test', motion.resource_groups, self.now)
            except Rejected as exc:
                self.assertTrue(str(exc).startswith('WAIT_REST'))
        self.fail('fresh stationary samples did not authorize takeover')

    def beat(self, motion, lease, epoch, identity='beat'):
        return replace(motion, command_id=identity, observation_id=motion.command_id,
            lease_id=lease, epoch=epoch, stamp=self.now, mode='trajectory_heartbeat',
            expected_revision=0, points=(), offsets=(), velocities=(), accelerations=())

    def test_prepared_replay_heartbeats_keep_one_validated_curve_and_source_deadlines(self):
        motion = self.prepared()
        self.engine.submit(motion, self.now)
        self.assertFalse(self.engine.lease_id or self.sent)
        prepared = self.engine.replay
        from bindu_execution.interpolation import PreparedSegment
        curve = PreparedSegment.from_description(self.profile, motion.names, prepared.description)
        self.assertAlmostEqual(curve.sample(.25).q[0]-motion.points[0][0], .01*.103515625)
        self.assertAlmostEqual(curve.sample(.25).v[0], .01*1.0546875)
        lease, epoch = self.take_replay(motion)
        heartbeat = self.beat(motion, lease, epoch)
        with patch('bindu_execution.delegated_position.PreparedReplay.create', side_effect=AssertionError('refit')):
            self.engine.submit(heartbeat, self.now)
        bound = self.io.prepared_replay
        self.assertEqual(bound.description, prepared.description)
        self.assertAlmostEqual(bound.effective, heartbeat.stamp+.12)
        self.assertEqual(self.io.source_motion.mode, 'prepared_trajectory')
        self.now += .02; self.observe()
        next_beat = self.beat(motion, lease, epoch, 'next')
        self.engine.submit(next_beat, self.now)
        self.assertIs(self.io.prepared_replay, bound)
        self.assertIs(self.io.source_motion, next_beat)
        with self.assertRaisesRegex(Rejected, 'VENDOR_TARGET_TIME'):
            self.engine.submit(replace(next_beat, command_id='long', valid_for=.4), self.now)
        with self.assertRaisesRegex(Rejected, 'IDENTITY'):
            self.engine.submit(replace(next_beat, command_id='bad', observation_id='other'), self.now)
        with self.assertRaisesRegex(Rejected, 'TIME|EXPIRED|STALE|INVALID_LEASE'):
            self.engine.submit(replace(next_beat, command_id='late'), self.now+.25)
        for _ in range(14):
            self.now += .02; self.observe(); self.engine.tick(self.now)
        self.assertFalse(self.engine.lease_id)

    def test_replay_takeover_waits_through_second_swing_and_repeated_feedback(self):
        motion = self.prepared(); self.engine.submit(motion, self.now)
        self.engine.io.replay_timing['completion_timeout_s'] = .3
        began = self.now
        for i in range(16):
            self.now = began+(i+1)*.02
            self.v['arm_l_joint_1'] = .02 if .08 <= self.now-began < .24 else 0.
            self.observe()
            with self.assertRaisesRegex(Rejected, 'WAIT_REST|REPLAY_NOT_SETTLED'):
                self.engine.acquire_replay('test', motion.resource_groups, self.now)
        self.assertFalse(self.engine.lease_id or self.sent)
        self.assertAlmostEqual(self.engine.replay_wait[0], began+.02)
        # A repeated request never buys another six seconds.
        with self.assertRaisesRegex(Rejected, 'REPLAY_NOT_SETTLED'):
            self.engine.acquire_replay('test', motion.resource_groups, self.now+.01)

    def test_replay_reference_is_authenticated_before_lease_and_rechecked_at_takeover(self):
        motion = self.prepared(held=True)
        held = ('instance', 1, 2, 3, (('arm_l_joint_1', 0.),))
        self.io.held_reference = lambda: held
        with self.assertRaisesRegex(Rejected, 'REFERENCE_INSTANCE_CHANGED'):
            self.engine.submit(replace(motion, observation_id='previous-executor'), self.now)
        self.engine.submit(motion, self.now)
        held = ('instance', 2, 2, 3, (('arm_l_joint_1', 0.),))
        with self.assertRaisesRegex(Rejected, 'HELD_REFERENCE_CHANGED'):
            self.engine.acquire_replay('test', motion.resource_groups, self.now)
        self.assertFalse(self.engine.lease_id or self.sent)

    def test_worker_certified_replay_commits_with_fresh_feedback_without_refitting(self):
        from bindu_execution.delegated_position import PreparedReplay
        motion = self.prepared(held=True)
        prepared = PreparedReplay.create(motion, self.profile)
        self.now += .055
        self.observe()
        self.io.held_reference = Mock(return_value=('instance', 1, 2, 3, ()))
        with patch.object(PreparedReplay, 'create', side_effect=AssertionError('refit')):
            self.engine.prepare_replay(motion, self.now, prepared=prepared)
        self.io.held_reference.assert_called_once()
        self.assertEqual(self.engine.replay.motion, motion)
        self.assertFalse(self.engine.lease_id or self.sent)

    def test_worker_certified_replay_rechecks_revision_and_expiry(self):
        from bindu_execution.delegated_position import PreparedReplay
        motion = self.prepared(held=True)
        prepared = PreparedReplay.create(motion, self.profile)
        self.engine.revision += 1
        with self.assertRaisesRegex(Rejected, 'STALE_REVISION'):
            self.engine.prepare_replay(motion, self.now, prepared=prepared)
        self.engine.revision = motion.expected_revision
        self.now += .251
        self.observe()
        with self.assertRaisesRegex(RuntimeError, 'SOURCE_COMMAND_EXPIRED'):
            self.engine.prepare_replay(motion, self.now, prepared=prepared)
        self.assertFalse(self.engine.lease_id or self.sent or self.engine.replay)

    def test_worker_certified_replay_does_not_bypass_stale_held_feedback(self):
        from bindu_execution.delegated_position import PreparedReplay
        motion = self.prepared(held=True)
        prepared = PreparedReplay.create(motion, self.profile)
        self.io.held_reference = Mock(side_effect=RuntimeError('SNOWBOT_PDO_NOT_ADVANCING'))
        with self.assertRaisesRegex(RuntimeError, 'PDO_NOT_ADVANCING'):
            self.engine.prepare_replay(motion, self.now, prepared=prepared)
        self.assertFalse(self.engine.lease_id or self.sent or self.engine.replay)

    def test_replay_frozen_sample_cannot_complete_takeover_window(self):
        motion = self.prepared(); self.engine.submit(motion, self.now)
        self.io.read_feedback = lambda now, dt: self.io.last_feedback
        for _ in range(20):
            self.now += .02
            with self.assertRaisesRegex(Rejected, 'WAIT_REST'):
                self.engine.acquire_replay('test', motion.resource_groups, self.now)
        self.assertFalse(self.engine.lease_id or self.sent)

    def test_replay_wrong_backend_mode_is_rejected_before_lease(self):
        motion = self.prepared()
        with self.assertRaisesRegex(Rejected, 'EXECUTION_MODE_MISMATCH'):
            self.engine.submit(replace(motion, mode='prepared_trajectory_impedance'), self.now)
        self.assertFalse(self.engine.lease_id or self.sent)

    def test_missing_nominal_reference_expires_arrival_without_claiming_completion(self):
        motion = self.prepared(held=True, mode='impedance')
        self.io.replay_timing['completion_timeout_s'] = .14
        self.io.sampled_reference = lambda: (self.now, {}, {}, {})
        self.engine.submit(motion, self.now)
        lease, epoch = self.take_replay(motion)
        self.engine.submit(self.beat(motion, lease, epoch), self.now)
        end = self.engine.replay.effective+motion.offsets[-1]
        step = 0
        while not self.engine.replay_result['final'] and self.now < end+.5:
            step += 1; self.now += .02; self.observe()
            if self.engine.stopping:
                self.engine.tick(self.now)
            else:
                try:
                    self.engine.submit(self.beat(motion, lease, epoch, 'next'+str(step)), self.now)
                except Rejected as exc:
                    self.assertEqual(str(exc), 'INVALID_LEASE')
                    self.assertEqual(self.engine.stopping[1], 'REPLAY_ARRIVAL_UNCONFIRMED')
        result = self.engine.replay_result
        self.assertEqual(result['error'], 'REPLAY_ARRIVAL_UNCONFIRMED')
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['completed'] or result['nominal_arrival_confirmed'])

    def test_replay_cancel_before_first_heartbeat_is_final_without_device_command(self):
        motion = self.prepared(); self.engine.submit(motion, self.now)
        lease, epoch = self.take_replay(motion)
        result = self.engine.replay_status(lease, epoch, self.now, stop=True)
        self.assertTrue(result['final'] and result['stop_confirmed'])
        self.assertFalse(result['completed'] or self.sent or self.engine.lease_id)

    def test_replay_service_credentials_allow_one_stop_and_final_query_after_revocation(self):
        import ast
        from bindu_contracts.runtime_mode import command_mode
        motion = self.prepared()
        path = ROOT/'src/integration/bindu_runtime/bindu_runtime/execution_node.py'
        cls = next(n for n in ast.parse(path.read_text()).body if getattr(n, 'name', '') == 'ExecutionNode')
        methods = [n for n in cls.body if getattr(n, 'name', '') in ('decode_motion', 'accept', 'lease', 'control')]
        seconds = lambda stamp: stamp.sec+stamp.nanosec*1e-9
        stamp = lambda t: S(sec=int(t*1e9)//1_000_000_000, nanosec=int(t*1e9)%1_000_000_000)
        ns = dict(Motion=Motion, Rejected=Rejected, seconds=seconds, stamp=stamp,
                  command_mode=command_mode, json=json)
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), ns)
        node = S(engine=self.engine, robot_io=self.io, profile=self.profile, closing=False,
                 runtime_mode='real', now=lambda: self.now, accepted_pub=Mock())
        node.accept = lambda msg: ns['accept'](node, msg)
        node.decode_motion = lambda msg: ns['decode_motion'](node, msg)
        def message(m):
            return S(**dict(command_id=m.command_id, task_id=m.task_id, observation_id=m.observation_id,
                runtime_mode=m.runtime_mode, schema_version=m.schema_version, mode=m.mode,
                lease_id=m.lease_id, epoch=m.epoch, profile_hash=m.profile_hash, stamp=stamp(m.stamp),
                valid_for=m.valid_for, duration=m.duration, joint_names=m.names, positions=m.positions,
                resource_group=m.group, resource_groups=m.resource_groups, rigid_reference=m.rigid_reference,
                expected_revision=m.expected_revision,
                velocity=S(linear=S(x=0., y=0., z=0.), angular=S(x=0., y=0., z=0.)),
                points=[S(positions=q, velocities=v, accelerations=a, effort=[], time_from_start=stamp(t))
                        for t,q,v,a in zip(m.offsets, m.points, m.velocities, m.accelerations)]))
        node.accept(message(motion))
        ready = ns['lease'](node, S(operation='ready', resources=motion.resource_groups), S())
        self.assertTrue(ready.ok)
        self.assertEqual(json.loads(ready.code)['period_s'], .02)
        request = S(operation='acquire_replay', owner='test', resources=motion.resource_groups)
        reply = ns['lease'](node, request, S())
        self.assertFalse(reply.ok)
        self.assertTrue(reply.code.startswith('WAIT_REST'))
        for _ in range(6):
            self.now += .02; self.observe(); reply = ns['lease'](node, request, S())
            if reply.ok: break
        self.assertTrue(reply.ok)
        lease, epoch = reply.lease_id, reply.epoch
        self.assertEqual(ns['lease'](node, S(operation='ready', resources=motion.resource_groups), S()).code, 'BUSY')
        node.accept(message(self.beat(motion, lease, epoch)))
        self.engine.tick(self.now)
        request = S(operation='replay_stop', lease_id=lease, epoch=epoch)
        reply = ns['control'](node, request, S())
        self.assertTrue(reply.ok)
        self.assertFalse(json.loads(reply.code)['final'])
        deadline = self.engine.stop_deadline
        again = ns['control'](node, request, S())
        self.assertEqual(self.engine.stop_deadline, deadline)
        for _ in range(8):
            self.now += .02; self.observe(); self.engine.tick(self.now)
        request.operation = 'replay_result'
        reply = ns['control'](node, request, S())
        result = json.loads(reply.code)
        self.assertTrue(result['final'] and result['stop_confirmed'])
        self.assertFalse(self.engine.lease_id or result['completed'])
        request.epoch += 1
        self.assertFalse(ns['control'](node, request, S()).ok)

    def test_replay_nominal_arrival_keeps_measured_compliance_distinct_and_waits_for_stop(self):
        for mode in ('position', 'impedance'):
            with self.subTest(mode=mode):
                self.setUp()
                motion = self.prepared(held=mode == 'impedance', mode=mode)
                self.engine.submit(motion, self.now)
                lease, epoch = self.take_replay(motion)
                self.engine.submit(self.beat(motion, lease, epoch), self.now)
                end = self.engine.replay.effective+1.
                final = dict(zip(motion.names, motion.points[-1]))
                self.io.sampled_reference = lambda: (self.now, final, {}, {})
                index = 0
                while not self.engine.replay_result['final'] and self.now < end+.5:
                    self.now += .02; index += 1
                    self.q.update(final)
                    if mode == 'impedance': self.q['arm_l_joint_1'] += .08
                    self.observe()
                    if not self.engine.stopping:
                        try:
                            self.engine.submit(self.beat(motion, lease, epoch, 'beat'+str(index)), self.now)
                        except Rejected as exc:
                            self.assertEqual(str(exc), 'INVALID_LEASE')
                            self.assertEqual(self.engine.stopping[1], 'REPLAY_COMPLETED')
                    else:
                        self.engine.tick(self.now)
                result = self.engine.replay_result
                self.assertEqual(result['nominal_arrival_confirmed'], mode == 'impedance')
                self.assertEqual(result['measured_arrival_confirmed'], mode == 'position')
                self.assertTrue(result['final'] and result['stop_confirmed'] and result['completed'])
                next_lease, next_epoch = self.engine.acquire('next', motion.resource_groups, 2., self.now)
                follow = Motion('follow', next_lease, next_epoch, self.profile.digest, 'next', '',
                    'joint_target', '', self.now, .25, names=motion.names,
                    positions=tuple(self.q[n] for n in motion.names), schema_version=2,
                    resource_groups=motion.resource_groups, runtime_mode='real')
                self.engine.submit(follow, self.now)
                self.now += .02; self.observe(); self.engine.tick(self.now)
                self.assertEqual(self.engine.active.command_id, 'follow')
                self.engine.replay_status(lease, epoch, self.now, stop=True)
                self.assertEqual(self.engine.active.command_id, 'follow')

    def test_replay_stop_failure_preserves_command_timeout_and_original_budget(self):
        motion = self.prepared(); self.engine.submit(motion, self.now)
        lease, epoch = self.take_replay(motion)
        self.engine.submit(self.beat(motion, lease, epoch), self.now)
        self.engine.tick(self.now)
        self.v['arm_l_joint_1'] = .03
        for _ in range(14):
            self.now += .02; self.observe(); self.engine.tick(self.now)
        self.assertEqual(self.engine.stopping[1], 'COMMAND_TIMEOUT')
        deadline = self.engine.stop_deadline
        self.engine.replay_status(lease, epoch, self.now, stop=True)
        self.assertEqual(self.engine.stop_deadline, deadline)
        self.now = deadline
        self.observe(); self.engine.tick(self.now)
        result = self.engine.replay_result
        self.assertTrue(result['error'].startswith('COMMAND_TIMEOUT;STOP_UNCONFIRMED'), result)
        self.assertFalse(result['stop_confirmed'] or result['completed'])

    def setUp(self):
        self.profile = Profile.load(package_share("snowbot_bringup")/'config/examples/snowbot/snowbot_vendor_position.json')
        self.now = 10.
        self.sent = []
        self.io = SnowbotIO(self.profile, lambda g, q: self.sent.append((set(g), dict(q))),
                            lambda: self.now, command_enabled=True)
        self.q = dict.fromkeys(self.profile.limits, 0.)
        self.q['arm_l_joint_2'], self.q['arm_r_joint_2'] = .2, -.2
        self.v = dict.fromkeys(self.q, 0.)
        self.observe()
        self.engine = DelegatedPositionExecutor(self.profile, self.io)

    def test_arm_feedback_does_not_require_gripper_slots(self):
        self.now += .01
        names = [n for g in ('waist', 'left_arm', 'right_arm') for n in GROUPS[g]]
        self.observe(positions={n:self.q[n] for n in names}, velocities={n:0. for n in names})
        self.io.require_feedback(('waist', 'left_arm', 'right_arm'))
        self.assertIn('left_tool_axis', self.io.missing_feedback_groups)

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
        old = Profile.load(package_share("snowbot_bringup")/'config/examples/snowbot/snowbot_shovel_sim.json')
        self.assertEqual(old.limits['body_joint_2'], [-.79, 1.83])
        for path in (CONFIG/'examples/snowbot/snowbot_shovel_sim.json',
                     package_share('bindu_runtime')/'config/simulation/wheel_sim.json'):
            with self.assertRaisesRegex(ValueError, 'PROFILE_REQUIRED'):
                SnowbotIO(Profile.load(path), None, lambda: self.now)

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

    def test_command_rejection_does_not_freeze_measured_feedback(self):
        self.io.targets={n:self.q[n] for n in GROUPS['left_arm']}
        self.io.check_health=Mock(side_effect=RuntimeError('SNOWBOT_TARGET_REJECTED:PREPARED_STATE_GAP'))
        self.now+=.01
        self.q['arm_l_joint_1']+=.001
        self.observe()
        feedback=self.io.read_feedback(self.now,.01)
        self.assertEqual(feedback.stamp,self.now)
        self.assertEqual(feedback.positions['arm_l_joint_1'],self.q['arm_l_joint_1'])
        with self.assertRaisesRegex(RuntimeError,'TARGET_REJECTED'):
            self.io.write_joints(dict(self.io.targets))

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




    def test_health_loss_during_hold_cannot_confirm_arrival(self):
        self.io.write_joints(dict.fromkeys(GROUPS['waist'], .1))
        self.engine.halt(self.now, 'CANCELED')
        self.io.check_health = Mock(side_effect=RuntimeError('SNOWBOT_SERVO_NOT_READY'))
        self.now += .02
        self.observe()
        self.engine.tick(self.now)
        self.assertTrue(self.io.stop_failures)


    def sampled_feedback(self, **changes):
        fields = dict(header=S(stamp=S(sec=10, nanosec=20000000)),
                      l_position=[.1]*8, r_position=[-.1]*8,
                      l_speed=[0.]*8, r_speed=[0.]*8,
                      head_position=[.2, .3], head_speed=[0.]*2,
                      torso_position=[.4, .5], torso_speed=[0.]*2,
                      pdo_sample=1, pdo_age=.001, pdo_wkc=42, pdo_expected_wkc=42,
                      feedback_valid_mask=0xfffff, feedback_status=[39]*20)
        return S(**dict(fields, **changes))







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
        self.health=SessionHealth(lambda:self.now,lambda:self.steady)
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

    def test_waist_arrival_gate_reports_the_required_joint_without_hiding_arm_offset(self):
        self.io.stop_requires_arrival = False
        self.io.stop_arrival_joints = ('body_joint_1', 'body_joint_2')
        self.engine.feedback.positions[self.names[0]] = .038
        self.engine.feedback.positions['body_joint_2'] = .007425
        result = self.diagnose(self.health, self.engine, self.now)
        self.assertEqual(result['reason'], 'STOP_POSITION_ERROR')
        self.assertEqual(result['worst_required_position_joint'], 'body_joint_2')
        self.assertAlmostEqual(result['max_required_position_error_rad'], .007425)
        self.assertEqual(result['arrival_required_joints'], ['body_joint_1', 'body_joint_2'])
        self.assertFalse(result['required_reference_arrived'])
        self.engine.feedback.positions['body_joint_2'] = .001
        result = self.diagnose(self.health, self.engine, self.now)
        self.assertEqual(result['reason'], 'WAIT_REST_WINDOW')
        self.assertTrue(result['required_reference_arrived'])
        self.assertFalse(result['reference_arrived'])

    def test_per_joint_stop_tolerances_are_reported_without_relaxing_other_checks(self):
        self.io.stop_requires_arrival = False
        self.io.stop_arrival_joints = ('body_joint_1', 'body_joint_2')
        self.engine.profile.stop_position_tolerances = dict.fromkeys(self.io.stop_arrival_joints, math.radians(1))
        self.engine.feedback.positions['body_joint_2'] = .006064
        self.engine.feedback.positions[self.names[0]] = .038
        result = self.diagnose(self.health, self.engine, self.now)
        self.assertEqual(result['reason'], 'WAIT_REST_WINDOW')
        self.assertEqual(result['arrival_position_tolerances_rad'],
                         dict.fromkeys(self.io.stop_arrival_joints, math.radians(1)))
        self.assertEqual(result['position_tolerance_rad'], .002)
        self.assertTrue(result['required_reference_arrived'])
        self.assertFalse(result['reference_arrived'])
        self.engine.feedback.positions['body_joint_2'] = .018
        self.assertEqual(self.diagnose(self.health, self.engine, self.now)['reason'], 'STOP_POSITION_ERROR')

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
        import numpy as np
        case=SnowbotStopDiagnosticTests();case.setUp()
        # Fixed ROS float64 arrays produce NumPy scalars; list-only fixtures
        # miss the numpy.bool result that previously killed record_stop_check.
        case.obs.hold_position=np.zeros(16)
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

    def test_session_capture_changes_generation_and_pause_does_not_renew_motion(self):
        from snowbot_hardware.position_session import PositionSessionDevices, PositionSessionIO
        from snowbot_hardware.session import SourceEnvelope
        from snowbot_hardware.session import CONTROL_GROUPS
        profile=Profile.load(package_share("snowbot_bringup")/'config/examples/snowbot/snowbot_vendor_position.json')
        devices=PositionSessionDevices.__new__(PositionSessionDevices)
        devices.node=S(now=lambda:10.)
        devices.health=S(reference_mode=2)
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
        self.assertEqual(flags,[])  # PAUSE does not wait for a fresh ROS measurement.
        self.assertEqual(io.send.call_count,2)
        self.assertFalse(io.requesting_pause)

    """Use installed message classes with mocked publishers; never create a node."""





if __name__ == '__main__': unittest.main()
