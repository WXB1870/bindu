"""Position-session source identity, independent PDO freshness and resource scope."""
from types import SimpleNamespace as S
from dataclasses import replace
import json
import unittest
from unittest.mock import Mock, patch

from bindu_runtime.snowbot_position_session import TrajectoryFault
from bindu_hardware.snowbot import GROUPS
from bindu_hardware.snowbot_session import ARMS, CONTROL_GROUPS, CONTROL_NAMES, CONTROL_MASK, SLOTS, SessionHealth, SourceEnvelope, decode_observation


def observation(now=10., **changes):
    data = dict(header=S(stamp=S(sec=int(now), nanosec=round((now-int(now))*1e9))),
        boot_id=1, pdo_sample=1, pdo_age=.001, wkc=67, expected_wkc=67,
        valid_mask=0xfffff, position=[.2]*20, speed=[0.]*20, current=[0.]*20,
        current_valid_mask=0xffff, torque_raw=[0]*20, torque_raw_valid_mask=0xf0000,
        status=[0x27]*20, session_reason=0, session_selected=False, session=0,
        command_sequence=0, hold_valid=False, hold_sample=0, hold_position=[0.]*16)
    data.update(changes)
    return S(**data)


class PositionSessionTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.steady = 1.
        self.health = SessionHealth(lambda: self.now, lambda: self.steady)
        for layer in ('upper', 'middle', 'servo'):
            self.health.state(layer, S(header=S(stamp=S(sec=10, nanosec=0)), layer=layer,
                startup_complete=True, feedback_ready=True, instance_id=12,
                control_source='topic' if layer == 'upper' else 'position',
                stop_policy='disable_no_load' if layer == 'servo' else 'servo_owned'))
        self.health.observe(observation())
        self.now += .001
        self.steady += .001
        self.health.observe(observation(self.now, pdo_sample=2))

    def motion(self, **changes):
        return S(command_id=changes.get('command_id', 'one'), stamp=changes.get('stamp', self.now),
                 valid_for=changes.get('valid_for', .25))

    def test_executor_republication_and_lease_heartbeat_cannot_renew_source(self):
        source = SourceEnvelope(11, lambda: self.now)
        motion = self.motion()
        first = source.command(motion)
        self.assertEqual(first['operation'], 1)
        self.assertEqual(first['expires_at_ns']-first['source_stamp_ns'], 200_000_000)
        for _ in range(30):
            self.now += .001
            self.assertIsNone(source.command(motion))
        self.assertEqual(source.sequence, 1)
        next_target = source.command(self.motion(command_id='two'))
        self.assertEqual(next_target['operation'], 0)
        self.assertEqual(next_target['sequence'], 2)

    def test_pause_and_resume_are_distinct_from_motion_refresh(self):
        source = SourceEnvelope(11, lambda: self.now)
        source.command(self.motion())
        self.now += .01
        pause = source.command(None, pause=True)
        self.assertEqual(pause['operation'], 2)
        resume = source.command(self.motion(command_id='resume'))
        self.assertEqual(resume['operation'], 1)
        with self.assertRaisesRegex(RuntimeError, 'EXPIRED'):
            source.command(self.motion(command_id='late', stamp=9.))

    def test_epoch_nanoseconds_do_not_round_ttl_above_protocol_limit(self):
        self.now = 1_792_567_890.1234567
        packet = SourceEnvelope(1, lambda: self.now).command(self.motion())
        self.assertEqual(packet['expires_at_ns']-packet['source_stamp_ns'], 200_000_000)

    def test_ros_publication_without_pdo_progress_is_rejected(self):
        self.health.require(CONTROL_GROUPS)
        self.now += .041
        self.steady += .041
        self.health.observe(observation(self.now, pdo_sample=2))
        with self.assertRaisesRegex(RuntimeError, 'NOT_ADVANCING'):
            self.health.require(CONTROL_GROUPS)

    def test_ros_feedback_budget_is_40ms_with_independent_20ms_pdo_limit(self):
        for age in (.021, .039):
            with self.subTest(ros_age=age):
                decode_observation(observation(pdo_age=0.), 10.+age)
        decode_observation(observation(0., pdo_age=0.), .040)
        with self.assertRaisesRegex(ValueError, 'PDO_STALE'):
            decode_observation(observation(pdo_age=0.), 10.041)
        with self.assertRaisesRegex(ValueError, 'PDO_STALE'):
            decode_observation(observation(pdo_age=.021), 10.)
        decode_observation(observation(pdo_age=.020), 10.)
        decode_observation(observation(pdo_age=.010), 10.029)
        with self.assertRaisesRegex(ValueError, 'PDO_STALE'):
            decode_observation(observation(pdo_age=.010), 10.031)

    def test_ros_progress_tolerates_39ms_but_repeated_samples_still_expire(self):
        self.now += .039
        self.steady += .039
        self.health.observe(observation(self.now, pdo_sample=3))
        self.assertEqual(self.health.advances, 2)
        self.health.require(CONTROL_GROUPS)
        self.now += .041
        self.steady += .041
        self.health.observe(observation(self.now, pdo_sample=3))
        with self.assertRaisesRegex(RuntimeError, 'NOT_ADVANCING'):
            self.health.require(CONTROL_GROUPS)

    def test_cached_ros_feedback_expires_after_40ms_without_new_observations(self):
        self.health.observation.pdo_age = 0.
        self.now += .039
        self.steady += .039
        self.health.require(CONTROL_GROUPS)
        self.now += .002
        self.steady += .002
        with self.assertRaisesRegex(RuntimeError, 'NOT_ADVANCING'):
            self.health.require(CONTROL_GROUPS)

    def test_wkc_and_fault_have_separate_failures_and_gripper_is_not_owned(self):
        with self.assertRaisesRegex(ValueError, 'WKC'):
            self.health.observe(observation(wkc=61))
        statuses = [0x27]*20
        statuses[0] |= 8
        self.health.observe(observation(pdo_sample=3, status=statuses))
        self.health.observe(observation(pdo_sample=4, status=statuses))
        with self.assertRaisesRegex(RuntimeError, 'DRIVE_FAULT'):
            self.health.require(CONTROL_GROUPS)
        statuses[0] = 0x27
        statuses[7] |= 8
        self.health.observe(observation(pdo_sample=5, status=statuses))
        self.health.observe(observation(pdo_sample=6, status=statuses))
        self.health.require(CONTROL_GROUPS)
        with self.assertRaisesRegex(RuntimeError, 'RESOURCE_LAYOUT'):
            self.health.require(('left_tool_axis',))

    def test_missing_waist_never_becomes_zero_or_ready(self):
        msg = observation(pdo_sample=3, valid_mask=0xffff)
        q, _, _ = decode_observation(msg, self.now)
        self.assertNotIn('body_joint_1', q)
        self.assertNotIn('head_joint_1', q)
        self.health.observe(msg)
        with self.assertRaisesRegex(RuntimeError, 'NOT_ADVANCING'):
            self.health.require(CONTROL_GROUPS)

    def test_waist_fault_invalidity_and_nonfinite_speed_block_coupled_control(self):
        for slot in (18, 19):
            status = [0x27]*20; status[slot] |= 8
            self.health.observe(observation(pdo_sample=self.health.sample+1, status=status))
            with self.assertRaisesRegex(RuntimeError, 'DRIVE_FAULT'):
                self.health.require(CONTROL_GROUPS)
            msg = observation(pdo_sample=self.health.sample+1, valid_mask=0xfffff & ~(1<<slot))
            self.health.observe(msg)
            with self.assertRaisesRegex(RuntimeError, 'NOT_ADVANCING'):
                self.health.require(CONTROL_GROUPS)
            msg = observation(pdo_sample=self.health.sample+1); msg.speed[slot] = float('nan')
            with self.assertRaisesRegex(ValueError, 'NONFINITE'): self.health.observe(msg)
        self.health.observe(observation(pdo_sample=self.health.sample+1))
        self.health.observe(observation(pdo_sample=self.health.sample+1))
        self.health.require(CONTROL_GROUPS)
        with self.assertRaisesRegex(RuntimeError, 'RESOURCE_LAYOUT'): self.health.require(ARMS)

    def test_restart_requires_reconnect_and_unconfirmed_policy_stays_blocked(self):
        with self.assertRaisesRegex(ValueError, 'RESTART'):
            self.health.observe(observation(boot_id=2))
        self.health.observe(observation(pdo_sample=3))
        self.health.observe(observation(pdo_sample=4))
        _, servo = self.health.states['servo']
        servo.stop_policy = 'unconfirmed'
        with self.assertRaisesRegex(RuntimeError, 'STOP_POLICY'):
            self.health.require(CONTROL_GROUPS)

    def test_bad_metadata_does_not_relabel_stale_feedback_as_current(self):
        for changes in (dict(pdo_age=-1), dict(pdo_age=.03), dict(position=[0.]*19)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.health.observe(observation(**changes))

    def test_observer_preserves_raw_csp_torque_and_its_validity(self):
        from tools.teleop.snowbot_pdo import observation_payload
        msg = observation()
        msg.torque_raw[16] = 123
        payload = observation_payload(msg)
        self.assertEqual(payload['torque_raw'][16], 123)
        self.assertEqual(payload['torque_raw_valid_mask'], 0xf0000)
        self.assertEqual(payload['current_valid_mask'], 0xffff)


class PauseHandoffTests(unittest.TestCase):
    """Exercise the real binding/executor with in-memory messages, without ROS."""
    def setUp(self):
        from tests.support.paths import ROOT
        from bindu_contracts.profile import Profile
        from bindu_execution.delegated_position import DelegatedPositionExecutor
        from bindu_runtime.snowbot_position_session import PositionSessionDevices, PositionSessionIO
        self.now, self.steady, self.sample = 10., 1., 0
        self.profile = Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_vendor_position.json')
        self.profile = replace(self.profile, stop_timeout=.05)
        self.q = dict.fromkeys(self.profile.limits, 0.)
        self.q['arm_l_joint_2'], self.q['arm_r_joint_2'] = .2, -.2
        self.q['body_joint_1'], self.q['body_joint_2'] = .12, -.08
        self.devices = d = PositionSessionDevices.__new__(PositionSessionDevices)
        d.node = S(now=lambda: self.now)
        d.health = SessionHealth(lambda: self.now, lambda: self.steady)
        d.envelopes = SourceEnvelope(11, d.node.now)
        d.arm_type = lambda: S(meta=S(), command=S())
        d.publishers = {'arms': Mock()}
        d.check_ready = lambda groups: d.check_health(groups, acquiring=True)
        self.io = d.io = PositionSessionIO(self.profile, d.send, d.node.now, command_enabled=True,
                                         check_ready=d.check_ready, check_health=d.check_health)
        self.io.after_capture = d.new_session
        self.io.pause_feedback = d.health.confirmed_hold
        self.update(); self.update()
        self.engine = DelegatedPositionExecutor(self.profile, self.io, steady=lambda: self.steady)

    def update(self, *, hold=None, velocity=0., waist_velocity=None, **changes):
        self.now += .005; self.steady += .005; self.sample += 1
        d = self.devices
        for layer in ('upper', 'middle', 'servo'):
            d.health.state(layer, S(header=observation(self.now).header, layer=layer, instance_id=12,
                startup_complete=True, feedback_ready=True, control_source='topic' if layer=='upper' else 'position',
                stop_policy='measured_hold'))
        msg = observation(self.now, pdo_sample=self.sample,
                          position=[self.q[n] for n in SLOTS], speed=[velocity]*20)
        if waist_velocity is not None: msg.speed[18:] = [waist_velocity]*2
        if hold is not None:
            session, sequence, boot, baseline = self.io.pause_request
            msg.session, msg.command_sequence, msg.boot_id = session, sequence, boot
            msg.session_selected, msg.session_reason, msg.hold_valid = True, 1, True
            msg.hold_sample, msg.hold_position = baseline, list(hold)
        for key, value in changes.items(): setattr(msg, key, value)
        q, v, stamp = d.health.observe(msg)
        self.io.observe(q, v, stamp, self.now, sample=msg.pdo_sample)

    def start_stop(self, command_id='move'):
        from bindu_contracts.contracts import Motion
        lease, epoch = self.engine.acquire('test', ('waist','left_arm','right_arm'), 2., self.now)
        names = tuple(GROUPS['left_arm'])
        motion = Motion(command_id, lease, epoch, self.profile.digest, 'test', 'pause-test',
            'joint_target', 'left_arm', self.now, .2, names=names,
            positions=tuple(self.q[n]+(.01 if n.endswith('_1') else 0) for n in names), runtime_mode='real')
        self.engine.submit(motion, self.now); self.engine.tick(self.now)
        self.now += .001; self.steady += .001
        self.devices.advance_session()
        self.engine.halt(self.now, revoke=True)
        self.assertTrue(self.engine.stopping)
        return [self.io.stop_targets[n] for n in CONTROL_NAMES]

    def test_delayed_pause_ack_still_needs_actual_position_and_velocity(self):
        hold = self.start_stop()
        self.update(); self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)  # Sender position at rest is not an ACK.
        hold[0] += .01; self.q['arm_l_joint_1'] = hold[0]
        self.update(hold=hold, velocity=.03); self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)
        self.assertAlmostEqual(self.engine.reference.q[self.engine.reference_names.index(CONTROL_NAMES[0])], hold[0])
        self.q['arm_l_joint_1'] += .003
        self.update(hold=hold); self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)  # Original 2 mrad tolerance applies.
        self.q['arm_l_joint_1'] = hold[0]
        self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)
        self.assertFalse(self.engine.stop_fault)

    def test_old_identity_incomplete_or_nonfinite_ack_cannot_finish_stop(self):
        hold = self.start_stop()
        for changes in (dict(session=123), dict(command_sequence=1), dict(hold_valid=False),
                        dict(hold_sample=0), dict(hold_sample=2**63), dict(hold_position=[0.]*14),
                        dict(hold_position=[float('nan')]*16)):
            with self.subTest(changes=changes):
                self.update(hold=hold, **changes); self.engine.tick(self.now)
                self.assertTrue(self.engine.stopping)
        self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)

    def test_changed_hold_ack_fails_instead_of_following_measured_drift(self):
        hold = self.start_stop()
        self.update(hold=hold, velocity=.03); self.engine.tick(self.now)
        hold[0] += .01; self.q['arm_l_joint_1'] = hold[0]
        self.update(hold=hold); self.engine.tick(self.now)
        self.assertEqual(self.engine.stop_fault, 'STOP_REFERENCE_INVALID')

    def test_late_ack_cannot_reset_stop_budget(self):
        hold = self.start_stop()
        deadline = self.engine.stop_deadline
        for _ in range(11): self.update(hold=hold)
        self.engine.tick(self.now)
        self.assertTrue(self.engine.stop_fault.startswith('STOP_UNCONFIRMED'))
        self.assertEqual(self.engine.stop_deadline, deadline)

    def test_retakeover_uses_new_session_and_rejects_old_pause_ack(self):
        hold = self.start_stop()
        old_request = self.io.pause_request
        self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)
        hold = self.start_stop('retakeover')
        self.assertNotEqual(self.io.pause_request[0], old_request[0])
        self.update(hold=hold, session=old_request[0]); self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)
        self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)

    def test_nonzero_waist_shares_one_pause_and_requires_fixed_point_and_rest(self):
        hold = self.start_stop()
        self.assertEqual(len(self.devices.publishers['arms'].publish.call_args_list), 2)
        packet = self.devices.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(packet.meta.operation, 2)
        self.assertEqual(packet.waist_position, [.12, -.08])
        hold[14] += .015; hold[15] -= .01
        self.q['body_joint_1'], self.q['body_joint_2'] = hold[14:]
        self.update(hold=hold, waist_velocity=.03); self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)
        self.q['body_joint_1'] += .003
        self.update(hold=hold); self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)
        self.q['body_joint_1'] = hold[14]
        self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)
        self.assertEqual(set(self.io.coupled_group_sets[0]), CONTROL_GROUPS)


class RealResourceTests(unittest.TestCase):
    def test_native_real_config_solves_and_transports_sixteen_controlled_joints(self):
        from tests.support.paths import ROOT
        from bindu_contracts.profile import Profile
        try:
            from bindu_kinematics.bimanual import BimanualQP
        except ImportError as exc:
            self.skipTest('Optional native QP environment required: '+str(exc))
        base = ROOT/'src/integration/bindu_runtime/config/robots/snowbot'
        cfg = json.loads((base/'bimanual_snowbot_real.json').read_text())
        self.assertEqual(tuple(cfg['resource_groups']), ('waist','left_arm','right_arm'))
        self.assertEqual(cfg['shared_motion_weight'], .1)
        cfg['urdf'] = str(ROOT/'src/hardware/bindu_description/urdf/snowbot_control.urdf')
        profile = Profile.load(base/'snowbot_vendor_position.json')
        solver = BimanualQP(cfg, profile); self.addCleanup(solver.close)
        self.assertEqual(solver.names, (*GROUPS['waist'], *GROUPS['left_arm'], *GROUPS['right_arm']))
        self.assertEqual(len(solver.model_names), 20)
        # Existing synthetic grasp fixture is measured input to this offline test.
        q = json.loads((base/'bimanual_snowbot_shovel.json').read_text())['demo']['initial_positions_rad']
        tool = solver.capture(q)
        target=tool.copy();target[0,3]+=.001
        output, _ = solver.solve(q, target, reference=[q[n] for n in solver.names], dt=.01, target_stamp=10.)
        self.assertEqual(len(output), 16)
        self.assertTrue(any(abs(output[i]-q[n])>1e-8 for i,n in enumerate(solver.names) if n in GROUPS['waist']))
        self.assertEqual(len(CONTROL_NAMES),16)
        self.assertEqual(CONTROL_MASK,0xc7f7f)
        from bindu_contracts.contracts import Motion
        rig = PauseHandoffTests(); rig.setUp(); rig.q.update(q); rig.update()
        lease, epoch = rig.engine.acquire('qp', tuple(cfg['resource_groups']), 2., rig.now)
        command = Motion('qp16', lease, epoch, profile.digest, 'qp', 'qp16', 'joint_target', '',
            rig.now, .2, names=solver.names, positions=tuple(output), runtime_mode='real',
            schema_version=2, resource_groups=tuple(cfg['resource_groups']))
        rig.engine.submit(command, rig.now); rig.engine.tick(rig.now)
        packet = rig.devices.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(packet.waist_position, list(output[:2]))
        self.assertEqual(packet.command.l_arm_position, list(output[2:9]))
        self.assertEqual(packet.command.r_arm_position, list(output[9:]))
        self.assertEqual(packet.meta.session, rig.devices.envelopes.session)
        for name in GROUPS['waist']:
            self.assertIn(name, solver.names)
            self.assertEqual(solver._q(q)[solver.model_names.index(name)], q[name])
            missing = dict(q); missing.pop(name)
            with self.assertRaisesRegex(ValueError, 'FEEDBACK_LAYOUT'): solver.capture(missing)


class TrajectoryWireTests(unittest.TestCase):
    def binding(self):
        from bindu_runtime.snowbot_position_session import SessionTrajectory
        rig = PauseHandoffTests(); rig.setUp(); d = rig.devices
        d.health.reference_mode = 1
        for _, state in d.health.states.values(): state.wire_version = 2
        d.health.states['servo'][1].reference_mode = 1
        d.arm_type = lambda: S(meta=S(), command=S(), trajectory=S())
        d.node.event = Mock()
        d.trajectory = SessionTrajectory(rig.profile)
        d.pending_trajectory, d.trajectory_instance = None, None
        d.trajectory_fault_kind, d.trajectory_fault_reason = TrajectoryFault.NONE, ''
        d.io.trajectory_reference = True
        d.io.wait_for_acceptance = lambda: d.pending_trajectory is not None
        return rig

    def send_source(self, rig, command_id='source'):
        d = rig.devices
        d.io.source_motion = S(command_id=command_id, stamp=rig.now, valid_for=.2, lease_id='lease', epoch=1)
        d.send(CONTROL_GROUPS, {n: rig.q[n] for n in CONTROL_NAMES})
        return d.publishers['arms'].publish.call_args.args[0]

    def acknowledge(self, rig, msg):
        servo = rig.devices.health.states['servo'][1]
        servo.session, servo.sequence = msg.meta.session, msg.meta.sequence

    def test_health_and_send_never_settle_or_dispatch_deferred_pause(self):
        rig = self.binding(); d = rig.devices
        first = self.send_source(rig); self.acknowledge(rig, first)
        pending = d.pending_trajectory
        d.io.deferred_pause = True
        d.node.event.reset_mock()
        with patch.object(d.io, 'stop') as stop, patch.object(d.trajectory, 'commit', wraps=d.trajectory.commit) as commit:
            for _ in range(3): d.check_health(CONTROL_GROUPS)
            self.send_source(rig, 'waiting')
            self.assertIs(d.pending_trajectory, pending)
            self.assertIsNone(d.trajectory.segment)
            self.assertTrue(d.io.deferred_pause)
            self.assertEqual(d.envelopes.sequence, 1)
            self.assertEqual(d.publishers['arms'].publish.call_count, 1)
            commit.assert_not_called(); stop.assert_not_called(); d.node.event.assert_not_called()
            d.advance_session(); d.advance_session()
            commit.assert_called_once(); stop.assert_called_once()
            self.assertEqual(d.node.event.call_count, 1)
            self.assertEqual(d.node.event.call_args.args[0], 'SEGMENT_ACCEPTED')
            self.assertFalse(d.io.deferred_pause)

    def test_fault_category_controls_pause_even_when_reason_text_is_misleading(self):
        rig = self.binding(); d = rig.devices
        d.trajectory.segment = object()
        d.io.stop_targets = {n: rig.q[n] for n in CONTROL_NAMES}
        for kind, reason, allowed in (
                (TrajectoryFault.REJECTED_WITH_REFERENCE, 'SNOWBOT_TRAJECTORY_INSTANCE_CHANGED_RECONNECT_REQUIRED', True),
                (TrajectoryFault.RECONNECT_REQUIRED, 'SNOWBOT_TRAJECTORY_REJECTED:looks_recoverable', False)):
            with self.subTest(kind=kind):
                d.trajectory_fault_kind, d.trajectory_fault_reason = kind, reason
                d.io.requesting_pause = True
                if allowed: d.check_health(CONTROL_GROUPS)
                else:
                    with self.assertRaisesRegex(RuntimeError, reason): d.check_health(CONTROL_GROUPS)
                d.io.requesting_pause = False
                with self.assertRaisesRegex(RuntimeError, reason): self.send_source(rig, 'forbidden-target')
                d.io.deferred_pause = True
                with patch.object(d.io, 'stop') as stop:
                    d.advance_session()
                    self.assertEqual(stop.call_count, int(allowed))
                self.assertIs(d.trajectory_fault_kind, kind)

    def test_stale_ack_cannot_commit_and_late_ack_latches_without_health_side_effects(self):
        rig = self.binding(); d = rig.devices
        first = self.send_source(rig); self.acknowledge(rig, first)
        pending = d.pending_trajectory
        rig.now += .03; rig.steady += .03
        d.advance_session()
        self.assertIs(d.pending_trajectory, pending)
        self.assertIsNone(d.trajectory.segment)
        with self.assertRaisesRegex(RuntimeError, 'NOT_ADVANCING'): d.check_health(CONTROL_GROUPS)
        self.assertIs(d.trajectory_fault_kind, TrajectoryFault.NONE)
        rig.now += .18; rig.steady += .18
        d.advance_session()
        self.assertIsNone(d.pending_trajectory)
        self.assertIsNone(d.trajectory.segment)
        self.assertIs(d.trajectory_fault_kind, TrajectoryFault.RECONNECT_REQUIRED)
        self.assertIn('ACCEPTANCE_UNCONFIRMED', d.trajectory_fault_reason)
        self.assertEqual([call.args[0] for call in d.node.event.call_args_list], ['SEGMENT_SENT'])

    def test_explicit_rejection_allows_one_deferred_pause_from_last_accepted_reference(self):
        rig = self.binding(); d = rig.devices
        first = self.send_source(rig); self.acknowledge(rig, first); d.advance_session()
        accepted = d.trajectory.segment
        middle = self.send_source(rig, 'rejected')
        d.io.targets = {n: rig.q[n] for n in CONTROL_NAMES}
        d.io.stop()
        self.assertTrue(d.io.deferred_pause)
        servo = d.health.states['servo'][1]
        servo.rejected_session, servo.rejected_sequence = middle.meta.session, middle.meta.sequence
        servo.reason = 'rejected_segment'
        d.advance_session()
        pause = d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(pause.meta.operation, 2)
        self.assertEqual(d.publishers['arms'].publish.call_count, 3)
        self.assertIs(d.trajectory.segment, accepted)
        self.assertIs(d.trajectory_fault_kind, TrajectoryFault.REJECTED_WITH_REFERENCE)
        self.assertFalse(d.io.deferred_pause)
        self.assertFalse(d.io.stop_failures)
        d.check_health(CONTROL_GROUPS)  # STOPPING can await this permitted PAUSE.
        d.advance_session()
        self.assertEqual(d.publishers['arms'].publish.call_count, 3)
        self.acknowledge(rig, pause); d.advance_session(); d.advance_session()
        self.assertIs(d.trajectory_fault_kind, TrajectoryFault.NONE)
        self.assertEqual(d.publishers['arms'].publish.call_count, 3)
        self.assertTrue(d.io.stop_targets)  # Software ACK still does not confirm physical rest.

    def test_pending_send_keeps_full_endpoint_checks_without_minting_source(self):
        rig = self.binding(); d = rig.devices
        del d.check_ready
        d.receivers = dict(arms='/robot_upperMonitor', middle='/dual_arm_control_node', head='/servo_node')
        routes = dict(command=('PositionSessionCommand', None, 'arms'),
                      upper=('PositionSessionUpper', 'arms', 'middle'),
                      servo=('PositionSessionServo', 'middle', 'head'))
        queries = []
        def infos(topic, subscription):
            queries.append(topic)
            kind, writer, reader = routes[topic.rsplit('/', 1)[1]]
            return [S(node_namespace='/', node_name=d.receivers[reader if subscription else writer][1:],
                      topic_type='snowbot_ctl_msgs/msg/'+kind)]
        def count(topic):
            queries.append(topic)
            return int(topic == '/bindu/position/command')
        d.node.get_publishers_info_by_topic = lambda t: infos(t, False)
        d.node.get_subscriptions_info_by_topic = lambda t: infos(t, True)
        d.node.count_publishers = count
        self.send_source(rig)
        for _ in range(3):
            queries.clear(); self.send_source(rig, 'waiting')
            self.assertEqual(len(queries), 8)
        d.node.count_publishers = lambda t: 2 if t == '/bindu/position/command' else 0
        with self.assertRaisesRegex(RuntimeError, 'COMMAND_PUBLISHER_CONFLICT'): self.send_source(rig, 'conflict')
        self.assertEqual(d.envelopes.sequence, 1)
        self.assertEqual(d.publishers['arms'].publish.call_count, 1)

    def test_deferred_pause_failure_stays_inside_original_stop_budgets(self):
        from bindu_contracts.contracts import Motion
        rig = self.binding(); d = rig.devices
        groups = ('left_arm', 'right_arm', 'waist')
        lease, epoch = rig.engine.acquire('test', groups, 2., rig.now)
        motion = Motion('inflight', lease, epoch, rig.profile.digest, 'test', '', 'joint_target', '', rig.now, .2,
            names=tuple(CONTROL_NAMES), positions=tuple(rig.q[n] for n in CONTROL_NAMES), runtime_mode='real',
            schema_version=2, resource_groups=groups)
        rig.engine.submit(motion, rig.now); rig.engine.tick(rig.now)
        first = d.publishers['arms'].publish.call_args.args[0]
        rig.engine.halt(rig.now, revoke=True)
        budgets = rig.engine.stop_deadline, rig.engine.stop_steady_deadline
        self.acknowledge(rig, first)
        with patch.object(d.io, 'stop', side_effect=RuntimeError('deferred_pause_failed')):
            d.advance_session()  # Must return so the original engine tick still runs.
            rig.engine.tick(rig.now)
        self.assertIs(d.trajectory_fault_kind, TrajectoryFault.RECONNECT_REQUIRED)
        self.assertEqual(d.trajectory_fault_reason, 'deferred_pause_failed')
        self.assertIn('vendor_hold', d.io.stop_failures)
        self.assertTrue(rig.engine.stop_fault.startswith('STOP_UNCONFIRMED'))
        self.assertEqual((rig.engine.stop_deadline, rig.engine.stop_steady_deadline), budgets)

    def setUp(self):
        from tests.support.paths import ROOT
        from bindu_contracts.profile import Profile
        from bindu_runtime.snowbot_position_session import SessionTrajectory
        self.profile = Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_vendor_position.json')
        self.planner = SessionTrajectory(self.profile)
        self.obs = observation(position=[0.]*20)
        self.obs.position[1], self.obs.position[9] = .2, -.2
        self.goal = {n: self.obs.position[SLOTS.index(n)] for n in CONTROL_NAMES}
        self.source = SourceEnvelope(91, lambda: self.now)
        self.now = 10.

    def plan(self, number, *, pause=False):
        motion = S(command_id=str(number), stamp=self.now, valid_for=.2)
        metadata = self.source.command(motion, pause=pause)
        proposal = self.planner.prepare(metadata, self.goal, self.obs, self.now)
        self.planner.commit(proposal)  # Fixture explicitly simulates servo acceptance.
        return proposal.wire, metadata

    def test_real_curve_description_retains_order_clock_and_source_identity(self):
        from bindu_execution.interpolation import PreparedSegment
        self.goal['arm_l_joint_1'] += .02
        self.goal['body_joint_1'] += .01
        wire, metadata = self.plan(1)
        self.assertEqual(wire['segment_id'], metadata['sequence'])
        self.assertEqual(wire['effective_at_ns'], 10_060_000_000)
        self.assertEqual(len(wire['control_count']), 16*len(wire['duration']))
        self.assertTrue(all(v <= .25 for v in wire['max_velocity']))
        self.assertTrue(all(a <= .5 for a in wire['max_acceleration']))
        imported = PreparedSegment.from_description(self.profile, CONTROL_NAMES, self.planner.segment.describe())
        for t in (0., .001, .02, imported.end):
            self.assertEqual(imported.sample(t), self.planner.segment.sample(t))
        self.now += .01
        self.assertIsNone(self.source.command(S(command_id='1', stamp=10., valid_for=.2)))
        self.assertEqual(metadata['expires_at_ns'], 10_200_000_000)

    def test_noncollinear_retarget_and_pause_continue_predicted_state(self):
        self.goal['arm_l_joint_1'] += .02
        self.plan(1)
        previous, switch = self.planner.segment, self.planner.effective_ns
        self.now += .02
        self.goal['arm_r_joint_1'] -= .01
        self.plan(2)
        expected = previous.sample((self.planner.effective_ns-switch)*1e-9)
        actual = self.planner.segment.sample(0.)
        for a, b in zip((expected.q, expected.v, expected.a), (actual.q, actual.v, actual.a)):
            for x, y in zip(a, b): self.assertAlmostEqual(x, y, places=10)
        previous, switch = self.planner.segment, self.planner.effective_ns
        self.now += .02
        wire, metadata = self.plan(3, pause=True)
        expected = previous.sample((wire['effective_at_ns']-switch)*1e-9)
        actual = self.planner.segment.sample(0.)
        for x, y in zip(actual.v, expected.v): self.assertAlmostEqual(x, y, places=10)
        self.assertLess(wire['effective_at_ns']+self.planner.segment.end*1e9, metadata['expires_at_ns'])
        self.assertTrue(all(abs(v)<1e-8 for v in self.planner.segment.sample(self.planner.segment.end).v))

    def test_moving_takeover_short_deadline_and_wrong_protocol_are_rejected(self):
        self.obs.speed[18] = .02
        with self.assertRaisesRegex(RuntimeError, 'REQUIRES_REST'): self.plan(1)
        self.obs.speed[18] = 0.
        with self.assertRaisesRegex(RuntimeError, 'DEADLINE_TOO_SHORT'):
            self.planner.prepare(dict(operation=1, sequence=1, expires_at_ns=10_050_000_000), self.goal, self.obs, 10.)
        health = PositionSessionTests(); health.setUp()
        health.health.reference_mode = 1
        with self.assertRaisesRegex(RuntimeError, 'PROTOCOL_CONFLICT'): health.health.require(CONTROL_GROUPS)
        for _, state in health.health.states.values(): state.wire_version=2
        health.health.states['servo'][1].reference_mode = 1
        health.health.require(CONTROL_GROUPS)

    def test_conservative_limits_reject_pause_that_cannot_fit_original_ttl(self):
        from bindu_execution.interpolation import PreparedSegment, State
        q = tuple(self.goal[n] for n in CONTROL_NAMES)
        velocity = (.25,)+(0.,)*15
        goal = list(q); goal[0] += .2
        previous = PreparedSegment.target(self.planner.profile, CONTROL_NAMES,
                                          State(q, velocity, (0.,)*16), goal, 0.)
        self.planner.segment, self.planner.effective_ns = previous, 10_000_000_000
        with self.assertRaisesRegex(ValueError, 'NO_FEASIBLE_STOP'):
            self.planner.prepare(dict(operation=2,sequence=2,expires_at_ns=10_200_000_000),
                                 self.goal,self.obs,10.)
        self.assertIs(self.planner.segment, previous)

    def test_binding_records_mapping_once_and_checks_pause_reference_basis(self):
        rig = PauseHandoffTests(); rig.setUp()
        devices = rig.devices
        devices.health.reference_mode = 1
        for _, state in devices.health.states.values(): state.wire_version=2
        devices.health.states['servo'][1].reference_mode = 1
        devices.arm_type = lambda: S(meta=S(), command=S(), trajectory=S())
        devices.node.event = Mock()
        devices.pending_trajectory, devices.trajectory_instance = None, None
        devices.trajectory_fault_kind, devices.trajectory_fault_reason = TrajectoryFault.NONE, ''
        def acknowledge(msg):
            servo = devices.health.states['servo'][1]
            servo.session, servo.sequence = msg.meta.session, msg.meta.sequence
        devices.publishers['arms'].publish.side_effect = acknowledge
        hold = rig.start_stop('source-v2')
        sent = [call.args[0] for call in devices.publishers['arms'].publish.call_args_list]
        self.assertEqual(len(sent), 2)
        self.assertEqual([p.meta.reference_mode for p in sent], [1, 1])
        self.assertEqual([p.trajectory.segment_id for p in sent], [1, 2])
        self.assertEqual([p.meta.source_command_id for p in sent], ['source-v2']*2)
        self.assertEqual(devices.node.event.call_count, 3)
        mappings = [json.loads(call.args[1].split(':', 1)[1]) for call in devices.node.event.call_args_list
                    if call.args[0] == 'SEGMENT_SENT']
        self.assertEqual(mappings[0]['session'], sent[0].meta.session)
        self.assertEqual(mappings[0]['expires_at_ns'], sent[0].meta.expires_at_ns)
        self.assertEqual(mappings[0]['lease_id'], sent[0].meta.lease_id)
        self.assertEqual(mappings[1]['segment_id'], 2)
        # A braking reference and an impedance nominal-hold ACK cannot confirm
        # a position stop. The same identity with its completed basis can.
        rig.update(hold=hold, reference_mode=1, segment_id=2, trajectory_phase=3, hold_basis=0)
        for _, state in devices.health.states.values(): state.wire_version=2
        devices.health.states['servo'][1].reference_mode = 1
        self.assertIsNone(devices.health.confirmed_hold(rig.io.pause_request))
        devices.health.observation.trajectory_phase = 4
        devices.health.observation.hold_basis = 1
        self.assertIsNone(devices.health.confirmed_hold(rig.io.pause_request))
        devices.health.observation.hold_basis = 0
        self.assertEqual(devices.health.confirmed_hold(rig.io.pause_request), dict(zip(CONTROL_NAMES, hold)))

    def test_dropped_rejected_ack_timeout_and_restart_never_advance_fake_reference(self):
        from bindu_runtime.snowbot_position_session import SessionTrajectory
        for failure in ('dropped', 'rejected', 'restart'):
            with self.subTest(failure=failure):
                rig = PauseHandoffTests(); rig.setUp(); devices = rig.devices
                devices.health.reference_mode = 1
                for _, state in devices.health.states.values(): state.wire_version = 2
                servo = devices.health.states['servo'][1]; servo.reference_mode = 1
                devices.node.event = Mock(); devices.arm_type = lambda: S(meta=S(), command=S(), trajectory=S())
                devices.trajectory = SessionTrajectory(rig.profile)
                devices.pending_trajectory, devices.trajectory_instance = None, None
                devices.trajectory_fault_kind, devices.trajectory_fault_reason = TrajectoryFault.NONE, ''
                target = {n: rig.q[n] for n in CONTROL_NAMES}
                def send(command):
                    devices.io.source_motion = S(command_id=command, stamp=rig.now, valid_for=.2, lease_id='lease', epoch=1)
                    devices.send(CONTROL_GROUPS, target)
                send('accepted')
                self.assertIsNone(devices.trajectory.segment)  # Publishing is not acceptance.
                first = devices.publishers['arms'].publish.call_args.args[0]
                servo.session, servo.sequence = first.meta.session, first.meta.sequence
                devices.advance_session()
                confirmed = devices.trajectory.segment
                send('middle')  # This segment is lost/rejected before servo adoption.
                middle = devices.publishers['arms'].publish.call_args.args[0]
                send('latest')
                self.assertEqual(devices.publishers['arms'].publish.call_count, 2)
                self.assertEqual(devices.envelopes.sequence, 2)
                self.assertIs(devices.trajectory.segment, confirmed)
                if failure == 'rejected':
                    servo.rejected_session, servo.rejected_sequence = middle.meta.session, middle.meta.sequence
                    servo.reason = 'trajectory_late_or_expired'
                    expected = 'TRAJECTORY_REJECTED'
                elif failure == 'restart':
                    servo.instance_id += 1; expected = 'INSTANCE_CHANGED_RECONNECT_REQUIRED'
                else:
                    rig.now += .201; expected = 'ACCEPTANCE_UNCONFIRMED'
                devices.advance_session()
                self.assertIn(expected, devices.trajectory_fault_reason)
                self.assertIs(devices.trajectory.segment, confirmed)
                self.assertIsNot(devices.trajectory_fault_kind, TrajectoryFault.NONE)
                with self.assertRaises(RuntimeError): send('dependent')
                self.assertEqual(devices.publishers['arms'].publish.call_count, 2)

    def test_upstream_goal_waits_for_actual_reference_then_expires_without_fallback(self):
        from bindu_runtime.snowbot_position_session import sampled_nominal_reference
        rig = PauseHandoffTests(); rig.setUp(); io = rig.io
        io.trajectory_reference = True
        goal = {n: rig.q[n]+.01 for n in CONTROL_NAMES}
        io.targets = dict(goal); io.reference = dict(goal)
        self.assertEqual(io.reference_positions, {})
        state = S(lease_active=True, lease_epoch=3,
                  reference=S(header=observation(rig.now).header, name=[], position=[]))
        # Service accepted precedes worker adoption/first PDO. New input waits;
        # it cannot bootstrap another solve from the queued goal.
        self.assertIsNone(sampled_nominal_reference(state, CONTROL_NAMES, 3, rig.now, .1,
                          wait_until=1.1, steady_now=1.))
        with self.assertRaisesRegex(RuntimeError, 'UNAVAILABLE'):
            sampled_nominal_reference(state, CONTROL_NAMES, 3, rig.now, .1,
                                      wait_until=1.1, steady_now=1.11)
        actual = {n: rig.q[n] for n in CONTROL_NAMES}
        sampled = rig.now-.005
        io.servo_reference = (sampled, actual, dict.fromkeys(actual, .03), dict.fromkeys(actual, .1))
        self.assertEqual(io.reference_positions, actual)
        self.assertNotEqual(io.reference_positions, goal)
        state.reference = S(header=observation(sampled).header, name=list(actual), position=list(actual.values()))
        value, at = sampled_nominal_reference(state, CONTROL_NAMES[::-1], 3, rig.now, .1)
        self.assertEqual(value, tuple(actual[n] for n in CONTROL_NAMES[::-1]))
        self.assertAlmostEqual(at, sampled)
        with self.assertRaisesRegex(RuntimeError, 'OWNER'):
            sampled_nominal_reference(state, CONTROL_NAMES, 4, rig.now, .1,
                                      wait_until=2., steady_now=1.)
        rig.now += .03
        self.assertEqual(io.reference_positions, {})

    def test_pause_waits_for_inflight_acceptance_without_resetting_stop_budget(self):
        from bindu_contracts.contracts import Motion
        rig = PauseHandoffTests(); rig.setUp(); d = rig.devices
        d.health.reference_mode = 1; d.io.trajectory_reference = True
        d.pending_trajectory, d.trajectory_instance = None, None
        d.trajectory_fault_kind, d.trajectory_fault_reason = TrajectoryFault.NONE, ''
        d.io.wait_for_acceptance = lambda: d.pending_trajectory is not None
        for _, state in d.health.states.values(): state.wire_version = 2
        d.health.states['servo'][1].reference_mode = 1
        d.arm_type = lambda: S(meta=S(), command=S(), trajectory=S()); d.node.event = Mock()
        groups = ('left_arm','right_arm','waist')
        lease, epoch = rig.engine.acquire('test', groups, 2., rig.now)
        names = tuple(CONTROL_NAMES)
        motion = Motion('inflight',lease,epoch,rig.profile.digest,'test','','joint_target','',rig.now,.2,
                        names=names,positions=tuple(rig.q[n] for n in names),runtime_mode='real',
                        schema_version=2,resource_groups=groups)
        rig.engine.submit(motion,rig.now);rig.engine.tick(rig.now)
        first = d.publishers['arms'].publish.call_args.args[0]
        rig.engine.halt(rig.now,revoke=True)
        budget = rig.engine.stop_deadline
        self.assertTrue(rig.io.deferred_pause);self.assertTrue(rig.engine.stopping)
        self.assertFalse(rig.io.stop_failures);self.assertEqual(d.publishers['arms'].publish.call_count,1)
        servo = d.health.states['servo'][1];servo.session,servo.sequence = first.meta.session,first.meta.sequence
        d.advance_session(); d.advance_session()
        self.assertFalse(rig.io.deferred_pause)
        self.assertEqual(d.publishers['arms'].publish.call_count,2)
        pause = d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(pause.meta.operation,2);self.assertEqual(rig.engine.stop_deadline,budget)
        expiry = pause.meta.expires_at_ns
        rig.io.stop();rig.io.stop()
        self.assertEqual(d.publishers['arms'].publish.call_count,2)
        self.assertEqual(d.pending_trajectory[0]['expires_at_ns'],expiry)


if __name__ == '__main__':
    unittest.main()
