"""Position-session source identity, independent PDO freshness and resource scope."""
from types import SimpleNamespace as S
from dataclasses import replace
import json
import unittest
from unittest.mock import Mock

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
        self.now += .025
        self.steady += .025
        self.health.observe(observation(self.now, pdo_sample=2))
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


if __name__ == '__main__':
    unittest.main()
