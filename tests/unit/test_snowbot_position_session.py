"""Position-session source identity, independent PDO freshness and resource scope."""
from bindu_contracts.resources import package_share
from types import SimpleNamespace as S
import ast
import math
from dataclasses import replace
from pathlib import Path
import json
import unittest
from unittest.mock import Mock, patch

from snowbot_hardware.protocol import GROUPS
from snowbot_hardware.session import ARMS, CONTROL_GROUPS, CONTROL_NAMES, CONTROL_MASK, SLOTS, SessionHealth, SourceEnvelope, decode_observation, new_session_id


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
                startup_complete=True, feedback_ready=True, instance_id=12, wire_version=3,
                control_source='topic' if layer == 'upper' else 'position',
                stop_policy='disable_no_load' if layer == 'servo' else 'servo_owned'))
        self.health.observe(observation())
        self.now += .001
        self.steady += .001
        self.health.observe(observation(self.now, pdo_sample=2))

    def motion(self, **changes):
        return S(command_id=changes.get('command_id', 'one'), stamp=changes.get('stamp', self.now),
                 valid_for=changes.get('valid_for', .25))

    def test_ordered_identity_handles_same_tick_rollback_restart_and_uint64_limit(self):
        first = new_session_id(10.)
        self.assertEqual(first, 10_000_000_000)
        self.assertEqual(new_session_id(10., first), first+1)
        self.assertEqual(new_session_id(9., first), first+1)
        self.assertGreater(new_session_id(10.001), first)  # New source process.
        self.assertEqual(new_session_id(1., (1 << 64)-2), (1 << 64)-1)
        with self.assertRaisesRegex(ValueError, 'ID_EXHAUSTED'):
            new_session_id(1., (1 << 64)-1)
        for now in (0., -1., float('nan'), float('inf')):
            with self.assertRaisesRegex(ValueError, 'CLOCK_INVALID'):
                new_session_id(now)

    def test_old_unordered_wire_versions_are_rejected_before_control(self):
        for layer in ('upper', 'middle', 'servo'):
            state = self.health.states[layer][1]
            for version in (2, 4):
                state.wire_version = version
                with self.assertRaisesRegex(RuntimeError, 'SESSION_PROTOCOL_CONFLICT:'+layer):
                    self.health.require(CONTROL_GROUPS, acquiring=True)
            state.wire_version = 3

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

    def test_target_stream_pause_has_own_fixed_stop_deadline(self):
        source = SourceEnvelope(11, lambda: self.now, target_stream=True)
        motion = self.motion(valid_for=.4)
        ordinary = source.command(motion)
        self.now += .01
        pause = source.command(None, pause=True)
        self.assertEqual(ordinary['expires_at_ns']-ordinary['source_stamp_ns'], 300_000_000)
        self.assertEqual(pause['expires_at_ns']-pause['source_stamp_ns'], 2_000_000_000)
        self.assertEqual(pause['operation'], 2)
        self.now += .02
        self.assertIsNone(source.command(motion))
        self.assertEqual(pause['expires_at_ns'], pause['source_stamp_ns']+2_000_000_000)
        legacy = SourceEnvelope(12, lambda: self.now).command(None, pause=True)
        self.assertEqual(legacy['expires_at_ns']-legacy['source_stamp_ns'], 200_000_000)

    def test_target_stream_ttl_uses_original_source_and_never_extends_short_commands(self):
        for stamp in (10., 1_792_567_890.1234567):
            for valid_for, ttl in ((.15, 150_000_000), (.4, 300_000_000)):
                with self.subTest(stamp=stamp, valid_for=valid_for):
                    self.now = stamp+.1
                    source = SourceEnvelope(11, lambda: self.now, target_stream=True)
                    motion = self.motion(stamp=stamp, valid_for=valid_for)
                    packet = source.command(motion)
                    self.assertEqual(packet['source_stamp_ns'], round(stamp*1e9))
                    self.assertEqual(packet['expires_at_ns'], round(stamp*1e9)+ttl)
                    self.now += .02
                    self.assertIsNone(source.command(motion))
                    self.assertEqual(source.sequence, 1)
                    self.now = stamp+min(valid_for, .3)
                    with self.assertRaisesRegex(RuntimeError, 'EXPIRED'):
                        source.command(self.motion(command_id='new-id-same-old-source',
                                                   stamp=stamp, valid_for=valid_for))

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

    def test_one_arm_feedback_offline_cannot_be_hidden_by_other_arm_progress(self):
        right_mask=sum(1<<SLOTS.index(name) for name in GROUPS['right_arm'])
        for sample in (3,4,5):
            msg=observation(self.now,pdo_sample=sample,valid_mask=0xfffff & ~right_mask)
            q,_,_=decode_observation(msg,self.now)
            self.assertTrue(all(name in q for name in GROUPS['left_arm']))
            self.assertTrue(all(name not in q for name in GROUPS['right_arm']))
            self.health.observe(msg)
            with self.assertRaisesRegex(RuntimeError,'NOT_ADVANCING'):
                self.health.require(CONTROL_GROUPS)
        # Restoration alone needs new complete observations; no command or
        # old target is generated by this read-only health gate.
        self.health.observe(observation(self.now,pdo_sample=6))
        with self.assertRaisesRegex(RuntimeError,'NOT_ADVANCING'):
            self.health.require(CONTROL_GROUPS)
        self.health.observe(observation(self.now,pdo_sample=7))
        self.health.require(CONTROL_GROUPS,acquiring=True)

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
        from snowbot_hardware.position_session import PositionSessionDevices, PositionSessionIO
        self.now, self.steady, self.sample = 10., 1., 0
        self.profile = Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_vendor_position.json')
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
                startup_complete=True, feedback_ready=True, wire_version=3, control_source='topic' if layer=='upper' else 'position',
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

    def enable_rest_window(self):
        self.profile = replace(self.profile, stop_timeout=.5)
        self.io.profile = self.engine.profile = self.profile
        self.io.stop_confirmation_window = .1
        self.io.stop_confirmation_max_gap = .04
        self.io.stop_requires_arrival = False
        self.io.target_stream = True

    def test_target_pause_low_waist_speed_cannot_hide_remaining_hold_travel(self):
        self.enable_rest_window()
        hold = self.start_stop()
        waist = CONTROL_NAMES.index('body_joint_2')
        # 118 A-return: 29.7 mrad in the old 65536-count coordinate,
        # converted to the selected 262144-count position coordinate.
        hold[waist] = self.q['body_joint_2'] + .0297 / 4
        for _ in range(24):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)
        self.assertIsNone(self.engine.stop_rest_window)
        self.q['body_joint_2'] = hold[waist] - .001
        for _ in range(24):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)
        self.assertFalse(self.engine.stop_fault)

    def test_target_pause_unreached_waist_expires_without_claiming_stop(self):
        self.enable_rest_window()
        hold = self.start_stop()
        hold[CONTROL_NAMES.index('body_joint_1')] += .007
        deadline = self.engine.stop_deadline
        for _ in range(105):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertEqual(self.engine.stop_deadline, deadline)
        self.assertTrue(self.engine.stop_fault.startswith('STOP_UNCONFIRMED'))
        self.assertFalse(any(e.state == 'CANCELED' for e in self.engine.events))

    def test_configured_waist_stop_tolerance_accepts_recorded_residual_after_rest(self):
        from bindu_contracts.profile import Profile
        self.enable_rest_window()
        selected = Profile.load(package_share('snowbot_bringup')/'config/robots/snowbot/snowbot_real_position.json')
        self.profile = replace(self.profile, stop_position_tolerances=selected.stop_position_tolerances)
        self.io.profile = self.engine.profile = self.profile
        self.assertEqual(self.profile.stop_position_tolerances,
                         dict.fromkeys(GROUPS['waist'], math.radians(1)))
        self.assertEqual(self.profile.feedback_tolerances['joint_position'], .002)
        hold = self.start_stop()
        hold[CONTROL_NAMES.index('body_joint_2')] += .006064
        for _ in range(20):
            self.update(hold=hold); self.engine.tick(self.now)
            self.assertTrue(self.engine.stopping)
        for _ in range(3):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)
        self.assertFalse(self.engine.stop_fault)

    def test_configured_waist_stop_tolerance_still_rejects_excess_position_error(self):
        self.enable_rest_window()
        self.profile = replace(self.profile, stop_position_tolerances={'body_joint_2':math.radians(1)})
        self.io.profile = self.engine.profile = self.profile
        hold = self.start_stop()
        hold[CONTROL_NAMES.index('body_joint_2')] += .018
        deadline = self.engine.stop_deadline
        for _ in range(105):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertEqual(self.engine.stop_deadline, deadline)
        self.assertTrue(self.engine.stop_fault.startswith('STOP_UNCONFIRMED'))

    def test_relaxed_arrival_keeps_original_speed_and_position_stability_checks(self):
        self.enable_rest_window()
        self.profile = replace(self.profile, stop_position_tolerances={'body_joint_2':math.radians(1)})
        self.io.profile = self.engine.profile = self.profile
        hold = self.start_stop()
        hold[CONTROL_NAMES.index('body_joint_2')] += .006064
        for _ in range(24):
            self.update(hold=hold, waist_velocity=.02); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stop_rest_window)
        for _ in range(16):
            self.update(hold=hold); self.engine.tick(self.now)
        self.q['body_joint_2'] += .003  # Inside arrival tolerance, beyond the 2 mrad rest anchor.
        self.update(hold=hold); self.engine.tick(self.now)
        for _ in range(8):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)
        for _ in range(14):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)

    def test_waist_override_does_not_relax_arm_arrival(self):
        self.enable_rest_window()
        self.io.stop_requires_arrival = True
        self.profile = replace(self.profile, stop_position_tolerances={'body_joint_2':math.radians(1)})
        self.io.profile = self.engine.profile = self.profile
        hold = self.start_stop()
        hold[CONTROL_NAMES.index('arm_l_joint_1')] += .003
        for _ in range(105):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertTrue(self.engine.stop_fault.startswith('STOP_UNCONFIRMED'))

    def test_target_pause_requires_continuous_measured_rest_window(self):
        self.enable_rest_window()
        hold = self.start_stop()
        deadline = self.engine.stop_deadline
        for _ in range(20):
            self.update(hold=hold); self.engine.tick(self.now)
            self.assertTrue(self.engine.stopping)
        self.update(hold=hold); self.engine.tick(self.now)
        self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)
        self.assertEqual(self.engine.stop_deadline, deadline)

    def test_target_pause_rest_does_not_claim_arrival_at_offset_reference(self):
        self.enable_rest_window()
        hold = self.start_stop()
        hold[0] -= .0452
        for _ in range(24):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)
        self.assertFalse(self.engine.stop_fault)
        self.assertEqual(self.io.pause_targets[CONTROL_NAMES[0]], hold[0])
        self.assertAlmostEqual(self.q[CONTROL_NAMES[0]]-hold[0], .0452)

    def test_target_pause_fixed_window_anchor_catches_drift_inside_arrival_tolerance(self):
        self.enable_rest_window()
        hold = self.start_stop()
        self.q[CONTROL_NAMES[0]] = hold[0]-.0015
        for _ in range(16):
            self.update(hold=hold); self.engine.tick(self.now)
        self.q[CONTROL_NAMES[0]] = hold[0]+.0015
        self.update(hold=hold); self.engine.tick(self.now)
        for _ in range(8):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)
        self.assertEqual(self.io.pause_targets[CONTROL_NAMES[0]], hold[0])
        for _ in range(14):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)

    def test_target_pause_unmatched_ack_cannot_start_rest_window(self):
        self.enable_rest_window()
        hold = self.start_stop()
        for _ in range(24):
            self.update(hold=hold, command_sequence=999); self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)
        self.assertIsNone(self.engine.stop_rest_window)

    def test_target_pause_velocity_and_feedback_gap_restart_window(self):
        self.enable_rest_window()
        hold = self.start_stop()
        for _ in range(16):
            self.update(hold=hold); self.engine.tick(self.now)
        self.update(hold=hold, velocity=.03); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stop_rest_window)
        for _ in range(16):
            self.update(hold=hold); self.engine.tick(self.now)
        # The observer may keep receiving while execution stalls. Do not infer
        # rest during that gap from only its first and final samples.
        for _ in range(10): self.update(hold=hold)
        self.engine.tick(self.now)
        self.assertTrue(self.engine.stopping)
        self.assertEqual(self.engine.stop_rest_window[0], self.steady)
        for _ in range(22):
            self.update(hold=hold); self.engine.tick(self.now)
        self.assertIsNone(self.engine.stopping)

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
        base = package_share('snowbot_bringup')/'config/robots/snowbot'
        cfg = json.loads((base/'bimanual_snowbot_real.json').read_text())
        self.assertEqual(tuple(cfg['resource_groups']), ('waist','left_arm','right_arm'))
        self.assertEqual(cfg['shared_motion_weight'], .1)
        cfg['urdf'] = str(package_share("snowbot_description")/'urdf/snowbot_control.urdf')
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


class TargetWireTests(unittest.TestCase):
    """Production binding with synthetic messages; no ROS or device access."""
    def setUp(self):
        self.rig = rig = PauseHandoffTests(); rig.setUp()
        self.d = d = rig.devices
        d.grippers=None
        d.health.reference_mode = 2
        d.trajectory_limits = dict(trajectory_max_velocity=.25, trajectory_max_acceleration=.5)
        for _, state in d.health.states.values(): state.wire_version = 5
        d.health.states['servo'][1].reference_mode = 2
        d.sent, d.target_instance, d.source_identity, d.target_fault = {}, None, None, ''
        d.applied_sequence = 0
        d.takeover_pending = None
        d.takeover_previous_hold = None
        d.io.trajectory_reference = True
        d.io.target_stream = True
        d.io.supports_rigid_reference = True
        d.node.event = Mock(); d.node.get_logger = Mock()
        d.arm_type = lambda: S(meta=S(), command=S(), trajectory=S(effective_at_ns=0,
            piece_start=[], duration=[], source_duration=[], source_elapsed=[], control_count=[], controls=[]))
        d.identity = Mock()
        d.receivers = {'head': '/servo_node'}
        d.feedback_topic='/bindu/position/observation'
        self.target = {n: rig.q[n] for n in CONTROL_NAMES}

    def send(self, command_id='move', *, pause=False, **changes):
        data = dict(command_id=command_id, lease_id='lease', epoch=3, stamp=self.rig.now, valid_for=.2)
        data.update(changes)
        self.d.io.source_motion = S(**data)
        self.d.io.sending_hold = pause
        try:
            self.d.send(CONTROL_GROUPS, dict(self.target))
        finally:
            self.d.io.sending_hold = False
        return self.d.publishers['arms'].publish.call_args.args[0]

    def receipt(self, packet, *, accepted=True, **changes):
        data = dict(header=observation(self.rig.now).header, layer='servo', wire_version=5,
            reference_mode=2, instance_id=12, feedback_boot_id=self.d.health.boot,
            state='accepted' if accepted else 'rejected', reason='target_invalid',
            session=packet.meta.session if accepted else 0, sequence=packet.meta.sequence if accepted else 0,
            rejected_session=0 if accepted else packet.meta.session,
            rejected_sequence=0 if accepted else packet.meta.sequence)
        data.update(changes)
        return S(**data)

    def update_target_feedback(self, **changes):
        self.rig.update(**changes)
        for _, state in self.d.health.states.values(): state.wire_version = 5
        self.d.health.states['servo'][1].reference_mode = 2

    def start_powered_hold(self):
        self.rig.enable_rest_window()
        self.d.startup_hold_state = 'waiting'
        self.d.node.closing = False
        self.d.advance_startup_hold(self.rig.engine)
        self.assertEqual(self.d.startup_hold_state, 'takeover')
        packet = self.d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(packet.meta.operation, 1)
        self.assertEqual(list(packet.command.l_arm_position), [self.rig.q[n] for n in GROUPS['left_arm']])
        self.d.receive_receipt(self.receipt(packet))
        self.d.applied_sequence = 1
        self.d.advance_startup_hold(self.rig.engine)
        self.assertEqual(self.d.startup_hold_state, 'stopping')
        self.assertFalse(self.rig.engine.lease_id)
        return self.d.publishers['arms'].publish.call_args.args[0]

    def test_startup_enables_once_then_confirms_powered_hold_without_vr(self):
        pause = self.start_powered_hold()
        self.assertEqual(pause.meta.operation, 2)
        hold = [self.rig.q[n] for n in CONTROL_NAMES]
        for _ in range(24):
            self.update_target_feedback(hold=hold, reference_mode=2, segment_id=pause.meta.sequence,
                trajectory_phase=4, hold_basis=0, reference_valid=True, output_valid=True)
            self.rig.engine.tick(self.rig.now)
            self.d.advance_startup_hold(self.rig.engine)
        self.assertEqual(self.d.startup_hold_state, 'ready')
        self.assertIsNone(self.rig.engine.stopping)
        self.assertEqual(self.d.publishers['arms'].publish.call_count, 2)
        for _ in range(10): self.d.advance_startup_hold(self.rig.engine)
        self.assertEqual(self.d.publishers['arms'].publish.call_count, 2)

    def test_read_held_reference_survives_new_lease_without_publishing(self):
        pause=self.start_powered_hold()
        hold=[self.rig.q[n] for n in CONTROL_NAMES]
        hold[0]-=.018
        for _ in range(24):
            self.update_target_feedback(hold=hold,reference_mode=2,segment_id=pause.meta.sequence,
                trajectory_phase=4,hold_basis=0,reference_valid=True,output_valid=True)
            self.rig.engine.tick(self.rig.now)
            self.d.advance_startup_hold(self.rig.engine)
        self.assertIsNone(self.rig.engine.stopping)
        self.d.node.engine=self.rig.engine
        self.d.node.instance_id='adapter'
        sent=self.d.publishers['arms'].publish.call_count
        old_request=self.d.io.pause_request
        # A can validate the exact completed hold before acquiring authority.
        idle=self.d.read_held_reference(None,S())
        self.assertTrue(idle.success,idle.message)
        self.assertEqual(json.loads(idle.message)['lease_id'],'')
        lease,epoch=self.rig.engine.acquire('follow',tuple(CONTROL_GROUPS),2.,self.rig.now)
        self.assertIsNone(self.d.io.pause_request)
        self.assertEqual(self.d.io.captured_pause_request,old_request)
        result=self.d.read_held_reference(None,S())
        self.assertTrue(result.success,result.message)
        value=json.loads(result.message)
        self.assertEqual((value['lease_id'],value['epoch']),(lease,epoch))
        self.assertEqual(value['positions'],dict(zip(CONTROL_NAMES,hold)))
        self.assertNotEqual(value['positions'][CONTROL_NAMES[0]],self.rig.q[CONTROL_NAMES[0]])
        obs=self.d.health.observation
        for field,bad in [('boot_id',999),('session',999),('command_sequence',999),
                          ('trajectory_phase',3),('hold_valid',False)]:
            before=getattr(obs,field);setattr(obs,field,bad)
            self.assertFalse(self.d.read_held_reference(None,S()).success,field)
            setattr(obs,field,before)
        self.rig.now+=.3;self.rig.steady+=.3
        self.assertFalse(self.d.read_held_reference(None,S()).success)
        self.assertEqual(self.d.publishers['arms'].publish.call_count,sent)

    def expired_completed_pause(self):
        self.test_startup_enables_once_then_confirms_powered_hold_without_vr()
        pause = self.d.publishers['arms'].publish.call_args.args[0]
        self.rig.now += 2.; self.rig.steady += 2.
        for _ in range(2):
            self.update_target_feedback(hold=[self.rig.q[n] for n in CONTROL_NAMES],
                reference_mode=2, segment_id=pause.meta.sequence, trajectory_phase=4,
                hold_basis=0, reference_valid=True, output_valid=True)
        servo = self.d.health.states['servo'][1]
        servo.rejected_session, servo.rejected_sequence = pause.meta.session, pause.meta.sequence
        servo.reason = 'duplicate_or_expired_frame'
        self.d.advance_session()
        return pause

    def test_completed_pause_expired_repeat_allows_fresh_capture_without_sending(self):
        pause = self.expired_completed_pause()
        self.assertIn('duplicate_or_expired_frame', self.d.target_fault)
        sent = self.d.publishers['arms'].publish.call_count
        lease, epoch = self.rig.engine.acquire('follow', tuple(CONTROL_GROUPS), 2., self.rig.now)
        self.assertTrue(lease)
        self.assertNotEqual(self.d.envelopes.session, pause.meta.session)
        self.assertEqual(self.d.target_fault, '')
        self.assertEqual(self.d.publishers['arms'].publish.call_count, sent)
        packet = self.send('new-follow', lease_id=lease, epoch=epoch)
        self.assertEqual(packet.meta.operation, 1)
        self.assertIsNotNone(self.d.takeover_pending)

    def test_expired_pause_recovery_requires_completed_unchanged_healthy_hold(self):
        for fault in ('pending_stop', 'stop_failed', 'target', 'unexpired',
                      'wrong_sequence', 'hold_changed', 'hold_invalid', 'instance_changed', 'other_rejection'):
            with self.subTest(fault=fault):
                self.setUp(); self.expired_completed_pause()
                metadata = self.d.sent[self.d.envelopes.sequence][0]
                if fault == 'pending_stop': self.d.io.stop_targets = dict(self.target)
                elif fault == 'stop_failed': self.d.io.stop_failures = ('vendor_hold',)
                elif fault == 'target': metadata['operation'] = 0
                elif fault == 'unexpired': metadata['expires_at_ns'] = round((self.rig.now+1)*1e9)
                elif fault == 'wrong_sequence': self.d.health.observation.command_sequence += 1
                elif fault == 'hold_changed': self.d.health.observation.hold_position[0] += .01
                elif fault == 'hold_invalid': self.d.health.observation.hold_valid = False
                elif fault == 'instance_changed': self.d.health.states['servo'][1].instance_id += 1
                else: self.d.target_fault = 'SNOWBOT_TARGET_REJECTED:target_invalid'
                with self.assertRaisesRegex(RuntimeError, 'TARGET_REJECTED'):
                    self.d.check_health(CONTROL_GROUPS, acquiring=True)

    def test_startup_failure_or_disabled_writes_never_retry_enable(self):
        self.d.node.closing = False
        self.d.startup_hold_state = 'disabled'
        self.d.advance_startup_hold(self.rig.engine)
        self.d.publishers['arms'].publish.assert_not_called()
        self.start_powered_hold()
        self.rig.engine.stop_fault = 'STOP_UNCONFIRMED'
        self.d.advance_startup_hold(self.rig.engine)
        self.assertEqual(self.d.startup_hold_state, 'failed')
        self.rig.engine.stop_fault = ''
        for _ in range(10): self.d.advance_startup_hold(self.rig.engine)
        self.assertEqual(self.d.publishers['arms'].publish.call_count, 2)

    def test_follow_after_startup_hold_keeps_lease_during_pdo_handoff(self):
        from bindu_contracts.contracts import Motion
        self.test_startup_enables_once_then_confirms_powered_hold_without_vr()
        old = self.d.health.observation
        groups = ('waist', 'left_arm', 'right_arm')
        lease, epoch = self.rig.engine.acquire('teleop', groups, 2., self.rig.now)
        names = tuple(n for group in groups for n in GROUPS[group])
        motion = Motion('first_follow', lease, epoch, self.rig.profile.digest,
            'test', '', 'joint_target', '', self.rig.now, .2, names=names,
            positions=tuple(self.target[n] for n in names), schema_version=2,
            resource_groups=groups, runtime_mode='real')
        self.rig.engine.submit(motion, self.rig.now)
        self.rig.engine.tick(self.rig.now)
        packet = self.d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(packet.meta.operation, 1)
        for i in range(7):
            self.held_feedback(session=old.session, command_sequence=old.command_sequence)
            if i == 5: self.d.receive_receipt(self.receipt(packet))
            self.d.advance_session()
            self.rig.engine.tick(self.rig.now)
            self.assertEqual(self.rig.engine.lease_id, lease)
            self.assertIsNone(self.rig.engine.stopping)
        self.assertEqual(self.d.publishers['arms'].publish.call_count, 3)

    def test_powered_hold_lifecycle_reuses_pause_until_explicit_fresh_motion(self):
        from bindu_contracts.contracts import Motion
        from bindu_execution.executor import Rejected
        self.test_startup_enables_once_then_confirms_powered_hold_without_vr()
        engine, publisher = self.rig.engine, self.d.publishers['arms'].publish
        # Exercise the same hold/release handlers used by PICO and tool finally,
        # without importing rclpy or creating a node or publisher.
        from tests.support.paths import ROOT
        source = ROOT/'src/integration/bindu_runtime/bindu_runtime/execution_node.py'
        node = next(n for n in ast.parse(source.read_text()).body
                    if isinstance(n, ast.ClassDef) and n.name == 'ExecutionNode')
        methods = [n for n in node.body if getattr(n, 'name', '') in ('lease', 'control')]
        namespace = {'Rejected': Rejected}
        module = ast.Module(body=[ast.ClassDef(name='Services', bases=[], keywords=[],
            body=methods, decorator_list=[])], type_ignores=[])
        exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), namespace)
        services = namespace['Services']()
        services.engine, services.robot_io, services.profile = engine, self.rig.io, self.rig.profile
        services.now, services.closing = lambda: self.rig.now, False
        groups = ('waist', 'left_arm', 'right_arm')
        names = tuple(n for group in groups for n in GROUPS[group])

        def follow(command_id):
            lease, epoch = engine.acquire('teleop', groups, 2., self.rig.now)
            # Acquiring authority alone preserves the already confirmed hold.
            before = publisher.call_count
            motion = Motion(command_id, lease, epoch, self.rig.profile.digest,
                'test', '', 'joint_target', '', self.rig.now, .2, names=names,
                positions=tuple(self.target[n] for n in names), schema_version=2,
                resource_groups=groups, runtime_mode='real')
            engine.submit(motion, self.rig.now); engine.tick(self.rig.now)
            self.assertEqual(publisher.call_count, before+1)
            packet = publisher.call_args.args[0]
            self.assertEqual(packet.meta.operation, 1)
            self.d.receive_receipt(self.receipt(packet))
            return motion

        self.assertEqual(publisher.call_count, 2)  # Startup TAKEOVER and PAUSE.
        motion = follow('explicit-follow')
        self.assertEqual(publisher.call_count, 3)
        hold_request = S(operation='hold', lease_id=motion.lease_id, epoch=motion.epoch)
        self.assertTrue(services.control(hold_request, S()).ok)
        pause = publisher.call_args.args[0]
        request, requested_at = self.rig.io.pause_request, self.rig.io.stop_requested_at
        budget = (engine.stop_deadline, engine.stop_steady_deadline)
        self.assertEqual((publisher.call_count, pause.meta.operation), (4, 2))
        for _ in range(3):
            self.assertTrue(services.control(hold_request, S()).ok)
            self.rig.io.stop()  # Repeated cleanup shares the original PAUSE.
        self.assertEqual(publisher.call_count, 4)
        self.assertEqual(self.rig.io.pause_request, request)
        self.assertEqual(self.rig.io.stop_requested_at, requested_at)
        self.assertEqual((engine.stop_deadline, engine.stop_steady_deadline), budget)
        hold = [self.rig.q[n] for n in CONTROL_NAMES]
        for _ in range(24):
            self.update_target_feedback(hold=hold, reference_mode=2, segment_id=pause.meta.sequence,
                trajectory_phase=4, hold_basis=0, reference_valid=True, output_valid=True)
            engine.tick(self.rig.now)
        self.assertIsNone(engine.stopping)
        release = S(operation='release', lease_id=motion.lease_id, epoch=motion.epoch)
        self.assertTrue(services.lease(release, S()).ok)
        self.assertEqual(services.lease(release, S()).code, 'INVALID_LEASE')
        self.rig.io.stop()
        self.assertEqual(publisher.call_count, 4)
        self.assertEqual(self.rig.io.pause_request, request)
        self.assertFalse(engine.stop_fault)
        with self.assertRaisesRegex(Rejected, 'INVALID_LEASE'):
            engine.submit(motion, self.rig.now)
        self.assertEqual(publisher.call_count, 4)
        engine.acquire('no-motion', groups, 2., self.rig.now)
        engine.halt(self.rig.now, revoke=True)
        self.assertEqual(publisher.call_count, 4)  # Empty lease emits no takeover or pause.
        fresh = follow('explicit-reengage')
        self.assertNotEqual(fresh.lease_id, motion.lease_id)
        self.assertGreater(fresh.epoch, motion.epoch)
        engine.tick(self.rig.now)
        self.assertEqual(publisher.call_count, 5)
        self.assertEqual([call.args[0].meta.operation for call in publisher.call_args_list],
                         [1, 2, 1, 2, 1])

    def test_unconfirmed_hold_cleanup_never_retries_takeover_or_releases_barrier(self):
        from bindu_execution.executor import Rejected
        self.rig.enable_rest_window()
        self.rig.start_stop()
        engine, publisher = self.rig.engine, self.d.publishers['arms'].publish
        request, budget = self.rig.io.pause_request, (engine.stop_deadline, engine.stop_steady_deadline)
        for _ in range(5):
            engine.halt(self.rig.now, revoke=True, emergency=True)
            self.rig.io.stop()
        self.assertEqual(publisher.call_count, 2)
        self.assertEqual(self.rig.io.pause_request, request)
        self.assertEqual((engine.stop_deadline, engine.stop_steady_deadline), budget)
        self.rig.now += .51; self.rig.steady += .51
        engine.halt(self.rig.now, revoke=True)
        self.assertIn('STOP_UNCONFIRMED', engine.stop_fault)
        with self.assertRaisesRegex(Rejected, 'STOP_UNCONFIRMED'):
            engine.acquire('reengage', tuple(CONTROL_GROUPS), 2., self.rig.now)
        self.assertEqual(publisher.call_count, 2)
        self.assertEqual(self.rig.io.pause_request, request)
        self.assertEqual((engine.stop_deadline, engine.stop_steady_deadline), budget)

    def pending_replacement_at_braking_boundary(self, *, confirm_takeover=True):
        from bindu_contracts.contracts import Motion
        groups=('waist','left_arm','right_arm')
        names=tuple(n for g in groups for n in GROUPS[g])
        engine=self.rig.engine
        lease,epoch=engine.acquire('follow',groups,2.,self.rig.now)
        first=Motion('old',lease,epoch,self.rig.profile.digest,'test','','joint_target','',
            self.rig.now,.4,names=names,positions=tuple(self.target[n] for n in names),
            schema_version=2,resource_groups=groups,runtime_mode='real')
        engine.submit(first,self.rig.now);engine.tick(self.rig.now)
        packet=self.d.publishers['arms'].publish.call_args.args[0]
        if confirm_takeover:self.d.receive_receipt(self.receipt(packet))
        # Executor/feedback continue while the target producer has no new input.
        for _ in range(47):
            self.update_target_feedback();engine.tick(self.rig.now)
        self.rig.now+=.02;self.rig.steady+=.02
        self.update_target_feedback()
        fresh=replace(first,command_id='replacement',stamp=self.rig.now-.035)
        engine.submit(fresh,self.rig.now)
        self.assertEqual(self.d.publishers['arms'].publish.call_count,1)
        return packet,fresh

    def test_accepted_replacement_is_dispatched_before_old_target_prebrake(self):
        old,motion=self.pending_replacement_at_braking_boundary()
        self.d.brake_before_source_expiry(self.rig.engine)
        packet=self.d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(packet.meta.source_command_id,motion.command_id)
        self.assertEqual(packet.meta.operation,0)
        self.assertEqual(packet.meta.source_stamp_ns,round(motion.stamp*1e9))
        self.assertEqual(packet.meta.expires_at_ns,round(motion.stamp*1e9)+300_000_000)
        self.assertIsNone(self.rig.engine.stopping)
        self.assertEqual(self.rig.engine.lease_id,motion.lease_id)
        self.assertEqual(old.meta.expires_at_ns,old.meta.source_stamp_ns+300_000_000)
        for _ in range(5):self.d.brake_before_source_expiry(self.rig.engine)
        self.assertEqual(self.d.publishers['arms'].publish.call_count,2)

    def test_accepted_replacement_does_not_extend_an_unconfirmed_takeover(self):
        old,_=self.pending_replacement_at_braking_boundary(confirm_takeover=False)
        self.d.brake_before_source_expiry(self.rig.engine)
        packet=self.d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(packet.meta.operation,2)
        self.assertFalse(self.rig.engine.lease_id)
        self.assertEqual(old.meta.expires_at_ns,old.meta.source_stamp_ns+300_000_000)
        self.assertEqual(self.d.publishers['arms'].publish.call_count,2)

    def test_replacement_still_obeys_endpoint_health_before_dispatch(self):
        self.pending_replacement_at_braking_boundary()
        self.d.target_fault='SNOWBOT_TARGET_INSTANCE_CHANGED_RECONNECT_REQUIRED'
        self.d.brake_before_source_expiry(self.rig.engine)
        self.assertFalse(self.rig.engine.lease_id)
        self.assertIsNone(self.rig.engine.active)
        self.assertEqual([c.args[0].meta.operation for c in self.d.publishers['arms'].publish.call_args_list],
                         [1,2])  # Only the original takeover and protective PAUSE.

    def test_target_loss_requests_one_pause_before_original_source_expiry(self):
        self.rig.enable_rest_window()
        self.d.startup_hold_state = 'waiting'; self.d.node.closing = False
        self.d.advance_startup_hold(self.rig.engine)
        first = self.d.publishers['arms'].publish.call_args.args[0]
        deadline = first.meta.expires_at_ns
        self.d.receive_receipt(self.receipt(first))
        for _ in range(15):
            self.update_target_feedback()
            self.d.brake_before_source_expiry(self.rig.engine)
        self.assertEqual(self.d.publishers['arms'].publish.call_count, 1)
        self.update_target_feedback()
        self.d.brake_before_source_expiry(self.rig.engine)
        pause = self.d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(pause.meta.operation, 2)
        self.assertLess(pause.meta.source_stamp_ns, deadline)
        self.assertEqual(first.meta.expires_at_ns, deadline)
        self.assertEqual(pause.meta.expires_at_ns-pause.meta.source_stamp_ns, 2_000_000_000)
        for _ in range(10): self.d.brake_before_source_expiry(self.rig.engine)
        self.assertEqual(self.d.publishers['arms'].publish.call_count, 2)

    def production_readiness(self):
        from snowbot_hardware.position_session import PositionSessionDevices
        self.d.receivers = dict(head='/servo_node', middle='/dual_arm_control_node', arms='/robot_upperMonitor')
        self.d.node.count_publishers = lambda topic: int(topic == '/bindu/position/command')
        self.d.check_ready = PositionSessionDevices.check_ready.__get__(self.d)
        self.rig.io.check_ready = self.d.check_ready

    def test_longer_source_ttl_brakes_at_180ms_with_dispatch_reserve(self):
        self.production_readiness()
        motion = self.source_motion()
        self.rig.engine.submit(motion, self.rig.now); self.rig.engine.tick(self.rig.now)
        first = self.d.publishers['arms'].publish.call_args.args[0]
        self.d.receive_receipt(self.receipt(first))
        for _ in range(35):  # 175 ms; retain 120 ms for the full PAUSE path.
            self.update_target_feedback(); self.rig.engine.tick(self.rig.now)
            self.d.brake_before_source_expiry(self.rig.engine)
            self.assertIsNone(self.rig.engine.stopping)
            self.assertEqual(self.d.publishers['arms'].publish.call_count, 1)
        self.update_target_feedback()
        self.d.brake_before_source_expiry(self.rig.engine)
        pause = self.d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(round((self.rig.now-motion.stamp)*1e9), 180_000_000)
        self.assertEqual(pause.meta.operation, 2)
        # Reproduce the observed 56 ms dispatch plus 10 ms preparation.
        effective=pause.meta.source_stamp_ns+56_000_000+10_000_000
        self.assertLess(effective,first.meta.expires_at_ns)
        self.assertFalse(self.rig.engine.lease_id)
        self.assertEqual(first.meta.expires_at_ns, round(motion.stamp*1e9)+300_000_000)
        for _ in range(10): self.d.brake_before_source_expiry(self.rig.engine)
        self.assertEqual(self.d.publishers['arms'].publish.call_count, 2)

    def source_motion(self, command_id='move', *, age=0., valid_for=.4):
        from bindu_contracts.contracts import Motion
        engine = self.rig.engine
        groups = ('waist', 'left_arm', 'right_arm')
        if not engine.lease_id:
            engine.acquire('test', groups, 2., self.rig.now)
        names = tuple(n for group in groups for n in GROUPS[group])
        return Motion(command_id, engine.lease_id, engine.epoch, self.rig.profile.digest,
            'test', '', 'joint_target', '', self.rig.now-age, valid_for, names=names,
            positions=tuple(self.target[n] for n in names), schema_version=2,
            resource_groups=groups, runtime_mode='real')

    def test_old_layer_state_preserves_newer_health_without_renewing_its_age(self):
        from copy import deepcopy
        old = deepcopy(self.d.health.states['servo'][1])
        self.rig.now += .01; self.rig.steady += .01
        current = deepcopy(old); current.header = observation(self.rig.now).header
        self.d.receive_state('servo', '/bindu/position/servo_state', 'head', current)
        received = self.d.health.received['servo']
        for frame in (current, old):
            self.rig.now += .01; self.rig.steady += .01
            self.d.receive_state('servo', '/bindu/position/servo_state', 'head', frame)
            self.assertIs(self.d.health.states['servo'][1], current)
            self.assertEqual(self.d.health.received['servo'], received)
        # Even an older queued frame arriving after the original layer timeout
        # cannot refresh current's timestamp or local receipt age.
        self.rig.now += .21; self.rig.steady += .21
        self.d.receive_state('servo', '/bindu/position/servo_state', 'head', old)
        self.assertIs(self.d.health.states['servo'][1], current)
        self.assertGreater(self.rig.steady-received, .2)
        self.assertEqual(self.d.health.received['servo'], received)

    def test_old_conflicting_layer_identity_or_malformed_stamp_still_invalidates(self):
        from copy import deepcopy
        for changes in (dict(instance_id=99), dict(wire_version=3), dict(reference_mode=0),
                        dict(control_source='impedance'), dict(feedback_boot_id=99),
                        dict(stop_policy='unconfirmed'), dict(header=S(stamp=S(sec=10, nanosec=-1))),
                        dict(header=S(stamp=S(sec=0, nanosec=0)))):
            with self.subTest(changes=changes):
                self.setUp()
                old = deepcopy(self.d.health.states['servo'][1])
                for key, value in changes.items(): setattr(old, key, value)
                self.d.receive_state('servo', '/bindu/position/servo_state', 'head', old)
                self.assertNotIn('servo', self.d.health.states)

    def test_layer_state_bad_clock_cannot_be_accepted_as_old_healthy_telemetry(self):
        current = self.d.health.states['servo'][1]
        for now in (float('nan'), float('inf'), 9.):
            with self.subTest(now=now):
                self.rig.now = now
                self.d.receive_state('servo', '/bindu/position/servo_state', 'head', current)
                self.assertNotIn('servo', self.d.health.states)

    def test_effective_source_deadline_rejects_before_acceptance_without_stop_footprint(self):
        from bindu_execution.executor import Rejected
        for age, valid_for in ((.3, .4), (.31, .4), (.1, .1)):
            with self.subTest(age=age, valid_for=valid_for):
                self.setUp(); self.production_readiness()
                motion = self.source_motion(age=age, valid_for=valid_for)
                with self.assertRaisesRegex(Rejected, 'SOURCE_COMMAND_EXPIRED'):
                    self.rig.engine.submit(motion, self.rig.now)
                self.assertIsNone(self.rig.engine.active)
                self.assertIsNone(self.rig.io.source_motion)
                self.assertFalse(self.rig.io.targets)
                self.assertFalse(self.d.sent)
                self.assertEqual(self.d.envelopes.sequence, 0)
                self.rig.engine.halt(self.rig.now, revoke=True)
                self.assertFalse(self.rig.engine.stop_fault)
                self.d.publishers['arms'].publish.assert_not_called()

    def test_expiry_between_acceptance_and_first_tick_has_no_phantom_stop(self):
        self.production_readiness()
        motion = self.source_motion(age=.299)
        self.rig.engine.submit(motion, self.rig.now)
        self.update_target_feedback()
        self.rig.engine.tick(self.rig.now)
        self.d.publishers['arms'].publish.assert_not_called()
        self.assertFalse(self.rig.io.targets)
        self.assertFalse(self.rig.io.stop_targets)
        self.assertFalse(self.rig.engine.stop_fault)
        self.assertIsNone(self.rig.engine.stopping)
        # Recovery requires a fresh explicit motion and new ownership; no old
        # candidate, source time or TTL is revived.
        fresh = self.source_motion('fresh-after-unsent')
        self.rig.engine.submit(fresh, self.rig.now); self.rig.engine.tick(self.rig.now)
        packet = self.d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(packet.meta.operation, 1)
        self.assertEqual(packet.meta.source_command_id, fresh.command_id)
        self.assertEqual(packet.meta.expires_at_ns-packet.meta.source_stamp_ns, 300_000_000)

    def test_unsent_expired_candidate_preserves_prior_real_send_for_pause(self):
        self.production_readiness()
        first = self.source_motion('first', age=.2995)
        self.rig.engine.submit(first, self.rig.now); self.rig.engine.tick(self.rig.now)
        self.d.receive_receipt(self.receipt(self.d.publishers['arms'].publish.call_args.args[0]))
        previous = dict(self.rig.io.targets)
        for name in self.target: self.target[name] += .001
        late = self.source_motion('late-next', age=.299)
        self.rig.engine.submit(late, self.rig.now)
        self.update_target_feedback(); self.rig.engine.tick(self.rig.now)
        self.assertEqual(self.d.publishers['arms'].publish.call_count, 2)
        self.assertEqual(self.d.publishers['arms'].publish.call_args.args[0].meta.operation, 2)
        self.assertEqual(self.rig.io.stop_targets, previous)
        self.assertTrue(self.rig.engine.stopping)
        self.assertFalse(self.rig.engine.stop_fault)

    def test_transport_call_exception_retains_unknown_delivery_stop_footprint(self):
        self.production_readiness()
        publisher = self.d.publishers['arms'].publish
        publisher.side_effect = [RuntimeError('delivery_unknown'), None]
        motion = self.source_motion()
        self.rig.engine.submit(motion, self.rig.now); self.rig.engine.tick(self.rig.now)
        self.assertEqual(publisher.call_count, 2)  # Unknown target followed by PAUSE.
        self.assertEqual([call.args[0].meta.operation for call in publisher.call_args_list], [1, 2])
        self.assertEqual(set(self.rig.io.stop_targets), set(CONTROL_NAMES))
        self.assertTrue(self.rig.engine.stopping)
        self.assertIsNone(self.rig.io.confirmed_stop_targets())
        self.assertFalse(self.rig.engine.lease_id)

    def test_old_wire_three_cannot_pass_target_stream_readiness(self):
        self.d.health.require(CONTROL_GROUPS)
        for layer in ('upper', 'middle', 'servo'):
            state = self.d.health.states[layer][1]
            state.wire_version = 3
            with self.assertRaisesRegex(RuntimeError, 'PROTOCOL_CONFLICT:'+layer):
                self.d.health.require(CONTROL_GROUPS)
            state.wire_version = 5

    def test_runtime_declares_no_lead_or_ack_gate_and_readonly_recipe_has_no_publisher(self):
        from snowbot_hardware.position_session import PositionSessionDevices
        modules = {'rcl_interfaces': S(), 'rcl_interfaces.msg': S(ParameterDescriptor=lambda **kw: S(**kw)),
            'snowbot_ctl_msgs': S(), 'snowbot_ctl_msgs.msg': S(PositionObservation=object,
                PositionSessionCommand=object, PositionSessionState=object),
            'std_msgs': S(), 'std_msgs.msg': S(Float64MultiArray=object),
            'rclpy': S(), 'rclpy.qos': S(qos_profile_sensor_data=S(depth=5))}
        parameters = {}; descriptors = {}
        def declare(key, default, descriptor):
            descriptors[key] = descriptor; parameters.setdefault(key, default)
            return S(value=parameters[key])
        node = S(runtime_mode='real', snowbot_execution_mode='position', snowbot_reference_mode=2,
            now=lambda: 10., declare_parameter=declare, get_parameter=lambda key: S(value=parameters[key]),
            create_subscription=Mock(), create_publisher=Mock(), create_service=Mock())
        modules['std_srvs.srv']=S(Trigger=object)
        with patch.dict('sys.modules', modules):
            created = PositionSessionDevices(node, self.rig.profile)
        qos=node.create_subscription.call_args_list[0].args[3]
        self.assertEqual(qos.depth,1)
        self.assertEqual(modules['rclpy.qos'].qos_profile_sensor_data.depth,5)
        self.assertIs(parameters['command_enabled'],False)
        self.assertTrue(descriptors['command_enabled'].read_only)
        self.assertFalse(created.io.command_enabled)
        self.assertEqual(created.startup_hold_state,'disabled')
        self.assertFalse(created.heartbeat_running)
        self.assertEqual(created.envelopes.pause_valid_for, 2.)
        created.new_session()
        self.assertEqual(created.envelopes.pause_valid_for, 2.)
        node.create_publisher.assert_not_called()
        self.assertNotIn('trajectory_lead_ms', descriptors)
        self.assertNotIn('trajectory_fast_receipt', descriptors)
        self.assertIn('/bindu/position/servo_receipt', [call.args[1] for call in node.create_subscription.call_args_list])
        self.assertEqual(created.trajectory_limits, dict(trajectory_max_velocity=.25, trajectory_max_acceleration=.5))
        for key in created.trajectory_limits:
            self.assertTrue(descriptors[key].read_only)
        parameters.update(trajectory_max_velocity=.4, trajectory_max_acceleration=.8)
        with patch.dict('sys.modules', modules):
            configured = PositionSessionDevices(node, self.rig.profile)
            self.assertEqual(configured.trajectory_limits, dict(trajectory_max_velocity=.4, trajectory_max_acceleration=.8))
            for key in configured.trajectory_limits:
                for invalid in (0., -1., float('nan'), float('inf'), True):
                    parameters[key] = invalid
                    with self.subTest(key=key, invalid=invalid), self.assertRaisesRegex(ValueError, 'TARGET_DYNAMIC_LIMIT'):
                        PositionSessionDevices(node, self.rig.profile)
                parameters[key] = configured.trajectory_limits[key]
        node.create_publisher.assert_not_called()

    def test_configurable_limits_preserve_each_axis_profile_and_pause_deadline(self):
        capped = CONTROL_NAMES[0]
        self.d.io.profile = replace(self.rig.profile, max_speed=.6, max_acceleration=1.2,
            joint_dynamics={capped: dict(velocity=.18, acceleration=.3, jerk=100.)})
        self.d.trajectory_limits = dict(trajectory_max_velocity=.4, trajectory_max_acceleration=.8)
        self.d.new_session()
        first = self.send()
        pause = self.send(pause=True)
        for packet in (first, pause):
            self.assertEqual(packet.trajectory.max_velocity, [.18]+[.4]*15)
            self.assertEqual(packet.trajectory.max_acceleration, [.3]+[.8]*15)
            self.assertEqual(packet.trajectory.max_jerk, [100.]+[162.]*15)
        self.assertEqual(first.meta.expires_at_ns-first.meta.source_stamp_ns, 200_000_000)
        self.assertEqual(pause.meta.expires_at_ns-pause.meta.source_stamp_ns, 2_000_000_000)

    def test_candidate_deployment_keeps_group_limits_and_geometry_through_target_and_pause_wire(self):
        from bindu_contracts.profile import Profile
        from bindu_runtime.teleop_deployment import load_deployment
        from tests.support.paths import ROOT
        runtime=ROOT/'src/integration/bindu_runtime'
        _,args=load_deployment(package_share("snowbot_bringup")/'config/deployments/snowbot_vr_real.json',runtime)
        robot_path=Path(args['robot_config'])
        robot=json.loads(robot_path.read_text())
        self.d.io.profile=Profile.load(robot_path.parent/robot['profile'])
        self.d.trajectory_limits={key:float(args[key]) for key in
                                 ('trajectory_max_velocity','trajectory_max_acceleration')}
        tool=json.loads((robot_path.parent/robot['teleop']['tool']).read_text())
        self.assertIs(tool['rigid_reference_prediction'],True)
        self.assertEqual(args['command_enabled'],'false')
        for rigid,pause in ((False,False),(True,False),(True,True)):
            self.d.new_session()
            packet=self.send(pause=pause,rigid_reference=rigid)
            self.assertEqual(list(packet.trajectory.max_velocity),[1.0]*14+[.375]*2)
            self.assertEqual(list(packet.trajectory.max_acceleration),[4.5]*14+[2.25]*2)
            self.assertEqual(list(packet.trajectory.max_jerk),[162.]*16)
            self.assertEqual(packet.trajectory.version,3 if rigid and not pause else 2)
            self.assertEqual(packet.meta.profile_hash,self.d.io.profile.digest)
            self.assertEqual(packet.meta.expires_at_ns-packet.meta.source_stamp_ns,
                             2_000_000_000 if pause else 200_000_000)

    def test_targets_keep_joint_order_identity_original_expiry_and_no_curve(self):
        first = self.send(stamp=self.rig.now-.01, valid_for=.15)
        self.assertEqual(first.meta.reference_mode, 2)
        self.assertEqual(first.meta.operation, 1)
        self.assertEqual((first.meta.source_command_id, first.meta.lease_id, first.meta.epoch), ('move','lease',3))
        self.assertEqual(first.meta.profile_hash, self.rig.profile.digest)
        self.assertEqual(first.meta.source_stamp_ns, round((self.rig.now-.01)*1e9))
        self.assertEqual(first.meta.expires_at_ns-first.meta.source_stamp_ns, 150_000_000)
        self.assertEqual(first.command.l_arm_position, [self.target[n] for n in GROUPS['left_arm']])
        self.assertEqual(first.command.r_arm_position, [self.target[n] for n in GROUPS['right_arm']])
        self.assertEqual(first.waist_position, [self.target[n] for n in GROUPS['waist']])
        self.assertEqual((first.trajectory.version, first.trajectory.segment_id), (2, 1))
        self.assertEqual(first.trajectory.effective_at_ns, 0)
        for field in ('piece_start','duration','source_duration','source_elapsed','control_count','controls'):
            self.assertEqual(getattr(first.trajectory, field), [])
        for field, getter, cap in (('max_velocity',self.rig.profile.speed,.25),
                ('max_acceleration',self.rig.profile.acceleration,.5),('max_jerk',self.rig.profile.jerk,162.)):
            self.assertEqual(getattr(first.trajectory,field), [min(cap,getter(n)) for n in CONTROL_NAMES])
        # Only the first takeover has an acceptance barrier. Repeated ticks
        # never issue a derived frame or renew its original source deadline.
        self.send(stamp=self.rig.now-.01, valid_for=.15)
        self.assertEqual(self.d.publishers['arms'].publish.call_count, 1)
        self.send('waiting-latest')
        self.assertEqual(self.d.publishers['arms'].publish.call_count,1)
        self.assertEqual(self.d.envelopes.sequence,1)
        self.assertEqual(self.d.takeover_pending[0]['expires_at_ns'],first.meta.expires_at_ns)
        self.d.receive_receipt(self.receipt(first))
        second = self.send('next')
        self.assertEqual((second.meta.operation, second.meta.sequence), (0,2))
        self.assertFalse(hasattr(self.d,'pending_trajectory'))
        self.assertFalse(hasattr(self.d,'trajectory'))

    def test_rigid_reference_is_explicit_recipe_and_pause_keeps_original_stop(self):
        self.d.new_session()
        first=self.send(rigid_reference=True)
        self.assertEqual(first.trajectory.version,3)
        self.assertEqual(first.meta.reference_mode,2)
        self.assertEqual(first.meta.expires_at_ns-first.meta.source_stamp_ns,200_000_000)
        self.assertEqual(first.trajectory.controls,[])
        self.d.receive_receipt(self.receipt(first))
        pause=self.send(pause=True,rigid_reference=True)
        self.assertEqual((pause.meta.operation,pause.trajectory.version),(2,2))
        self.assertEqual(pause.meta.expires_at_ns-pause.meta.source_stamp_ns,2_000_000_000)

    def test_rigid_reference_scope_is_validated_before_device_acceptance(self):
        from bindu_contracts.contracts import Motion
        lease,epoch=self.rig.engine.acquire('rigid',CONTROL_GROUPS,2.,self.rig.now)
        names=tuple(n for group in CONTROL_GROUPS for n in self.rig.profile.groups[group])
        motion=Motion('rigid',lease,epoch,self.rig.profile.digest,'test','','joint_target','',
            self.rig.now,.2,names=names,positions=tuple(self.target[n] for n in names),
            schema_version=2,resource_groups=CONTROL_GROUPS,runtime_mode='real',rigid_reference=True)
        with self.assertRaisesRegex(Exception,'RIGID_REFERENCE_SCOPE'):
            self.rig.engine.submit(replace(motion,rigid_reference=1),self.rig.now)
        self.rig.io.supports_rigid_reference=False
        with self.assertRaisesRegex(Exception,'RIGID_REFERENCE_UNSUPPORTED'):
            self.rig.engine.submit(motion,self.rig.now)
        self.rig.io.supports_rigid_reference=True
        self.rig.engine.submit(motion,self.rig.now)
        self.assertTrue(self.rig.io.source_motion.rigid_reference)

    def test_pause_preempts_without_receipt_and_retains_identity_after_motion_clear(self):
        first = self.send()
        self.d.io.source_motion = None
        self.d.io.sending_hold = True
        self.d.send(CONTROL_GROUPS, self.target)
        self.d.io.sending_hold = False
        pause = self.d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(pause.meta.operation,2)
        self.assertEqual(pause.meta.sequence,2)
        self.assertEqual(pause.meta.source_command_id,first.meta.source_command_id)
        self.assertEqual(pause.meta.lease_id,first.meta.lease_id)
        self.assertEqual(pause.meta.source_stamp_ns, round(self.rig.now*1e9))
        self.assertIsNone(self.d.takeover_pending)
        self.assertEqual(self.d.io.pause_request[:2],(pause.meta.session,2))

    def test_pause_from_executor_never_waits_and_keeps_original_stop_budget(self):
        self.rig.enable_rest_window()
        hold = self.rig.start_stop()
        self.assertEqual(self.d.publishers['arms'].publish.call_count,2)
        pause = self.d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(pause.meta.operation,2)
        budget=(self.rig.engine.stop_deadline,self.rig.engine.stop_steady_deadline)
        self.rig.io.stop();self.rig.io.stop()
        self.assertEqual(self.d.publishers['arms'].publish.call_count,2)
        self.assertEqual(budget,(self.rig.engine.stop_deadline,self.rig.engine.stop_steady_deadline))
        self.assertTrue(self.rig.engine.stopping)
        self.assertIsNone(self.rig.io.confirmed_stop_targets())
        for _ in range(24):
            self.update_target_feedback(hold=hold,reference_mode=2,segment_id=2,
                trajectory_phase=4,hold_basis=0,reference_valid=True,output_valid=True)
            self.rig.engine.tick(self.rig.now)
        self.assertIsNone(self.rig.engine.stopping)

    def test_applied_requires_actual_valid_servo_sample_and_is_distinct_from_accepted(self):
        packet=self.send()
        self.d.receive_receipt(self.receipt(packet))
        self.assertEqual(self.d.node.event.call_args.args[0],'TARGET_ACCEPTED')
        self.rig.now+=.001;self.rig.steady+=.001
        stamp=self.rig.now
        msg=observation(stamp,pdo_sample=self.d.health.sample+1,session_selected=True,
            session=packet.meta.session,command_sequence=packet.meta.sequence,segment_id=packet.meta.sequence,
            reference_mode=2,trajectory_phase=1,reference_valid=True,output_valid=True,sample_time_ns=round(stamp*1e9),
            reference_position=[self.target[n] for n in CONTROL_NAMES],
            reference_speed=[0.]*16,reference_acceleration=[0.]*16)
        self.d.receive(msg)
        self.assertEqual(self.d.node.event.call_args.args[0],'TARGET_ACCEPTED')
        self.rig.now+=.001;self.rig.steady+=.001
        msg.header=observation(self.rig.now).header
        msg.pdo_sample+=1;msg.sample_time_ns=round(self.rig.now*1e9);msg.output_valid=True
        msg.trajectory_phase=2
        self.d.receive(msg)
        self.assertEqual(self.d.node.event.call_args.args[0],'TARGET_APPLIED')
        count=self.d.node.event.call_count
        self.d.receive(msg)
        self.assertEqual(self.d.node.event.call_count,count)
        self.assertEqual(self.d.io.servo_reference[1],self.target)

    def test_invalid_source_or_target_is_rejected_before_sequence_or_publish(self):
        for changes in ({'lease_id':''},{'command_id':'x'*129},{'epoch':True},{'epoch':-1}):
            with self.subTest(changes=changes), self.assertRaisesRegex(RuntimeError,'IDENTITY'):
                self.send(**changes)
            self.assertEqual(self.d.envelopes.sequence,0)
        self.target['body_joint_1'] = float('nan')
        with self.assertRaisesRegex(RuntimeError,'TARGET_LAYOUT_OR_LIMIT'): self.send()
        self.assertEqual(self.d.envelopes.sequence,0)
        self.d.publishers['arms'].publish.assert_not_called()

    def test_receipt_is_telemetry_and_old_rejection_cannot_latch_newer_source(self):
        first = self.send('first');self.d.receive_receipt(self.receipt(first))
        second = self.send('second')
        self.d.receive_receipt(self.receipt(first,accepted=False))
        self.assertEqual(self.d.target_fault,'')
        self.d.receive_receipt(self.receipt(second))
        self.assertEqual(self.d.node.event.call_args.args[0],'TARGET_ACCEPTED')
        self.assertFalse(hasattr(self.d.io,'servo_reference') and self.d.io.servo_reference)
        self.d.receive_receipt(self.receipt(second,accepted=False))
        self.assertEqual(self.d.target_fault,'SNOWBOT_TARGET_REJECTED:target_invalid')
        with self.assertRaisesRegex(RuntimeError,'TARGET_REJECTED'): self.send('third')
        # A reject never prevents a separately identified PAUSE request.
        self.assertEqual(self.send('second',pause=True).meta.operation,2)

    def test_receipts_validate_protocol_owner_boot_time_and_endpoint(self):
        packet = self.send()
        events = self.d.node.event.call_count
        for changes in ({'wire_version':2},{'wire_version':3},{'wire_version':4},{'reference_mode':1},{'instance_id':13},
                {'feedback_boot_id':9},{'header':S(stamp=S(sec=9,nanosec=1_010_000_000))},
                {'header':S(stamp=S(sec=-1,nanosec=0))},{'session':0},{'sequence':99},{'layer':'middle'},
                {'header':observation(self.rig.now+.001).header},
                {'header':observation(self.rig.now-.001).header}):
            self.d.receive_receipt(self.receipt(packet,**changes))
            self.assertEqual(self.d.node.event.call_count,events)
        self.d.identity.side_effect = RuntimeError('wrong publisher')
        self.d.receive_receipt(self.receipt(packet))
        self.assertEqual(self.d.node.event.call_count,events)

    def test_restart_requires_reconnect_bounded_bookkeeping_and_old_results_do_not_restore(self):
        first=self.send('first');self.d.receive_receipt(self.receipt(first))
        for i in range(40): self.send(str(i))
        self.assertEqual(len(self.d.sent),32)
        servo = self.d.health.states['servo'][1]
        servo.rejected_session,servo.rejected_sequence,servo.reason = self.d.envelopes.session,1,'old_reject'
        self.d.advance_session()
        self.assertEqual(self.d.target_fault,'')
        servo.instance_id += 1
        self.d.advance_session()
        self.assertIn('RECONNECT_REQUIRED',self.d.target_fault)
        servo.instance_id -= 1
        self.d.advance_session()
        self.assertIn('RECONNECT_REQUIRED',self.d.target_fault)
        self.d.new_session()
        self.assertEqual(self.d.sent,{})
        self.assertIsNone(self.d.source_identity)
        self.assertEqual(self.d.target_fault,'')

    def test_takeover_only_handshake_preserves_source_expiry_and_latest_target(self):
        first=self.send('takeover')
        deadline=first.meta.expires_at_ns
        for number in range(8):
            self.rig.now+=.01
            # Keep valid feedback flowing, as a real input loop would.
            self.rig.update()
            for _,state in self.d.health.states.values():state.wire_version=5
            self.d.health.states['servo'][1].reference_mode=2
            self.send('queued-'+str(number))
        self.assertEqual(self.d.envelopes.sequence,1)
        self.assertEqual(self.d.takeover_pending[0]['expires_at_ns'],deadline)
        self.assertEqual(self.d.publishers['arms'].publish.call_count,1)
        self.d.receive_receipt(self.receipt(first))
        self.assertIsNone(self.d.takeover_pending)
        self.assertEqual(self.d.publishers['arms'].publish.call_count,1)
        latest=self.send('queued-7')
        self.assertEqual(latest.meta.source_command_id,'queued-7')
        self.assertEqual((latest.meta.sequence,latest.meta.operation),(2,0))
        self.send('next-without-ack')
        self.assertEqual(self.d.envelopes.sequence,3)

    def held_feedback(self, **changes):
        data = dict(session_selected=True, session=17, command_sequence=2,
            session_reason=1, hold_valid=True, reference_mode=2, trajectory_phase=4,
            reference_valid=True, output_valid=True)
        data.update(changes)
        self.update_target_feedback(**data)

    def test_old_hold_is_allowed_until_new_takeover_pdo_even_after_ack(self):
        self.held_feedback()
        self.d.new_session()
        first = self.send('follow')
        # Reproduce the field ordering: old held PDO at +20 ms, ACK at
        # +28 ms, then the first new-session PDO. ACK is not applied feedback.
        for _ in range(4): self.held_feedback()
        self.d.check_health(CONTROL_GROUPS)
        self.d.receive_receipt(self.receipt(first))
        self.assertIsNone(self.d.takeover_pending)
        self.held_feedback()
        self.d.check_health(CONTROL_GROUPS)
        self.assertEqual(self.d.applied_sequence, 0)
        self.assertIsNone(self.d.io.servo_reference)
        self.rig.now += .001; self.rig.steady += .001; self.rig.sample += 1
        msg = observation(self.rig.now, pdo_sample=self.d.health.sample+1,
            session_selected=True, session=first.meta.session, command_sequence=1,
            segment_id=1, reference_mode=2, trajectory_phase=2,
            reference_valid=True, output_valid=True, sample_time_ns=round(self.rig.now*1e9),
            reference_position=list(self.target.values()), reference_speed=[0.]*16,
            reference_acceleration=[0.]*16)
        self.d.receive(msg)
        self.d.check_health(CONTROL_GROUPS)
        self.assertEqual(self.d.applied_sequence, 1)
        self.held_feedback()
        with self.assertRaisesRegex(RuntimeError, 'SESSION_OWNER_CONFLICT'):
            self.d.check_health(CONTROL_GROUPS)

    def test_handoff_does_not_accept_other_owners_or_unhealthy_hold(self):
        self.held_feedback()
        self.d.new_session()
        self.send()
        for changes in (dict(session=18), dict(command_sequence=3),
                dict(session_reason=0), dict(hold_valid=False), dict(trajectory_phase=3),
                dict(reference_valid=False), dict(output_valid=False)):
            with self.subTest(changes=changes):
                self.held_feedback(**changes)
                with self.assertRaisesRegex(RuntimeError, 'SESSION_OWNER_CONFLICT'):
                    self.d.check_health(CONTROL_GROUPS)
        self.held_feedback(status=[0x2f]*20)
        with self.assertRaisesRegex(RuntimeError, 'DRIVE_FAULT'):
            self.d.check_health(CONTROL_GROUPS)
        self.held_feedback()
        self.held_feedback()
        self.d.health.states['servo'][1].instance_id += 1
        with self.assertRaisesRegex(RuntimeError, 'SESSION_OWNER_CONFLICT'):
            self.d.check_health(CONTROL_GROUPS)
        self.held_feedback()
        self.rig.steady += .041
        with self.assertRaisesRegex(RuntimeError, 'PDO_NOT_ADVANCING'):
            self.d.check_health(CONTROL_GROUPS)

    def test_handoff_deadline_is_original_takeover_even_after_ack_and_new_targets(self):
        for acknowledged in (False, True):
            with self.subTest(acknowledged=acknowledged):
                self.setUp()
                self.held_feedback()
                self.d.new_session()
                first = self.send()
                if acknowledged: self.d.receive_receipt(self.receipt(first))
                for i in range(39):
                    self.held_feedback()
                    self.send('next-'+str(i))
                    self.d.check_health(CONTROL_GROUPS)
                self.rig.now = first.meta.expires_at_ns*1e-9
                self.held_feedback()
                with self.assertRaisesRegex(RuntimeError, 'SESSION_OWNER_CONFLICT'):
                    self.d.check_health(CONTROL_GROUPS)

    def test_takeover_handoff_does_not_confirm_pause_with_old_hold(self):
        self.held_feedback()
        self.d.new_session()
        self.send()
        pause = self.send(pause=True)
        self.held_feedback()
        self.assertIsNone(self.d.io.confirmed_stop_targets())
        hold = list(self.target.values())
        self.update_target_feedback(hold=hold, reference_mode=2, segment_id=pause.meta.sequence,
            trajectory_phase=4, hold_basis=0, reference_valid=True, output_valid=True)
        self.assertEqual(self.d.io.confirmed_stop_targets(), self.target)

    def test_takeover_late_ack_rejection_and_restart_cannot_unblock_ordinary_stream(self):
        first=self.send()
        self.rig.now+=.201
        self.d.receive_receipt(self.receipt(first,header=observation(first.meta.source_stamp_ns*1e-9).header))
        self.assertIn('TAKEOVER_UNCONFIRMED_RECONNECT_REQUIRED',self.d.target_fault)
        self.assertIsNotNone(self.d.takeover_pending)
        self.d.new_session()
        self.rig.update()
        for _,state in self.d.health.states.values():state.wire_version=5
        self.d.health.states['servo'][1].reference_mode=2
        first=self.send('new')
        self.d.receive_receipt(self.receipt(first,accepted=False))
        self.assertIn('TARGET_REJECTED',self.d.target_fault)
        self.d.receive_receipt(self.receipt(first))
        self.assertIsNotNone(self.d.takeover_pending)
        self.d.new_session()
        first=self.send('restart')
        self.d.health.states['servo'][1].instance_id+=1
        self.d.receive_receipt(self.receipt(first))
        self.assertIn('INSTANCE_CHANGED_RECONNECT_REQUIRED',self.d.target_fault)
        self.assertIsNotNone(self.d.takeover_pending)

    def test_takeover_selected_state_releases_only_matching_source_and_instance(self):
        first=self.send()
        state=self.d.health.states['servo'][1]
        state.selected=True;state.session=first.meta.session;state.sequence=2
        state.feedback_boot_id=self.d.health.boot
        self.d.advance_session();self.assertIsNotNone(self.d.takeover_pending)
        state.sequence=first.meta.sequence
        for header,boot in ((S(stamp=S(sec=9,nanosec=1_010_000_000)),self.d.health.boot),
                (state.header,self.d.health.boot+1)):
            original=state.header;state.header=header;state.feedback_boot_id=boot
            self.d.advance_session();self.assertIsNotNone(self.d.takeover_pending)
            state.header=original
        state.feedback_boot_id=self.d.health.boot
        self.d.advance_session();self.assertIsNone(self.d.takeover_pending)
        self.send('ordinary')
        self.assertEqual(self.d.envelopes.sequence,2)

    def test_old_curve_protocol_and_target_impedance_combination_rejected(self):
        for mode in (0,1):
            self.d.health.states['servo'][1].reference_mode=mode
            with self.assertRaisesRegex(RuntimeError,'PROTOCOL_CONFLICT'): self.d.check_health(CONTROL_GROUPS)
        with self.assertRaisesRegex(ValueError,'PURE_POSITION'):
            SessionHealth(lambda:10.,execution_mode='impedance',reference_mode=2)
        with self.assertRaisesRegex(ValueError,'REFERENCE_MODE'):
            SessionHealth(lambda:10.,reference_mode=1)

    def test_pause_basis_uses_servo_fixed_reference_and_never_claims_measured_rest(self):
        self.send()
        self.send(pause=True)
        hold = [self.target[n] for n in CONTROL_NAMES]
        self.rig.update(hold=hold, reference_mode=2, segment_id=2, trajectory_phase=3, hold_basis=0,reference_valid=True,output_valid=True)
        for _,state in self.d.health.states.values():state.wire_version=5
        self.d.health.states['servo'][1].reference_mode=2
        self.assertIsNone(self.d.health.confirmed_hold(self.d.io.pause_request))
        self.d.health.observation.trajectory_phase=4
        self.d.health.observation.hold_basis=1
        self.assertIsNone(self.d.health.confirmed_hold(self.d.io.pause_request))
        self.d.health.observation.hold_basis=0
        self.assertEqual(self.d.health.confirmed_hold(self.d.io.pause_request),dict(zip(CONTROL_NAMES,hold)))
        # Normal PAUSE confirmation cannot be fabricated by expiry fallback,
        # revocation, fault, or invalid sampled output, even at measured rest.
        for changes in ({'session_reason':2},{'session_reason':3},{'session_reason':7},
                        {'output_valid':False},{'reference_valid':False}):
            previous={key:getattr(self.d.health.observation,key) for key in changes}
            for key,value in changes.items():setattr(self.d.health.observation,key,value)
            self.assertIsNone(self.d.health.confirmed_hold(self.d.io.pause_request))
            for key,value in previous.items():setattr(self.d.health.observation,key,value)


    def test_upstream_goal_waits_for_actual_reference_then_expires_without_fallback(self):
        from snowbot_hardware.position_session import sampled_nominal_reference
        io,rig = self.d.io,self.rig
        goal = {n: rig.q[n]+.01 for n in CONTROL_NAMES}
        io.targets = dict(goal); io.reference = dict(goal)
        self.assertEqual(io.reference_positions, {})
        state = S(lease_active=True,lease_epoch=3,reference=S(header=observation(rig.now).header,name=[],position=[]))
        self.assertIsNone(sampled_nominal_reference(state, CONTROL_NAMES, 3, rig.now, .1,
                          wait_until=1.1, steady_now=1.))
        with self.assertRaisesRegex(RuntimeError, 'UNAVAILABLE'):
            sampled_nominal_reference(state,CONTROL_NAMES,3,rig.now,.1,wait_until=1.1,steady_now=1.11)
        actual = {n: rig.q[n] for n in CONTROL_NAMES}; sampled=rig.now-.005
        io.servo_reference=(sampled,actual,dict.fromkeys(actual,.03),dict.fromkeys(actual,.1))
        self.assertEqual(io.reference_positions,actual)
        self.assertNotEqual(io.reference_positions,goal)
        state.reference=S(header=observation(sampled).header,name=list(actual),position=list(actual.values()))
        value,at=sampled_nominal_reference(state,CONTROL_NAMES[::-1],3,rig.now,.1)
        self.assertEqual(value,tuple(actual[n] for n in CONTROL_NAMES[::-1]));self.assertAlmostEqual(at,sampled)
        with self.assertRaisesRegex(RuntimeError,'OWNER'):
            sampled_nominal_reference(state,CONTROL_NAMES,4,rig.now,.1,wait_until=2.,steady_now=1.)
        rig.now += .03
        self.assertEqual(io.reference_positions,{})


if __name__ == '__main__':
    unittest.main()


class DelayedFeedbackPauseTests(unittest.TestCase):
    """Exercise stop-only admission with production checks and memory endpoints."""

    def setUp(self):
        from snowbot_hardware.position_session import PositionSessionDevices
        self.fixture = TargetWireTests()
        self.fixture.setUp()
        self.d = self.fixture.d
        self.d.envelopes = SourceEnvelope(11, self.d.node.now, target_stream=True)
        self.d.node.count_publishers = lambda topic: int(topic == '/bindu/position/command')
        self.d.receivers.update(arms='upper', middle='middle')
        self.d.check_ready = lambda groups: PositionSessionDevices.check_ready(self.d, groups)
        self.d.io.check_ready = self.d.check_ready

    def delay_following(self):
        packet = self.fixture.send()
        self.d.io.targets = dict(self.fixture.target)
        self.d.io.invalid = 'SNOWBOT_PDO_STALE'
        self.fixture.rig.now += .06
        self.fixture.rig.steady += .06
        return packet

    def test_delayed_feedback_blocks_following_but_sends_one_identified_pause(self):
        original = self.delay_following()
        with self.assertRaises(RuntimeError):
            self.fixture.send('another-target')
        # Revoking the motion must not manufacture a new source identity.
        self.d.io.source_motion = None
        self.d.io.stop()
        pause = self.d.publishers['arms'].publish.call_args.args[0]
        self.assertEqual(pause.meta.operation, 2)
        for field in ('session', 'feedback_boot_id', 'middle_instance_id',
                      'source_command_id', 'lease_id', 'epoch', 'profile_hash'):
            self.assertEqual(getattr(pause.meta, field), getattr(original.meta, field))
        self.assertEqual(pause.meta.sequence, original.meta.sequence + 1)
        self.assertEqual(pause.meta.expires_at_ns - pause.meta.source_stamp_ns, 2_000_000_000)
        self.assertFalse(self.d.io.stop_failures)
        self.assertIsNone(self.d.io.confirmed_stop_targets())
        count = self.d.publishers['arms'].publish.call_count
        request = self.d.io.pause_request
        for _ in range(5):
            self.fixture.rig.now += .02
            self.d.io.stop()
        self.assertEqual(self.d.publishers['arms'].publish.call_count, count)
        self.assertEqual(self.d.io.pause_request, request)

    def test_pause_cannot_cross_boot_or_instance_change_or_missing_authority(self):
        for changed in ('boot', 'servo', 'middle', 'sent'):
            with self.subTest(changed=changed):
                self.setUp()
                self.delay_following()
                if changed == 'boot':
                    self.d.health.boot += 1
                elif changed == 'sent':
                    self.d.sent.clear()
                else:
                    self.d.health.states[changed][1].instance_id += 1
                count = self.d.publishers['arms'].publish.call_count
                self.d.io.stop()
                self.assertEqual(self.d.publishers['arms'].publish.call_count, count)
                self.assertEqual(set(self.d.io.stop_failures), CONTROL_GROUPS)
                self.assertIsNone(self.d.io.confirmed_stop_targets())

    def test_pause_retains_endpoint_and_competing_publisher_checks(self):
        for conflict in ('legacy', 'duplicate', 'endpoint'):
            with self.subTest(conflict=conflict):
                self.setUp()
                self.delay_following()
                if conflict == 'endpoint':
                    self.d.identity.side_effect = RuntimeError('endpoint mismatch')
                else:
                    self.d.node.count_publishers = lambda topic: (
                        2 if conflict == 'duplicate' and topic == '/bindu/position/command'
                        else int(topic == '/bindu/position/command' or
                                 (conflict == 'legacy' and topic == '/joint_position_cmd')))
                count = self.d.publishers['arms'].publish.call_count
                self.d.io.stop()
                self.assertEqual(self.d.publishers['arms'].publish.call_count, count)
                self.assertTrue(self.d.io.stop_failures)

    def test_stop_without_owned_targets_does_not_publish_or_enable(self):
        self.d.io.stop()
        self.d.publishers['arms'].publish.assert_not_called()
