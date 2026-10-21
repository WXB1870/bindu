"""Position-session source identity, independent PDO freshness and resource scope."""
from types import SimpleNamespace as S
import unittest

from bindu_hardware.snowbot_session import ARMS, SessionHealth, SourceEnvelope, decode_observation


def observation(now=10., **changes):
    data = dict(header=S(stamp=S(sec=int(now), nanosec=round((now-int(now))*1e9))),
        boot_id=1, pdo_sample=1, pdo_age=.001, wkc=67, expected_wkc=67,
        valid_mask=0xfffff, position=[.2]*20, speed=[0.]*20, current=[0.]*20,
        current_valid_mask=0xffff, torque_raw=[0]*20, torque_raw_valid_mask=0xf0000,
        status=[0x27]*20, session_reason=0, session_selected=False, session=0)
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
        self.health.require(ARMS)
        self.now += .025
        self.steady += .025
        self.health.observe(observation(self.now, pdo_sample=2))
        with self.assertRaisesRegex(RuntimeError, 'NOT_ADVANCING'):
            self.health.require(ARMS)

    def test_wkc_and_fault_have_separate_failures_and_gripper_is_not_owned(self):
        with self.assertRaisesRegex(ValueError, 'WKC'):
            self.health.observe(observation(wkc=61))
        statuses = [0x27]*20
        statuses[0] |= 8
        self.health.observe(observation(pdo_sample=3, status=statuses))
        self.health.observe(observation(pdo_sample=4, status=statuses))
        with self.assertRaisesRegex(RuntimeError, 'DRIVE_FAULT'):
            self.health.require(ARMS)
        statuses[0] = 0x27
        statuses[7] |= 8
        self.health.observe(observation(pdo_sample=5, status=statuses))
        self.health.observe(observation(pdo_sample=6, status=statuses))
        self.health.require(ARMS)
        with self.assertRaisesRegex(RuntimeError, 'ARMS_ONLY'):
            self.health.require(('left_tool_axis',))

    def test_head_waist_absence_never_becomes_zero_measurement(self):
        msg = observation(pdo_sample=3, valid_mask=0xffff)
        q, _, _ = decode_observation(msg, self.now)
        self.assertNotIn('body_joint_1', q)
        self.assertNotIn('head_joint_1', q)
        self.health.observe(msg)
        self.health.require(ARMS)

    def test_restart_requires_reconnect_and_unconfirmed_policy_stays_blocked(self):
        with self.assertRaisesRegex(ValueError, 'RESTART'):
            self.health.observe(observation(boot_id=2))
        self.health.observe(observation(pdo_sample=3))
        self.health.observe(observation(pdo_sample=4))
        _, servo = self.health.states['servo']
        servo.stop_policy = 'unconfirmed'
        with self.assertRaisesRegex(RuntimeError, 'STOP_POLICY'):
            self.health.require(ARMS)

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


if __name__ == '__main__':
    unittest.main()
