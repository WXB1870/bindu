"""Measured coupling must stay distinct from target following and free axial roll."""
from tests.support.paths import ROOT
import copy
import importlib.util
from pathlib import Path
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location('snowbot_errors', ROOT/'tools/snowbot_errors.py')
errors = importlib.util.module_from_spec(spec)
spec.loader.exec_module(errors)


def pose(x=0., roll=0.):
    t = np.eye(4); t[0, 3] = x
    c, s = np.cos(roll), np.sin(roll)
    t[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return t


class EndEffectorErrorTests(unittest.TestCase):
    def setUp(self):
        self.config = dict(left_to_tool=pose().ravel().tolist(), right_to_tool=pose(-.5).ravel().tolist(),
            right_axial_grasp=dict(axis_in_tool=[0, 0, 1], grip_in_right_flange_m=[0, 0, 0]),
            demo=dict(grip_offset_m=[0, 0, 0]), grasp_position_tolerance_m=.005, grasp_rotation_tolerance_rad=.02)
        self.metric = errors.EndEffectorErrors(self.config, lambda q:(pose(q['left']), pose(q['right'], q['roll'])))
        self.q = dict(left=0., right=.5, roll=0.)
        self.data = dict(valid=True, runtime_mode='real', feedback_stamp=10., q=self.q,
                         pico=dict(stamp=10., teleop_mode='bimanual_tool', phase='following', grasp_applied='rigid'))
        self.capture()
        self.target()

    def capture(self, mode='rigid'):
        self.metric.event('BIMANUAL_CAPTURED', dict(feedback=copy.copy(self.q), grasp_mode=mode), 'task', 9.8)
        self.data['pico']['grasp_applied'] = mode

    def target(self, x=.1, task='task', stamp=9.99):
        self.metric.event('BIMANUAL_QP_RESULT', dict(target=pose(x).ravel().tolist(),
            input_stamp=stamp, teleop_mode='bimanual_tool'), task, stamp)

    def sample(self, now=10.01):
        return self.metric.sample(self.data, now)

    def test_synchronized_hands_can_lag_target_with_zero_coupling_error(self):
        value = self.sample()
        np.testing.assert_allclose(value['left'], [100, 0], atol=1e-9)
        np.testing.assert_allclose(value['right'], [100, 0], atol=1e-9)
        np.testing.assert_allclose(value['coupled'], [0, 0], atol=1e-9)
        self.assertEqual(value['runtime_mode'], 'real')

    def test_measured_right_drift_is_reported_in_mm_independent_of_target(self):
        self.q['right'] += .006
        self.assertAlmostEqual(self.sample()['coupled'][0], 6)
        self.assertEqual(self.sample()['position_limit_mm'], 5)

    def test_rigid_roll_is_an_error_but_axial_roll_is_free(self):
        self.q['roll'] = .5
        self.assertAlmostEqual(self.sample()['coupled'][1], np.degrees(.5))
        self.q['roll'] = 0.
        self.capture('right_axial'); self.target()
        self.q['roll'] = .5
        self.metric.last = None
        value = self.sample()
        self.assertAlmostEqual(value['coupled'][1], 0.)
        self.assertAlmostEqual(value['right'][1], 0.)

    def test_rigid_recapture_retains_current_roll(self):
        self.q['roll'] = .5
        self.capture(); self.target()
        self.assertAlmostEqual(self.sample()['coupled'][1], 0.)

    def test_capture_does_not_zero_existing_calibration_position_error(self):
        self.q['right'] += .004
        self.capture(); self.target()
        self.assertAlmostEqual(self.sample()['coupled'][0], 4.)

    def test_missing_or_stale_target_is_null_not_zero(self):
        self.metric.target = None
        self.assertIsNone(self.sample()['left'])
        self.target(stamp=9.)
        self.metric.last = None
        self.assertIsNone(self.sample()['right'])
        self.assertTrue(self.sample()['valid'])

    def test_target_from_other_task_or_future_is_not_used(self):
        self.metric.target = None
        self.target(task='other')
        self.assertIsNone(self.sample()['left'])
        self.target(stamp=10.02)
        self.metric.last = None
        self.assertIsNone(self.sample(10.03)['left'])

    def test_stale_feedback_mode_change_and_grasp_transition_make_gaps(self):
        self.assertFalse(self.sample(10.3)['valid'])
        self.data['pico']['grasp_applied'] = 'right_axial'
        self.assertFalse(self.sample()['valid'])
        self.data['pico']['grasp_applied'] = 'rigid'
        self.data['pico']['phase'] = 'switching'
        self.assertFalse(self.sample()['valid'])
        self.data['pico']['teleop_mode'] = 'independent_arms'
        self.assertFalse(self.sample()['valid'])
        self.assertIsNone(self.metric.capture)

    def test_fast_target_stream_selects_latest_target_not_after_feedback(self):
        self.target(x=.2,stamp=10.02)
        value=self.sample(10.03)
        np.testing.assert_allclose(value['left'],[100.,0.],atol=1e-9)
        self.assertEqual(value['target_stamp'],9.99)
        self.data['feedback_stamp']=10.12;self.data['pico']['stamp']=10.12
        self.target(x=.3,stamp=10.14)
        value=self.sample(10.15)
        np.testing.assert_allclose(value['left'],[200.,0.],atol=1e-9)
        self.assertEqual(value['target_stamp'],10.02)
        self.capture();self.metric.last=None
        self.assertIsNone(self.sample(10.15)['left'])
        self.assertLessEqual(len(self.metric.targets),64)

    def test_timestamp_reversal_requires_recapture_and_samples_are_bounded_rate(self):
        first = self.sample()
        self.data['feedback_stamp'] += .02
        self.assertIs(self.sample(10.03), first)
        self.data['feedback_stamp'] = 9.99
        self.assertFalse(self.sample()['valid'])
        self.assertIsNone(self.metric.capture)

    def test_malformed_target_is_rejected(self):
        with self.assertRaises(ValueError):
            errors.transform(np.zeros((4, 4)))

    def test_axial_tilt_is_still_an_error(self):
        actual = np.eye(4); a = .04
        actual[:3, :3] = [[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]]
        self.assertAlmostEqual(errors.pose_error(actual, np.eye(4), axial=True)[1], np.degrees(a))
