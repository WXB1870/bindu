import math
import unittest
import numpy as np
from bindu_lio.localization import (
    RegistrationConfig, ScanMapRegistration, correction_distance,
    initial_map_from_odom, inverse_se3, pose_matrix, rigid_transform,
)


class LioGeometryTests(unittest.TestCase):
    def test_initial_pose_includes_current_odometry(self):
        odom = pose_matrix([2., 1., 0.], [0., 0., math.sin(.4), math.cos(.4)])
        correction = pose_matrix([.3, -.4, .1], [0., 0., math.sin(.1), math.cos(.1)])
        seed = correction @ odom
        np.testing.assert_allclose(initial_map_from_odom(seed, odom), correction, atol=1e-12)
        self.assertGreater(np.linalg.norm(seed-correction), 1.)

    def test_rigid_pose_rejects_reflection_scale_nan_and_invalid_quaternion(self):
        for bad in (np.diag([-1., 1., 1., 1.]), np.diag([2., 1., 1., 1.]), np.full((4, 4), np.nan)):
            with self.assertRaises(ValueError):
                rigid_transform(bad)
        with self.assertRaises(ValueError):
            pose_matrix([0., 0., 0.], [0., 0., 0., 0.])

    def test_configuration_rejects_nonfinite_and_out_of_range(self):
        for values in ({'max_rmse': float('nan')}, {'min_fitness': 1.2}, {'map_voxel': 0.}, {'min_points': 3.2}):
            with self.assertRaises(ValueError):
                RegistrationConfig(**values)

    def test_distance_handles_pi_rotation(self):
        translated = pose_matrix([.3, .4, 0.], [0., 0., 1., 0.])
        distance, angle = correction_distance(np.eye(4), translated)
        self.assertAlmostEqual(distance, .5)
        self.assertAlmostEqual(angle, math.pi)


class ActualRegistrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import open3d  # noqa: F401
        except ImportError:
            raise unittest.SkipTest('optional Open3D dependency unavailable')
        rng = np.random.default_rng(42)
        cls.points = rng.uniform([-4., -3., -1.], [5., 4., 2.], (5000, 3))

    def test_actual_icp_recovers_known_three_dimensional_transform(self):
        expected = pose_matrix([.18, -.12, .08], [0., 0., math.sin(.025), math.cos(.025)])
        scan = (inverse_se3(expected) @ np.c_[self.points, np.ones(len(self.points))].T).T[:, :3]
        matcher = ScanMapRegistration(self.points)
        actual, fitness, rmse = matcher.match(scan, np.eye(4), np.eye(4))
        error, angle = correction_distance(expected, actual)
        self.assertLess(error, .01)
        self.assertLess(angle, .005)
        self.assertGreater(fitness, .95)
        self.assertLess(rmse, .05)

    def test_bad_overlap_is_rejected(self):
        matcher = ScanMapRegistration(self.points)
        with self.assertRaisesRegex(ValueError, 'REGISTRATION_REJECTED'):
            matcher.match(self.points+100., np.eye(4), np.eye(4))

    def test_large_correction_is_not_silently_published(self):
        matcher = ScanMapRegistration(self.points, RegistrationConfig(max_translation_step=.01))
        with self.assertRaisesRegex(ValueError, 'REGISTRATION_REJECTED'):
            matcher.match(self.points+[.15, 0., 0.], np.eye(4), np.eye(4))

    def test_nonfinite_and_empty_scan_are_rejected(self):
        matcher = ScanMapRegistration(self.points)
        with self.assertRaises(ValueError):
            matcher.match([[float('nan'), 0., 0.]], np.eye(4), np.eye(4))
        with self.assertRaisesRegex(ValueError, 'INSUFFICIENT_REGISTRATION_POINTS'):
            matcher.match(np.empty((0, 3)), np.eye(4), np.eye(4))


if __name__ == '__main__':
    unittest.main()
