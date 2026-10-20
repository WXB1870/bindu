"""Airy boundary checks use synthetic ROS messages and never open a device."""
from tests.support.paths import ROOT
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

from bindu_navigation.airy_scan import AiryScanConfig, cloud_stamp, project_cloud


CONFIG = ROOT/'src/integration/bindu_runtime/config/robots/wheel_gen2/navigation_wheel_gen2_airy.json'


def calibrated():
    data = json.loads(CONFIG.read_text())
    data.update(base_frame='base_link', base_from_lidar=[.4, 0., .6, 0., 0., 0., 1.],
                min_height=.1, max_height=.9, min_bins=1)
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder)/'sensor.json'
        path.write_text(json.dumps(data))
        return AiryScanConfig.load(path)


def cloud(points, stamp=100.):
    header = Header()
    header.frame_id = 'rslidar'
    header.stamp.sec = int(stamp)
    header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    return create_cloud_xyz32(header, points)


class AiryScanTests(unittest.TestCase):
    def test_document_snapshot_requires_measured_calibration(self):
        with self.assertRaisesRegex(ValueError, 'AIRY_CALIBRATION_REQUIRED'):
            AiryScanConfig.load(CONFIG)
        config = calibrated()
        self.assertEqual((config.source_domain, config.control_domain), (42, 0))
        self.assertEqual(config.source_topic, '/rslidar_points')

    def test_projection_keeps_source_stamp_and_unknown_sectors(self):
        message = cloud([[2., 0., 0.], [3., 0., 0.], [1., 0., 1.], [0., 2., 0.]])
        scan = project_cloud(message, calibrated(), 100.1)
        self.assertEqual(cloud_stamp(message), 100.)
        self.assertEqual(scan.header.stamp, message.header.stamp)
        self.assertEqual(scan.header.frame_id, 'wheel_gen2_nav_scan')
        ranges = [value for value in scan.ranges if math.isfinite(value)]
        self.assertEqual(sorted(ranges), [2., 2.])
        self.assertTrue(any(math.isnan(value) for value in scan.ranges))

    def test_pitch_calibration_projects_into_horizontal_base_frame(self):
        config = calibrated()
        data = vars(config).copy()
        # 90 degree rotation about Y: sensor +Z points in base +X.
        data['base_from_lidar'] = (0., 0., .6, 0., math.sqrt(.5), 0., math.sqrt(.5))
        config = AiryScanConfig(**data)
        scan = project_cloud(cloud([[0., 0., 2.]]), config, 100.1)
        self.assertEqual([value for value in scan.ranges if math.isfinite(value)], [2.])

    def test_bad_source_age_and_layout_do_not_publish(self):
        config = calibrated()
        message = cloud([[2., 0., 0.]])
        for mutate, code in [
            (lambda m: setattr(m.header, 'frame_id', 'base_link'), 'AIRY_FRAME_MISMATCH'),
            (lambda m: setattr(m, 'is_bigendian', True), 'AIRY_CLOUD_LAYOUT'),
            (lambda m: setattr(m, 'row_step', m.row_step-1), 'AIRY_CLOUD_LAYOUT'),
            (lambda m: setattr(m.fields[0], 'datatype', PointField.UINT8), 'AIRY_CLOUD_LAYOUT'),
        ]:
            bad = copy.deepcopy(message)
            mutate(bad)
            with self.subTest(code=code), self.assertRaisesRegex(ValueError, code):
                project_cloud(bad, config, 100.1)
        for now, last in [(99.99, 0.), (100.31, 0.), (100.1, 100.)]:
            with self.subTest(now=now, last=last), self.assertRaisesRegex(ValueError, 'AIRY_STALE_CLOUD'):
                project_cloud(message, config, now, last)

    def test_sparse_cloud_rejected_and_calibration_must_be_finite(self):
        config = calibrated()
        with self.assertRaisesRegex(ValueError, 'AIRY_SCAN_TOO_SPARSE'):
            project_cloud(cloud([[2., 0., 2.]]), config, 100.1)
        raw = json.loads(CONFIG.read_text())
        raw.update(base_frame='base_link', base_from_lidar=[0., 0., 0., 0., 0., 0., 0.],
                   min_height=0., max_height=1.)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'bad.json'
            path.write_text(json.dumps(raw))
            with self.assertRaisesRegex(ValueError, 'AIRY_EXTRINSIC'):
                AiryScanConfig.load(path)


if __name__ == '__main__':
    unittest.main()
