"""Measured PDO feedback contract; all native checks run without ROS or devices."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from tests.support.paths import ROOT
from tools.deployment import prepare_snowbot_vendor as vendor


class VendorFeedbackTests(unittest.TestCase):
    def test_native_sampling_contract(self):
        compiler = shutil.which('c++')
        if not compiler:
            self.skipTest('C++ compiler unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(Path(tmp)/'feedback')
            subprocess.run([compiler, '-std=c++17', '-O2', '-pthread',
                            '-I'+str(vendor.ASSETS),
                            str(ROOT/'tests/native/test_snowbot_feedback.cpp'), '-o', binary],
                           check=True, capture_output=True, text=True)
            result = subprocess.run([binary], check=True, capture_output=True, text=True, timeout=20)
            self.assertIn('snapshot checks passed', result.stdout)

    def test_combined_preparation_pins_feedback_source_and_leaves_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source, assets = root/'original', root/'assets'
            source.mkdir(); assets.mkdir()
            for package in ('servo_control', 'dual_arm_control'):
                (source/package/'include/servo_control').mkdir(parents=True)
            (source/'value').write_text('original\n')
            (source/'feedback').write_text('legacy\n')
            (assets/'position_118.patch').write_text(
                '--- a/value\n+++ b/value\n@@ -1 +1 @@\n-original\n+position\n')
            (assets/'feedback_118.patch').write_text(
                '--- a/feedback\n+++ b/feedback\n@@ -1 +1 @@\n-legacy\n+measured\n')
            (assets/'position_contract.h').write_text('// position\n')
            (assets/'feedback_snapshot.h').write_text('// feedback\n')
            with patch.object(vendor, 'ASSETS', assets), patch.object(
                    vendor, 'SITE_118_FILES', {'value': vendor.digest(source/'value')}), patch.object(
                    vendor, 'SITE_118_FEEDBACK_FILES', {'feedback': vendor.digest(source/'feedback')}):
                prepared = vendor.prepare_118(source, root/'prepared', feedback=True)
                (source/'feedback').write_text('drift\n')
                with self.assertRaisesRegex(ValueError, 'SOURCE_VERSION_MISMATCH:feedback'):
                    vendor.prepare_118(source, root/'rejected', feedback=True)
            self.assertEqual((prepared/'src/value').read_text(), 'position\n')
            self.assertEqual((prepared/'src/feedback').read_text(), 'measured\n')
            self.assertEqual((source/'value').read_text(), 'original\n')
            manifest = json.loads((prepared/'bindu_vendor_build.json').read_text())
            self.assertEqual(manifest['feedback_helper_sha256'],
                             hashlib.sha256(b'// feedback\n').hexdigest())
            self.assertFalse(manifest['hardware_started'])
