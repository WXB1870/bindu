"""Local copy/version safeguards and native position contract; no ROS or devices."""
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


class VendorPositionTests(unittest.TestCase):
    def fixture(self, root):
        source, assets = root/'original', root/'assets'
        source.mkdir(); assets.mkdir()
        for package in ('servo_control', 'dual_arm_control'):
            (source/package/'include').mkdir(parents=True)
        (source/'value').write_text('original\n')
        (assets/'position_118.patch').write_text(
            '--- a/value\n+++ b/value\n@@ -1 +1 @@\n-original\n+fixed\n')
        (assets/'position_contract.h').write_text('// helper\n')
        expected = {'value': hashlib.sha256(b'original\n').hexdigest()}
        return source, assets, expected

    def test_prepare_copies_without_mutating_original_and_records_version(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source, assets, expected=self.fixture(root)
            with patch.object(vendor, 'ASSETS', assets), patch.object(vendor, 'SITE_118_FILES', expected):
                result=vendor.prepare_118(source, root/'prepared')
            self.assertEqual((source/'value').read_text(), 'original\n')
            self.assertEqual((result/'src/value').read_text(), 'fixed\n')
            for package in ('dual_arm_control', 'servo_control'):
                self.assertEqual((result/'src'/package/'include/bindu_position_contract.h').read_text(), '// helper\n')
            manifest=json.loads((result/'bindu_vendor_build.json').read_text())
            self.assertFalse(manifest['hardware_started'])
            self.assertFalse(manifest['full_driver_build_verified'])

    def test_changed_source_is_rejected_before_creating_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source, assets, expected=self.fixture(root)
            (source/'value').write_text('site changed\n')
            with patch.object(vendor, 'ASSETS', assets), patch.object(vendor, 'SITE_118_FILES', expected):
                with self.assertRaisesRegex(ValueError, 'VERSION_MISMATCH'):
                    vendor.prepare_118(source, root/'prepared')
            self.assertFalse((root/'prepared').exists())

    def test_source_output_overlap_existing_output_and_symlinks_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source, assets, expected=self.fixture(root)
            with patch.object(vendor, 'SITE_118_FILES', expected):
                for output in (source, source/'inside', root):
                    with self.assertRaisesRegex(ValueError, 'OVERLAP'):
                        vendor.prepare_118(source, output)
                (root/'prepared').mkdir()
                with self.assertRaisesRegex(ValueError, 'OUTPUT_EXISTS'):
                    vendor.prepare_118(source, root/'prepared')
                (source/'link').symlink_to(root/'prepared')
                with self.assertRaisesRegex(ValueError, 'SOURCE_SYMLINK'):
                    vendor.prepare_118(source, root/'another')

    def test_patch_failure_cleans_stage_without_partial_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source, assets, expected=self.fixture(root)
            (assets/'position_118.patch').write_text('invalid patch\n')
            with patch.object(vendor, 'ASSETS', assets), patch.object(vendor, 'SITE_118_FILES', expected):
                with self.assertRaises(subprocess.CalledProcessError):
                    vendor.prepare_118(source, root/'prepared')
            self.assertFalse((root/'prepared').exists())
            self.assertFalse(list(root.glob('.snowbot-118-*')))

    def test_native_clock_and_capture_contract(self):
        compiler=shutil.which('clang++') or shutil.which('g++')
        if not compiler:self.skipTest('C++ compiler unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            binary=Path(tmp)/'position-check'
            subprocess.run([compiler, '-std=c++17', str(ROOT/'tests/native/test_snowbot_position.cpp'),
                            '-o', str(binary)], check=True, capture_output=True, text=True)
            result=subprocess.run([str(binary)], check=True, capture_output=True, text=True)
            self.assertIn('21 position contract checks passed', result.stdout)


if __name__ == '__main__':
    unittest.main()
