"""Offline native guard and wire metadata checks; no ROS or hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from tests.support.paths import ROOT
from bindu_hardware.snowbot import guard_array
from tools.deployment import prepare_snowbot_vendor as vendor


class VendorGuardTests(unittest.TestCase):
    def test_native_deadlines_and_hold(self):
        compiler = shutil.which('c++')
        if not compiler:
            self.skipTest('C++ compiler unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(Path(tmp)/'guard')
            subprocess.run([compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(vendor.ASSETS),
                            str(ROOT/'tests/native/test_snowbot_guard_118.cpp'), '-o', binary],
                           check=True, capture_output=True, text=True)
            result = subprocess.run([binary], check=True, capture_output=True, text=True, timeout=10)
            self.assertIn('checks passed', result.stdout)

    def test_guard_array_preserves_native_prefix_and_full_session(self):
        prefix = [-.2, 0., 0., 100., 30.]
        result = guard_array(prefix, 1700000000000000000, 123, 2**64-2, True)
        self.assertEqual(result, [*prefix, 1700000000., 123., 4294967295., 4294967294., 1.])
        self.assertEqual(prefix, [-.2, 0., 0., 100., 30.])
        for stamp, seq, session, hold in ((0, 1, 1, False), (1, 2**53, 1, False),
                                         (1, 1, 0, False), (1, 1, 2**64, False),
                                         (1, 1, 1, 1)):
            with self.assertRaisesRegex(ValueError, 'GUARD_METADATA'):
                guard_array(prefix, stamp, seq, session, hold)
