"""Bindu adapter metadata encoding; vendor implementation tests live in Snowbot."""
import unittest
from snowbot_hardware.protocol import guard_array

class VendorGuardMetadataTests(unittest.TestCase):
    def test_guard_array_preserves_native_prefix_and_full_session(self):
        prefix = [-.2, 0., 0., 100., 30.]
        result = guard_array(prefix, 1700000000000000000, 123, 2**64-2, True)
        self.assertEqual(result, [*prefix, 1700000000., 123., 4294967295., 4294967294., 1.])
        self.assertEqual(prefix, [-.2, 0., 0., 100., 30.])
