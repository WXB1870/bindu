"""Bindu adapter metadata encoding; vendor implementation tests live in Snowbot."""
import unittest
from snowbot_hardware.protocol import guard_array

class VendorGuardMetadataTests(unittest.TestCase):
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
