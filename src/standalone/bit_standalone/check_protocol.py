"""Byte-layout checks only, with no ROS or CAN dependency."""

import unittest

from bit_standalone.protocol_preview import encode_translation_preview


class ProtocolChecks(unittest.TestCase):
    def test_full_scale_and_zero(self):
        self.assertEqual(encode_translation_preview(1.0, -1.0, 1.0), bytes([127, 129, 0, 0, 0, 0, 0, 0]))
        self.assertEqual(encode_translation_preview(0.0, 0.0, 1.0), bytes(8))

    def test_truncation_matches_existing_encoder(self):
        self.assertEqual(encode_translation_preview(0.5, -0.5, 1.0)[:2], bytes([63, 193]))

    def test_configurable_scale_and_saturation(self):
        self.assertEqual(encode_translation_preview(0.5, -0.5, 2.0)[:2], bytes([31, 225]))
        self.assertEqual(encode_translation_preview(5.0, -5.0, 1.0)[:2], bytes([127, 129]))

    def test_invalid_inputs(self):
        self.assertEqual(encode_translation_preview(float("nan"), 0.2, 1.0), bytes(8))
        with self.assertRaises(ValueError):
            encode_translation_preview(0.1, 0.2, 0.0)


if __name__ == "__main__":
    unittest.main()
