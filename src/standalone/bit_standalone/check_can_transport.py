"""Offline frame/freshness checks. Does not import ROS or open a socket."""

import struct
import unittest

from bit_standalone.can_transport import velocity_can_frame


class CanTransportChecks(unittest.TestCase):
    def test_standard_frame_and_signed_payload(self):
        frame = velocity_can_frame(1.0, -1.0, 1.0, 0.01, 0.02, 0.1)
        self.assertEqual(len(frame), 16)
        can_id, dlc, payload = struct.unpack("=IB3x8s", frame)
        self.assertEqual(can_id, 0x190)
        self.assertEqual(dlc, 8)
        self.assertEqual(payload, bytes([127, 129, 0, 0, 0, 0, 0, 0]))

    def test_scaling_and_saturation_preserve_translation_only(self):
        payload = velocity_can_frame(0.5, -3.0, 2.0, 0, 0, 0.1)[8:]
        self.assertEqual(payload, bytes([31, 129, 0, 0, 0, 0, 0, 0]))

    def test_stale_received_or_stale_stamped_command_sends_zero(self):
        for received_age, message_age in ((0.101, 0), (0, 0.101), (float("inf"), 0)):
            self.assertEqual(velocity_can_frame(1, 1, 1, received_age, message_age, 0.1)[8:], bytes(8))

    def test_future_or_invalid_time_sends_zero(self):
        for received_age, message_age in ((0, -0.01), (-0.01, 0), (0, float("nan"))):
            self.assertEqual(velocity_can_frame(1, 1, 1, received_age, message_age, 0.1)[8:], bytes(8))

    def test_invalid_velocity_sends_zero(self):
        self.assertEqual(velocity_can_frame(float("nan"), 1, 1, 0, 0, 0.1)[8:], bytes(8))


if __name__ == "__main__":
    unittest.main()
