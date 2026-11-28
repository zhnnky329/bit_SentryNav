"""Classic Linux SocketCAN framing and the navigation command freshness rule."""

import math
import struct

from bit_standalone.protocol_preview import encode_translation_preview


def velocity_can_frame(vx, vy, scale, received_age, message_age, timeout):
    # Standard 11-bit ID, DLC 8, then the 0x190 payload shared with IRobot-SentryNav27.
    # Stale or invalid commands zero vx/vy but keep byte7 fixed at 1.
    fresh = all(math.isfinite(age) and 0.0 <= age <= timeout for age in (received_age, message_age))
    payload = encode_translation_preview(vx if fresh else 0.0, vy if fresh else 0.0, scale)
    return struct.pack("=IB3x8s", 0x190, 8, payload)
