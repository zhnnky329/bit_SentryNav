"""Classic Linux SocketCAN framing and the navigation command freshness rule."""

import math
import struct

from bit_standalone.protocol_preview import encode_translation_preview


def velocity_can_frame(vx, vy, scale, received_age, message_age, timeout):
    # Standard 11-bit ID, DLC 8, then the existing firmware's eight payload bytes.
    fresh = all(math.isfinite(age) and 0.0 <= age <= timeout for age in (received_age, message_age))
    payload = encode_translation_preview(vx, vy, scale) if fresh else bytes(8)
    return struct.pack("=IB3x8s", 0x190, 8, payload)
