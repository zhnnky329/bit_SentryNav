"""0x190 command layout aligned with IRobot-SentryNav27; byte7 is fixed to 1."""

import math
import struct

# IRobot-SentryNav27 layout: vx(int8), vy(int8), target_yaw(int16 big-endian),
# left, behind, right, byte7. This standalone path only fills translation, so
# bytes 2-6 stay zero; byte7 is always 1 as required by the current firmware.
_BYTE7_FIXED = 1


def encode_translation_preview(vx, vy, speed_at_full_scale_mps):
    if not math.isfinite(speed_at_full_scale_mps) or speed_at_full_scale_mps <= 0.0:
        raise ValueError("speed_at_full_scale_mps must be positive and finite")
    if not math.isfinite(vx) or not math.isfinite(vy):
        vx = vy = 0.0
    x = int(max(-1.0, min(1.0, vx / speed_at_full_scale_mps)) * 127.0)
    y = int(max(-1.0, min(1.0, vy / speed_at_full_scale_mps)) * 127.0)
    return struct.pack(">bbhBBBB", x, y, 0, 0, 0, 0, _BYTE7_FIXED)
