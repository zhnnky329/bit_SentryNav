"""Offline comparison with the existing 0x190 translational command layout."""

import math
import struct


def encode_translation_preview(vx, vy, speed_at_full_scale_mps):
    if not math.isfinite(speed_at_full_scale_mps) or speed_at_full_scale_mps <= 0.0:
        raise ValueError("speed_at_full_scale_mps must be positive and finite")
    if not math.isfinite(vx) or not math.isfinite(vy):
        return bytes(8)
    x = int(max(-1.0, min(1.0, vx / speed_at_full_scale_mps)) * 127.0)
    y = int(max(-1.0, min(1.0, vy / speed_at_full_scale_mps)) * 127.0)
    return struct.pack("bb6x", x, y)
