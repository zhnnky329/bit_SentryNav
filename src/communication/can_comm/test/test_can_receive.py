import socket

import pytest

from can_receive import CAN_FRAME, YAW_FEEDBACK_ID, decode_yaw


@pytest.mark.parametrize(('encoded', 'expected'), [
    ('2328', 90.0), ('dcd8', -90.0), ('ffff', -0.01), ('0000', 0.0),
])
def test_feedback_angle(encoded, expected):
    data = bytes.fromhex('12 34 56 78 9a bc ' + encoded)
    assert decode_yaw(CAN_FRAME.pack(YAW_FEEDBACK_ID, 8, data)) == expected


@pytest.mark.parametrize(('can_id', 'dlc'), [
    (0x148, 8), (YAW_FEEDBACK_ID, 7),
    (YAW_FEEDBACK_ID | socket.CAN_EFF_FLAG, 8),
    (YAW_FEEDBACK_ID | socket.CAN_RTR_FLAG, 8),
])
def test_unrelated_frames(can_id, dlc):
    assert decode_yaw(CAN_FRAME.pack(can_id, dlc, bytes(8))) is None


def test_truncated_frame():
    assert decode_yaw(bytes(15)) is None
