import socket

from can_send import CAN_FRAME, COMMAND_ID, encode_command


def test_zero_command():
    assert encode_command(0.0, 0.0, 0, 0, 0) == bytes(8)


def test_velocity_encoding_and_limits():
    data = encode_command(1.0, -1.0, 0, 0, 0)
    assert data[0] == 0x7F
    assert data[1] == 0x81

    data = encode_command(2.0, -2.0, 0, 0, 0)
    assert data[0] == 0x7F
    assert data[1] == 0x81


def test_target_yaw_encoding():
    assert encode_command(0.0, 0.0, 0, 0, 0, 90.0)[2:4] == bytes.fromhex('2328')
    assert encode_command(0.0, 0.0, 0, 0, 0, -90.0)[2:4] == bytes.fromhex('dcd8')

    clamped = encode_command(0.0, 0.0, 0, 0, 0, 200.0)
    assert int.from_bytes(clamped[2:4], 'big', signed=True) == 18000


def test_presence_flags():
    data = encode_command(0.0, 0.0, 1, 0, 1)
    assert data[4:8] == bytes((1, 0, 1, 0))

    data = encode_command(0.0, 0.0, 5, 0, 0)
    assert data[4:8] == bytes((1, 0, 0, 0))


def test_can_frame_round_trip():
    data = encode_command(0.5, -0.5, 0, 1, 0)
    frame = CAN_FRAME.pack(COMMAND_ID, 8, data)
    can_id, dlc, payload = CAN_FRAME.unpack(frame)
    assert can_id == COMMAND_ID
    assert dlc == 8
    assert payload == data


def test_standard_frame_id():
    assert not COMMAND_ID & socket.CAN_EFF_FLAG
