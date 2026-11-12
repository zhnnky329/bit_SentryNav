"""2027 referee frame validation shared by mock Tx paths (CRC8/MAXIM, CRC16/X25)."""


def crc8(data):
    value = 0xff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8c if value & 1 else 0)
    return value


def crc16(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def parse_interaction_frame(data):
    raw = bytes(data)
    if len(raw) < 15 or raw[0] != 0xa5:
        raise ValueError("frame too short or wrong SOF")
    length = int.from_bytes(raw[1:3], "little")
    if length < 6 or length > 118 or len(raw) != length + 9:
        raise ValueError("0x0301 payload length mismatch")
    if crc8(raw[:4]) != raw[4] or crc16(raw[:-2]) != int.from_bytes(raw[-2:], "little"):
        raise ValueError("frame CRC mismatch")
    cmd_id = int.from_bytes(raw[5:7], "little")
    if cmd_id != 0x0301:
        raise ValueError("not a 0x0301 interaction frame")
    return {
        "seq": raw[3],
        "data_cmd_id": int.from_bytes(raw[7:9], "little"),
        "sender_id": int.from_bytes(raw[9:11], "little"),
        "receiver_id": int.from_bytes(raw[11:13], "little"),
        "payload": raw[13:-2],
    }
