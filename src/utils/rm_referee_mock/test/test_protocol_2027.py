"""Pure protocol tests; run with python3 -m unittest test/test_protocol_2027.py."""
import unittest

from rm_referee_mock.match_control_status import MatchControlStatus
from rm_referee_mock.match_control_supply import MatchControlSupply
from rm_referee_mock.protocol.frame_2027 import crc8, crc16, parse_interaction_frame


class Choice:
    def __init__(self, value):
        self._value = value

    def currentIndex(self):
        return self._value

    def value(self):
        return self._value

    def isChecked(self):
        return bool(self._value)

    def get_confirmed_text(self):
        return "00:00"


class Scenario(MatchControlStatus, MatchControlSupply):
    def __init__(self, robot_id=7):
        self.spinBox_robot_id = Choice(robot_id)
        self.checkBox_2 = Choice(1)
        self.auto_supply_zone_detected = False
        self.comboBox_small_rune = Choice(2)
        self.comboBox_big_rune = Choice(1)
        self.comboBox_fortress_buff = Choice(2)
        self.comboBox_outpost_buff = Choice(1)
        self.comboBox_opponent_fortress_gain = Choice(4)
        self.comboBox_opponent_outpost_gain = Choice(4)
        self.checkBox_base_buff = Choice(0)
        self.comboBox_central_highland = Choice(0)
        self.comboBox_r_outpost_control = Choice(1)
        self.comboBox_b_outpost_control = Choice(2)
        self.comboBox_r_base_control = Choice(0)
        self.comboBox_b_base_control = Choice(2)
        self.lineEdit_dart_time = Choice(0)
        self.comboBox_dart_target = Choice(0)
        self.comboBox_central_buff = Choice(0)
        self.comboBox_game_type = Choice(1)

    def _is_rmul_game_type(self):
        return False

    def _parse_time_to_seconds(self, text):
        return 0


class Protocol2027Test(unittest.TestCase):
    def test_event_red_side(self):
        scenario = Scenario(7)
        word = scenario.get_event_data_value()
        self.assertEqual((word >> 1) & 3, 1)  # 已激活的小符
        self.assertEqual((word >> 3) & 3, 2)  # 正在激活的大符
        self.assertEqual((word >> 5) & 7, 2)  # R 堡垒敌方激活
        self.assertEqual((word >> 25) & 7, 4)  # B 前哨未下发
        self.assertEqual((word >> 28) & 3, 1)  # R 前哨临时控制
        self.assertEqual((word >> 30) & 3, 2)  # B 前哨未控制
        word2 = scenario.get_event_data_2_value()
        self.assertEqual(word2 & 3, 0)
        self.assertEqual((word2 >> 2) & 3, 2)
        self.assertEqual(scenario.get_rfid_status_value()["rfid_status"] & 1, 1)
        self.assertEqual((scenario.get_rfid_status_value()["rfid_status"] >> 8) & 1, 1)

    def test_event_blue_side(self):
        scenario = Scenario(107)
        word = scenario.get_event_data_value()
        self.assertEqual((word >> 5) & 7, 4)
        self.assertEqual((word >> 8) & 7, 2)
        self.assertEqual((word >> 22) & 7, 4)
        self.assertEqual((word >> 25) & 7, 1)
        self.assertEqual((scenario.get_rfid_status_value()["rfid_status"] >> 9) & 1, 1)

    def test_interaction_crc_and_lengths(self):
        payload = bytes([0x00, 0x02, 0x07, 0x00, 0x09, 0x00, 0xaa])
        frame = bytearray([0xa5, len(payload), 0, 4, 0])
        frame[4] = crc8(frame[:4])
        frame.extend([0x01, 0x03])
        frame.extend(payload)
        frame.extend(crc16(frame).to_bytes(2, "little"))
        parsed = parse_interaction_frame(frame)
        self.assertEqual((parsed["sender_id"], parsed["receiver_id"], parsed["payload"]),
                         (7, 9, b"\xaa"))
        frame[-1] ^= 1
        with self.assertRaisesRegex(ValueError, "CRC"):
            parse_interaction_frame(frame)
        with self.assertRaisesRegex(ValueError, "length"):
            parse_interaction_frame(bytes(frame[:-1]))


if __name__ == "__main__":
    unittest.main()
