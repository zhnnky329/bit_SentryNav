#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include "librm/device/referee/referee.hpp"
#include "librm/modules/crc.hpp"

namespace {
using rm::u8;
using rm::u16;
using Revision = rm::device::RefereeRevision;
using Protocol = rm::device::RefereeProtocol<Revision::kRm2027V100>;
using Referee = rm::device::Referee<Revision::kRm2027V100>;

std::vector<u8> MakeFrame(u16 cmd, const std::vector<u8>& payload, u8 seq = 1) {
  std::vector<u8> frame{0xa5, static_cast<u8>(payload.size() & 0xff),
                        static_cast<u8>(payload.size() >> 8), seq, 0};
  frame[4] = rm::modules::Crc8(frame.data(), 4, rm::modules::CRC8_INIT);
  frame.push_back(static_cast<u8>(cmd & 0xff));
  frame.push_back(static_cast<u8>(cmd >> 8));
  frame.insert(frame.end(), payload.begin(), payload.end());
  const u16 crc = rm::modules::Crc16(frame.data(), frame.size(), rm::modules::CRC16_INIT);
  frame.push_back(static_cast<u8>(crc & 0xff));
  frame.push_back(static_cast<u8>(crc >> 8));
  return frame;
}
void Feed(Referee& referee, const std::vector<u8>& frame) {
  for (u8 byte : frame) referee << byte;
}
bool Check(bool condition, const char* label) {
  if (!condition) std::cerr << "FAIL: " << label << '\n';
  return condition;
}
}  // namespace

int main() {
  Referee referee;
  int callbacks = 0;
  u16 last_cmd = 0;
  referee.AttachCallback([&](u16 cmd, u8) { ++callbacks; last_cmd = cmd; });
  // 独立手算：0x0101 两个小端 uint32，检测第二个字和 R/B 阵地位没有截断。
  Feed(referee, MakeFrame(0x0101, {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0}));
  bool ok = Check(callbacks == 1 && last_cmd == 0x0101, "valid 0x0101 callback") &&
            Check(referee.data().event_data.event_data == 0x78563412 &&
                  referee.data().event_data.event_data_2 == 0xf0debc9a,
                  "0x0101 two words");
  Feed(referee, MakeFrame(0x0101, {1, 2, 3, 4}));
  Feed(referee, MakeFrame(0x0101, {1, 2, 3, 4, 5, 6, 7, 8, 9}));
  ok &= Check(callbacks == 1, "reject short and long fixed payloads");
  auto bad_crc = MakeFrame(0x0101, std::vector<u8>(8, 0));
  bad_crc.back() ^= 0xff;
  Feed(referee, bad_crc);
  Feed(referee, MakeFrame(0x7777, std::vector<u8>(8, 0)));
  ok &= Check(callbacks == 1, "reject CRC and unknown command");
  Feed(referee, MakeFrame(0x0003, {1,0, 0,0, 3,0, 4,0, 0,0, 7,0,
                                   0x34,0x12, 0,0, 0x78,0x56, 0,0,
                                   0xbc,0x9a, 0xf0,0xde}));
  ok &= Check(callbacks == 2 && referee.data().game_robot_HP.outpost_R_HP == 0x1234 &&
              referee.data().game_robot_HP.outpost_B_HP == 0x5678 &&
              referee.data().game_robot_HP.ally_control_degree == 0x9abc &&
              referee.data().game_robot_HP.enemy_control_degree == 0xdef0,
              "0x0003 offsets and control degrees");
  Feed(referee, MakeFrame(0x0206, {0x50, 0x34, 0x12, 0x9c, 0xff, 0x10, 0x00}));
  ok &= Check(callbacks == 3 && referee.data().hurt_data.HP_deduction_reason == 5 &&
              referee.data().hurt_data.hit_sequence == 0x1234 &&
              referee.data().hurt_data.hit_position_x == -100 &&
              referee.data().hurt_data.hit_position_y == 16, "0x0206 hit sequence and signed coordinates");
  Feed(referee, MakeFrame(0x0209, {0x00, 0x00, 0x40, 0x00, 0x34, 0x12}));
  ok &= Check(callbacks == 4 && referee.data().rfid_status.rfid_status == (1U << 22) &&
              referee.data().rfid_status.rfid_status_2 == 0x1234,
              "0x0209 tunnel bit and 16-bit second word");
  Feed(referee, MakeFrame(0x0204, {1, 2, 0, 3, 4, 5, 0}));
  Feed(referee, MakeFrame(0x0204, {1, 2, 0, 3, 4, 5, 0, 6}));
  ok &= Check(callbacks == 5 && referee.data().buff.attack_buff == 5,
              "0x0204 seven-byte buff and rejection of old eight-byte frame");
  const std::vector<u8> header{0x00,0x02, 0x09,0x00, 0x07,0x00};
  Feed(referee, MakeFrame(0x0301, {0,2,9,0,7}));
  Feed(referee, MakeFrame(0x0301, std::vector<u8>(119, 0)));
  ok &= Check(callbacks == 5, "reject invalid robot interaction lengths");
  auto interaction = header;
  interaction.insert(interaction.end(), {0xaa, 0xbb});
  Feed(referee, MakeFrame(0x0301, interaction));
  ok &= Check(callbacks == 6 && referee.last_payload_length() == 8 &&
              referee.data().robot_interaction.data_cmd_id == 0x0200 &&
              referee.data().robot_interaction.user_data[0] == 0xaa,
              "valid variable interaction");
  Feed(referee, MakeFrame(0x0301, header));
  ok &= Check(callbacks == 7 && referee.last_payload_length() == 6 &&
              referee.data().robot_interaction.user_data[0] == 0,
              "short interaction clears previous tail");
  // 309-byte frame exceeds the old 128-byte buffer. Check every byte, not just
  // the callback, to detect truncation and a bad offset into the destination.
  std::vector<u8> large(300);
  for (size_t i = 0; i < large.size(); ++i) large[i] = static_cast<u8>(i);
  Feed(referee, MakeFrame(0x0310, large));
  ok &= Check(callbacks == 8 &&
              std::memcmp(referee.data().robot_custom_data_2.data, large.data(), 300) == 0,
              "300-byte custom payload survives complete frame");
  Feed(referee, MakeFrame(0x0311, std::vector<u8>(30, 0)));  // previous layout
  Feed(referee, MakeFrame(0x0311, std::vector<u8>(60, 0x5a)));
  ok &= Check(callbacks == 9 && referee.data().custom_control.data[59] == 0x5a,
              "60-byte control and rejection of old layout");
  Feed(referee, MakeFrame(0x020e, {1,2,0x34,0x12,0x78,0x56,0xbc,0x9a}));
  ok &= Check(callbacks == 10 && referee.data().radar_info.radar_info_2 == 2 &&
              referee.data().radar_info.fortify_center_x == 0x1234 &&
              referee.data().radar_info.fortify_center_y == 0x5678 &&
              referee.data().radar_info.fortify_radius == 0x9abc,
              "radar fortification layout");
  auto bad_header = MakeFrame(0x0101, std::vector<u8>(8, 0));
  bad_header[4] ^= 1;
  Feed(referee, bad_header);
  Feed(referee, {0x13, 0x00, 0xff});  // noise between frames
  Feed(referee, MakeFrame(0x0101, {1,0,0,0,2,0,0,0}));
  ok &= Check(callbacks == 11 && referee.data().event_data.event_data_2 == 2,
              "resynchronize after bad header and noise");
  return ok ? 0 : 1;
}
