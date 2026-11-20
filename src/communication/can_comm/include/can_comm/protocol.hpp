#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

namespace can_comm
{

constexpr uint32_t kCommandId = 0x190;
constexpr uint32_t kYawFeedbackId = 0x149;

inline int8_t quantizeVelocity(const double value)
{
  return static_cast<int8_t>(std::clamp(value, -1.0, 1.0) * 127.0);
}

inline std::array<uint8_t, 8> encodeCommand(
  const double vx, const double vy, const bool left, const bool behind, const bool right,
  const double target_yaw_deg = 0.0)
{
  const auto yaw = static_cast<int16_t>(std::clamp(target_yaw_deg, -180.0, 180.0) * 100.0);
  return {
    static_cast<uint8_t>(quantizeVelocity(vx)),
    static_cast<uint8_t>(quantizeVelocity(vy)),
    static_cast<uint8_t>((static_cast<uint16_t>(yaw) >> 8) & 0xFF),
    static_cast<uint8_t>(yaw & 0xFF),
    static_cast<uint8_t>(left ? 1 : 0),
    static_cast<uint8_t>(behind ? 1 : 0),
    static_cast<uint8_t>(right ? 1 : 0),
    0,
  };
}

inline std::optional<double> decodeYaw(
  const uint32_t can_id, const uint8_t dlc, const uint8_t * data)
{
  if (can_id != kYawFeedbackId || dlc != 8) {
    return std::nullopt;
  }
  const auto raw = static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) | data[7]);
  return raw / 100.0;
}

}  // namespace can_comm
