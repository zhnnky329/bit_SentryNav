#include <gtest/gtest.h>

#include <linux/can.h>

#include <array>
#include <cstdint>

#include "can_comm/protocol.hpp"

using can_comm::decodeYaw;
using can_comm::encodeCommand;
using can_comm::kYawFeedbackId;

TEST(EncodeCommand, Zero)
{
  const auto data = encodeCommand(0.0, 0.0, false, false, false);
  EXPECT_EQ(data, (std::array<uint8_t, 8>{0, 0, 0, 0, 0, 0, 0, 0}));
}

TEST(EncodeCommand, VelocityLimits)
{
  auto data = encodeCommand(1.0, -1.0, false, false, false);
  EXPECT_EQ(data[0], 0x7F);
  EXPECT_EQ(data[1], 0x81);

  data = encodeCommand(2.0, -2.0, false, false, false);
  EXPECT_EQ(data[0], 0x7F);
  EXPECT_EQ(data[1], 0x81);
}

TEST(EncodeCommand, TargetYaw)
{
  auto data = encodeCommand(0.0, 0.0, false, false, false, 90.0);
  EXPECT_EQ(data[2], 0x23);
  EXPECT_EQ(data[3], 0x28);

  data = encodeCommand(0.0, 0.0, false, false, false, -90.0);
  EXPECT_EQ(data[2], 0xDC);
  EXPECT_EQ(data[3], 0xD8);

  data = encodeCommand(0.0, 0.0, false, false, false, 200.0);
  EXPECT_EQ(data[2], 0x46);
  EXPECT_EQ(data[3], 0x50);
}

TEST(EncodeCommand, PresenceFlags)
{
  const auto data = encodeCommand(0.0, 0.0, true, false, true);
  EXPECT_EQ(data[4], 1);
  EXPECT_EQ(data[5], 0);
  EXPECT_EQ(data[6], 1);
  EXPECT_EQ(data[7], 0);
}

TEST(DecodeYaw, Angles)
{
  uint8_t data[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0x23, 0x28};
  ASSERT_TRUE(decodeYaw(kYawFeedbackId, 8, data).has_value());
  EXPECT_DOUBLE_EQ(*decodeYaw(kYawFeedbackId, 8, data), 90.0);

  data[6] = 0xDC;
  data[7] = 0xD8;
  EXPECT_DOUBLE_EQ(*decodeYaw(kYawFeedbackId, 8, data), -90.0);

  data[6] = 0xFF;
  data[7] = 0xFF;
  EXPECT_DOUBLE_EQ(*decodeYaw(kYawFeedbackId, 8, data), -0.01);
}

TEST(DecodeYaw, RejectsOtherFrames)
{
  uint8_t data[8] = {};
  EXPECT_FALSE(decodeYaw(0x148, 8, data).has_value());
  EXPECT_FALSE(decodeYaw(kYawFeedbackId, 7, data).has_value());
  EXPECT_FALSE(decodeYaw(kYawFeedbackId | CAN_EFF_FLAG, 8, data).has_value());
  EXPECT_FALSE(decodeYaw(kYawFeedbackId | CAN_RTR_FLAG, 8, data).has_value());
}
