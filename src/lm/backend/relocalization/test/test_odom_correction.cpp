// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "relocalizer/odom_correction.hpp"

using relocalizer::OdomCorrection;

TEST(OdomCorrection, HoldsTheNewestMeasurementOnlyUntilExpiry)
{
  OdomCorrection held;
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0.5);
  const rclcpp::Time measured(1, 0, RCL_ROS_TIME);
  OdomCorrection::Transform fix;
  fix.header.stamp = measured;
  fix.transform.translation.x = 3.0;
  int sent = 0;
  auto send = [&](const auto & tf) {
    ++sent;
    EXPECT_EQ(rclcpp::Time(tf.header.stamp), measured + timeout);
    EXPECT_DOUBLE_EQ(tf.transform.translation.x, 3.0);
  };
  held.publish(measured, timeout, send);
  held.update(fix);
  fix.header.stamp = measured - rclcpp::Duration::from_seconds(0.1);
  fix.transform.translation.x = 9.0;
  held.update(fix);
  held.publish(measured + timeout, timeout, send);
  held.publish(measured + timeout + rclcpp::Duration(0, 1), timeout, send);
  EXPECT_EQ(sent, 1);
}

TEST(OdomCorrection, ClearAndClockRewindRequireANewFix)
{
  OdomCorrection held;
  const rclcpp::Time measured(2, 0, RCL_ROS_TIME);
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0.5);
  OdomCorrection::Transform fix;
  fix.header.stamp = measured;
  int sent = 0;
  auto send = [&](const auto &) { ++sent; };
  held.update(fix);
  held.clear();
  held.publish(measured, timeout, send);
  held.update(fix);
  held.publish(measured - timeout, timeout, send);
  held.publish(measured, timeout, send);
  EXPECT_EQ(sent, 0);
  held.update(fix);
  held.publish(measured, timeout, send);
  EXPECT_EQ(sent, 1);
}
