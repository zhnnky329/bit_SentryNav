// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <mutex>
#include <optional>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>

namespace relocalizer
{
// Keep the measurement stamp intact. Only the TF broadcast gets the control-time stamp.
class OdomCorrection
{
public:
  using Transform = geometry_msgs::msg::TransformStamped;

  void update(const Transform & measurement)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!measurement_ || rclcpp::Time(measurement.header.stamp) >
      rclcpp::Time(measurement_->header.stamp))
    {
      measurement_ = measurement;
    }
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    measurement_.reset();
  }

  template<typename Publish>
  void publish(const rclcpp::Time & now, const rclcpp::Duration & timeout, Publish send)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!measurement_) {
      return;
    }
    const auto age = now - rclcpp::Time(measurement_->header.stamp, now.get_clock_type());
    if (age.nanoseconds() < 0 || age > timeout) {
      measurement_.reset();
      return;
    }
    auto transform = *measurement_;
    transform.header.stamp = now;
    send(transform);
  }

private:
  std::mutex mutex_;
  std::optional<Transform> measurement_;
};
}  // namespace relocalizer
