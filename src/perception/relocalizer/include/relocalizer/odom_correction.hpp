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
    if (latched_) {
      return;
    }
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
    latched_ = false;
  }

  // Keep publishing the last accepted correction even after its measurement
  // stamp ages out. Used when the relocalizer is intentionally frozen after
  // the initial fix. Returns false if there is no correction to latch (for
  // example if the timeout expired first).
  bool latch()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!measurement_) {
      return false;
    }
    latched_ = true;
    return true;
  }

  bool hasMeasurement()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return measurement_.has_value();
  }

  template<typename Publish>
  void publish(const rclcpp::Time & now, const rclcpp::Duration & timeout, Publish send)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!measurement_) {
      return;
    }
    if (!latched_) {
      const auto age = now - rclcpp::Time(measurement_->header.stamp, now.get_clock_type());
      if (age.nanoseconds() < 0 || age > timeout) {
        measurement_.reset();
        return;
      }
    }
    auto transform = *measurement_;
    transform.header.stamp = now;
    send(transform);
  }

private:
  std::mutex mutex_;
  std::optional<Transform> measurement_;
  bool latched_{false};
};
}  // namespace relocalizer
