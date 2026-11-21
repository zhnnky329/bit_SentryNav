#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "body_cloud.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using Cloud = sensor_msgs::msg::PointCloud2;

class LivoxBodyCloudRelay : public rclcpp::Node
{
public:
  LivoxBodyCloudRelay()
  : Node("livox_body_cloud_relay"),
    body_frame_(declare_parameter<std::string>("body_frame", "base_link")),
    buffer_(get_clock()), listener_(buffer_)
  {
    publisher_ = create_publisher<Cloud>("/cloud_registered_body", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<Cloud>("/livox/lidar_filtered", rclcpp::SensorDataQoS(),
      [this](Cloud::ConstSharedPtr cloud) { relay(*cloud); });
  }

private:
  void relay(const Cloud & input)
  {
    if (input.data.empty()) {
      return;
    }
    tf2::Transform to_body;
    try {
      const auto transform = buffer_.lookupTransform(
        body_frame_, input.header.frame_id, rclcpp::Time(input.header.stamp));
      tf2::fromMsg(transform.transform, to_body);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
      return;
    }

    try {
      Cloud output = relocalizer::body_cloud(input, to_body, body_frame_);
      if (output.width > 0) {
        publisher_->publish(output);
      }
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
    }
  }

  std::string body_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Subscription<Cloud>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxBodyCloudRelay>());
  rclcpp::shutdown();
  return 0;
}
