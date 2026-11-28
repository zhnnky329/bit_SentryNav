#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "body_cloud.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using Cloud = sensor_msgs::msg::PointCloud2;
using CustomMsg = livox_ros_driver2::msg::CustomMsg;

class LivoxBodyCloudRelay : public rclcpp::Node
{
public:
  LivoxBodyCloudRelay()
  : Node("livox_body_cloud_relay"),
    body_frame_(declare_parameter<std::string>("body_frame", "base_link")),
    input_topic_(declare_parameter<std::string>("input_topic", "/livox/lidar")),
    output_topic_(declare_parameter<std::string>("output_topic", "/cloud_registered_body")),
    input_format_(declare_parameter<std::string>("input_format", "custom")),
    buffer_(get_clock()), listener_(buffer_)
  {
    publisher_ = create_publisher<Cloud>(output_topic_, rclcpp::SensorDataQoS());
    if (input_format_ == "custom") {
      custom_subscription_ = create_subscription<CustomMsg>(
        input_topic_, rclcpp::SensorDataQoS(),
        [this](CustomMsg::ConstSharedPtr cloud) { relay_custom(*cloud); });
    } else if (input_format_ == "pointcloud2") {
      pointcloud_subscription_ = create_subscription<Cloud>(
        input_topic_, rclcpp::SensorDataQoS(),
        [this](Cloud::ConstSharedPtr cloud) { relay_cloud(*cloud); });
    } else {
      throw std::invalid_argument("input_format must be 'custom' or 'pointcloud2'");
    }
    RCLCPP_INFO(
      get_logger(), "relaying %s (%s) to %s in %s", input_topic_.c_str(),
      input_format_.c_str(), output_topic_.c_str(), body_frame_.c_str());
  }

private:
  void relay_custom(const CustomMsg & input)
  {
    if (input.points.empty()) {
      return;
    }
    Cloud cloud;
    cloud.header = input.header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(input.points.size());
    cloud.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(input.points.size());
    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
    for (const auto & point : input.points) {
      *x_it = point.x;
      *y_it = point.y;
      *z_it = point.z;
      ++x_it;
      ++y_it;
      ++z_it;
    }
    relay_cloud(cloud);
  }

  void relay_cloud(const Cloud & input)
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
  std::string input_topic_;
  std::string output_topic_;
  std::string input_format_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Subscription<Cloud>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<CustomMsg>::SharedPtr custom_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxBodyCloudRelay>());
  rclcpp::shutdown();
  return 0;
}
