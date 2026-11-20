#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <linux/can.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "can_comm/can_socket.hpp"
#include "can_comm/protocol.hpp"

namespace can_comm
{

class CanReceiveNode : public rclcpp::Node
{
public:
  CanReceiveNode()
  : Node("can_receive")
  {
    interface_ = declare_parameter<std::string>("interface", "can0");
    const auto yaw_topic = declare_parameter<std::string>("yaw_topic", "/can_comm/yaw_deg");
    auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);
    if (publish_rate_hz <= 0.0) {
      publish_rate_hz = 100.0;
    }

    yaw_pub_ = create_publisher<std_msgs::msg::Float32>(yaw_topic, 10);
    fd_ = openCanSocket(interface_, &kYawFeedbackId);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      [this]() {receive();});

    RCLCPP_INFO(
      get_logger(), "CAN 接收节点已启动 | %s 0x%X -> %s @ %.0f Hz",
      interface_.c_str(), kYawFeedbackId, yaw_topic.c_str(), publish_rate_hz);
  }

  ~CanReceiveNode() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

private:
  void receive()
  {
    struct can_frame frame {};
    std::optional<double> latest;
    while (::read(fd_, &frame, sizeof(frame)) == CAN_MTU) {
      if (const auto yaw = decodeYaw(frame.can_id, frame.can_dlc, frame.data)) {
        latest = yaw;
      }
    }

    if (latest) {
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(*latest);
      yaw_pub_->publish(msg);
    }
  }

  std::string interface_;
  int fd_ = -1;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace can_comm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<can_comm::CanReceiveNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("can_receive"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
