#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sentry_msgs/msg/armor_presence.hpp>
#include <std_msgs/msg/float32.hpp>

#include <linux/can.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "can_comm/can_socket.hpp"
#include "can_comm/protocol.hpp"

namespace can_comm
{

class CanSendNode : public rclcpp::Node
{
public:
  CanSendNode()
  : Node("can_send")
  {
    interface_ = declare_parameter<std::string>("interface", "can0");
    frequency_ = declare_parameter<double>("send_frequency_hz", 500.0);
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto armor_presence_topic =
      declare_parameter<std::string>("armor_presence_topic", "/detector/armor_presence");
    const auto target_yaw_topic =
      declare_parameter<std::string>("target_yaw_topic", "/can_comm/target_yaw");
    cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 0.1);
    if (frequency_ <= 0.0) {
      frequency_ = 500.0;
    }

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        last_cmd_vel_time_ = now();
        cmd_vel_received_ = true;
      });

    armor_presence_sub_ = create_subscription<sentry_msgs::msg::ArmorPresence>(
      armor_presence_topic, 10,
      [this](const sentry_msgs::msg::ArmorPresence::SharedPtr msg) {
        left_ = msg->left;
        behind_ = msg->behind;
        right_ = msg->right;
      });

    target_yaw_sub_ = create_subscription<std_msgs::msg::Float32>(
      target_yaw_topic, 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        target_yaw_ = msg->data;
      });

    fd_ = openCanSocket(interface_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / frequency_),
      [this]() {sendFrame();});

    RCLCPP_INFO(
      get_logger(), "CAN 发送节点已启动 | %s 0x%X @ %.0f Hz | 订阅 %s, %s, %s",
      interface_.c_str(), kCommandId, frequency_, cmd_vel_topic.c_str(),
      armor_presence_topic.c_str(), target_yaw_topic.c_str());
  }

  ~CanSendNode() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

private:
  void sendFrame()
  {
    double vx = 0.0;
    double vy = 0.0;
    if (cmd_vel_received_ && (now() - last_cmd_vel_time_).seconds() <= cmd_vel_timeout_s_) {
      vx = vx_;
      vy = vy_;
    }

    const auto payload = encodeCommand(vx, vy, left_, behind_, right_, target_yaw_);
    struct can_frame frame {};
    frame.can_id = kCommandId;
    frame.can_dlc = 8;
    std::copy(payload.begin(), payload.end(), frame.data);

    if (::write(fd_, &frame, sizeof(frame)) < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "发送 0x%X 失败: %s",
        kCommandId, std::strerror(errno));
    }
  }

  std::string interface_;
  double frequency_ = 500.0;
  double cmd_vel_timeout_s_ = 0.1;
  int fd_ = -1;

  double vx_ = 0.0;
  double vy_ = 0.0;
  double target_yaw_ = 0.0;
  uint8_t left_ = 0;
  uint8_t behind_ = 0;
  uint8_t right_ = 0;
  bool cmd_vel_received_ = false;
  rclcpp::Time last_cmd_vel_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sentry_msgs::msg::ArmorPresence>::SharedPtr armor_presence_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_yaw_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace can_comm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<can_comm::CanSendNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("can_send"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
