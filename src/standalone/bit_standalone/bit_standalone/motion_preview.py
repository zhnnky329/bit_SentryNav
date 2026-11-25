"""Convert odom velocity into base_link (big gimbal axes); no hardware I/O."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from bit_standalone.frames import world_velocity_to_base


class MotionPreview(Node):
    def __init__(self):
        super().__init__("bit_motion_preview")
        self.timeout = self.declare_parameter("timeout_s", 0.1).value
        if not math.isfinite(self.timeout) or self.timeout <= 0.0:
            raise ValueError("timeout_s must be positive and finite")
        self.command = None
        self.orientation = None
        self.command_time = self.odom_time = float("-inf")
        self.odom_stamp_ns = 0
        self.command_sub = self.create_subscription(Twist, "/bit/cmd_vel_world", self.on_command, 1)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odometry, qos_profile_sensor_data)
        self.publisher = self.create_publisher(TwistStamped, "/bit/cmd_vel_base", 1)
        self.timer = self.create_timer(0.01, self.publish_preview, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, message):
        self.command = (message.linear.x, message.linear.y)
        self.command_time = time.monotonic()

    def on_odometry(self, message):
        q = message.pose.pose.orientation
        self.orientation = (q.x, q.y, q.z, q.w)
        self.odom_time = time.monotonic()
        self.odom_stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def publish_preview(self):
        now = time.monotonic()
        result = TwistStamped()
        ros_now = self.get_clock().now()
        result.header.stamp = ros_now.to_msg()
        result.header.frame_id = "base_link"
        odom_age = (ros_now.nanoseconds - self.odom_stamp_ns) * 1e-9
        if (now - self.command_time <= self.timeout and now - self.odom_time <= self.timeout
                and 0.0 <= odom_age <= self.timeout):
            values = (*self.command, *self.orientation)
            if all(math.isfinite(value) for value in values) and sum(value * value for value in self.orientation) > 0.0:
                velocity = world_velocity_to_base(self.command[0], self.command[1], self.orientation)
                result.twist.linear.x, result.twist.linear.y, result.twist.linear.z = map(float, velocity)
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = MotionPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
