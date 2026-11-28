"""Transmit base_link planar velocity through the IRobot-SentryNav27 0x190 layout."""

import math
import socket
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from bit_standalone.can_transport import velocity_can_frame


class CanBridge(Node):
    def __init__(self):
        super().__init__("bit_can_bridge")
        interface = self.declare_parameter("interface", "can0").value
        frequency = self.declare_parameter("send_frequency_hz", 500.0).value
        self.timeout = self.declare_parameter("cmd_vel_timeout_s", 0.1).value
        self.scale = self.declare_parameter("speed_at_full_scale_mps", 1.0).value
        if any(not math.isfinite(value) or value <= 0.0 for value in (frequency, self.timeout, self.scale)):
            raise ValueError("send_frequency_hz, cmd_vel_timeout_s and speed_at_full_scale_mps must be positive and finite")
        if self.get_parameter("use_sim_time").value:
            raise ValueError("CAN transmission requires live wall time, not use_sim_time")
        self.command = (0.0, 0.0)
        self.received_at = float("-inf")
        self.stamp_ns = 0
        # Interface setup/bitrate is an operator responsibility; never change the network here.
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.setblocking(False)
        try:
            self.socket.bind((interface,))
        except OSError:
            self.socket.close()
            raise
        self.subscription = self.create_subscription(TwistStamped, "/bit/cmd_vel_base", self.on_command, 1)
        self.timer = self.create_timer(1.0 / frequency, self.send_frame, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"CAN enabled: {interface}, 0x190 @ {frequency:g} Hz, base_link vx/vy, byte7=1")

    def on_command(self, message):
        if message.header.frame_id != "base_link":
            self.received_at = float("-inf")
            raise ValueError(f"Expected base_link velocity; received {message.header.frame_id!r}")
        self.command = (message.twist.linear.x, message.twist.linear.y)
        self.received_at = time.monotonic()
        self.stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def send_frame(self):
        frame = velocity_can_frame(
            *self.command, self.scale,
            time.monotonic() - self.received_at,
            (self.get_clock().now().nanoseconds - self.stamp_ns) * 1e-9,
            self.timeout,
        )
        # Socket errors propagate and stop this node; do not silently report a successful send.
        if self.socket.send(frame) != len(frame):
            raise OSError("Incomplete SocketCAN frame write")

    def close(self):
        self.timer.cancel()
        try:
            self.socket.send(velocity_can_frame(0.0, 0.0, self.scale, 0.0, 0.0, self.timeout))
        except OSError as error:
            self.get_logger().error(f"Final zero CAN command could not be sent: {error}")
        finally:
            self.socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CanBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
