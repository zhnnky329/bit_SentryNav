"""Publish candidate payload bytes for recording only; never opens a CAN interface."""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from bit_standalone.protocol_preview import encode_translation_preview


class CanPreview(Node):
    def __init__(self):
        super().__init__("bit_can_preview")
        self.scale = self.declare_parameter("speed_at_full_scale_mps", 1.0).value
        if not math.isfinite(self.scale) or self.scale <= 0.0:
            raise ValueError("speed_at_full_scale_mps must be positive and finite")
        self.publisher = self.create_publisher(UInt8MultiArray, "/bit/can_0x190_payload_preview", 1)
        self.subscription = self.create_subscription(
            TwistStamped, "/bit/cmd_vel_base", self.on_command, 1
        )

    def on_command(self, message):
        result = UInt8MultiArray()
        # Input is gimbal-relative translation; the embedded side owns gimbal-to-chassis rotation.
        result.data = list(encode_translation_preview(
            message.twist.linear.x, message.twist.linear.y, self.scale
        ))
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = CanPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
