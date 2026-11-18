#!/usr/bin/env python3
import socket
import struct

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sentry_msgs.msg import ArmorPresence


COMMAND_ID = 0x190
# Linux struct can_frame uses host byte order for the ID, then DLC and padding.
CAN_FRAME = struct.Struct('=IB3x8s')


def encode_command(vx, vy, left, behind, right, target_yaw_deg=0.0):
    """Encode the 0x190 payload."""
    vx_q = int(max(-1.0, min(1.0, vx)) * 127.0)
    vy_q = int(max(-1.0, min(1.0, vy)) * 127.0)
    yaw_q = int(max(-180.0, min(180.0, target_yaw_deg)) * 100.0)
    return bytes((
        vx_q & 0xFF,
        vy_q & 0xFF,
        (yaw_q >> 8) & 0xFF,
        yaw_q & 0xFF,
        1 if left else 0,
        1 if behind else 0,
        1 if right else 0,
        0,
    ))


class CanSend(Node):
    def __init__(self):
        super().__init__('can_send')
        self.interface = self.declare_parameter('interface', 'can0').value
        self.frequency = self.declare_parameter('send_frequency_hz', 500.0).value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        armor_presence_topic = self.declare_parameter(
            'armor_presence_topic', '/detector/armor_presence').value
        self.cmd_vel_timeout = self.declare_parameter(
            'cmd_vel_timeout_s', 0.1).value
        if self.frequency <= 0.0:
            self.frequency = 500.0

        self.vx = 0.0
        self.vy = 0.0
        self.left = 0
        self.behind = 0
        self.right = 0
        self.last_cmd_vel_time = None

        self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)
        self.create_subscription(
            ArmorPresence, armor_presence_topic, self.on_armor_presence, 10)
        self.get_logger().info(
            f'订阅 {cmd_vel_topic} 和 {armor_presence_topic}，'
            f'准备在 {self.interface} 上发送 0x{COMMAND_ID:X}')

    def start(self, bus):
        self.bus = bus
        self.create_timer(1.0 / self.frequency, self.send_frame)
        self.get_logger().info(
            f'CAN 发送节点已启动 | {self.interface} '
            f'0x{COMMAND_ID:X} @ {self.frequency:.0f} Hz')

    def on_cmd_vel(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.last_cmd_vel_time = self.get_clock().now()

    def on_armor_presence(self, msg):
        self.left = msg.left
        self.behind = msg.behind
        self.right = msg.right

    def send_frame(self):
        if (self.last_cmd_vel_time is None or
                (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds
                / 1e9 > self.cmd_vel_timeout):
            vx = 0.0
            vy = 0.0
        else:
            vx = self.vx
            vy = self.vy

        # target_yaw 按当前协议固定发 0，接入 /target_yaw 后再传入实值。
        data = encode_command(vx, vy, self.left, self.behind, self.right)
        try:
            self.bus.send(CAN_FRAME.pack(COMMAND_ID, 8, data))
        except OSError as error:
            self.get_logger().error(
                f'发送 0x{COMMAND_ID:X} 失败: {error}', throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = CanSend()
    try:
        with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
            bus.bind((node.interface,))
            bus.setblocking(False)
            node.start(bus)
            rclpy.spin(node)
    except OSError as error:
        node.get_logger().error(f'打开 CAN 接口 {node.interface} 失败: {error}')
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
