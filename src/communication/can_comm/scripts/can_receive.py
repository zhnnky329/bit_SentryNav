#!/usr/bin/env python3
import socket
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


YAW_FEEDBACK_ID = 0x149
# Linux struct can_frame uses host byte order for the ID, then DLC and padding.
CAN_FRAME = struct.Struct('=IB3x8s')


def decode_yaw(frame):
    if len(frame) != CAN_FRAME.size:
        return None
    can_id, dlc, data = CAN_FRAME.unpack(frame)
    if can_id != YAW_FEEDBACK_ID or dlc != 8:
        return None
    return int.from_bytes(data[6:8], 'big', signed=True) / 100.0


def main():
    rclpy.init()
    node = Node('can_receive')
    interface = node.declare_parameter('interface', 'can0').value
    yaw_topic = node.declare_parameter('yaw_topic', '/can_comm/yaw_deg').value
    publisher = node.create_publisher(Float32, yaw_topic, 10)
    try:
        with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
            mask = socket.CAN_SFF_MASK | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
            bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                           struct.pack('=II', YAW_FEEDBACK_ID, mask))
            bus.bind((interface,))
            bus.setblocking(False)
            node.get_logger().info(f'正在监听 {interface} 的 yaw 回传帧 0x149')

            def receive():
                # Bound each batch so a busy interface cannot monopolize the executor.
                for _ in range(64):
                    try:
                        frame = bus.recv(CAN_FRAME.size)
                    except BlockingIOError:
                        break
                    yaw = decode_yaw(frame)
                    if yaw is not None:
                        publisher.publish(Float32(data=yaw))

            node.create_timer(0.01, receive)
            rclpy.spin(node)
    except OSError as error:
        node.get_logger().error(f'读取 CAN 接口 {interface} 失败: {error}')
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
