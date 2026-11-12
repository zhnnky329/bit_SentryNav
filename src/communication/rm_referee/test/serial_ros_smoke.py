#!/usr/bin/env python3
"""Linux PTY -> real RefereeNode -> ROS, plus Tx -> PTY. No physical device.
Source the freshly built workspace; use an unused ROS_DOMAIN_ID.
"""
import os
import pty
import select
import signal
import struct
import subprocess
import tempfile
import time
import tty
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from rm_referee_msgs.msg import EventData, GameRobotHP, RobotInteraction, RobotStatus
from rm_referee_msgs.srv import Tx


def crc(data, initial, polynomial):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


def frame(cmd, payload):
    raw = bytearray(b'\xa5' + struct.pack('<HB', len(payload), 1))
    raw.append(crc(raw, 0xff, 0x8c))
    raw.extend(struct.pack('<H', cmd) + payload)
    raw.extend(struct.pack('<H', crc(raw, 0xffff, 0x8408)))
    return bytes(raw)


def main():
    master, slave = pty.openpty()
    tty.setraw(slave)
    rclpy.init()
    node = rclpy.create_node('referee_serial_smoke')
    received = {name: [] for name in ('event_data', 'game_robot_hp', 'robot_interaction', 'robot_status')}
    subscriptions = [node.create_subscription(cls, '/rm_referee/' + name,
        lambda msg, key=name: received[key].append(msg), qos_profile_sensor_data)
        for name, cls in zip(received, (EventData, GameRobotHP, RobotInteraction, RobotStatus))]
    executable = Path(get_package_prefix('rm_referee')) / 'lib/rm_referee/referee_node'
    log = tempfile.NamedTemporaryFile(prefix='referee_serial_', suffix='.log', delete=False)
    proc = subprocess.Popen([str(executable), '--ros-args', '-p', 'enable_normal:=true',
        '-p', 'enable_vt:=false', '-p', 'record_raw_data:=false',
        '-p', 'normal_tty_device:=' + os.ttyname(slave)], stdout=log, stderr=log)

    def wait_for(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
            if proc.poll() is not None:
                raise AssertionError('node exited; inspect ' + log.name)
        raise AssertionError('timeout; inspect ' + log.name)

    def deliver(cmd, payload, topic):
        old = len(received[topic])
        os.write(master, frame(cmd, payload))
        wait_for(lambda: len(received[topic]) > old)
        return received[topic][-1]

    try:
        wait_for(lambda: all(node.count_publishers('/rm_referee/' + name) for name in received), 10)
        end = time.monotonic() + 1
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        hp = struct.pack('<12H', 1, 0, 3, 4, 0, 7, 600, 10000, 500, 9000, 45, 21)
        before_id = deliver(0x0003, hp, 'game_robot_hp')
        assert before_id.ally_outpost_hp == 65535
        status = bytes([7]) + bytes(16)
        deliver(0x0201, status, 'robot_status')
        message = deliver(0x0003, hp, 'game_robot_hp')
        assert (message.ally_outpost_hp, message.enemy_outpost_hp,
                message.ally_control_degree, message.enemy_control_degree) == (600, 500, 45, 21)
        # Deliberately split one frame into three serial writes.
        raw = frame(0x0101, struct.pack('<II', 0x12345678, 0x9abcdef0))
        os.write(master, raw[:3]); time.sleep(.02)
        os.write(master, raw[3:9]); time.sleep(.02)
        os.write(master, raw[9:])
        wait_for(lambda: bool(received['event_data']))
        assert received['event_data'][-1].event_data_2 == 0x9abcdef0
        msg = deliver(0x0301, struct.pack('<HHH', 0x0200, 9, 7) + b'\xaa\xbb', 'robot_interaction')
        assert (msg.sender_id, msg.receiver_id, list(msg.payload)) == (9, 7, [170, 187])
        old = len(received['robot_interaction'])
        for sender, receiver, subcmd in [(109,7,0x0200), (9,3,0x0200), (9,7,0x0120)]:
            os.write(master, frame(0x0301, struct.pack('<HHH', subcmd,sender,receiver) + b'\x00'))
        # A following valid event acts as a barrier: all earlier bytes were decoded.
        deliver(0x0101, bytes(8), 'event_data')
        end = time.monotonic() + .3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        assert len(received['robot_interaction']) == old, 'invalid interaction published'
        deliver(0x0201, bytes([107]) + bytes(16), 'robot_status')
        message = deliver(0x0003, hp, 'game_robot_hp')
        assert (message.ally_outpost_hp, message.enemy_outpost_hp) == (500,600)
        client = node.create_client(Tx, '/rm_referee/tx')
        assert client.wait_for_service(timeout_sec=5)
        outgoing = frame(0x0301, struct.pack('<HHHI', 0x0120,107,0x8080,0))
        request = Tx.Request(); request.data = list(outgoing)
        future = client.call_async(request)
        wait_for(future.done)
        assert future.result().ok
        echoed = b''
        deadline = time.monotonic() + 3
        while len(echoed) < len(outgoing) and time.monotonic() < deadline:
            if select.select([master], [], [], .1)[0]:
                echoed += os.read(master, 4096)
        assert echoed == outgoing, 'Tx changed or truncated bytes'
        print('PASS: split serial frames, ROS fields, unknown/red/blue ID, interaction filtering, Tx byte equality')
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill(); proc.wait()
            print('WARNING: node did not exit gracefully')
        node.destroy_node(); rclpy.shutdown()
        os.close(master); os.close(slave); log.close()
        print('Node log:', log.name)
    assert proc.returncode == 0, 'node shutdown failed'


if __name__ == '__main__':
    main()
