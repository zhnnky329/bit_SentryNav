#!/usr/bin/env python3
"""Record a live ROS 2 graph and a SocketCAN interface in one session."""

import argparse
import csv
from datetime import datetime
import errno
import os
from pathlib import Path
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time


# Linux socket options not exported by Python on Ubuntu 24.04.
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
SO_RXQ_OVFL = getattr(socket, "SO_RXQ_OVFL", 40)
CAN_RAW_ERR_FILTER = getattr(socket, "CAN_RAW_ERR_FILTER", 2)
CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG = 0x80000000, 0x40000000, 0x20000000
CAN_EFF_MASK, CAN_SFF_MASK = 0x1FFFFFFF, 0x7FF

TIMESPEC = struct.Struct("@ll")
UINT32 = struct.Struct("=I")
ANCILLARY_SIZE = socket.CMSG_SPACE(TIMESPEC.size) + socket.CMSG_SPACE(UINT32.size)
CAN_COLUMNS = (
    "unix_time_ns", "direction", "can_id", "frame_type", "id_format",
    "length", "payload_hex", "fd_flags_hex", "socket_dropped_total",
)


def open_can(interface):
    bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        bus.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
        bus.setsockopt(socket.SOL_SOCKET, SO_RXQ_OVFL, 1)
        bus.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, UINT32.pack(CAN_EFF_MASK))
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        bus.bind((interface,))
        error = bus.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
        if error:
            raise OSError(error, os.strerror(error))
        bus.settimeout(0.2)
    except OSError:
        bus.close()
        raise
    return bus


def can_row(frame, ancillary, flags):
    timestamp = time.time_ns()
    dropped = 0
    for level, event, value in ancillary:
        if level == socket.SOL_SOCKET and event == SO_TIMESTAMPNS:
            seconds, nanoseconds = TIMESPEC.unpack_from(value)
            timestamp = seconds * 1_000_000_000 + nanoseconds
        elif level == socket.SOL_SOCKET and event == SO_RXQ_OVFL:
            dropped = UINT32.unpack_from(value)[0]

    raw_id = UINT32.unpack_from(frame)[0]
    extended = bool(raw_id & CAN_EFF_FLAG)
    error = bool(raw_id & CAN_ERR_FLAG)
    remote = bool(raw_id & CAN_RTR_FLAG)
    fd = len(frame) == 72
    length = frame[4]
    can_id = raw_id & (CAN_EFF_MASK if extended or error else CAN_SFF_MASK)
    direction = "TX_LOCAL" if flags & socket.MSG_DONTROUTE else "RX_BUS"
    frame_type = "error" if error else "rtr" if remote else "fd" if fd else "data"
    payload = "" if remote else frame[8:8 + length].hex().upper()
    return (
        timestamp, direction, f"0x{can_id:08X}" if extended or error else f"0x{can_id:03X}",
        frame_type, "extended" if extended else "" if error else "standard", length,
        payload, f"0x{frame[5]:02X}" if fd else "", dropped,
    )


def stop_rosbag(process):
    if process.poll() is not None:
        return process.returncode
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        print("ROS bag 停止超时，正在终止录制进程。", file=sys.stderr, flush=True)
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-interface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument(
        "--output-dir", type=Path, default=Path(__file__).resolve().parents[1] / "bag" / "field_capture",
        help="parent directory for capture sessions",
    )
    args = parser.parse_args()
    ros2 = shutil.which("ros2")
    if ros2 is None:
        parser.exit(1, "找不到 ros2。请先 source Jazzy 和工程的 install/setup 文件。\n")

    stopped = False

    def request_stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    session = None
    try:
        with open_can(args.can_interface) as bus:
            args.output_dir.mkdir(parents=True, exist_ok=True)
            prefix = datetime.now().strftime("%Y%m%d_%H%M%S_")
            session = Path(tempfile.mkdtemp(prefix=prefix, dir=args.output_dir)).resolve()
            with (session / "can.csv").open("w", newline="", encoding="utf-8") as can_file:
                writer = csv.writer(can_file)
                writer.writerow(CAN_COLUMNS)
                process = subprocess.Popen(
                    [ros2, "bag", "record", "--all", "--include-hidden-topics",
                     "--storage", "mcap", "--max-bag-size", "1073741824",
                     "--disable-keyboard-controls", "--output", str(session / "rosbag")],
                    start_new_session=True,
                )
                print(f"录制目录 {session}", flush=True)
                print(f"监听 {args.can_interface}，等待 ROS bag 显示 Recording。Ctrl+C 停止。", flush=True)

                frames = 0
                dropped = 0
                try:
                    next_flush = time.monotonic()
                    while not stopped:
                        now = time.monotonic()
                        if now >= next_flush:
                            can_file.flush()
                            next_flush = now + 0.2
                            if process.poll() is not None:
                                raise RuntimeError(f"ROS bag 提前退出，退出码 {process.returncode}")
                        try:
                            frame, ancillary, flags, _address = bus.recvmsg(72, ANCILLARY_SIZE)
                        except TimeoutError:
                            continue
                        row = can_row(frame, ancillary, flags)
                        writer.writerow(row)
                        frames += 1
                        dropped = max(dropped, row[-1])
                finally:
                    rosbag_status = stop_rosbag(process)
            if rosbag_status != 0 or not (session / "rosbag" / "metadata.yaml").is_file():
                raise RuntimeError(f"ROS bag 未确认保存完成，退出码 {rosbag_status}")
            print(f"已保存 {session}")
            print(f"CAN 帧 {frames}，SocketCAN 接收队列报告丢帧 {dropped}。")
            return 0
    except (OSError, RuntimeError) as exc:
        if isinstance(exc, OSError) and exc.errno in (errno.ENODEV, errno.ENETDOWN) and session is None:
            print(f"无法监听 {args.can_interface}。请检查实车 CAN 接口是否存在并已启用。", file=sys.stderr)
        print(f"录制失败：{exc}", file=sys.stderr)
        if session is not None:
            print(f"已有文件保存在 {session}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
