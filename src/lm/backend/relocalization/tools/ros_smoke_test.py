#!/usr/bin/env python3
# Copyright 2026 bw_nav contributors
# SPDX-License-Identifier: Apache-2.0
"""End-to-end smoke test for the relocalizer node.

Publishes one recorded sweep as a sensor_msgs/PointCloud2, waits for the node to
localize, checks the broadcast world->body transform, and finally exercises the
``~/relocalize`` trigger service.  Run the node first, e.g.:

    ros2 launch relocalizer relocalizer.launch.py sweep_topic:=/sweep_test

then:

    python3 src/lm/backend/relocalization/tools/ros_smoke_test.py --sweep .validation/sweep.pcd
"""

from __future__ import annotations

import argparse
import json
import math
import sys

import numpy as np

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def read_pcd(path):
    with open(path, "rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("unexpected end of PCD header")
            words = line.decode("ascii", "replace").strip().split()
            if not words:
                continue
            header[words[0]] = words[1:]
            if words[0] == "DATA":
                break
        count = int(header["POINTS"][0])
        fields = header.get("FIELDS", [])
        if len(fields) < 3:
            raise RuntimeError("PCD must contain x y z fields")
        sizes = [int(value) for value in header.get("SIZE", [])]
        point_step = sum(sizes)
        payload = stream.read(count * point_step)
    points = np.frombuffer(payload, dtype=np.float32).reshape(count, -1)
    return np.ascontiguousarray(points[:, :3], dtype=np.float32)


class SmokeNode(Node):
    def __init__(self, sweep, body_frame, world_frame, expected):
        super().__init__("relocalizer_smoke_test")
        self.sweep = sweep
        self.body_frame = body_frame
        self.world_frame = world_frame
        self.expected = expected
        self.status = None
        self.pose = None
        self.proposals = None
        self.transform = None
        self.service_result = None
        self.service_future = None
        self.done = False

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PointCloud2, "/sweep_test", qos)
        self.create_subscription(String, "/relocalizer/status", self.on_status, 10)
        self.create_subscription(PoseStamped, "/relocalizer/pose", self.on_pose, 10)
        self.create_subscription(PoseArray, "/relocalizer/proposals", self.on_proposals, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.client = self.create_client(Trigger, "/relocalizer/relocalize")
        self.timer = self.create_timer(0.15, self.publish_sweep)
        self.deadline = self.get_clock().now().nanoseconds + int(30.0e9)

    def publish_sweep(self):
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.body_frame
        message.height = 1
        message.width = int(self.sweep.shape[0])
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * message.width
        message.data = self.sweep.astype("<f4").tobytes()
        message.is_dense = True
        self.publisher.publish(message)
        if self.get_clock().now().nanoseconds > self.deadline:
            self.done = True

    def on_status(self, message):
        try:
            self.status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if (self.status.get("state") == "TRACKING" and self.pose is not None
                and self.service_future is None):
            self.call_service()

    def call_service(self):
        if not self.client.service_is_ready():
            return
        self.service_future = self.client.call_async(Trigger.Request())

    def on_pose(self, message):
        self.pose = message

    def on_proposals(self, message):
        self.proposals = message

    def on_tf(self, message):
        for transform in message.transforms:
            if (transform.header.frame_id == self.world_frame
                    and transform.child_frame_id == self.body_frame):
                self.transform = transform

    def spin_until_done(self):
        while rclpy.ok() and not self.done:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.service_future is not None and self.service_future.done():
                response = self.service_future.result()
                self.service_result = {"success": response.success, "message": response.message}
                self.service_future = None
            status = self.status or {}
            if (status.get("state") == "TRACKING" and self.transform is not None
                    and self.service_result is not None):
                break

    def report(self):
        errors = []
        if self.status is None or self.status.get("state") != "TRACKING":
            errors.append("node never reached TRACKING")
        if self.transform is None:
            errors.append("no world->body transform observed")
        if self.pose is None:
            errors.append("no pose published")
        if self.proposals is None or not self.proposals.poses:
            errors.append("no proposals published")
        if self.service_result is None:
            errors.append("relocalize service was not exercised")
        elif not self.service_result["success"]:
            errors.append(f"relocalize service failed: {self.service_result['message']}")

        if self.transform is not None:
            translation = self.transform.transform.translation
            rotation = self.transform.transform.rotation
            heading = math.degrees(math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)))
            distance = math.hypot(
                translation.x - self.expected[0], translation.y - self.expected[1])
            turn = abs((heading - self.expected[2] + 180.0) % 360.0 - 180.0)
            if distance > 0.5:
                errors.append(f"translation error {distance:.3f} m exceeds 0.5 m")
            if turn > 5.0:
                errors.append(f"heading error {turn:.2f} deg exceeds 5 deg")
            result = {
                "x": translation.x, "y": translation.y, "heading_deg": heading,
                "distance_error": distance, "heading_error": turn,
            }
        else:
            result = {}
        return {"ok": not errors, "errors": errors, "transform": result,
                "status": self.status, "service": self.service_result}


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", required=True, help="PCD file with one body-frame sweep")
    parser.add_argument("--body-frame", default="body")
    parser.add_argument("--world-frame", default="map")
    parser.add_argument("--expect-x", type=float, default=0.0)
    parser.add_argument("--expect-y", type=float, default=0.0)
    parser.add_argument("--expect-heading", type=float, default=0.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = SmokeNode(
        read_pcd(args.sweep), args.body_frame, args.world_frame,
        (args.expect_x, args.expect_y, args.expect_heading))
    try:
        node.spin_until_done()
        report = node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
