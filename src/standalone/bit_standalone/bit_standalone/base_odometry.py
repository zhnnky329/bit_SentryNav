"""Publish odom -> base_link -> livox_frame; base_link follows the big gimbal."""

from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from bit_standalone.frames import reference_from_imu, transform_odometry


class BaseOdometry(Node):
    def __init__(self):
        super().__init__("bit_base_odometry")
        self.declare_parameters("", [
            ("base_to_lidar_xyz", [0.22, 0.0, 0.0]),
            ("base_to_lidar_rpy_deg", [0.0, 45.0, 0.0]),
            ("imu_to_lidar_xyz", [-0.011, -0.02329, 0.04412]),
            ("imu_to_lidar_rotation", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
        ])
        xyz, rpy, imu_xyz, imu_rotation = [
            self.get_parameter(name).value for name in (
                "base_to_lidar_xyz", "base_to_lidar_rpy_deg",
                "imu_to_lidar_xyz", "imu_to_lidar_rotation",
            )
        ]
        self.offset, self.rotation = reference_from_imu(xyz, rpy, imu_xyz, imu_rotation)
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        mount = TransformStamped()
        mount.header.stamp = self.get_clock().now().to_msg()
        mount.header.frame_id = "base_link"
        mount.child_frame_id = "livox_frame"
        mount.transform.translation.x, mount.transform.translation.y, mount.transform.translation.z = map(float, xyz)
        q = mount.transform.rotation
        q.x, q.y, q.z, q.w = map(float, Rotation.from_euler("xyz", rpy, degrees=True).as_quat())
        self.static_broadcaster.sendTransform(mount)
        self.publisher = self.create_publisher(Odometry, "/odom", 10)
        self.subscription = self.create_subscription(
            Odometry, "/aft_mapped_to_init", self.on_odometry, qos_profile_sensor_data
        )

    def on_odometry(self, raw):
        pose, twist = raw.pose.pose, raw.twist.twist
        p, q, v, w = transform_odometry(
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            [twist.linear.x, twist.linear.y, twist.linear.z],
            [twist.angular.x, twist.angular.y, twist.angular.z],
            self.offset, self.rotation,
        )
        odom = Odometry()
        odom.header = deepcopy(raw.header)
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, p)
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = map(float, q)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, v)
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float, w)
        # Covariance is unspecified; no independent chassis attitude is inferred.
        self.publisher.publish(odom)
        transform = TransformStamped()
        transform.header = deepcopy(odom.header)
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, p)
        transform.transform.rotation = deepcopy(odom.pose.pose.orientation)
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = BaseOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
