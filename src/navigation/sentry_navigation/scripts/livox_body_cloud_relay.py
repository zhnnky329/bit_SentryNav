#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


class LivoxBodyCloudRelay(Node):
    def __init__(self):
        super().__init__('livox_body_cloud_relay')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, '/cloud_registered_body', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, '/livox/lidar_filtered', self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            'Relaying /livox/lidar_filtered (livox_frame) -> '
            '/cloud_registered_body (base_link) for relocalizer')

    def on_cloud(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'livox_frame', Time())
        except Exception:
            return

        try:
            raw = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.asarray(raw, dtype=np.float64).reshape(-1, 3)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'cannot read /livox/lidar_filtered: {exc}',
                throttle_duration_sec=2.0)
            return

        if points.shape[0] == 0:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        body = points @ rot.T + np.array(
            [translation.x, translation.y, translation.z])

        header = msg.header
        header.frame_id = 'base_link'
        out = point_cloud2.create_cloud_xyz32(
            header, body.astype(np.float32).tolist())
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxBodyCloudRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
