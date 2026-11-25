"""Offline mathematical checks; no ROS, compilation, or hardware required."""

import unittest

import numpy as np
from numpy.testing import assert_allclose
from scipy.spatial.transform import Rotation

from bit_standalone.frames import (
    reference_from_imu, transform_odometry, world_velocity_to_base,
)


class FrameChecks(unittest.TestCase):
    def test_gimbal_spin_does_not_move_navigation_center(self):
        offset, rotation = reference_from_imu(
            [0.22, 0, 0], [0, 45, 0], [-0.011, -0.02329, 0.04412], np.eye(3).ravel()
        )
        for yaw in (-170, -90, 0, 70, 179):
            heading = Rotation.from_euler("z", yaw, degrees=True)
            imu_heading = heading * Rotation.from_matrix(rotation)
            arm = heading.apply(offset)
            omega_world = np.array([0, 0, 1.3])
            gimbal = transform_odometry(
                np.array([1, 2, 0]) + arm, imu_heading.as_quat(),
                imu_heading.inv().apply(np.cross(omega_world, arm)),
                imu_heading.inv().apply(omega_world), offset, rotation,
            )
            assert_allclose(gimbal[0], [1, 2, 0], atol=1e-12)
            assert_allclose(Rotation.from_quat(gimbal[1]).as_matrix(), heading.as_matrix(), atol=1e-12)
            assert_allclose(gimbal[2], [0, 0, 0], atol=1e-12)
            assert_allclose(gimbal[3], [0, 0, 1.3], atol=1e-12)

    def test_translation_and_gimbal_spin_recover_base_velocity(self):
        offset, rotation = reference_from_imu(
            [0.22, 0, 0], [0, 45, 0], [-0.011, -0.02329, 0.04412], np.eye(3).ravel()
        )
        heading = Rotation.from_euler("z", 90, degrees=True)
        imu_heading = heading * Rotation.from_matrix(rotation)
        arm = heading.apply(offset)
        velocity_world = np.array([0.2, -0.1, 0])
        omega_world = np.array([0, 0, -0.8])
        gimbal = transform_odometry(
            np.array([3, -2, 0]) + arm, imu_heading.as_quat(),
            imu_heading.inv().apply(velocity_world + np.cross(omega_world, arm)),
            imu_heading.inv().apply(omega_world), offset, rotation,
        )
        assert_allclose(gimbal[2], heading.inv().apply(velocity_world), atol=1e-12)
        command = world_velocity_to_base(0.2, -0.1, gimbal[1])
        assert_allclose(command, gimbal[2], atol=1e-12)

    def test_world_velocity_at_ninety_degree_heading(self):
        heading = Rotation.from_euler("z", 90, degrees=True).as_quat()
        assert_allclose(world_velocity_to_base(1.0, 0.0, heading), [0, -1, 0], atol=1e-12)
        assert_allclose(world_velocity_to_base(0.0, 1.0, heading), [1, 0, 0], atol=1e-12)

    def test_tilted_base_velocity_matches_full_tf_rotation(self):
        attitude = Rotation.from_euler("xyz", [12, 25, 90], degrees=True)
        command = world_velocity_to_base(0.2, -0.1, attitude.as_quat())
        assert_allclose(attitude.apply(command), [0.2, -0.1, 0], atol=1e-12)

    def test_world_velocity_round_trip(self):
        for yaw in (-150, -30, 0, 45, 170):
            heading = Rotation.from_euler("z", yaw, degrees=True)
            velocity = world_velocity_to_base(0.2, -0.1, heading.as_quat())
            assert_allclose(heading.apply(velocity), [0.2, -0.1, 0], atol=1e-12)

    def test_identity_preserves_motion(self):
        p, q, v, w = transform_odometry(
            [1, 2, 3], [0, 0, 0, 1], [0.4, -0.2, 0], [0, 0, 0.5], np.zeros(3), np.eye(3)
        )
        assert_allclose(p, [1, 2, 3])
        assert_allclose(q, [0, 0, 0, 1])
        assert_allclose(v, [0.4, -0.2, 0])
        assert_allclose(w, [0, 0, 0.5])

    def test_45_degree_mount_keeps_forward_speed(self):
        mount = Rotation.from_euler("y", 45, degrees=True)
        p, q, v, w = transform_odometry(
            [0.22, 0, 0], mount.as_quat(), mount.inv().apply([1, 0, 0]),
            [0, 0, 0], np.array([0.22, 0, 0]), mount.as_matrix(),
        )
        assert_allclose(p, [0, 0, 0], atol=1e-12)
        assert_allclose(Rotation.from_quat(q).as_matrix(), np.eye(3), atol=1e-12)
        assert_allclose(v, [1, 0, 0], atol=1e-12)

    def test_rotation_about_chassis_center_has_zero_translation(self):
        mount = Rotation.from_euler("y", 45, degrees=True)
        heading = Rotation.from_euler("z", 90, degrees=True)
        r_world_imu = heading * mount
        offset_world = heading.apply([0.22, 0, 0])
        omega_world = np.array([0, 0, 2])
        v_world_imu = np.cross(omega_world, offset_world)
        p, q, v, w = transform_odometry(
            np.array([3, 4, 0]) + offset_world, r_world_imu.as_quat(),
            r_world_imu.inv().apply(v_world_imu), r_world_imu.inv().apply(omega_world),
            np.array([0.22, 0, 0]), mount.as_matrix(),
        )
        assert_allclose(p, [3, 4, 0], atol=1e-12)
        assert_allclose(Rotation.from_quat(q).as_matrix(), heading.as_matrix(), atol=1e-12)
        assert_allclose(v, [0, 0, 0], atol=1e-12)
        assert_allclose(w, [0, 0, 2], atol=1e-12)

    def test_sensor_internal_extrinsic_composition(self):
        r_base_lidar = Rotation.from_euler("xyz", [0, 45, 10], degrees=True).as_matrix()
        r_imu_lidar = Rotation.from_euler("xyz", [2, 3, 4], degrees=True).as_matrix()
        t_imu_lidar = np.array([-0.011, -0.02329, 0.04412])
        p, r = reference_from_imu([0.22, 0, 0], [0, 45, 10], t_imu_lidar, r_imu_lidar.ravel())
        assert_allclose(r @ r_imu_lidar, r_base_lidar, atol=1e-12)
        assert_allclose(p + r @ t_imu_lidar, [0.22, 0, 0], atol=1e-12)

    def test_tf_chain_reconstructs_lidar_pose(self):
        base_lidar_xyz = np.array([0.22, 0, 0])
        base_lidar_rotation = Rotation.from_euler("y", 45, degrees=True)
        imu_lidar_xyz = np.array([-0.011, -0.02329, 0.04412])
        imu_lidar_rotation = Rotation.from_euler("xyz", [2, -3, 1], degrees=True)
        offset, rotation = reference_from_imu(
            base_lidar_xyz, [0, 45, 0], imu_lidar_xyz, imu_lidar_rotation.as_matrix().ravel()
        )
        imu_pose = Rotation.from_euler("xyz", [10, 35, 80], degrees=True)
        imu_position = np.array([2, 3, 0.4])
        p, q, _, _ = transform_odometry(
            imu_position, imu_pose.as_quat(), [0, 0, 0], [0, 0, 0], offset, rotation
        )
        base_pose = Rotation.from_quat(q)
        assert_allclose(p + base_pose.apply(base_lidar_xyz),
                        imu_position + imu_pose.apply(imu_lidar_xyz), atol=1e-12)
        assert_allclose((base_pose * base_lidar_rotation).as_matrix(),
                        (imu_pose * imu_lidar_rotation).as_matrix(), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
