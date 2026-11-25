"""Rigid-body transforms; T_A_B maps coordinates from B into A."""

import numpy as np
from scipy.spatial.transform import Rotation


def world_velocity_to_base(vx, vy, quaternion_xyzw):
    """Express the odom-plane velocity in base_link using the same rotation as TF."""
    return Rotation.from_quat(quaternion_xyzw).inv().apply([vx, vy, 0.0])


def reference_from_imu(reference_lidar_xyz, reference_lidar_rpy_deg, imu_lidar_xyz, imu_lidar_rotation):
    r_reference_lidar = Rotation.from_euler("xyz", reference_lidar_rpy_deg, degrees=True).as_matrix()
    r_imu_lidar = np.asarray(imu_lidar_rotation, dtype=float).reshape(3, 3)
    r_reference_imu = r_reference_lidar @ r_imu_lidar.T
    p_reference_imu = np.asarray(reference_lidar_xyz) - r_reference_imu @ np.asarray(imu_lidar_xyz)
    return p_reference_imu, r_reference_imu


def transform_odometry(position, quaternion_xyzw, velocity, angular_velocity, p_reference_imu, r_reference_imu):
    r_world_imu = Rotation.from_quat(quaternion_xyzw).as_matrix()
    r_world_reference = r_world_imu @ r_reference_imu.T
    offset_world = r_world_reference @ p_reference_imu
    p_world_reference = np.asarray(position) - offset_world
    omega_world = r_world_imu @ np.asarray(angular_velocity)
    v_world_reference = r_world_imu @ np.asarray(velocity) - np.cross(omega_world, offset_world)
    return (
        p_world_reference,
        Rotation.from_matrix(r_world_reference).as_quat(),
        r_world_reference.T @ v_world_reference,
        r_world_reference.T @ omega_world,
    )
