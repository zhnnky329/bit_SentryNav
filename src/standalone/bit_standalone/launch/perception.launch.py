import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    replay = mode == "replay"
    pointlio_config = LaunchConfiguration("pointlio_config").perform(context)
    with open(pointlio_config, encoding="utf-8") as stream:
        lio = yaml.safe_load(stream)["/**"]["ros__parameters"]
    components = []
    if not replay:
        driver_config = LaunchConfiguration("driver_config").perform(context)
        if not os.path.isfile(driver_config):
            raise RuntimeError("live mode requires driver_config:=/absolute/path/to/MID360_config.json")
        components.append(ComposableNode(
            package="livox_ros_driver2", plugin="livox_ros::DriverNode", name="livox_driver_node",
            parameters=[{
                "xfer_format": 1, "multi_topic": 0, "data_src": 0,
                "publish_freq": 20.0, "output_data_type": 0, "frame_id": "livox_frame",
                "user_config_path": driver_config, "enable_internal_lidar_merge": False,
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        ))
    components.append(ComposableNode(
        package="point_lio", plugin="point_lio::LaserMappingNode", name="laserMapping",
        parameters=[pointlio_config, {
            "use_sim_time": replay,
            "publish.tf_send_en": False,
            "publish.world_frame": "odom",
            "publish.imu_frame": "livox_imu",
            "publish.scan_bodyframe_pub_en": False,
            "mapping.extrinsic_est_en": False,
        }],
        extra_arguments=[{"use_intra_process_comms": True}],
    ))
    return [
        ComposableNodeContainer(
            name="livox_pointlio_container", namespace="", package="rclcpp_components",
            executable="component_container_mt", output="screen",
            composable_node_descriptions=components,
        ),
        Node(
            package="bit_standalone", executable="base_odometry", name="bit_base_odometry",
            output="screen", parameters=[LaunchConfiguration("robot_config"), {
                "use_sim_time": replay,
                "imu_to_lidar_xyz": lio["mapping"]["extrinsic_T"],
                "imu_to_lidar_rotation": lio["mapping"]["extrinsic_R"],
            }],
        ),
        Node(
            package="rviz2", executable="rviz2", output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{"use_sim_time": replay}],
            arguments=["-d", os.path.join(get_package_share_directory("bit_standalone"), "config", "perception.rviz")],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory("bit_standalone")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="replay", choices=["replay", "live"]),
        DeclareLaunchArgument("driver_config", default_value=os.path.join(share, "config", "MID360_config.json")),
        DeclareLaunchArgument("pointlio_config", default_value=os.path.join(share, "config", "pointlio.yaml")),
        DeclareLaunchArgument("robot_config", default_value=os.path.join(share, "config", "robot.yaml")),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
