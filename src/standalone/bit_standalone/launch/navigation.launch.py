import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_setup(context):
    share = get_package_share_directory("bit_standalone")
    mode = LaunchConfiguration("mode").perform(context)
    replay = mode == "replay"
    enable_can = LaunchConfiguration("enable_can").perform(context) == "true"
    if enable_can and replay:
        raise RuntimeError("enable_can is only allowed with mode:=live; replay must not drive the car")
    map_file = os.path.abspath(os.path.expanduser(LaunchConfiguration("map").perform(context)))
    pcd_file = os.path.abspath(os.path.expanduser(LaunchConfiguration("pcd").perform(context)))
    radius = float(LaunchConfiguration("robot_radius").perform(context))
    if not all(os.path.isfile(path) for path in (map_file, pcd_file)):
        raise RuntimeError("map and pcd must point to existing files for the same scene")
    if not math.isfinite(radius) or radius <= 0.0:
        raise RuntimeError("robot_radius must be a positive finite navigation envelope radius in meters")
    relocalizer_share = get_package_share_directory("relocalizer")

    params = ParameterFile(RewrittenYaml(
        source_file=os.path.join(share, "config", "nav2_jazzy.yaml"),
        param_rewrites={
            "use_sim_time": "true" if replay else "false",
            "robot_radius": str(radius),
            "default_nav_to_pose_bt_xml": os.path.join(share, "behavior_tree", "navigate_to_pose.xml"),
            "planner_server.ros__parameters.MincoPlanner.rog_map.projection.prior_map.yaml_path": map_file,
        },
        convert_types=True,
    ), allow_substs=True)

    actions = [
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(share, "launch", "perception.launch.py")),
                    launch_arguments={
                        "mode": mode,
                        "driver_config": LaunchConfiguration("driver_config"),
                        "robot_config": LaunchConfiguration("robot_config"),
                        "rviz": "false",
                    }.items(),
                ),
            ],
            scoped=True,
        ),
        Node(
            package="relocalizer", executable="livox_body_cloud_relay", name="livox_body_cloud_relay",
            output="screen", parameters=[{
                "use_sim_time": replay,
                "body_frame": "base_link",
                "input_topic": LaunchConfiguration("relocalizer_input_topic"),
                "input_format": LaunchConfiguration("relocalizer_input_format"),
            }],
        ),
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(relocalizer_share, "launch", "relocalizer.launch.py")),
                    launch_arguments={
                        "config": os.path.join(relocalizer_share, "config", "relocalizer.yaml"),
                        "plan_descriptor": map_file,
                        "surface_map": pcd_file,
                        "sweep_topic": "/cloud_registered_body",
                        "body_frame": "base_link",
                        "world_frame": "map",
                        "odom_frame": "odom",
                        "automatic": "true",
                        "use_sim_time": "true" if replay else "false",
                        "rviz": "false",
                    }.items(),
                ),
            ],
            scoped=True,
        ),
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            output="screen", parameters=[params, {"yaml_filename": map_file}],
        ),
        Node(
            package="nav2_planner", executable="planner_server", name="planner_server",
            output="screen", parameters=[params],
        ),
        Node(
            package="nav2_controller", executable="controller_server", name="controller_server",
            output="screen", parameters=[params],
            remappings=[
                ("/aft_mapped_to_init", "/odom"),
                ("cmd_vel", "/bit/cmd_vel_world"),
                ("/cmd_vel_mpc", "/bit/cmd_vel_mpc_preview"),
            ],
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
            output="screen", parameters=[params],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[{
                "use_sim_time": replay, "autostart": True,
                "node_names": ["map_server", "planner_server", "controller_server", "bt_navigator"],
            }],
        ),
        Node(
            package="bit_standalone", executable="motion_preview", name="bit_motion_preview",
            output="screen", parameters=[{"use_sim_time": replay}],
        ),
        Node(
            package="bit_standalone", executable="can_preview", name="bit_can_preview",
            output="screen", parameters=[os.path.join(share, "config", "communication.yaml"), {"use_sim_time": replay}],
        ),
        Node(
            package="bit_standalone", executable="can_bridge", name="bit_can_bridge",
            condition=IfCondition(LaunchConfiguration("enable_can")),
            output="screen", parameters=[os.path.join(share, "config", "communication.yaml"), {"use_sim_time": False}],
        ),
        Node(
            package="rviz2", executable="rviz2", output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{"use_sim_time": replay}],
            arguments=["-d", os.path.join(share, "config", "navigation.rviz")],
        ),
    ]
    return actions


def generate_launch_description():
    share = get_package_share_directory("bit_standalone")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="replay", choices=["replay", "live"]),
        DeclareLaunchArgument("enable_can", default_value="false", choices=["false", "true"]),
        DeclareLaunchArgument("map", default_value=os.path.join(share, "map", "map.yaml"), description="Absolute path to the scene Nav2 map YAML"),
        DeclareLaunchArgument("pcd", default_value=os.path.join(share, "map", "map.pcd"), description="Absolute path to the matching scene PCD"),
        DeclareLaunchArgument("robot_radius", default_value="0.14142135623730953", description="Circular costmap/corridor radius derived from the previous +/-0.10 m square footprint"),
        DeclareLaunchArgument("relocalizer_input_topic", default_value="/livox/lidar", description="Raw cloud topic for the body-frame relay"),
        DeclareLaunchArgument("relocalizer_input_format", default_value="custom", choices=["custom", "pointcloud2"], description="Raw cloud message type for the body-frame relay"),
        DeclareLaunchArgument("driver_config", default_value=os.path.join(share, "config", "MID360_config.json")),
        DeclareLaunchArgument("robot_config", default_value=os.path.join(share, "config", "robot.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
