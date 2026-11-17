# Copyright 2026 bw_nav contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the relocalizer with the sample site configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("relocalizer")
    default_config = os.path.join(share, "config", "relocalizer.yaml")
    default_rviz = os.path.join(share, "rviz", "relocalizer.rviz")
    parameter_types = {
        "plan_descriptor": str,
        "surface_map": str,
        "sweep_topic": str,
        "body_frame": str,
        "world_frame": str,
        "odom_frame": str,
        "automatic": bool,
    }
    supplied_parameters = set()

    def capture_overrides(context):
        # Capture explicit launch arguments before declarations add defaults.
        # This also preserves an explicitly empty intermediate frame.
        supplied_parameters.update(
            name for name in parameter_types if name in context.launch_configurations)

    def launch_node(context):
        overrides = {}
        for name, value_type in parameter_types.items():
            if name in supplied_parameters:
                value = LaunchConfiguration(name)
                # The ros2 launch CLI requires a nonempty argument token.
                if name == "odom_frame" and value.perform(context) in ('""', "''"):
                    value = ""
                overrides[name] = ParameterValue(value, value_type=value_type)
        return [Node(
            package="relocalizer",
            executable="relocalizer_node",
            name="relocalizer",
            output="screen",
            parameters=[LaunchConfiguration("config"), overrides],
        )]

    arguments = [
        DeclareLaunchArgument(
            "config", default_value=default_config, description="parameter file"),
        DeclareLaunchArgument(
            "plan_descriptor", default_value="",
            description="override the blocked-cell raster YAML path in config"),
        DeclareLaunchArgument(
            "surface_map", default_value="", description="override the 3-D map PCD in config"),
        DeclareLaunchArgument(
            "sweep_topic", default_value="",
            description="override the body-frame point surface topic in config"),
        DeclareLaunchArgument(
            "body_frame", default_value="", description="override the input frame in config"),
        DeclareLaunchArgument(
            "world_frame", default_value="", description="override the output frame in config"),
        DeclareLaunchArgument(
            "odom_frame", default_value="",
            description="override the intermediate frame; empty selects direct world -> body"),
        DeclareLaunchArgument(
            "automatic", default_value="",
            description="override automatic searching in config (true/false)"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="start RViz with the relocalizer display"),
        DeclareLaunchArgument(
            "rviz_config", default_value=default_rviz, description="RViz display config"),
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="relocalizer_rviz",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription([
        OpaqueFunction(function=capture_overrides),
        *arguments,
        OpaqueFunction(function=launch_node),
        rviz_node,
    ])
