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


def launch_relocalizer(context):
    maps = {}
    for name in ("plan_descriptor", "surface_map"):
        path = LaunchConfiguration(name).perform(context)
        if path:
            maps[name] = ParameterValue(path, value_type=str)
    return [Node(
        package="relocalizer",
        executable="relocalizer_node",
        name="relocalizer",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {
                **maps,
                "sweep_topic": LaunchConfiguration("sweep_topic"),
                "body_frame": LaunchConfiguration("body_frame"),
                "world_frame": LaunchConfiguration("world_frame"),
                "odom_frame": ParameterValue(LaunchConfiguration("odom_frame"), value_type=str),
                "automatic": ParameterValue(LaunchConfiguration("automatic"), value_type=bool),
            },
        ],
    )]


def generate_launch_description():
    share = get_package_share_directory("relocalizer")
    default_config = os.path.join(share, "config", "relocalizer.yaml")
    default_rviz = os.path.join(share, "rviz", "relocalizer.rviz")

    arguments = [
        DeclareLaunchArgument(
            "config", default_value=default_config, description="parameter file"),
        DeclareLaunchArgument(
            "plan_descriptor", default_value="",
            description="blocked-cell raster YAML (PGM descriptor)"),
        DeclareLaunchArgument(
            "surface_map", default_value="", description="3-D surface map PCD"),
        DeclareLaunchArgument(
            "sweep_topic", default_value="/cloud_registered_body",
            description="body-frame point surface topic"),
        DeclareLaunchArgument(
            "body_frame", default_value="body", description="frame of the input surface"),
        DeclareLaunchArgument(
            "world_frame", default_value="map", description="frame of the output pose"),
        DeclareLaunchArgument(
            "odom_frame", default_value="",
            description="optional intermediate frame; publishes world -> odom_frame"),
        DeclareLaunchArgument(
            "automatic", default_value="true", description="search without a service call"),
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

    return LaunchDescription(arguments + [OpaqueFunction(function=launch_relocalizer), rviz_node])
