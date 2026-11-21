"""实车传感器、里程计、地图定位与 Nav2 的统一入口。"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _site_file(site, key, directory):
    value = site.get(key) if isinstance(site, dict) else None
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'场地配置缺少文件路径 {key}')
    path = (directory / Path(value).expanduser()).resolve()
    if not path.is_file():
        raise FileNotFoundError(f'场地配置 {key} 对应的文件不存在: {path}')
    return str(path)


def _include(package, filename, **arguments):
    path = get_package_share_path(package) / 'launch' / filename
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())


def _start(context):
    config = get_package_share_path('bringup') / 'config'
    site_path = Path(LaunchConfiguration('site').perform(context)).expanduser().absolute()
    site = yaml.safe_load(site_path.read_text(encoding='utf-8'))
    map_yaml = _site_file(site, 'map_yaml', site_path.parent)
    automatic = IfCondition(LaunchConfiguration('use_relocalizer')).evaluate(context)

    actions = [
        _include('livox_ros_driver2', 'msg_MID360_launch.py'),
        Node(package='cpp_lidar_filter', executable='lidar_filter_node', name='my_lidar_filter',
             output='screen', parameters=[str(config / 'lidar_filter.yaml')]),
        _include('super_lio', 'livox_mid360.launch.py', rviz='false'),
        Node(package='sentry_navigation', executable='tf_only_odom', output='screen',
             parameters=[str(config / 'lidar.yaml'), {'publish_map_to_odom': not automatic}]),
    ]
    if automatic:
        actions.extend([
            Node(package='relocalizer', executable='livox_body_cloud_relay', output='screen'),
            _include('relocalizer', 'relocalizer.launch.py', plan_descriptor=map_yaml,
                     surface_map=_site_file(site, 'surface_map', site_path.parent),
                     sweep_topic='/cloud_registered_body', body_frame='base_link',
                     world_frame='map', odom_frame='odom', automatic='true', rviz='false'),
        ])

    actions.append(_include(
        'sentry_navigation', 'navigation.launch.py', map=map_yaml,
        params_file=str(config / 'nav2_params.yaml'), use_rviz=LaunchConfiguration('use_rviz'),
        rviz_config=str(config / 'nav2_default_view.rviz')))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('site', default_value=str(
            get_package_share_path('bringup') / 'config/site.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_relocalizer', default_value='true'),
        OpaqueFunction(function=_start),
    ])
