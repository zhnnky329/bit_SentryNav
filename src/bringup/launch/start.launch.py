"""实车传感器、里程计、地图定位与 Nav2 的统一入口。"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _site_file(site, key, directory):
    value = site.get(key) if isinstance(site, dict) else None
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'场地配置缺少文件路径 {key}')
    path = (directory / Path(value).expanduser()).resolve()
    if not path.is_file():
        raise FileNotFoundError(f'场地配置 {key} 对应的文件不存在: {path}')
    return str(path)


def _start(context):
    config_path = Path(LaunchConfiguration('config').perform(context)).expanduser().absolute()
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    grid_map_yaml = _site_file(config, 'grid_map_yaml', config_path.parent)
    for key in ('use_relocalizer', 'use_rviz'):
        if not isinstance(config.get(key), bool):
            raise ValueError(f'{config_path} 中的 {key} 必须为 true 或 false')
    automatic = config['use_relocalizer']

    arguments = {
        'grid_map_yaml': grid_map_yaml,
        'use_relocalizer': str(automatic).lower(),
        'use_rviz': str(config['use_rviz']).lower(),
        'publish_map_to_odom': 'false' if automatic else 'true',
    }
    if automatic:
        arguments['pcd_map'] = _site_file(config, 'pcd_map', config_path.parent)

    sequence = get_package_share_path('bringup') / 'config/navigation_sequence.yaml'
    return [IncludeLaunchDescription(str(sequence), launch_arguments=arguments.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(
            get_package_share_path('bringup') / 'config/bringup.yaml')),
        OpaqueFunction(function=_start),
    ])
