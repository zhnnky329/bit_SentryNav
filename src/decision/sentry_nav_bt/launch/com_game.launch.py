"""简易赛专用入口；不改变原 sentry_nav_bt.launch.py 的默认赛季树。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = Path(get_package_share_directory('sentry_nav_bt'))
    base_launch = package_dir / 'launch' / 'sentry_nav_bt.launch.py'
    simple_tree = package_dir / 'config' / 'com_game.xml'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('validate_bt_only', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(base_launch)),
            launch_arguments={
                'bt_xml_filename': str(simple_tree),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'validate_bt_only': LaunchConfiguration('validate_bt_only'),
            }.items(),
        ),
    ])
