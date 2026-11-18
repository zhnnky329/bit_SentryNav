from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('can_comm'), 'config', 'can_params.yaml')

    return LaunchDescription([
        Node(
            package='can_comm',
            executable='can_send',
            name='can_send',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='can_comm',
            executable='can_receive',
            name='can_receive',
            output='screen',
            parameters=[config_file],
        ),
    ])
