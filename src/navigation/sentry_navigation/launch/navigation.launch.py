"""显示场地图，并在导航 TF 就绪后启动官方 Nav2 导航服务。"""
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    clock = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}
    monitor = Node(package='sentry_navigation', executable='tf_monitor.py',
                   name='tf_monitor', output='screen', parameters=[clock])

    def start_navigation(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            raise RuntimeError(f'TF 检查退出码为 {event.returncode}，导航服务未启动')
        path = get_package_share_path('nav2_bringup') / 'launch/navigation_launch.py'
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)),
            launch_arguments={'params_file': LaunchConfiguration('params_file'),
                              'use_sim_time': use_sim_time, 'autostart': 'true',
                              'use_composition': 'false'}.items())]

    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=str(
            get_package_share_path('nav2_bringup') / 'rviz/nav2_default_view.rviz')),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[clock, {'yaml_filename': ParameterValue(
                 LaunchConfiguration('map'), value_type=str)}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[clock, {'autostart': True, 'node_names': ['map_server']}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[clock]),
        RegisterEventHandler(OnProcessExit(target_action=monitor, on_exit=start_navigation)),
        monitor,
    ])
