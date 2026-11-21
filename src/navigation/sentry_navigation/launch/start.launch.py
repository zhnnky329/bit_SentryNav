from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise FileNotFoundError(f'Map YAML does not exist: {map_path}')
    bringup = Path(get_package_share_directory('bringup'))
    livox = Path(get_package_share_directory('livox_ros_driver2'))
    lio = Path(get_package_share_directory('super_lio'))
    navigation = Path(get_package_share_directory('sentry_navigation'))
    relocalizer = Path(get_package_share_directory('relocalizer'))
    use_relocalizer = LaunchConfiguration('use_relocalizer').perform(
        context).strip().lower() in {'1', 'true', 'yes', 'on'}

    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(livox / 'launch/msg_MID360_launch.py'))),
        Node(package='cpp_lidar_filter', executable='lidar_filter_node',
             name='my_lidar_filter', output='screen', parameters=[{
                 'input_topic': '/livox/lidar/pointcloud',
                 'output_topic': '/livox/lidar_filtered',
                 'min_x': -0.2, 'max_x': 0.4, 'min_y': -0.2, 'max_y': 0.2,
                 'min_z': -0.1, 'max_z': 0.2, 'negative': True, 'leaf_size': 0.05,
             }]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(lio / 'launch/livox_mid360.launch.py')), launch_arguments={'rviz': 'false'}.items()),
        Node(package='sentry_navigation', executable='tf_only_odom', name='tf_only_odom',
             output='screen', parameters=[
                 str(bringup / 'config/lidar.yaml'),
                 {'publish_map_to_odom': not use_relocalizer},
             ]),
        Node(package='sentry_navigation', executable='livox_body_cloud_relay.py',
             name='livox_body_cloud_relay', output='screen',
             condition=IfCondition(LaunchConfiguration('use_relocalizer'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(relocalizer / 'launch/relocalizer.launch.py')),
            launch_arguments={
                'plan_descriptor': str(bringup / 'map' / 'map.yaml'),
                'surface_map': str(bringup / 'map' / 'map.pcd'),
                'sweep_topic': '/cloud_registered_body',
                'body_frame': 'base_link',
                'world_frame': 'map',
                'odom_frame': 'odom',
                'automatic': 'true',
                'rviz': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_relocalizer')),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(navigation / 'launch/navigation_launch.py')), launch_arguments={
                'map': str(map_path),
                'use_rviz': LaunchConfiguration('use_rviz').perform(context),
                'rviz_config': str(bringup / 'config' / 'nav2_default_view.rviz'),
             }.items()),
    ]


def generate_launch_description():
    default_map = str(Path(get_package_share_directory('bringup')) / 'map' / 'map.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Absolute path to training map YAML (default: bringup/map/map.yaml)',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_relocalizer',
            default_value='true',
            description='Use relocalizer for map->odom (default: true); false uses /initialpose',
        ),
        OpaqueFunction(function=_start),
    ])
