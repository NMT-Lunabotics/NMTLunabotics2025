import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the configuration directory (slam_config/config)
    config_dir = os.path.join(
        get_package_share_directory('slam_config'),
        'config'
    )

    # Paths to parameter files in slam_config/config
    slam_toolbox_params = os.path.join(config_dir, 'slam_params.yaml')

    # Rplidar launch
    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_s1_launch.py'
    )

    # Path to SLAM Toolbox launch file
    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'  # Using online_async launch
    )

    # Include rplidar Launch File
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file),
    )

    # Laser scan matching
    laser_scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[{
            'publish_odom': 'odom',
            'publish_tf': True,
            'max_angular_correction_deg': 20
        }]
    )

    # Include SLAM Toolbox Launch File
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={
            'slam_params_file': slam_toolbox_params,
            'use_sim_time': 'false'
        }.items()
    )

    # Static Transforms
    base_footprint_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'laser']
    )

    return LaunchDescription([
        rplidar_launch,
        laser_scan_matcher_node,
        base_footprint_tf_node,
        laser_tf_node,
        slam_toolbox_launch,
    ])
