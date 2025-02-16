from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    motor_config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'motor_control_params.yaml'
    )

    actuator_config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'motor_control_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='motor_control',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen',
            parameters=[motor_config]
        ),
        Node(
            package='motor_control',
            executable='actuator_control_node',
            name='actuator_control_node',
            output='screen',
            parameters=[actuator_config]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('moon_serial'),
                    'launch',
                    'serial_launch.py'
                ])
            ])
        )
    ])
