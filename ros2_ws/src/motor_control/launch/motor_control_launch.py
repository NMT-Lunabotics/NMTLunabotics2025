from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen',
            parameters=[
                {'cmd_vel_topic': 'cmd_vel'},
                {'wheel_base': 0.6},
                {'wheel_diameter': 0.3},
                {'max_rpm': 30},
                {'min_rpm': 2},
            ]
        ),
        Node(
            package='motor_control',
            executable='actuator_control_node',
            name='actuator_control_node',
            output='screen',
            parameters=[
                {'joy_topic': '/joy'},
                {'actuator_max_vel': 25},
                {'bucket_axis': 3},
                {'arm_axis': 4}
            ]
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
