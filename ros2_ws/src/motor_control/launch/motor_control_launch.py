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
                {'wheel_base': 0.5},
                {'wheel_diameter': 0.1},
                {'max_rpm': 100},
                {'min_rpm': 10},
                {'arduino_serial_device': '/dev/ttyACM0'}
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
                {'arm_axis': 2}
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
