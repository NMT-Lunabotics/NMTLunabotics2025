from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
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
            parameters=[config]
        )
    ])
