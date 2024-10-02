from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    motor_ctrl_config = os.path.join(
        get_package_share_directory('motor_ctrl'),
        'config',
        'motor_ctrl_config.yaml'
    )

    return LaunchDescription([
        # Load parameters for motor_ctrl_node
        DeclareLaunchArgument(
            'motor_ctrl_config',
            default_value=motor_ctrl_config,
            description='Path to the motor control configuration file'
        ),

        # Run the motor_ctrl_node
        Node(
            package='motor_ctrl',
            executable='motor_ctrl_node',
            name='motor_ctrl_node',
            output='screen',
            parameters=[LaunchConfiguration('motor_ctrl_config')]
        ),

        # Run the can_raw_node
        Node(
            package='can_raw',
            executable='can_raw_node',
            name='can_raw_node',
            output='screen'
        ),

        # Run the can_raw_input_node
        Node(
            package='can_raw_input',
            executable='can_raw_input_node',
            name='can_raw_input_node',
            output='screen'
        ),

        # Run the can_convert_node
        Node(
            package='can_convert',
            executable='can_convert_node',
            name='can_convert_node',
            output='screen'
        ),
    ])