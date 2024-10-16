from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    motor_ctrl_config = PathJoinSubstitution([
        FindPackageShare('motor_ctrl'),
        'config',
        'motor_ctrl_config.yaml'
    ])

    return LaunchDescription([
        Node(
            package='motor_ctrl',
            executable='motor_ctrl_node',
            name='motor_ctrl_node',
            output='screen',
            parameters=[motor_ctrl_config]
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