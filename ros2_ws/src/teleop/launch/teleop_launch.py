from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    
    # Declare the launch argument for device_id
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='Joystick device ID'
    )

    # Create a LaunchConfiguration to use the argument
    device_id = LaunchConfiguration('device_id')
    return LaunchDescription([
        # Start the joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            # Adjust the joystick device if necessary
            parameters=[{
                'device_id': device_id,
                'deadzone': 0.2
            }]
        ),

        # Start the teleop_twist_joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'require_enable_button': True,
                'enable_button': 4,
                'axis_linear.x': 1,
                'scale_linear.x': 0.25,
                'axis_angular.yaw': 0,
                'scale_angular.yaw': 0.25,
                'inverted_reverse': False
            }]
        ),
    ])
