from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moon_serial',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            parameters=[
                {'baud_rate': 115200},
                {'serial_device': '/dev/ttyACM0'}
            ]
        ),
        Node(
            package='moon_serial',
            executable='serial_convert_node',
            name='serial_convert_node'
        )
    ])