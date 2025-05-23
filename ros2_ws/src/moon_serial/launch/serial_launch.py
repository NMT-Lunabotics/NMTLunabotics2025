from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    # Define the LED-off command
    led_off_command = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/led_control', 'moon_messages/Leds', 
            '{red: true, yellow: false, green: false, blue: false}'
        ],
        output='screen'
    )

    led_on_command = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/led_control', 'moon_messages/Leds', 
            '{red: false, yellow: false, green: true}'
        ],
        output='screen'
    )

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
        ),
        Node(  # Add the heartbeat_publisher node
            package='moon_serial',
            executable='heartbeat_publisher',
            name='heartbeat_publisher',
            output='screen'
        ),
        TimerAction(
            period=2.0,  # Wait 2 seconds after nodes start
            actions=[led_on_command],
        ),
        # Turn off the LED when either node exits
        RegisterEventHandler(
            OnProcessExit(
                target_action=Node(
                    package='moon_serial',
                    executable='serial_bridge_node',
                    name='serial_bridge_node'
                ),
                on_exit=[led_off_command]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=Node(
                    package='moon_serial',
                    executable='serial_convert_node',
                    name='serial_convert_node'
                ),
                on_exit=[led_off_command]
            )
        )
    ])