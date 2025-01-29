from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            name='republish',
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', 'image_raw/compressed'),
                ('out', 'image_raw/uncompressed')
            ]
        ),
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            remappings=[
                ('image', 'image_raw/uncompressed'),
            ]
        )
    ])
