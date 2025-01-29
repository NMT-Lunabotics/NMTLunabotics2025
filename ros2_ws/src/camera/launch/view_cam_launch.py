from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_raw',
            description='Topic to remap the image topic'
        ),
        DeclareLaunchArgument(
            'image_transport',
            default_value='theora',
            description='Image transport parameter'
        ),
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            remappings=[
                ('image', LaunchConfiguration('image_topic'))
            ],
            parameters=[{
                'image_transport': LaunchConfiguration('image_transport')
            }]
        )
    ])