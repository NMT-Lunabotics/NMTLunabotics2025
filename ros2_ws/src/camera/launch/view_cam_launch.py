from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/usb_cam/image_raw',
        description='Topic for the image'
    )

    return LaunchDescription([
        image_topic_arg,
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            output='screen',
            parameters=[{'image_transport': 'theora'}],
            remappings=[('image', LaunchConfiguration('image_topic'))]
        )
    ])