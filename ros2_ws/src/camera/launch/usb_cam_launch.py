from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 320,  # Lower resolution
                'image_height': 240,
                'pixel_format': 'mjpeg',  # Compressed format
                'framerate': 10.0,  # Lower frame rate
                'camera_frame_id': 'usb_cam'
            }]
        )
    ])