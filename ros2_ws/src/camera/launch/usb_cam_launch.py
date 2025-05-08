from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cameras = [
        {'name': 'camera1', 'video_device': '/dev/video0'},
        # Add more cameras as needed
    ]

    nodes = []
    for camera in cameras:
        nodes.append(
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name=camera['name'],
                output='screen',
                parameters=[{
                    'video_device': camera['video_device'],
                    'image_width': 320,  # Lower resolution
                    'image_height': 240,
                    'pixel_format': 'raw_mjpeg',  # Change from 'mjpeg' to 'yuyv'
                    'framerate': 10.0,  # Lower frame rate
                    'camera_frame_id': camera['name']
                }],
                remappings=[
                    ('/image_raw', f'/{camera["name"]}/image')
                ]
            )
        )

    return LaunchDescription(nodes)