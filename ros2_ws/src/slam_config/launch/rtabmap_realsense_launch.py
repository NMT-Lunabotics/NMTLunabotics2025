from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to the RealSense camera and RTAB-Map launch files
    realsense_launch_file = os.path.join(
        get_package_share_directory(
            'realsense2_camera'), 'launch', 'rs_launch.py'
    )
    rtabmap_launch_file = os.path.join(
        get_package_share_directory(
            'rtabmap_launch'), 'launch', 'rtabmap.launch.py'
    )

    # Launch arguments
    return LaunchDescription([
        # RealSense D435 camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'depth_module.depth_profile': '640x480x30',
                'color_module.color_profile': '640x480x30',
                'enable_depth': 'true',
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'enable_color': 'true',
                'enable_gryo': 'true',
                'enable_accel': 'true',
                'enable_sync': 'true',
                'unite_imu_method': '1'
            }.items()
        ),

        # RTAB-Map SLAM launch with visual odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments={
                'depth_topic': '/camera/camera/depth/image_rect_raw',
                'rgb_topic': '/camera/camera/color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'frame_id': 'camera_link',
                'subscribe_depth': 'true',
                'subscribe_rgb': 'false',
                'subscribe_stereo': 'true',
                'visual_odometry': 'true',
                'approx_sync': 'true',
                'queue_size': '10'
            }.items()
        ),

    ])
