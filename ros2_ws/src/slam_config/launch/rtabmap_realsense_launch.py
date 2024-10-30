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
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
    )
    rtabmap_launch_file = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
    )

    # Launch arguments
    return LaunchDescription([
        # RealSense D435 camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'depth_module.profile': '640x480x30',      # Set resolution and frame rate
                'enable_depth': 'true',
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'enable_color': 'true',                   # Disable color to prioritize depth
                'unite_imu_method': '1'                    # Use IMU data for better odometry (if available)
            }.items()
        ),
        
        # RTAB-Map SLAM launch with visual odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments={
                'frame_id': 'camera_link',                  # Set frame ID for RealSense camera
                'subscribe_depth': 'true',
                'subscribe_rgb': 'false',
                'subscribe_stereo': 'true',                 # Use stereo camera setup
                'visual_odometry': 'true',                  # Enable visual odometry
                'approx_sync': 'true',                      # Synchronize messages approximately
                'queue_size': '10'                          # Set queue size for topic synchronization
            }.items()
        ),
       
    ])
