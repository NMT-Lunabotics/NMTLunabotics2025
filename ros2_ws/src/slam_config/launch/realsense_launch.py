from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_launch_file = os.path.join(
        get_package_share_directory(
            'realsense2_camera'), 'launch', 'rs_launch.py'
    )
    realsense_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_depth': 'true',
            'pointcloud.enable': 'true',
            'colorizer.enable': 'false',
            'enable_color': 'true',
            'enable_rgbd': 'false',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'unite_imu_method': '2',
            'serial_no': '_306322300659',
            'camera_name': 'camera_1',
        }.items()
        
    ) 
    realsense_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_depth': 'true',
            'pointcloud.enable': 'true',
            'colorizer.enable': 'false',
            'enable_color': 'true',
            'enable_rgbd': 'false',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'unite_imu_method': '2',
            'serial_no': '_311322301206',
            'camera_name': 'camera_2',
        }.items()
        
    )
    return LaunchDescription([
        realsense_launch_1, 
        realsense_launch_2
    ])