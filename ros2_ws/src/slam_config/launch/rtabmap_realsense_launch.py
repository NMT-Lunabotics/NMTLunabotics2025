from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to the launch files
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    rtabmap_launch_path = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py'
    )

    # Include the RealSense launch file with parameters directly in launch_arguments
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'enable_color': 'true',
            'enable_sync': 'true',
        }.items()
    )

    # Include the RTAB-Map launch file with parameters directly in launch_arguments
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'depth_topic': '/camera/camera/depth/image_rect_raw',
            'rgb_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'rtabmapviz': 'false',
            'localization': 'true',
            'approx_sync': 'true',
        }.items()
    )

    # Static transform publisher from base_link to camera_link
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        output='screen',
        parameters=[],
        arguments=[
            '0',    # x translation
            '0',    # y translation
            '0',    # z translation
            '0',    # x rotation (radians)
            '0',    # y rotation (radians)
            '0',    # z rotation (radians)
            'base_link',  # parent frame
            'camera_link'  # child frame
        ]
    )

    return LaunchDescription([
        # Launch the RealSense camera
        realsense_launch,

        # Launch RTAB-Map with the provided parameters
        rtabmap_launch,

        # Publish the static transform from base_link to camera_link
        static_transform_publisher,
    ])
