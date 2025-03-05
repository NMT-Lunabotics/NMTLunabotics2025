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

    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_s1_launch.py'
    )

    rtabmap_launch_file = os.path.join(
        get_package_share_directory(
            'rtabmap_launch'), 'launch', 'rtabmap.launch.py'
    )

    realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'depth_module.depth_profile': '640x480x30',
                'rgb_camera.color_profile': '640x480x30',
                'enable_depth': 'true',
                'enable_color': 'true',
                'enable_rgbd': 'true',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'enable_sync': 'true',
                'align_depth.enable': 'true',
                'unite_imu_method': '2'
            }.items()
        )
    
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file),
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{'use_mag': False, 
                        'world_frame':'enu', 
                        'publish_tf':False}],
        remappings=[('imu/data_raw', '/camera/camera/imu')])

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_file),
        launch_arguments={
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'rgb_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'imu_topic': '/imu/data',
            'scan_topic': '/scan',
            'wait_imu_to_init': 'true',
            'frame_id': 'camera_link',
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'subscribe_stereo': 'false',
            'subscribe_scan': 'true',
            'visual_odometry': 'true',
            'approx_sync': 'true',
            'queue_size': '10',
            'rtabmap_viz': 'false',
            'database_path': ''
        }.items()
    )
    
    # Launch arguments
    return LaunchDescription([
        rplidar_launch,
        realsense_launch,
        imu_filter_node,
        rtabmap_launch
    ])

    static_transform_publisher = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'laser']
    )

    return LaunchDescription([
        rplidar_launch,
        realsense_launch,
        imu_filter_node,
        rtabmap_launch,
        static_transform_publisher
    ])