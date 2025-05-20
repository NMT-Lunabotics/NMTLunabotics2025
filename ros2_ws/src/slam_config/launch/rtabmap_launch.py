from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare a launch argument for localization mode
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch in localization mode (true/false)'
    )

    localization = LaunchConfiguration('localization')

    rtabmap_config_file = os.path.join(
        get_package_share_directory('slam_config'),
                                       'config', 'rtabmap_config.yaml')


    realsense_launch_file = os.path.join(
        get_package_share_directory(
            'slam_config'), 'launch', 'realsense_launch.py'
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

    urdf_file = os.path.join(
        get_package_share_directory('slam_config'), 'urdf', 'goliath.urdf'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
    )
    
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file),
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{'use_mag': False, 
                        'world_frame':'enu', 
                        'publish_tf':False}],
        remappings=[('imu/data_raw', '/camera/camera_1/imu')])

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_file),
        launch_arguments={
            'depth_topic': '/camera/camera_1/aligned_depth_to_color/image_raw',
            'rgb_topic': '/camera/camera_1/color/image_raw',
            'camera_info_topic': '/camera/camera_1/color/camera_info',
            'imu_topic': '/imu/data',
            'scan_topic': '/scan',
            
            # RTAB Map Functionality
            'localization': 'false',
            'approx_sync': 'true',
            # 'mapping': 'true',
            'visual_odometry': 'true',

            # RTAB Map Frames and TFs
            'publish_tf': 'true',
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            # Occupancy Grid
            # 'Grid/FromDepth': 'true',
            # 'Grid/3D': 'false',
            # 'Grid/CellSize': '0.05',
            # # 'Grid/DepthDecimation': '5' # Change as needed
            # 'Grid/RangeMin': '0.3',
            # 'Grid/RangeMax': '5.0',
            'grid_map_publisher_rate': '1.0',
            # 'grid_frame_id': 'map',

            # Publishing/Subscribing
            'queue_size': '10',
            'wait_imu_to_init': 'true',
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'subscribe_stereo': 'false',
            'subscribe_scan': 'true',
            # 'Mem/IncrementalMemory': 'true',
            'rtabmap_args':
                '--ros-args --params-file '
                '/home/luna/ros2_ws/src/slam_config/config/rtabmap_config.yaml'
            
    }.items()
    )

    return LaunchDescription([
        localization_arg,
        robot_state_publisher_node,
        rplidar_launch,
        realsense_launch,
        imu_filter_node,
        rtabmap_launch,
    ])