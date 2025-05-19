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
        PythonLaunchDescriptionSource(rtabmap_launch_file)
    )

    # Launch arguments
    return LaunchDescription([
        robot_state_publisher_node,
        rplidar_launch,
        realsense_launch,
        imu_filter_node,
        rtabmap_launch,
    ])