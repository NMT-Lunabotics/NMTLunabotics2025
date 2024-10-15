from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare the arguments with their default values
    launch_motors_arg = DeclareLaunchArgument(
        'launch_motors',
        default_value='true',
        description='Launch motor_control if true'
    )
    
    launch_mapping_arg = DeclareLaunchArgument(
        'launch_mapping',
        default_value='true',
        description='Launch slam_config if true'
    )
    
    # Define paths to the launch files from other packages
    motor_control_launch_path = os.path.join(
        FindPackageShare('motor_control').find('motor_control'),
        'launch',
        'motor_control_launch.py'
    )

    slam_mapping_launch_path = os.path.join(
        FindPackageShare('slam_config').find('slam_config'),
        'launch',
        'rtabmap_realsense_launch.py'
    )

    # Include the motor control launch file if 'launch_motors' is true
    motor_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motor_control_launch_path),
        condition=IfCondition(LaunchConfiguration('launch_motors'))
    )

    # Include the mapping (slam) launch file if 'launch_mapping' is true
    slam_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_mapping_launch_path),
        condition=IfCondition(LaunchConfiguration('launch_mapping'))
    )

    # Log messages to indicate what is being launched
    log_motors = LogInfo(condition=IfCondition(LaunchConfiguration('launch_motors')), msg="Launching motor control...")
    log_mapping = LogInfo(condition=IfCondition(LaunchConfiguration('launch_mapping')), msg="Launching SLAM mapping...")

    return LaunchDescription([
        launch_motors_arg,
        launch_mapping_arg,
        motor_control_launch,
        slam_mapping_launch,
        log_motors,
        log_mapping
    ])
