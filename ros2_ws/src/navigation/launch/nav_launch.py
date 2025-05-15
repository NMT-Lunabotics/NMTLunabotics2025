from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_pkg = get_package_share_directory('nav2_bringup')
    navigation_pkg = get_package_share_directory('navigation')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': os.path.join(navigation_pkg, 'config', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        nav2
    ])
