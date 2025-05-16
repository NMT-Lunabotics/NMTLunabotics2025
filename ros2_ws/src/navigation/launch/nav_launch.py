import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    lifecycle_nodes = [
        'bt_navigator',
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'waypoint_follower',
        'velocity_smoother',
    ]

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    nav_dir = os.path.join(get_package_share_directory('navigation'))
    nav2_params_file = os.path.join(nav_dir, 'config', 'nav2_params.yaml')

    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('params_file', nav2_params_file)
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        nav2,
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': autostart},
                        {'node_names': lifecycle_nodes}]
        ),
    ])
