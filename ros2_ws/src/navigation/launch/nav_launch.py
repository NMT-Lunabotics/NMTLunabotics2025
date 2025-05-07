from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
            'map_server',
            'amcl',
            'planner_server',
            'controller_server',
            'recoveries_server',
            'bt_navigator'
            ]
        }]
    )
    
    return LaunchDescription([
        lifecycle_manager,
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_dir + '/launch/navigation_launch.py'),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': get_package_share_directory('navigation') + '/config/nav2_params.yaml'
            }.items()
        )
    ])
