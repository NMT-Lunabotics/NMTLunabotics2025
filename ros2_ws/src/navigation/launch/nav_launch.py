# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

def launch_setup(context, *args, **kwargs):

    # Configurations
    autostart = LaunchConfiguration('autostart')
    lifecycle_nodes = ['bt_navigator',
                       'controller_server',
                       'planner_server',
                       'smoother_server',
                       'behavior_server',
                       'robot_state_publisher',
                       'waypoint_follower',
                       'velocity_smoother',
    ]

    # Directories
    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')

    
    nav_dir = os.path.join(get_package_share_directory('navigation'))
    
    nav2_params_file = os.path.join(nav_dir, 'config', 'nav2_params.yaml')

    # Paths
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    # Includes
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('params_file', nav2_params_file)
        ]
    )


    return [
        # Nodes to launch
        nav2,
        autostart,
        lifecycle_nodes,        
    ]

def generate_launch_description():
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
    ])