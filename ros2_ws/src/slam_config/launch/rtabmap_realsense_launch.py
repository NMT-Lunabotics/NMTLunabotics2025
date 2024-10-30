from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    return LaunchDescription([
        # Include the RealSense camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'realsense2_camera'), 'launch', 'rs_launch.py')
            )
        ),

        # Group for stereo image processing
        GroupAction([
            Node(
                package='stereo_image_proc',
                executable='stereo_image_proc',
                name='stereo_image_proc',
                namespace='stereo',
                output='screen'
            ),
            Node(
                package='nodelet',
                executable='nodelet',
                name='disparity2depth',
                namespace='stereo',
                output='screen',
                arguments=['standalone', 'rtabmap_util/disparity_to_depth']
            )
        ]),

        # Odometry node using viso2_ros package
        Node(
            package='viso2_ros',
            executable='stereo_odometer',
            name='stereo_odometer',
            output='screen',
            remappings=[
                ('stereo', 'stereo'),
                ('image', 'image_rect')
            ],
            parameters=[
                {'base_link_frame_id': '/base_link'},
                {'odom_frame_id': '/odom'},
                {'ref_frame_change_method': 1}
            ]
        ),

        # RTAB-Map SLAM group
        GroupAction([
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                arguments=['--delete_db_on_start'],
                parameters=[
                    {'subscribe_depth': True},
                    {'subscribe_laserScan': False},
                    {'frame_id': '/base_link'},
                    {'queue_size': 30},
                    {'approx_sync': False},
                    {'Vis/MinInliers': '12'}
                ],
                remappings=[
                    ('rgb/image', '/camera/camera/depth/image_rect_raw'),
                    ('rgb/camera_info', '/camera/camera/depth/image_rect_raw'),
                    ('depth/image', '/camera/camera/depth/image_rect_raw'),
                    ('odom', '/stereo_odometer/odometry')
                ]
            )
        ])
    ])
