from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Node Penggabung Pointcloud
        Node(
            package='pointcloud_concatenate',
            executable='pointcloud_concatenate_node',
            name='pc_concat',
            parameters=[{
                'clouds': 4,
                'target_frame': 'base_link',
                'hz': 10.0
            }],
            output='screen'
        ),

        # 2. Node SLAM (Localization)
        Node(
            package='localization',
            executable='slam_rgbd_cam_node',
            name='slam_node',
            parameters=[{
                'min_cam_d': 0.6,
                'max_cam_d': 8.0
            }],
            output='screen'
        ),

        # 3. Node UDP Bridge (Sudah diperbaiki namanya)
        Node(
            package='udp_bot',
            executable='udp_bot_node', # Menggunakan nama hasil pengecekan pkg executables
            name='udp_bridge',
            output='screen'
        )
    ])
