"""
Launch file untuk testing SLAM pipeline dengan pointcloud_concatenate dan UDP bridge.

Penggunaan:
  ros2 launch localization slam_test.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. Node Penggabung Pointcloud (4 kamera -> 1 pointcloud)
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

        # 2. Node SLAM (lidarslam_ros2 architecture)
        Node(
            package='localization',
            executable='slam_rgbd_cam_node',
            name='slam_rgbd_cam',
            parameters=[{
                'registration_method': 'GICP',
                'gicp_corr_dist_threshold': 5.0,
                'trans_for_mapupdate': 0.5,
                'vg_size_for_input': 0.1,
                'vg_size_for_map': 0.05,
                'num_targeted_cloud': 10,
                'use_min_max_filter': True,
                'scan_min_range': 0.8,
                'scan_max_range': 4.5,
                'set_initial_pose': True,
                'use_odom': True,
                'publish_tf': True,
                'debug_flag': False,
                'input_cloud_topic': '/full_pointcloud',
            }],
            output='screen'
        ),

        # 3. Node UDP Receiver (Odometry dari STM32)
        Node(
            package='localization',
            executable='udp_receiver',
            name='udp_receiver',
            output='screen'
        ),
    ])
