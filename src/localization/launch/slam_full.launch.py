"""
Launch file untuk 3D SLAM pipeline lengkap.
Menjalankan SLAM node dengan konfigurasi dari YAML.

Penggunaan:
  ros2 launch localization slam_full.launch.py
  ros2 launch localization slam_full.launch.py use_sim_time:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_localization = get_package_share_directory('localization')

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Gunakan simulation time (true untuk Gazebo)')

    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(pkg_localization, 'config', 'slam_params.yaml'),
        description='Path ke file konfigurasi SLAM')

    map_save_path_arg = DeclareLaunchArgument(
        'map_save_path', default_value='/tmp/slam_map',
        description='Path untuk menyimpan map')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_config = LaunchConfiguration('slam_config')

    # SLAM Node
    slam_node = Node(
        package='localization',
        executable='slam_rgbd_cam_node',
        name='slam_rgbd_cam',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )

    # SLAM Listen Node (monitor pose output)
    slam_listen = Node(
        package='localization',
        executable='slam_listen_node',
        name='slam_listen',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        map_save_path_arg,
        slam_node,
        slam_listen,
    ])
