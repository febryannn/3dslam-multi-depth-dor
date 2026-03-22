"""
Launch file untuk Localization menggunakan pre-built map.
Robot melakukan lokalisasi terhadap peta PCD yang sudah dibangun oleh SLAM.

Penggunaan:
  ros2 launch localization localization.launch.py pcd_path:=/path/to/map.pcd
  ros2 launch localization localization.launch.py pcd_path:=/tmp/slam_map.pcd use_sim_time:=true
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
        description='Gunakan simulation time')

    loc_config_arg = DeclareLaunchArgument(
        'loc_config',
        default_value=os.path.join(pkg_localization, 'config', 'localization_params.yaml'),
        description='Path ke file konfigurasi localization')

    pcd_path_arg = DeclareLaunchArgument(
        'pcd_path', default_value='/tmp/slam_map.pcd',
        description='Path ke PCD map file')

    use_sim_time = LaunchConfiguration('use_sim_time')
    loc_config = LaunchConfiguration('loc_config')
    pcd_path = LaunchConfiguration('pcd_path')

    # Localization Node
    loc_node = Node(
        package='localization',
        executable='rgbd_cam_loc_node',
        name='rgbd_cam_loc',
        parameters=[
            loc_config,
            {
                'use_sim_time': use_sim_time,
                'pcd_path': pcd_path,
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        loc_config_arg,
        pcd_path_arg,
        loc_node,
    ])
