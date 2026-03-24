import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('graph_based_slam'), 'config')
    params_file = os.path.join(config_dir, 'graph_based_slam_params.yaml')

    graph_based_slam_node = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        graph_based_slam_node,
    ])
