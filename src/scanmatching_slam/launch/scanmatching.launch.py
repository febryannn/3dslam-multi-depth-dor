import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('scanmatching_slam'),
        'config',
        'scanmatching_params.yaml'
    )

    scanmatching_node = Node(
        package='scanmatching_slam',
        executable='scanmatching_node',
        name='scanmatching_slam',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        scanmatching_node,
    ])
