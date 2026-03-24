import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dynamic_object_removal'),
        'config',
        'dynamic_object_removal_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='dynamic_object_removal',
            executable='dynamic_object_removal_node',
            name='dynamic_object_removal',
            output='screen',
            parameters=[config],
        ),
    ])
