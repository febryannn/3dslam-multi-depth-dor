import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robot_description'
    xacro_file = 'robot.xacro'

    # Path ke file xacro
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_path = os.path.join(pkg_path, 'urdf', xacro_file)

    # Robot State Publisher (Menerjemahkan URDF agar dimengerti RViz)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro_path])}]
    )

    # Joint State Publisher GUI (Agar bisa geser-geser sendi robot nanti)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
