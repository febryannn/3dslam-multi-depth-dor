import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world', default_value='myworld.world',
        description='Name of the Gazebo world file to load'
    )

    pkg_gazebo = get_package_share_directory('agv_robot_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                pkg_gazebo,
                'worlds',
                LaunchConfiguration('world')
            ]),
            TextSubstitution(text=' -r -v -v1')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(gazebo_launch)
    return ld
