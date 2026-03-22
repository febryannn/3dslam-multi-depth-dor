import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('prefix', default_value='',
                          description='Prefix for robot joints and links'),
    DeclareLaunchArgument('use_gazebo', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation')
]


def generate_launch_description():
    pkg_share = FindPackageShare('agv_robot_description')
    default_urdf = PathJoinSubstitution(
        [pkg_share, 'urdf', 'robots', 'robot_3d.urdf.xacro'])
    default_rviz = PathJoinSubstitution(
        [pkg_share, 'rviz', 'display.rviz'])

    jsp_gui = LaunchConfiguration('jsp_gui')
    rviz_config = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_jsp_gui = DeclareLaunchArgument('jsp_gui', default_value='true')
    declare_rviz_config = DeclareLaunchArgument('rviz_config_file', default_value=default_rviz)
    declare_urdf = DeclareLaunchArgument('urdf_model', default_value=default_urdf)
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    robot_description = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'prefix:=', LaunchConfiguration('prefix'), ' ',
        'use_gazebo:=', LaunchConfiguration('use_gazebo')
    ]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description}])

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(jsp_gui))

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui))

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(declare_jsp_gui)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_urdf)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_sim_time)
    ld.add_action(joint_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_node)
    return ld
