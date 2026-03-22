import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description = get_package_share_directory('agv_robot_description')
    pkg_gazebo = get_package_share_directory('agv_robot_gazebo')
    pkg_pointcloud = get_package_share_directory('pointcloud_concatenate')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    world_arg = DeclareLaunchArgument('world', default_value='myworld.world')
    model_arg = DeclareLaunchArgument(
        'model', default_value='robot_3d.urdf.xacro')
    pointcloud_concatenate_arg = DeclareLaunchArgument(
        'pointcloud_concatenate', default_value='true')

    urdf_file = PathJoinSubstitution([
        pkg_description, "urdf", "robots", LaunchConfiguration('model')
    ])

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'world.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_description, 'rviz', 'display.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    spawn_urdf = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0", "-y", "-2.0", "-z", "0.5", "-Y", "1.59"
        ],
        output="screen",
        parameters=[{'use_sim_time': True}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro', ' ', urdf_file]),
            'use_sim_time': True,
        }],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/cam_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/cam_back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/cam_left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/cam_right/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(rviz_arg)
    ld.add_action(world_arg)
    ld.add_action(model_arg)
    ld.add_action(pointcloud_concatenate_arg)
    ld.add_action(world_launch)
    ld.add_action(spawn_urdf)
    ld.add_action(robot_state_publisher)
    ld.add_action(gz_bridge)
    return ld
