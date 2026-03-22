"""
=================================================================
  GAZEBO LOCALIZATION - Navigasi dengan Pre-built Map
=================================================================

  Gunakan setelah map sudah dibangun dengan gazebo_slam.launch.py.

  Penggunaan:
    ros2 launch agv_sim gazebo_localization.launch.py pcd_path:=/tmp/slam_map.pcd

=================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_agv_sim = get_package_share_directory('agv_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_localization = get_package_share_directory('localization')

    # Arguments
    declare_world = DeclareLaunchArgument(
        'world', default_value='complex_lab.world')
    declare_pcd_path = DeclareLaunchArgument(
        'pcd_path', default_value='/tmp/slam_map.pcd',
        description='Path ke PCD map file hasil SLAM')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true')
    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop', default_value='true')

    world_name = LaunchConfiguration('world')
    pcd_path = LaunchConfiguration('pcd_path')
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')

    # URDF
    xacro_file = os.path.join(pkg_agv_sim, 'urdf', 'agv.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Paths
    rviz_config = os.path.join(pkg_agv_sim, 'rviz', 'slam_config.rviz')
    loc_config = os.path.join(pkg_localization, 'config', 'localization_params.yaml')
    world_file = os.path.join(pkg_agv_sim, 'worlds')

    # Gazebo
    set_gz_resource = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_agv_sim, '..')]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', world_file, '/', world_name],
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-string', robot_desc, '-name', 'agv_robot', '-z', '0.2'],
        output='screen')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cloud_in1/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cloud_in2/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cloud_in3/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cloud_in4/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen')

    odom_relay = Node(
        package='agv_sim', executable='odom_to_udp_relay',
        parameters=[{'use_sim_time': True}], output='screen')

    # Localization Node (bukan SLAM)
    loc_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='localization',
                executable='rgbd_cam_loc_node',
                name='rgbd_cam_loc',
                parameters=[
                    loc_config,
                    {
                        'use_sim_time': True,
                        'pcd_path': pcd_path,
                    },
                ],
                output='screen',
            ),
        ],
    )

    # PointCloud to Grid (menggunakan map dari localization)
    pc2grid = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='pointcloud_to_grid',
                executable='pointcloud_to_grid_node',
                parameters=[{
                    'use_sim_time': True,
                    'cloud_in_topic': '/loc/map_cloud',
                    'maph_topic_name': '/height_grid',
                    'mapi_topic_name': '/intensity_grid',
                    'cell_size': 0.1,
                    'length_x': 25.0,
                    'length_y': 25.0,
                }],
                output='screen'),
        ],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz))

    teleop = Node(
        package='udp_bot', executable='teleop',
        output='screen', prefix='xterm -e',
        condition=IfCondition(use_teleop))

    return LaunchDescription([
        declare_world,
        declare_pcd_path,
        declare_use_rviz,
        declare_use_teleop,
        set_gz_resource,
        gz_sim,
        spawn_robot,
        rsp,
        bridge,
        odom_relay,
        loc_node,
        pc2grid,
        rviz_node,
        teleop,
    ])
