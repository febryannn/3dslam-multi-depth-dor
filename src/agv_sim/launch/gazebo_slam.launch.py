"""
=================================================================
  GAZEBO 3D SLAM - Launch File
=================================================================

  Pipeline:
    1. Gazebo Harmonic + AGV robot (4 depth cameras)
    2. ROS-Gazebo Bridge (clock, odom, cmd_vel, 4x point cloud)
    3. Robot State Publisher (URDF → TF tree termasuk optical frames)
    4. PointCloud Concatenate (merge 4 kamera via TF2 → /full_pointcloud)
    5. 3D SLAM Node (scan-to-submap matching, arsitektur lidarslam_ros2)
    6. PointCloud → 2D Grid (untuk navigasi)
    7. RViz2 + Teleop

  Penggunaan:
    ros2 launch agv_sim gazebo_slam.launch.py

  Setelah mapping selesai, simpan map:
    ros2 service call /slam/save_map std_srvs/srv/Empty
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
    # === Package paths ===
    pkg_agv_sim = get_package_share_directory('agv_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_localization = get_package_share_directory('localization')

    # === Launch Arguments ===
    declare_world = DeclareLaunchArgument(
        'world', default_value='complex_lab.world',
        description='World file')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Gazebo tanpa GUI')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Jalankan RViz2')

    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop', default_value='true',
        description='Jalankan keyboard teleop')

    declare_slam_config = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(pkg_localization, 'config', 'slam_params.yaml'),
        description='SLAM config YAML')

    declare_map_save_path = DeclareLaunchArgument(
        'map_save_path', default_value='/tmp/slam_map',
        description='Path simpan map')

    # === Substitutions ===
    world_name = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')
    slam_config = LaunchConfiguration('slam_config')
    map_save_path = LaunchConfiguration('map_save_path')

    # === URDF ===
    xacro_file = os.path.join(pkg_agv_sim, 'urdf', 'agv.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    rviz_config = os.path.join(pkg_agv_sim, 'rviz', 'slam_config.rviz')
    world_file = os.path.join(pkg_agv_sim, 'worlds')

    # === 1. Gazebo ===
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

    # === 2. Spawn Robot ===
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'agv_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.2',
        ],
        output='screen',
    )

    # === 3. Robot State Publisher (TF tree termasuk optical frames) ===
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # === 4. ROS ↔ Gazebo Bridge ===
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
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
        output='screen',
    )

    # === 5. PointCloud Concatenate (merge 4 kamera via TF2) ===
    # Menggunakan TF2 untuk transform dari optical frame → base_link
    pointcloud_concat = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='pointcloud_concatenate',
                executable='pointcloud_concatenate_node',
                name='pointcloud_concatenate',
                remappings=[
                    ('cloud_in1', '/cloud_in1/points'),
                    ('cloud_in2', '/cloud_in2/points'),
                    ('cloud_in3', '/cloud_in3/points'),
                    ('cloud_in4', '/cloud_in4/points'),
                ],
                parameters=[{
                    'use_sim_time': True,
                    'target_frame': 'base_link',
                    'clouds': 4,
                    'hz': 10.0,
                }],
                output='screen',
            ),
        ],
    )

    # === 6. SLAM Node (scan-to-submap matching) ===
    slam_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='localization',
                executable='slam_rgbd_cam_node',
                name='slam_rgbd_cam',
                parameters=[
                    slam_config,
                    {
                        'use_sim_time': True,
                        'map_save_path': map_save_path,
                    },
                ],
                output='screen',
            ),
        ],
    )

    # === 7. PointCloud → 2D Grid ===
    pointcloud_to_grid = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='pointcloud_to_grid',
                executable='pointcloud_to_grid_node',
                name='pointcloud_to_grid',
                parameters=[{
                    'use_sim_time': True,
                    'cloud_in_topic': '/slam/map_cloud',
                    'mapi_topic_name': '/intensity_grid',
                    'maph_topic_name': '/height_grid',
                    'cell_size': 0.1,
                    'length_x': 25.0,
                    'length_y': 25.0,
                    'position_x': 0.0,
                    'position_y': 0.0,
                    'height_factor': 50.0,
                    'intensity_factor': 100.0,
                }],
                output='screen',
            ),
        ],
    )

    # === 8. Odom Relay ===
    odom_relay = Node(
        package='agv_sim',
        executable='odom_to_udp_relay',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # === 9. Teleop ===
    teleop = Node(
        package='udp_bot',
        executable='teleop',
        output='screen',
        prefix='xterm -e',
        condition=IfCondition(use_teleop),
    )

    # === 10. RViz2 ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # Arguments
        declare_world,
        declare_headless,
        declare_use_rviz,
        declare_use_teleop,
        declare_slam_config,
        declare_map_save_path,

        # Gazebo
        set_gz_resource,
        gz_sim,
        spawn_robot,

        # Infrastructure
        robot_state_publisher,
        ros_gz_bridge,
        odom_relay,

        # Point Cloud Pipeline
        pointcloud_concat,

        # SLAM (delayed - needs pointcloud_concat + TF ready)
        slam_node,
        pointcloud_to_grid,

        # UI
        rviz_node,
        teleop,
    ])
