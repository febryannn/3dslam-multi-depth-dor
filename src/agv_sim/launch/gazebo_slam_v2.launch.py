"""
=================================================================
  GAZEBO 3D SLAM V2 - Launch File (lidarslam_ros2 Architecture)
=================================================================

  Architecture:
    Frontend: scanmatching_slam (scan-to-submap, publishes MapArray)
    Backend:  graph_based_slam (g2o loop closure + pose graph optimization)
    Filter:   dynamic_object_removal (removes moving objects)

  Pipeline:
    1. Gazebo Harmonic + AGV robot (4 RGBD cameras)
    2. ROS-Gazebo Bridge (clock, odom, cmd_vel, 4x point cloud)
    3. Robot State Publisher (URDF -> TF tree)
    4. PointCloud Concatenate (merge 4 kamera -> /full_pointcloud)
    5. Dynamic Object Removal (/full_pointcloud -> /filtered_pointcloud)
    6. Scanmatching SLAM Frontend (scan-to-submap, publishes map_array)
    7. Graph-Based SLAM Backend (loop closure + g2o optimization)
    8. PointCloud -> 2D Grid
    9. RViz2 + Teleop

  Penggunaan:
    ros2 launch agv_sim gazebo_slam_v2.launch.py
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
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # === Package paths ===
    pkg_agv_sim = get_package_share_directory('agv_sim')
    pkg_description = get_package_share_directory('agv_robot_description')
    pkg_gazebo = get_package_share_directory('agv_robot_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_scanmatching = get_package_share_directory('scanmatching_slam')
    pkg_graph_slam = get_package_share_directory('graph_based_slam')
    pkg_dor = get_package_share_directory('dynamic_object_removal')

    # === Launch Arguments ===
    declare_world = DeclareLaunchArgument(
        'world', default_value='myworld.world')
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true')
    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop', default_value='true')
    declare_use_dor = DeclareLaunchArgument(
        'use_dynamic_removal', default_value='true',
        description='Enable dynamic object removal')

    world_name = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')

    # Paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'robots', 'robot_3d.urdf.xacro')
    rviz_config = os.path.join(pkg_agv_sim, 'rviz', 'slam_config.rviz')
    world_dir = os.path.join(pkg_gazebo, 'worlds')
    scanmatching_config = os.path.join(pkg_scanmatching, 'config', 'scanmatching_params.yaml')
    graph_slam_config = os.path.join(pkg_graph_slam, 'config', 'graph_based_slam_params.yaml')
    dor_config = os.path.join(pkg_dor, 'config', 'dynamic_object_removal_params.yaml')

    # === 1. Gazebo ===
    set_gz_resource = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_description, '..')]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', world_dir, '/', world_name],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # === 2. Spawn Robot ===
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'agv_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '-2.0', '-z', '0.5', '-Y', '1.59',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # === 3. Robot State Publisher ===
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro', ' ', urdf_file]),
            'use_sim_time': True,
        }],
        output='screen',
    )

    # === 4. ROS <-> Gazebo Bridge ===
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cam_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cam_back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cam_left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cam_right/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # === 5. PointCloud Concatenate (merge 4 kamera via TF2) ===
    pointcloud_concat = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='pointcloud_concatenate',
                executable='pointcloud_concatenate_node',
                name='pointcloud_concatenate',
                remappings=[
                    ('cloud_in1', '/cam_front/points'),
                    ('cloud_in2', '/cam_back/points'),
                    ('cloud_in3', '/cam_left/points'),
                    ('cloud_in4', '/cam_right/points'),
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

    # === 6. Dynamic Object Removal ===
    dynamic_removal = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='dynamic_object_removal',
                executable='dynamic_object_removal_node',
                name='dynamic_object_removal',
                parameters=[
                    dor_config,
                    {
                        'use_sim_time': True,
                        'input_topic': '/full_pointcloud',
                        'output_topic': '/filtered_pointcloud',
                    },
                ],
                output='screen',
            ),
        ],
    )

    # === 7. Scanmatching SLAM Frontend ===
    scanmatching_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='scanmatching_slam',
                executable='scanmatching_node',
                name='scanmatching_slam',
                parameters=[
                    scanmatching_config,
                    {
                        'use_sim_time': True,
                        'input_cloud_topic': '/filtered_pointcloud',
                    },
                ],
                output='screen',
            ),
        ],
    )

    # === 8. Graph-Based SLAM Backend (g2o loop closure) ===
    graph_slam_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='graph_based_slam',
                executable='graph_based_slam_node',
                name='graph_based_slam',
                parameters=[
                    graph_slam_config,
                    {'use_sim_time': True},
                ],
                remappings=[
                    ('map_array', '/scanmatching_slam/map_array'),
                    ('modified_map', '/graph_based_slam/modified_map'),
                    ('modified_map_array', '/graph_based_slam/modified_map_array'),
                    ('modified_path', '/graph_based_slam/modified_path'),
                ],
                output='screen',
            ),
        ],
    )

    # === 9. PointCloud -> 2D Grid ===
    pointcloud_to_grid = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='pointcloud_to_grid',
                executable='pointcloud_to_grid_node',
                name='pointcloud_to_grid',
                parameters=[{
                    'use_sim_time': True,
                    'cloud_in_topic': '/scanmatching_slam/map',
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

    # === 10. Teleop ===
    teleop = Node(
        package='udp_bot',
        executable='teleop',
        output='screen',
        prefix='xterm -e',
        condition=IfCondition(use_teleop),
    )

    # === 11. RViz2 ===
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
        declare_use_dor,

        # Gazebo
        set_gz_resource,
        gz_sim,
        spawn_robot,

        # Infrastructure
        robot_state_publisher,
        ros_gz_bridge,

        # Point Cloud Pipeline
        pointcloud_concat,
        dynamic_removal,

        # SLAM (Frontend + Backend)
        scanmatching_node,
        graph_slam_node,
        pointcloud_to_grid,

        # UI
        rviz_node,
        teleop,
    ])
