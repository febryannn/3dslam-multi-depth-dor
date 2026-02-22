import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_agv_sim = get_package_share_directory('agv_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup URDF
    xacro_file = os.path.join(pkg_agv_sim, 'urdf', 'agv.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Paths untuk Config
    rviz_config_dir = os.path.join(pkg_agv_sim, 'rviz', 'slam_config.rviz')
    world_file = os.path.join(pkg_agv_sim, 'worlds', 'complex_lab.world')

    # Environment
    set_gz_resource = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_agv_sim, '..')]
    )

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn Robot
    create_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-string', robot_desc, '-name', 'agv_robot', '-z', '0.2'],
        output='screen')

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen')

    # PERBAIKAN 1: Tambahkan /points pada topik kamera agar PointCloud tertangkap
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

    # PERBAIKAN 2: Ubah input topik enumerasi agar sesuai dengan output bridge
    pointcloud_concat = Node(
        package='pointcloud_concatenate',
        executable='pointcloud_concatenate_node',
        name='pointcloud_concatenate',
        parameters=[{
            'use_sim_time': True,
            'input_topics': ['/cloud_in1/points', '/cloud_in2/points', '/cloud_in3/points', '/cloud_in4/points'],
            'output_frame': 'base_link',
        }],
        remappings=[('output', 'full_pointcloud')],
        output='screen')

    # Odom Relay & SLAM
    odom_relay = Node(
        package='agv_sim', executable='odom_to_udp_relay',
        parameters=[{'use_sim_time': True}], output='screen')

    slam = Node(
        package='localization', executable='slam_rgbd_cam_node',
        name='slam_rgbd_cam',
        parameters=[{'use_sim_time': True, 'min_cam_d': 0.6, 'max_cam_d': 8.0}],
        output='screen')

    # RViz & Teleop
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}], output='screen')

    teleop = Node(
        package='udp_bot', executable='teleop',
        output='screen', prefix='gnome-terminal --')

    # PERBAIKAN 3: static_tf DIHAPUS karena konflik dengan gz_frame_id di URDF Anda

    return LaunchDescription([
        set_gz_resource,
        gz_sim,
        create_entity,
        rsp,
        bridge,
        pointcloud_concat,
        odom_relay,
        slam,
        rviz,
        teleop,
    ])
