import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Memastikan sistem menggunakan jam dunia nyata (bukan simulasi)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ========================================================================
    # 1. STATIC TRANSFORM PUBLISHER (KALIBRASI POSISI KAMERA)
    # NANTI KAMU HARUS GANTI ANGKA 0.5, 0.0, Dst SESUAI UKURAN PENGGARIS DI ROBOT FISIK!
    # ========================================================================
    tf_kinect1 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_kinect1',
        arguments=['0.5', '0.0', '0.4', '0.0', '0.0', '0.0', 'base_link', 'camera_link_1']
    )
    tf_kinect2 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_kinect2',
        arguments=['-0.5', '0.0', '0.4', '3.14159', '0.0', '0.0', 'base_link', 'camera_link_2']
    )
    tf_kinect3 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_kinect3',
        arguments=['0.0', '0.3', '0.4', '1.5708', '0.0', '0.0', 'base_link', 'camera_link_3']
    )
    tf_kinect4 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_kinect4',
        arguments=['0.0', '-0.3', '0.4', '-1.5708', '0.0', '0.0', 'base_link', 'camera_link_4']
    )

    # ========================================================================
    # 2. NODE DRIVER KAMERA FISIK (KINECT V1) - DIBEKUKAN SEMENTARA
    # ========================================================================
    # kinect1_node = Node(
    #     package='kinect_ros2', executable='kinect_ros2_node',
    #     name='kinect1_driver', remappings=[('/depth/points', '/cloud_in1')],
    #     parameters=[{'frame_id': 'camera_link_1'}]
    # )
    # kinect2_node = Node(
    #     package='kinect_ros2', executable='kinect_ros2_node',
    #     name='kinect2_driver', remappings=[('/depth/points', '/cloud_in2')],
    #     parameters=[{'frame_id': 'camera_link_2'}]
    # )
    # kinect3_node = Node(
    #     package='kinect_ros2', executable='kinect_ros2_node',
    #     name='kinect3_driver', remappings=[('/depth/points', '/cloud_in3')],
    #     parameters=[{'frame_id': 'camera_link_3'}]
    # )
    # kinect4_node = Node(
    #     package='kinect_ros2', executable='kinect_ros2_node',
    #     name='kinect4_driver', remappings=[('/depth/points', '/cloud_in4')],
    #     parameters=[{'frame_id': 'camera_link_4'}]
    # )

    # ========================================================================
    # 3. NODE KOMUNIKASI MOTOR (UDP) & PENGGABUNG POINTCLOUD
    # ========================================================================
    # NODE UDP SUDAH DIAKTIFKAN DI SINI:
    udp_receiver_node = Node(
        package='localization', executable='udp_receiver', name='udp_receiver_node'
    )
    
    # Penggabung kamera MASIH DIBEKUKAN karena belum ada kameranya:
    # pointcloud_concat_node = Node(
    #     package='localization', executable='pointcloud_concatenate_node',
    #     name='pointcloud_concatenate_node', parameters=[{'use_sim_time': use_sim_time}]
    # )

    # ========================================================================
    # 4. NODE UTAMA SLAM (ALGORITMA C++ KITA)
    # ========================================================================
    slam_main_node = Node(
        package='localization',
        executable='slam_rgbd_cam_node',
        name='slam_rgbd_cam_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'min_cam_d': 0.6},
            {'max_cam_d': 8.0}
        ],
        output='screen'
    )

    # ========================================================================
    # 5. RVIZ2 UNTUK PEMANTAUAN
    # ========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('agv_sim'), 'rviz', 'slam_config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Kumpulkan semua node untuk diluncurkan bersamaan
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Gunakan waktu nyata untuk hardware'),
        
        # Node yang aktif sekarang:
        tf_kinect1, tf_kinect2, tf_kinect3, tf_kinect4,
        slam_main_node,
        udp_receiver_node, # <--- NODE UDP SUDAH MASUK DAFTAR AKTIF
        rviz_node
        
        # Node yang dimatikan sementara:
        # kinect1_node, kinect2_node, kinect3_node, kinect4_node,
        # pointcloud_concat_node
    ])
