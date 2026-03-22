cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(udp_bot_msgs REQUIRED)
find_package(marvelmind_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation registration)
find_package(octomap REQUIRED)

# Include directories dan definisi PCL & Octomap
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS} # Tambahan Octomap
)
add_definitions(${PCL_DEFINITIONS})

# --- Daftar Executable Node ---

# 1. Fusion Node
add_executable(fusion_node src/fusion_gps_odom.cpp)
ament_target_dependencies(fusion_node rclcpp geometry_msgs udp_bot_msgs marvelmind_msgs tf2 tf2_ros tf2_geometry_msgs)

# 2. Odom Node
add_executable(odom_node src/odom.cpp)
ament_target_dependencies(odom_node rclcpp geometry_msgs udp_bot_msgs marvelmind_msgs tf2 tf2_ros tf2_geometry_msgs)

# 3. EKF Node
add_executable(ekf_node src/ekf_gps_odom.cpp)
ament_target_dependencies(ekf_node rclcpp geometry_msgs udp_bot_msgs marvelmind_msgs tf2 tf2_ros tf2_geometry_msgs)

# 4. UKF Node
add_executable(ukf_node src/ukf_gps_odom.cpp)
ament_target_dependencies(ukf_node rclcpp geometry_msgs udp_bot_msgs marvelmind_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)

# 5. Recording Node
add_executable(recording_node src/record_data.cpp)
ament_target_dependencies(recording_node rclcpp geometry_msgs udp_bot_msgs marvelmind_msgs tf2 tf2_ros tf2_geometry_msgs)

# 6. SLAM Listen Node
add_executable(slam_listen_node src/slam_listen.cpp)
ament_target_dependencies(slam_listen_node rclcpp geometry_msgs udp_bot_msgs marvelmind_msgs tf2 tf2_ros tf2_geometry_msgs)

# 7. SLAM RGBD Cam Node (scan-to-submap, arsitektur lidarslam_ros2)
add_executable(slam_rgbd_cam_node src/slam_rgbd_cam.cpp)
target_link_libraries(slam_rgbd_cam_node ${PCL_LIBRARIES})
ament_target_dependencies(slam_rgbd_cam_node rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs tf2_eigen pcl_conversions)

# 8. RGBD Cam Localization Node (ICP-based localization vs pre-built map)
add_executable(rgbd_cam_loc_node src/rgbd_cam_loc.cpp)
target_link_libraries(rgbd_cam_loc_node ${PCL_LIBRARIES})
ament_target_dependencies(rgbd_cam_loc_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions)

# Node Penerima UDP Odometry
add_executable(udp_receiver src/udp_receiver.cpp)
ament_target_dependencies(udp_receiver rclcpp nav_msgs tf2 tf2_ros tf2_geometry_msgs)

# --- Install Targets ---
install(TARGETS
  fusion_node
  odom_node
  ekf_node
  ukf_node
  recording_node
  slam_listen_node
  slam_rgbd_cam_node
  rgbd_cam_loc_node
  udp_receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config maps launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
