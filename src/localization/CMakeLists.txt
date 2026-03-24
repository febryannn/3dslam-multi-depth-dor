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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation registration)
find_package(g2o REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# SLAM RGBD Cam Node (scan-to-submap + g2o backend loop closure)
add_executable(slam_rgbd_cam_node src/slam_rgbd_cam.cpp)
target_link_libraries(slam_rgbd_cam_node
  ${PCL_LIBRARIES}
  g2o::core
  g2o::solver_eigen
  g2o::types_slam3d
)
ament_target_dependencies(slam_rgbd_cam_node rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs tf2_eigen pcl_conversions)

# RGBD Cam Localization Node (ICP-based localization vs pre-built map)
add_executable(rgbd_cam_loc_node src/rgbd_cam_loc.cpp)
target_link_libraries(rgbd_cam_loc_node ${PCL_LIBRARIES})
ament_target_dependencies(rgbd_cam_loc_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions)

# Node Penerima UDP Odometry
add_executable(udp_receiver src/udp_receiver.cpp)
ament_target_dependencies(udp_receiver rclcpp nav_msgs tf2 tf2_ros tf2_geometry_msgs)

# Install targets
install(TARGETS
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
