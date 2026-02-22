cmake_minimum_required(VERSION 3.8)
project(agv_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(udp_bot_msgs REQUIRED)

# Node: odom_to_udp_relay (jembatan odom Gazebo -> udp_bot_msgs untuk SLAM)
add_executable(odom_to_udp_relay src/odom_to_udp_relay.cpp)
ament_target_dependencies(odom_to_udp_relay
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  udp_bot_msgs
)
install(TARGETS odom_to_udp_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  worlds
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
