cmake_minimum_required(VERSION 3.8)
project(navigasi)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# main nav
add_executable(movebase_nav src/movebase_nav_differential.cpp)

# simple controllers
add_executable(simple_nav_diff src/simple_nav_differential.cpp)
add_executable(simple_nav_omni src/simple_nav_omni.cpp)

ament_target_dependencies(movebase_nav rclcpp geometry_msgs nav_msgs tf2 tf2_ros)
ament_target_dependencies(simple_nav_diff rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(simple_nav_omni rclcpp geometry_msgs nav_msgs)

install(TARGETS
  movebase_nav
  simple_nav_diff
  simple_nav_omni
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

