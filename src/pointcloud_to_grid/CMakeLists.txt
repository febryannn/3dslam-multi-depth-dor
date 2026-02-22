cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_grid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Executable
add_executable(pointcloud_to_grid_node 
  src/pointcloud_to_grid_node.cpp
)

ament_target_dependencies(pointcloud_to_grid_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
)

target_link_libraries(pointcloud_to_grid_node
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS pointcloud_to_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install config files if exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
