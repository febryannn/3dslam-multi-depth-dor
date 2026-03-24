cmake_minimum_required(VERSION 3.8)
project(dynamic_object_removal)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters octree)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(dynamic_object_removal_node src/dynamic_object_removal_node.cpp)
ament_target_dependencies(dynamic_object_removal_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(dynamic_object_removal_node ${PCL_LIBRARIES})

install(TARGETS dynamic_object_removal_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
