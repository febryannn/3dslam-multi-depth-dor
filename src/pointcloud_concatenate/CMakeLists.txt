cmake_minimum_required(VERSION 3.8)
project(pointcloud_concatenate)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

include_directories(include ${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_lib src/pointcloud_concatenate.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp sensor_msgs pcl_conversions pcl_ros tf2 tf2_ros tf2_sensor_msgs)
target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})

add_executable(pointcloud_concatenate_node src/pointcloud_concatenate_node.cpp)
target_link_libraries(pointcloud_concatenate_node ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_concatenate_node rclcpp sensor_msgs pcl_conversions tf2 tf2_ros)

install(TARGETS ${PROJECT_NAME}_lib pointcloud_concatenate_node DESTINATION lib/${PROJECT_NAME})
ament_package()
