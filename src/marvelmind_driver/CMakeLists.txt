cmake_minimum_required(VERSION 3.8)
project(marvelmind_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  include
  include/marvelmind_driver
)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HedgePos.msg"
  "msg/HedgePosA.msg"
  "msg/HedgePosAng.msg"
  "msg/BeaconPosA.msg"
  "msg/HedgeImuRaw.msg"
  "msg/HedgeImuFusion.msg"
  "msg/BeaconDistance.msg"
  "msg/HedgeTelemetry.msg"
  "msg/HedgeQuality.msg"
  "msg/MarvelmindWaypoint.msg"
  "msg/MarvelmindUserData.msg"
  "srv/MarvelmindAPI.srv"
  DEPENDENCIES std_msgs
)

# Executable: marvelmind_api_ros ONLY
add_executable(marvelmind_node
  src/marvelmind_api_ros.cpp
  src/marvelmind_api.c
)

ament_target_dependencies(marvelmind_node 
  rclcpp 
  std_msgs
)

# Link with generated messages
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} 
  "rosidl_typesupport_cpp"
)
target_link_libraries(marvelmind_node 
  "${cpp_typesupport_target}"
  ${CMAKE_DL_LIBS}
)

# Install
install(TARGETS marvelmind_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
