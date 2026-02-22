cmake_minimum_required(VERSION 3.8)
project(udp_bot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KirimKecepatanUdp.msg"
  "msg/KirimOffsetUdp.msg"
  "msg/TerimaUdp.msg"
)

ament_package()

