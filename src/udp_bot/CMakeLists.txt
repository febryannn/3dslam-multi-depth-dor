cmake_minimum_required(VERSION 3.8)
project(udp_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(udp_bot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)
# Ini memungkinkan Anda menggunakan #include "udp_bot/UdpSocket.h"
# atau #include "UdpSocket.h" tergantung struktur folder include/udp_bot
include_directories(include/udp_bot)

# ── udp_bot_node ──────────────────────────────────────────────
add_executable(udp_bot_node src/udp_bot_node.cpp)
ament_target_dependencies(udp_bot_node
  rclcpp
  udp_bot_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── teleop ───────────────────────────────────────────────────
add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop
  rclcpp
  udp_bot_msgs
  geometry_msgs
)

# ── wifi_node (UnicastLan) ───────────────────────────────────
# PERBAIKAN: Menambahkan executable untuk wifi.cpp
add_executable(wifi_node src/wifi.cpp)
ament_target_dependencies(wifi_node
  rclcpp
  udp_bot_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── install executables ───────────────────────────────────────
install(TARGETS
  udp_bot_node
  teleop
  wifi_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── install headers ───────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include
)

# testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
