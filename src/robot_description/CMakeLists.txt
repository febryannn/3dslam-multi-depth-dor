cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 1. Install folder launch (agar ros2 launch bisa menemukannya)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install folder urdf/xacro (agar robot_state_publisher bisa membacanya)
install(DIRECTORY urdf  # Asumsi folder xacro anda bernama 'urdf' atau letakkan file xacro di folder ini
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install folder meshes (PENTING: agar RViz bisa memuat file agv.stl)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
