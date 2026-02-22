cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Cari Paket Dependensi
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate Service (Pastikan file srv/InputPos.srv sudah ada)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InputPos.srv"
  DEPENDENCIES geometry_msgs
)

# 3. Build Executables

# Node Manual Mapping
add_executable(manual_mapping src/manual_mapping.cpp)
ament_target_dependencies(manual_mapping 
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions
)
target_link_libraries(manual_mapping ${PCL_LIBRARIES})

# Node Simple Mapping
add_executable(simple_mapping src/simple_mapping.cpp)
ament_target_dependencies(simple_mapping 
  rclcpp sensor_msgs pcl_conversions
)
target_link_libraries(simple_mapping ${PCL_LIBRARIES})

# Node Converter
add_executable(converter src/converter.cpp)
ament_target_dependencies(converter rclcpp sensor_msgs)

# 4. Hubungkan Service Header ke Executable
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(manual_mapping "${cpp_typesupport_target}")

# 5. INSTRUKSI INSTALL (Agar 'ros2 run' bisa menemukan file)
install(TARGETS
  manual_mapping
  simple_mapping
  converter
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
