cmake_minimum_required(VERSION 3.8)
project(ndt_omp_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common registration filters)
find_package(OpenMP)

if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Header-only library — export include dir
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
