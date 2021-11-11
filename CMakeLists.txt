cmake_minimum_required(VERSION 3.5)
project(camera_lite)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node 
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge)

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
