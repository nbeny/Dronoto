cmake_minimum_required(VERSION 3.16)
project(dronoto_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dronoto_msgs REQUIRED)
find_package(dronoto_core REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(safety_supervisor_node src/safety_supervisor_node.cpp)
ament_target_dependencies(safety_supervisor_node
  rclcpp dronoto_msgs dronoto_core px4_msgs
)

install(TARGETS safety_supervisor_node DESTINATION lib/${PROJECT_NAME})

ament_package()
