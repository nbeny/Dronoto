cmake_minimum_required(VERSION 3.16)
project(dronoto_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dronoto_msgs REQUIRED)
find_package(dronoto_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mission_executive_node src/mission_executive_node.cpp)
ament_target_dependencies(mission_executive_node
  rclcpp dronoto_msgs dronoto_core geometry_msgs std_msgs std_srvs
)

install(TARGETS mission_executive_node DESTINATION lib/${PROJECT_NAME})

ament_package()
