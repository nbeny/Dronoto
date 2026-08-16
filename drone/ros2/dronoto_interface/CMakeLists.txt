cmake_minimum_required(VERSION 3.16)
project(dronoto_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(dronoto_msgs REQUIRED)
find_package(dronoto_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(px4_interface_node src/px4_interface_node.cpp)
ament_target_dependencies(px4_interface_node
  rclcpp px4_msgs dronoto_msgs dronoto_core nav_msgs geometry_msgs std_srvs tf2_ros
)
target_link_libraries(px4_interface_node Eigen3::Eigen)

install(TARGETS px4_interface_node DESTINATION lib/${PROJECT_NAME})

ament_package()
