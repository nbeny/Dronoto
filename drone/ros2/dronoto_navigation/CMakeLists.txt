cmake_minimum_required(VERSION 3.16)
project(dronoto_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dronoto_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(trajectory_follower_node src/trajectory_follower_node.cpp)
ament_target_dependencies(trajectory_follower_node
  rclcpp dronoto_msgs geometry_msgs std_msgs
)
target_link_libraries(trajectory_follower_node Eigen3::Eigen)

install(TARGETS trajectory_follower_node DESTINATION lib/${PROJECT_NAME})

ament_package()
