cmake_minimum_required(VERSION 3.8)
project(arm_probe_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(gap_explorer_interfaces REQUIRED)

add_executable(arm_probe_node src/arm_probe_node.cpp)

ament_target_dependencies(
  arm_probe_node
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  gap_explorer_interfaces
)

install(TARGETS
  arm_probe_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()