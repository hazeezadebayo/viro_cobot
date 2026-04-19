cmake_minimum_required(VERSION 3.8)
project(cobot_control_mux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(control_mux_node src/control_mux_node.cpp)
ament_target_dependencies(control_mux_node
  rclcpp
  trajectory_msgs
  std_msgs
  sensor_msgs
)

add_executable(moveit_effort_bridge src/moveit_effort_bridge.cpp)
ament_target_dependencies(moveit_effort_bridge
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  std_msgs
  sensor_msgs
)

install(TARGETS
  control_mux_node
  moveit_effort_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
