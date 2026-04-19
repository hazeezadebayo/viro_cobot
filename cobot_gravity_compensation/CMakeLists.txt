cmake_minimum_required(VERSION 3.8)
project(cobot_gravity_compensation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

# Gravity Compensator Library (The Core Math)
add_library(gravity_comp_lib
  src/gravity_compensator.cpp
)
ament_target_dependencies(gravity_comp_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  kdl_parser
  urdf
)
target_link_libraries(gravity_comp_lib
  orocos-kdl
  ${Eigen3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# ROS 2 Node
add_executable(gravity_comp_node
  src/gravity_comp_node.cpp
)
ament_target_dependencies(gravity_comp_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  kdl_parser
  urdf
)
target_link_libraries(gravity_comp_node
  gravity_comp_lib
  orocos-kdl
  ${Eigen3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Install
install(TARGETS
  gravity_comp_lib
  gravity_comp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
