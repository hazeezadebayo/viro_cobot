cmake_minimum_required(VERSION 3.8)
project(cobot_force_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

# Wrench Estimator Library
add_library(wrench_estimator_lib
  src/wrench_estimator.cpp
)
ament_target_dependencies(wrench_estimator_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  kdl_parser
  urdf
)
target_link_libraries(wrench_estimator_lib
  orocos-kdl
  ${Eigen3_LIBRARIES}
)

# ROS 2 Node
add_executable(wrench_estimator_node
  src/wrench_estimator_node.cpp
)
ament_target_dependencies(wrench_estimator_node
  rclcpp
  sensor_msgs
  geometry_msgs
  kdl_parser
  urdf
)
target_link_libraries(wrench_estimator_node
  wrench_estimator_lib
  orocos-kdl
  ${Eigen3_LIBRARIES}
)

# Install
install(TARGETS
  wrench_estimator_lib
  wrench_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
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
