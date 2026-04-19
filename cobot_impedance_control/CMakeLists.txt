cmake_minimum_required(VERSION 3.8)
project(cobot_impedance_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(trajectory_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

add_library(impedance_controller_lib
  src/impedance_controller.cpp
)
ament_target_dependencies(impedance_controller_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  # trajectory_msgs
  std_msgs
  kdl_parser
)
target_link_libraries(impedance_controller_lib
  orocos-kdl
  ${Eigen3_LIBRARIES}
)

add_executable(impedance_node
  src/impedance_node.cpp
)
ament_target_dependencies(impedance_node
  rclcpp
  sensor_msgs
  geometry_msgs
  # trajectory_msgs
  std_msgs
  kdl_parser
)
target_link_libraries(impedance_node
  impedance_controller_lib
  orocos-kdl
  ${Eigen3_LIBRARIES}
)

install(TARGETS
  impedance_controller_lib
  impedance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
