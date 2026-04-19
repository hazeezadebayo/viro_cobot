cmake_minimum_required(VERSION 3.8)
project(cobot_admittance_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

add_executable(admittance_node
  src/admittance_node.cpp
)
ament_target_dependencies(admittance_node
  rclcpp
  geometry_msgs
  sensor_msgs
  kdl_parser
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)
target_link_libraries(admittance_node
  orocos-kdl
  ${Eigen3_LIBRARIES}
)

install(TARGETS
  admittance_node
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
