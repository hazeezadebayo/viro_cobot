cmake_minimum_required(VERSION 3.8)
project(cobot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

install(
    DIRECTORY config launch worlds urdf meshes rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()