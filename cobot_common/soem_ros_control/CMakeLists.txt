cmake_minimum_required(VERSION 3.5)
project(soem_ros_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_SKIP_BUILD_RPATH FALSE)

set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${PROJECT_BINARY_DIR}/../soem_ethercat_driver")

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(soem_ethercat_driver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)


set(DEPENDENCIES
  "rclcpp"
  "rclpy"
  "std_msgs"
  "std_srvs"
  "trajectory_msgs"
  "sensor_msgs"
  "tf2"
  "hardware_interface"
  "controller_interface"
  "soem_ethercat_driver"
  "pluginlib"
  "rosidl_default_generators"
)

add_library(soem_hardware_interface
  SHARED
  src/soem_hardware_interface.cpp
)

target_include_directories(soem_hardware_interface
  PRIVATE
  include
)

ament_target_dependencies(soem_hardware_interface
  ${DEPENDENCIES}
)

target_link_libraries(soem_hardware_interface
  Boost::thread
  Boost::system
)


pluginlib_export_plugin_description_file(hardware_interface soem_hardware_interface.xml)

install(
  TARGETS soem_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
