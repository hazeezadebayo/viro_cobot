cmake_minimum_required(VERSION 3.5)
project(soem_ethercat_driver)

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

add_compile_options(-std=c++17 -std=gnu++17)

# set(INSTALL_LIB_DIR "${PROJECT_BINARY_DIR}/lib")

set(CMAKE_SKIP_BUILD_RPATH FALSE)

set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${PROJECT_BINARY_DIR}")

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(soem_ros2 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread system)


include_directories(
  include
  /usr/include/
)

add_library(${PROJECT_NAME}_lib
  SHARED
  "src/ethercat_driver.cpp"
  "src/ethercat_module_client.cpp"
  "src/ethercat_manager.cpp"
)

# Set include directories for the library
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib 
  ${Boost_LIBRARIES}
)

set(DEPENDENCIES
  rclcpp
  rclpy
  std_msgs
  soem_ros2
  trajectory_msgs
  sensor_msgs
  std_srvs
  tf2
  hardware_interface
  rosidl_default_generators
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

# Register the library for other packages
ament_export_dependencies(Boost)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(${PROJECT_NAME}_lib)
ament_export_include_directories(include)

# The test executable
set(EXECUTABLE_NAME "ethercat_driver_test")
add_executable(${EXECUTABLE_NAME} 
src/ethercat_driver_test.cpp
)

target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${EXECUTABLE_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
