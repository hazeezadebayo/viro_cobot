cmake_minimum_required(VERSION 3.8)
project(cobot_api)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  #message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_py REQUIRED)
find_package(cobot_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
# find_package(rviz_visual_tools REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
# find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(std_srvs REQUIRED)    ####
find_package(moveit_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# ---------------cpp-------------
# Include Cpp "include" directory
include_directories(
  include
  # ${PROJECT_SOURCE_DIR}/include/robot_library
  ${ament_INCLUDE_DIRS}
  )
# ----- moveit dependencies -------
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  geometry_msgs
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  cobot_msgs
  std_srvs
  # rviz_visual_tools
  moveit_planners_ompl
  # moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  control_msgs
  moveit_servo
)
# -------------------------------
# Create Cpp executable
add_executable(move_group_interface_tutorial_cart
    src/move_group_interface_tutorial_cart.cpp)
target_include_directories(move_group_interface_tutorial_cart
    PUBLIC include)
ament_target_dependencies(move_group_interface_tutorial_cart
    ${THIS_PACKAGE_INCLUDE_DEPENDS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(move_group_interface_tutorial_cart
  yaml-cpp)
# Install Cpp executables
install(TARGETS
move_group_interface_tutorial_cart
  DESTINATION lib/${PROJECT_NAME})
# -------------------------------
# Create Cpp executable
add_executable(move_group_interface_tutorial_joint
    src/move_group_interface_tutorial_joint.cpp)
target_include_directories(move_group_interface_tutorial_joint
    PUBLIC include)
ament_target_dependencies(move_group_interface_tutorial_joint
    ${THIS_PACKAGE_INCLUDE_DEPENDS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(move_group_interface_tutorial_joint
  yaml-cpp)
# Install Cpp executables
install(TARGETS
  move_group_interface_tutorial_joint
  DESTINATION lib/${PROJECT_NAME})
# -------------------------------
# Create Cpp executable
add_executable(move_group_service_node
    src/move_group_service_node.cpp)
target_include_directories(move_group_service_node
    PUBLIC include)
ament_target_dependencies(move_group_service_node
    ${THIS_PACKAGE_INCLUDE_DEPENDS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
# Install Cpp executables
install(TARGETS
  move_group_service_node
  DESTINATION lib/${PROJECT_NAME})
# -------------------------------
# Create Cpp executable
add_executable(cobot_group_teleop_api
    src/cobot_group_teleop_api.cpp)
target_include_directories(cobot_group_teleop_api
    PUBLIC include)
ament_target_dependencies(cobot_group_teleop_api
    ${THIS_PACKAGE_INCLUDE_DEPENDS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
# Install Cpp executables
install(TARGETS
  cobot_group_teleop_api
  DESTINATION lib/${PROJECT_NAME})
# -------------------------------





# --------------python-----------
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
# chmod +x scripts/cobot_job_client.py
install(PROGRAMS
  scripts/cobot_job_client.py
  scripts/joint_traj_ctrlr_test.py
  scripts/cobot_gui_client.py
  DESTINATION lib/${PROJECT_NAME}
)
# -------------------------------
## general path install
## copy launch files etc to share.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
# -------------------------------
# -------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
