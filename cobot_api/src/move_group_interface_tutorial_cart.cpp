#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <thread>
#include <chrono>
#include <yaml-cpp/yaml.h>

// ros2 launch cobot_api cobot_api.launch.py move_group_demo:=cart

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    // publish to goal pose topic
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    goal_pub_ = move_group_node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_state", 10);

    // We spin up a SingleThreadedExecutor for the current state monitor to get
    // information about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // getting path
    std::string ros_pkg = "cobot_api";
    std::string ros_pkg_path = ament_index_cpp::get_package_share_directory(ros_pkg);
    std::string relative_yaml_path = "/config/move_group_trajectory.yaml";
    std::string yaml_path = ros_pkg_path.append(relative_yaml_path);

    std::vector<std::vector<double>> target_group_positions_list; // friction_parameters = yaml_list["friction_params"].as<std::vector<double>>();

    try {
        // parsing yaml params
        YAML::Node yaml_config = YAML::LoadFile(yaml_path);
        // if (yaml_config["move_group_trajectory"]) {
        if (yaml_config["move_group_trajectory"] && yaml_config["move_group_trajectory"]["cart_trajectory"]) {
          YAML::Node jointTrajectory = yaml_config["move_group_trajectory"]["cart_trajectory"];
          for (const auto& pos : jointTrajectory) {
              std::vector<double> cart_positions;
              for (const auto& val : pos.second) {
                  cart_positions.push_back(val.as<double>());
              }

              // print statements 
              std::cout << "Cart Positions: "; for (const auto& value : cart_positions) { std::cout << value << " "; } std::cout << std::endl;
              target_group_positions_list.push_back(cart_positions);
          }   
        }
        else {
            RCLCPP_ERROR(LOGGER, "Could not find expected nodes in the YAML file: %s", yaml_path.c_str()); // ROS_ERROR("Yaml path: %s", yaml_path.c_str());
            assert(false);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Error while reading the YAML file: %s", e.what());
        assert(false);
    }

    static const std::string PLANNING_GROUP = "cobot_arm"; // panda_arm
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str()); // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(LOGGER, "Available Planning Groups:"); // We can get a list of all the groups in the robot:
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    move_group.setMaxVelocityScalingFactor(0.40); // 0.05
    move_group.setMaxAccelerationScalingFactor(0.30); // 0.05

    // Delay for 5 seconds using ROS-specific function
    std::this_thread::sleep_for(std::chrono::seconds(5));

    move_group.setJointValueTarget(target_group_positions_list[0]);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // galactic
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // humble
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
        move_group.move();

    // Delay for 1 seconds using ROS-specific function
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!target_group_positions_list.empty()) {
        target_group_positions_list.erase(target_group_positions_list.begin());
    }

    while (true) {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      
      for (auto &target_group_positions : target_group_positions_list) {
        // Set target pose: Set orientation and position for target_pose based on target_group_positions
        geometry_msgs::msg::Pose target_pose;
        
        double r = target_group_positions[3], p = target_group_positions[4], y = target_group_positions[5];
        tf2::Quaternion q;
        q.setRPY(r, p, y);
        tf2::convert(q, target_pose.orientation); 
        target_pose.orientation = target_pose.orientation; 

        target_pose.position.x = target_group_positions[0];
        target_pose.position.y = target_group_positions[1];
        target_pose.position.z = target_group_positions[2];     
        waypoints.push_back(target_pose);

        // RCLCPP_INFO(LOGGER, "===> Goal pose initialized: ");
        // Show target pose: Create a PoseStamped message to be published
        geometry_msgs::msg::PoseStamped pose_stamped_msg;
        pose_stamped_msg.header.frame_id = "base_link"; // Set the frame id
        pose_stamped_msg.header.stamp = move_group_node->now();
        pose_stamped_msg.pose.position = target_pose.position;
        pose_stamped_msg.pose.orientation = target_pose.orientation;  // Set the orientation
        goal_pub_->publish(pose_stamped_msg);
      }

      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double res = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      RCLCPP_INFO(LOGGER, "\n Cartesian path plan (%.2f%% res acheived) and  %lu points", res * 100.0, waypoints.size());
      if(res<0.7) // 0.8 // 0.9
      {
          RCLCPP_ERROR(LOGGER, "Cartesian plan only solved %f of the %lu points", 100.0 * res, waypoints.size());
          return 0;
      }

      // Move the manipulator according to calculated cartesian(linear) path
      // galactic
      // bool success = (move_group.execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      // humble
      bool success = (move_group.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success)  {
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Adjust sleep duration
        RCLCPP_INFO(LOGGER, "===> finished successfully: ");
      }      
      
    }

  // END_TUTORIAL
  std::this_thread::sleep_for(std::chrono::seconds(5));
  rclcpp::shutdown();
  return 0;
}
















/* 
"move_group_trajectory.yaml" sample content:

move_group_trajectory:
# -----------------------------------------------------------------------------------
# MoveGroup joint space goals demo:
# move_group_trajectory:
#   joint_trajectory:
#       pos1: [j1,  j2,  j3,  j4,  j5,  j6]
#       pos2: [j1,  j2,  j3,  j4,  j5,  j6]
# -----------------------------------------------------------------------------------
    joint_trajectory: 
        pos1:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pos2:  [1.57, 0.0, 0.75, 0.0, 1.0, -1.0]
        pos3:  [0.0, 0.0, 1.57, 0.0, 1.5, 0.0]
        pos4:  [-1.57, 0.0, 0.75, 0.0, 1.0, 1.0]
        pos5:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pos6:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pos7:  [-1.57, 0.0, 0.75, 0.0, 1.0, -1.0]
        pos8:  [0.0, 0.0, 1.57, 0.0, 1.5, 0.0]
        pos9:  [1.57, 0.0, 0.75, 0.0, 1.0, 1.0]
        pos10: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# -----------------------------------------------------------------------------------
# MoveGroup cartesian goals demo executable
# first pose should be a joint_space goal so as to not give cartesian plan error.
# move_group_trajectory:
#   cart_trajectory:
#       joint_pos0: [j1,  j2,  j3,  j4,  j5,  j6]
#       pos1: [x,  y,   z,  roll, pitch, yaw]
#       pos2: [x,  y,   z,  roll, pitch, yaw]
# -----------------------------------------------------------------------------------
    cart_trajectory:
        joint_pos0: [1.1605692565236883, -0.378789872255207, 2.1052728637579925, 0.8838470401018528, 1.5555991649132412, -2.7956814099957525]
        pos1:  [-0.57686, -0.06178, 0.12705, -3.1191507, 0.0242322, -1.5410678]
        pos2:  [-0.57979, -0.15956,   0.12492, -3.1190814, 0.0240148, -1.5411366]
        pos3:  [-0.49434, -0.16213, 0.12676, -3.1190882, 0.0241914, -1.5411103]
        pos4:  [-0.49217, -0.089334, 0.12852, -3.1190276, 0.0242998, -1.5410274]
*/