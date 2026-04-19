#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <chrono>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// ros2 launch cobot_api cobot_api.launch.py move_group_demo:=joint

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_ =
    //  move_group_node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_state", 10);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);

    std::thread([&executor]() { executor.spin(); }).detach();

    std::vector<std::vector<double>> joint_group_positions_list; // friction_parameters = yaml_list["friction_params"].as<std::vector<double>>();

    // getting path
    std::string ros_pkg = "cobot_api";
    std::string ros_pkg_path = ament_index_cpp::get_package_share_directory(ros_pkg);
    std::string relative_yaml_path = "/config/move_group_trajectory.yaml";
    std::string yaml_path = ros_pkg_path.append(relative_yaml_path);

    try {
        // parsing yaml params
        YAML::Node yaml_config = YAML::LoadFile(yaml_path);
        if (yaml_config["move_group_trajectory"]) {
        YAML::Node jointTrajectory = yaml_config["move_group_trajectory"]["joint_trajectory"];
        
        for (const auto& pos : jointTrajectory) {
            std::vector<double> joint_positions;
            for (const auto& val : pos.second) {
                joint_positions.push_back(val.as<double>());
            }
            joint_group_positions_list.push_back(joint_positions);
        }   
        }
        else {
            RCLCPP_ERROR(LOGGER, "Could not find yaml parameters! at Yaml path: %s", yaml_path.c_str()); // ROS_ERROR("Yaml path: %s", yaml_path.c_str());
            assert(false);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Error while reading the YAML file: %s", e.what());
        assert(false);
    }


    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, "cobot_arm");

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.

    move_group.setMaxVelocityScalingFactor(0.50); // 0.05
    move_group.setMaxAccelerationScalingFactor(0.40); // 0.05

    // -----------------------------------------------------
    // if we desire a while loop for which the motion never stops and restarted at every end:
    // -----------------------------------------------------
    size_t num_positions = joint_group_positions_list.size();
    size_t current_index = 0;
    while (true) {
        auto& joint_group_positions = joint_group_positions_list[current_index];
        move_group.setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // galactic
        // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // humble
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
            move_group.move();

        current_index = (current_index + 1) % num_positions; // Cycle through positions

        std::this_thread::sleep_for(std::chrono::seconds(2)); // sleep duration
    }
    // -----------------------------------------------------
    // -----------------------------------------------------
    // if we desire a for loop for which the motion is done once and never restarted
    // -----------------------------------------------------
    /* 
    for (auto &joint_group_positions : joint_group_positions_list) {

        move_group.setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // galactic
        // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // humble
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        move_group.move();
    }
    */

    // // move_group.stop();
    std::this_thread::sleep_for(std::chrono::seconds(1000));
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