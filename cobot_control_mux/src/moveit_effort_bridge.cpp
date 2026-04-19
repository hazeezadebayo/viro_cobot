#include <memory>
#include <string>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace cobot_control_mux
{

/**
 * @brief MoveIt-to-Effort Bridge Node
 * 
 * Implements the FollowJointTrajectory action server.
 * Translates the trajectory into direct effort commands via a PD loop.
 */
class MoveItEffortBridge : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  MoveItEffortBridge() : Node("moveit_effort_bridge")
  {
    // 1. Parameters
    this->declare_parameter("joint_names", std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    this->declare_parameter("effort_topic", "/moveit/effort");
    
    // Increased Gains for Authoritative Control (Stronger Grip)
    this->declare_parameter("p_gains", std::vector<double>{400.0, 400.0, 400.0, 150.0, 150.0, 150.0}); // {100.0, 100.0, 100.0, 40.0, 40.0, 40.0}
    this->declare_parameter("d_gains", std::vector<double>{20.0, 20.0, 20.0, 5.0, 5.0, 5.0}); // {5.0, 5.0, 5.0, 1.0, 1.0, 1.0}

    joint_names_ = this->get_parameter("joint_names").as_string_array();
    p_gains_ = this->get_parameter("p_gains").as_double_array();
    d_gains_ = this->get_parameter("d_gains").as_double_array();

    q_actual_.assign(joint_names_.size(), 0.0);
    qd_actual_.assign(joint_names_.size(), 0.0);

    // 2. Action Server
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "/joint_trajectory_controller/follow_joint_trajectory",
      std::bind(&MoveItEffortBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveItEffortBridge::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveItEffortBridge::handle_accepted, this, std::placeholders::_1));

    // 3. Feedback Subs & Pubs
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&MoveItEffortBridge::joint_state_callback, this, std::placeholders::_1));
    
    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      this->get_parameter("effort_topic").as_string(), 10);

    // 4. Control Timer (100Hz)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MoveItEffortBridge::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "MoveIt Effort Bridge active on /joint_trajectory_controller/follow_joint_trajectory");
  }

private:
  // --- Action Server Callbacks ---
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received trajectory goal with %zu points", goal->trajectory.points.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
    active_goal_ = nullptr;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    active_goal_ = goal_handle;
    start_time_ = this->get_clock()->now();
  }

  // --- Core Logic ---
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it != msg->name.end()) {
        size_t idx = std::distance(msg->name.begin(), it);
        q_actual_[i] = msg->position[idx];
        qd_actual_[i] = msg->velocity[idx];
      }
    }
  }

  void control_loop()
  {
    std_msgs::msg::Float64MultiArray effort_msg;
    effort_msg.data.assign(joint_names_.size(), 0.0);

    if (active_goal_) {
      auto current_time = this->get_clock()->now();
      double elapsed_s = (current_time - start_time_).seconds();
      
      const auto & trajectory = active_goal_->get_goal()->trajectory;
      
      // Find the appropriate waypoint based on elapsed time
      trajectory_msgs::msg::JointTrajectoryPoint target_point;
      bool found = false;
      
      for (const auto & point : trajectory.points) {
        if (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 > elapsed_s) {
          target_point = point;
          found = true;
          break;
        }
      }

      // If finished, succeed the goal
      if (!found) {
        RCLCPP_INFO(this->get_logger(), "Trajectory completed successfully");
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        active_goal_->succeed(result);
        active_goal_ = nullptr;
      } else {
        // PD Control: tau = Kp(q_target - q_actual) + Kd(qd_target - qd_actual)
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          // Find mapping from trajectory joint name to our joint name list
          auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), joint_names_[i]);
          if (it != trajectory.joint_names.end()) {
            size_t idx = std::distance(trajectory.joint_names.begin(), it);
            double q_target = target_point.positions[idx];
            double qd_target = target_point.velocities.empty() ? 0.0 : target_point.velocities[idx];
            
            effort_msg.data[i] = p_gains_[i] * (q_target - q_actual_[i]) + d_gains_[i] * (qd_target - qd_actual_[i]);
          }
        }
      }
    } else {
      // Explicitly zero out when idle to signal the Mux that inhibition isn't needed
      effort_msg.data.assign(joint_names_.size(), 0.0);
    }

    effort_pub_->publish(effort_msg);
  }

  std::vector<std::string> joint_names_;
  std::vector<double> p_gains_;
  std::vector<double> d_gains_;
  std::vector<double> q_actual_;
  std::vector<double> qd_actual_;

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleFollowJointTrajectory> active_goal_;
  rclcpp::Time start_time_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace cobot_control_mux

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cobot_control_mux::MoveItEffortBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
