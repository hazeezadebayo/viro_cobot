#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace cobot_control_mux
{

/**
 * @brief Unified Control Multiplexer
 * 
 * Sums effort commands from multiple sources (Gravity, Admittance, Impedance)
 * and publishes the result to the direct effort controller.
 * Also keeps the trajectory controller synchronized with the current position.
 */
class ControlMuxNode : public rclcpp::Node
{
public:
  ControlMuxNode() : Node("control_mux_node")
  {
    // 1. Parameters
    this->declare_parameter("update_rate_hz", 100.0);
    this->declare_parameter("joint_names", std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    this->declare_parameter("gravity_effort_topic", "/gravity/effort");
    this->declare_parameter("admittance_effort_topic", "/admittance/effort");
    this->declare_parameter("impedance_effort_topic", "/impedance/effort");
    this->declare_parameter("direct_effort_topic", "/effort_direct_controller/commands");
    this->declare_parameter("jtc_sync_topic", "/joint_trajectory_controller/joint_trajectory");
    this->declare_parameter("moveit_effort_topic", "/moveit/effort");

    joint_names_ = this->get_parameter("joint_names").as_string_array();

    // 2. Effort Sources (All these will be summed)
    setup_effort_source(this->get_parameter("gravity_effort_topic").as_string(), "Gravity");
    setup_effort_source(this->get_parameter("admittance_effort_topic").as_string(), "Admittance");
    setup_effort_source(this->get_parameter("impedance_effort_topic").as_string(), "Impedance");
    setup_effort_source(this->get_parameter("moveit_effort_topic").as_string(), "MoveIt");

    // 3. Feedback for Synchronization
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&ControlMuxNode::joint_state_callback, this, std::placeholders::_1));

    // 4. Outputs
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("jtc_sync_topic").as_string(), 10);
    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      this->get_parameter("direct_effort_topic").as_string(), 10);

    // 5. Timer for Summation and Sync
    double period_s = 1.0 / this->get_parameter("update_rate_hz").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_s), std::bind(&ControlMuxNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Control Mux started at %.1f Hz. Summing Gravity, Admittance, and Impedance.", this->get_parameter("update_rate_hz").as_double());
  }

private:
  void setup_effort_source(const std::string & topic, const std::string & label)
  {
    source_labels_[topic] = label;
    effort_subs_.push_back(this->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic, 10, [this, topic](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        last_efforts_[topic] = msg->data;
        last_msg_time_[topic] = this->get_clock()->now();
      }));
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (q_actual_.size() != joint_names_.size()) q_actual_.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it != msg->name.end()) {
        q_actual_[i] = msg->position[std::distance(msg->name.begin(), it)];
      }
    }
    joint_state_received_ = true;
  }

  void timer_callback()
  {
    auto now = this->get_clock()->now();
    auto timeout = 500ms;

    // --- 1. Effort Summation ---
    std_msgs::msg::Float64MultiArray sum_msg;
    sum_msg.data.assign(joint_names_.size(), 0.0);
    
    std::string active_sources = "";

    for (auto const& [topic, data] : last_efforts_) {
      if ((now - last_msg_time_[topic]) < timeout) {
        if (data.size() == joint_names_.size()) {
          for (size_t i = 0; i < data.size(); ++i) {
            sum_msg.data[i] += data[i];
          }
          active_sources += source_labels_[topic] + " ";
        }
      }
    }
    
    // Always publish summation to provide direct control baseline
    effort_pub_->publish(sum_msg);

    // Occasional logging of active and MISSING sources
    static int log_count = 0;
    if (++log_count >= 500) {
        if (!active_sources.empty()) {
            RCLCPP_INFO(this->get_logger(), "Participating sources: [ %s]", active_sources.c_str());
        }
        
        // Check for "Ghost" topics (registered but silent)
        for (auto const& [topic, label] : source_labels_) {
            if (last_msg_time_[topic].nanoseconds() == 0) {
                RCLCPP_WARN(this->get_logger(), "Topic SILENT (Never received): %s", topic.c_str());
            } else if ((now - last_msg_time_[topic]) > timeout) {
                RCLCPP_WARN(this->get_logger(), "Topic TIMEOUT (Stale): %s", topic.c_str());
            }
        }
        log_count = 0;
    }

    // --- 2. JTC Synchronization ---
    if (joint_state_received_) {
      trajectory_msgs::msg::JointTrajectory sync_traj;
      sync_traj.header.stamp = now;
      sync_traj.joint_names = joint_names_;

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = q_actual_;
      point.time_from_start = rclcpp::Duration::from_seconds(0.01);
      
      sync_traj.points.push_back(point);
      trajectory_pub_->publish(sync_traj);
    }
  }

  std::vector<std::string> joint_names_;
  std::vector<double> q_actual_;
  bool joint_state_received_{false};
  
  std::map<std::string, std::vector<double>> last_efforts_;
  std::map<std::string, rclcpp::Time> last_msg_time_;
  std::map<std::string, std::string> source_labels_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr> effort_subs_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace cobot_control_mux

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cobot_control_mux::ControlMuxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
