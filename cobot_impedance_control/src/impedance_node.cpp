#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "cobot_impedance_control/impedance_controller.hpp"

namespace cobot_impedance_control
{

class ImpedanceNode : public rclcpp::Node
{
public:
  ImpedanceNode() : Node("impedance_node")
  {
    // 1. Parameters
    this->declare_parameter("base_link", "base_link");
    this->declare_parameter("end_effector", "wrist_3_link");
    this->declare_parameter("robot_description", "");
    this->declare_parameter("stiffness", std::vector<double>{200.0, 200.0, 200.0, 10.0, 10.0, 10.0});
    this->declare_parameter("damping", std::vector<double>{20.0, 20.0, 20.0, 1.0, 1.0, 1.0});
    this->declare_parameter("joint_names", std::vector<std::string>{});
    this->declare_parameter("setpoint_topic", "/cobot/setpoint_pose");

    // 2. Initialize Controller
    std::string urdf = this->get_parameter("robot_description").as_string();
    std::string base = this->get_parameter("base_link").as_string();
    std::string ee = this->get_parameter("end_effector").as_string();
    joint_names_ = this->get_parameter("joint_names").as_string_array();

    if (urdf.empty()) {
      RCLCPP_ERROR(this->get_logger(), "robot_description is empty!");
      return;
    }

    controller_ = std::make_unique<ImpedanceController>(urdf, base, ee);
    
    update_gains();

    // 3. Subscription for Setpoint
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->get_parameter("setpoint_topic").as_string(), 10, std::bind(&ImpedanceNode::target_pose_callback, this, std::placeholders::_1));
    
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
       "/joint_states", 10, std::bind(&ImpedanceNode::joint_state_callback, this, std::placeholders::_1));

    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/impedance/effort", 10);

    RCLCPP_INFO(this->get_logger(), "Impedance Node initialized (Safety Stiffness Mode).");
  }

private:
  void update_gains()
  {
    auto k = this->get_parameter("stiffness").as_double_array();
    auto d = this->get_parameter("damping").as_double_array();
    Eigen::Vector6d k_vec, d_vec;
    for(int i=0; i<6; ++i) { k_vec(i) = k[i]; d_vec(i) = d[i]; }
    controller_->set_gains(k_vec, d_vec);
  }

  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    target_pos_(0) = msg->pose.position.x;
    target_pos_(1) = msg->pose.position.y;
    target_pos_(2) = msg->pose.position.z;
    
    target_quat_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    target_captured_ = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    size_t nj = joint_names_.size();
    if (nj == 0) return;

    Eigen::VectorXd q(nj);
    Eigen::VectorXd qd(nj);

    // Map joints
    for (size_t i = 0; i < nj; ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it != msg->name.end()) {
        size_t index = std::distance(msg->name.begin(), it);
        q(i) = msg->position[index];
        qd(i) = msg->velocity[index];
      }
    }

    // Capture first pose as target if no external setpoint received
    if (!target_captured_) {
       // KDL Forward Kinematics would be better, but for now we wait for setpoint or use controller's internal storage
       // For a cleaner MVL, we let the controller handle it or wait for the first user setpoint message.
       return;
    }

    // Compute Torques
    Eigen::VectorXd tau = controller_->compute_torques(q, qd, target_pos_, target_quat_);

    // Publish Effort
    std_msgs::msg::Float64MultiArray effort_msg;
    effort_msg.data.assign(tau.data(), tau.data() + tau.size());
    effort_pub_->publish(effort_msg);
  }

  std::unique_ptr<ImpedanceController> controller_;
  std::vector<std::string> joint_names_;
  Eigen::Vector3d target_pos_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond target_quat_ = Eigen::Quaterniond::Identity();
  bool target_captured_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
};

} // namespace cobot_impedance_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cobot_impedance_control::ImpedanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
