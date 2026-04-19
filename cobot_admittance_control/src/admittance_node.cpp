#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace cobot_admittance_control
{

/**
 * @brief Simple Admittance Node (Direct Force-To-Torque Mapping)
 * 
 * This node implements pure transparency: tau = J^T * F_ext.
 * Simplified for maximum readability and zero-config operation.
 */
class AdmittanceNode : public rclcpp::Node
{
public:
  AdmittanceNode() : Node("admittance_node")
  {
    // 1. Core Parameters (Core Physics Only)
    this->declare_parameter("base_link", "base_link");
    this->declare_parameter("end_effector", "link_6");
    this->declare_parameter("robot_description", "");
    this->declare_parameter("joint_names", std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    this->declare_parameter("damping_factor", 2.0);   // Nms/rad
    this->declare_parameter("filter_alpha", 0.5);     // 0.0 to 1.0 (Low-pass)

    base_link = this->get_parameter("base_link").as_string();
    ee_link = this->get_parameter("end_effector").as_string();
    joint_names_ = this->get_parameter("joint_names").as_string_array();

    q_actual_ = Eigen::VectorXd::Zero(joint_names_.size());
    q_actual_dot_ = Eigen::VectorXd::Zero(joint_names_.size());
    filter_val_ = Eigen::VectorXd::Zero(6);

    // 2. Communication
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/cobot/wrist_ft", 10, [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        // Simple 1st-order Low-Pass Filter (AlphaFilter)
        double alpha = this->get_parameter("filter_alpha").as_double();
        filter_val_(0) = alpha * msg->wrench.force.x + (1.0 - alpha) * filter_val_(0);
        filter_val_(1) = alpha * msg->wrench.force.y + (1.0 - alpha) * filter_val_(1);
        filter_val_(2) = alpha * msg->wrench.force.z + (1.0 - alpha) * filter_val_(2);
        filter_val_(3) = alpha * msg->wrench.torque.x + (1.0 - alpha) * filter_val_(3);
        filter_val_(4) = alpha * msg->wrench.torque.y + (1.0 - alpha) * filter_val_(4);
        filter_val_(5) = alpha * msg->wrench.torque.z + (1.0 - alpha) * filter_val_(5);
        wrench_received_ = true;
      });
    
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (q_actual_.size() != (int)joint_names_.size()) q_actual_.resize(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
          if (it != msg->name.end()) {
            size_t idx = std::distance(msg->name.begin(), it);
            q_actual_(i) = msg->position[idx];
            q_actual_dot_(i) = msg->velocity[idx];
          }
        }
        joint_state_received_ = true;
      });

    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/admittance/effort", 10);

    // Initial prime
    std_msgs::msg::Float64MultiArray prime_msg;
    prime_msg.data.resize(6, 0.0);
    effort_pub_->publish(prime_msg);

    // 3. Initialize Kinematics (KDL)
    std::string urdf_xml = this->get_parameter("robot_description").as_string();
    KDL::Tree tree;
    if (kdl_parser::treeFromString(urdf_xml, tree)) {
      tree.getChain(base_link, ee_link, kdl_chain_);
      jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
      RCLCPP_INFO(this->get_logger(), "Admittance setup: %s -> %s", base_link.c_str(), ee_link.c_str());
    }

    // 4. Update Loop (100Hz)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&AdmittanceNode::control_loop, this));
  }

private:
  void control_loop()
  {
    if (!joint_state_received_ || !jac_solver_) return;

    // Output Result (Clean External Influence)
    Eigen::VectorXd F_ext = filter_val_;

    // 2. Mapping: tau = J^T * F_ext
    KDL::Jacobian jac(joint_names_.size());
    KDL::JntArray q_kdl(q_actual_.size());
    for(int i=0; i<q_actual_.size(); ++i) q_kdl(i) = q_actual_(i);
    jac_solver_->JntToJac(q_kdl, jac);
    
    Eigen::VectorXd tau_admittance = jac.data.transpose() * F_ext;

    // Optional: Virtual Damping for stability
    double damping = this->get_parameter("damping_factor").as_double();
    Eigen::VectorXd tau = tau_admittance - (damping * q_actual_dot_);

    std_msgs::msg::Float64MultiArray effort_msg;
    effort_msg.data.assign(tau.data(), tau.data() + tau.size());
    effort_pub_->publish(effort_msg);
  }

  Eigen::Matrix<double, 6, 1> filter_val_ = Eigen::Matrix<double, 6, 1>::Zero();
  bool wrench_received_{false};
  Eigen::VectorXd q_actual_;
  Eigen::VectorXd q_actual_dot_;
  bool joint_state_received_{false};

  std::string base_link, ee_link;
  std::vector<std::string> joint_names_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
};

} // namespace cobot_admittance_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cobot_admittance_control::AdmittanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
