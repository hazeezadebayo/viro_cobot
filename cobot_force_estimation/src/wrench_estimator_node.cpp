#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "cobot_force_estimation/wrench_estimator.hpp"

namespace cobot_force_estimation
{

class WrenchEstimatorNode : public rclcpp::Node
{
public:
  WrenchEstimatorNode() : Node("wrench_estimator_node")
  {
    // 1. Parameters
    this->declare_parameter("base_link", "base_link");
    this->declare_parameter("end_effector", "link_6");
    this->declare_parameter("publishing_frequency", 50.0);
    this->declare_parameter("lpf_alpha", 0.1); // Low pass filter alpha

    // 2. Load URDF
    std::string urdf_xml;
    this->declare_parameter("robot_description", "");
    this->get_parameter("robot_description", urdf_xml);

    if (urdf_xml.empty()) {
        RCLCPP_INFO(this->get_logger(), "Retrieving robot_description from global parameters...");
        // In many setups, it's provided by robot_state_publisher
    }

    // 3. Setup Estimator
    try {
        // We'll wait until we get the URDF
        auto base = this->get_parameter("base_link").as_string();
        auto ee = this->get_parameter("end_effector").as_string();
        
        // This is a placeholder since urdf might be empty at start
        // We will initialize in the first callback if needed
    } catch (...) {}

    // 4. Communication
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&WrenchEstimatorNode::joint_callback, this, std::placeholders::_1));
    
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/cobot/estimated_wrench", 10);

    RCLCPP_INFO(this->get_logger(), "Virtual Wrench Estimator Node started.");
  }

private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!estimator_) {
        std::string urdf_xml;
        // Search for robot_description in parameters (might be in a different node)
        // For simplicity, we assume it's passed or available.
        if (!this->get_parameter("robot_description", urdf_xml) || urdf_xml.empty()) {
            return; 
        }
        estimator_ = std::make_unique<WrenchEstimator>(
            urdf_xml, 
            this->get_parameter("base_link").as_string(), 
            this->get_parameter("end_effector").as_string());
        
        joint_names_ = estimator_->get_joint_names();
        nj_ = joint_names_.size();
        filtered_wrench_.setZero();
    }

    Eigen::VectorXd q(nj_);
    Eigen::VectorXd qd(nj_);
    Eigen::VectorXd tau(nj_);

    // Robust mapping by name
    for (size_t i = 0; i < nj_; ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
        if (it != msg->name.end()) {
            size_t idx = std::distance(msg->name.begin(), it);
            q(i) = msg->position[idx];
            qd(i) = msg->velocity[idx];
            tau(i) = msg->effort[idx];
        } else {
            RCLCPP_WARN_ONCE(this->get_logger(), "Joint %s not found in JointState message!", joint_names_[i].c_str());
            q(i) = 0.0;
            qd(i) = 0.0;
            tau(i) = 0.0;
        }
    }

    // Estimate Wrench
    Eigen::Matrix<double, 6, 1> raw_wrench = estimator_->estimate_wrench(q, qd, tau);

    // Apply Low Pass Filter
    double alpha = this->get_parameter("lpf_alpha").as_double();
    filtered_wrench_ = alpha * raw_wrench + (1.0 - alpha) * filtered_wrench_;

    // Publish
    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = this->get_clock()->now();
    wrench_msg.header.frame_id = this->get_parameter("end_effector").as_string();
    
    wrench_msg.wrench.force.x = filtered_wrench_(0);
    wrench_msg.wrench.force.y = filtered_wrench_(1);
    wrench_msg.wrench.force.z = filtered_wrench_(2);
    wrench_msg.wrench.torque.x = filtered_wrench_(3);
    wrench_msg.wrench.torque.y = filtered_wrench_(4);
    wrench_msg.wrench.torque.z = filtered_wrench_(5);

    wrench_pub_->publish(wrench_msg);
  }

  std::unique_ptr<WrenchEstimator> estimator_;
  std::vector<std::string> joint_names_;
  size_t nj_{0};

  Eigen::Matrix<double, 6, 1> filtered_wrench_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
};

} // namespace cobot_force_estimation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cobot_force_estimation::WrenchEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
