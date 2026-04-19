#include "cobot_gravity_compensation/gravity_comp_node.hpp"
#include <urdf/model.h>

namespace cobot_gravity_compensation
{

GravityCompNode::GravityCompNode(const rclcpp::NodeOptions & options)
: Node("gravity_comp_node", options)
{
  // 1. Parameters
  this->declare_parameter("base_link", "base_link");
  this->declare_parameter("end_effector", "link_6");
  this->declare_parameter("joint_names", std::vector<std::string>{});
  this->declare_parameter("robot_description", "");
  this->declare_parameter("direct_torque_topic", "/gravity/effort");

  std::string base_link = this->get_parameter("base_link").as_string();
  std::string ee_link = this->get_parameter("end_effector").as_string();
  joint_names_ = this->get_parameter("joint_names").as_string_array();
  std::string direct_torque_topic = this->get_parameter("direct_torque_topic").as_string();
  std::string urdf_xml = this->get_parameter("robot_description").as_string();

  if (urdf_xml.empty()) {
    RCLCPP_ERROR(this->get_logger(), "robot_description parameter is empty!");
    return;
  }

  // 2. Initialize the Math Logic
  try {
    compensator_ = std::make_unique<GravityCompensator>(urdf_xml, base_link, ee_link);
    compensator_->set_friction_params(load_friction_params());
    RCLCPP_INFO(this->get_logger(), "Gravity Compensator initialized for %zu joints.", compensator_->get_num_joints());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Initialization failed: %s", e.what());
    return;
  }

  // 3. Setup Dynamic Parameters
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&GravityCompNode::on_parameter_set, this, std::placeholders::_1));

  // 4. Setup Communication
  direct_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(direct_torque_topic, 10);
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&GravityCompNode::joint_state_callback, this, std::placeholders::_1));

  initialized_ = true;
}

rcl_interfaces::msg::SetParametersResult GravityCompNode::on_parameter_set(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    std::string name = param.get_name();
    if (name.find("friction.") == 0) {
      RCLCPP_INFO(this->get_logger(), "Friction parameter changed: %s", name.c_str());
    }
  }

  std::map<std::string, FrictionParams> new_params = load_friction_params();
  if (compensator_) {
    compensator_->set_friction_params(new_params);
  }

  return result;
}

std::map<std::string, FrictionParams> GravityCompNode::load_friction_params()
{
  std::map<std::string, FrictionParams> params;
  for (const auto & name : joint_names_) {
    FrictionParams p;
    this->declare_parameter("friction." + name + ".f_v", 0.0);
    this->declare_parameter("friction." + name + ".f_c", 0.0);
    this->declare_parameter("friction." + name + ".f_s", 0.0);
    this->declare_parameter("friction." + name + ".f_1", 10.0);
    this->declare_parameter("friction." + name + ".f_2", 100.0);

    this->get_parameter("friction." + name + ".f_v", p.f_v);
    this->get_parameter("friction." + name + ".f_c", p.f_c);
    this->get_parameter("friction." + name + ".f_s", p.f_s);
    this->get_parameter("friction." + name + ".f_1", p.f_1);
    this->get_parameter("friction." + name + ".f_2", p.f_2);
    
    params[name] = p;
  }
  return params;
}

void GravityCompNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!initialized_) return;

  size_t nj = joint_names_.size();
  Eigen::VectorXd q(nj);
  Eigen::VectorXd qd(nj);

  bool found_all = true;
  for (size_t i = 0; i < nj; ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
    if (it != msg->name.end()) {
      size_t index = std::distance(msg->name.begin(), it);
      q(i) = msg->position[index];
      qd(i) = msg->velocity[index];
    } else {
      found_all = false;
      break;
    }
  }

  if (!found_all) return;

  // Compute Compensation (Purely Gravity + Friction)
  Eigen::VectorXd tau = compensator_->compute_compensation(q, qd);

  // Publish Efforts
  std_msgs::msg::Float64MultiArray direct_msg;
  direct_msg.data.assign(tau.data(), tau.data() + tau.size());
  direct_torque_pub_->publish(direct_msg);
}

}  // namespace cobot_gravity_compensation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cobot_gravity_compensation::GravityCompNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
