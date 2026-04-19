#ifndef COBOT_GRAVITY_COMPENSATION__GRAVITY_COMP_NODE_HPP_
#define COBOT_GRAVITY_COMPENSATION__GRAVITY_COMP_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "cobot_gravity_compensation/gravity_compensator.hpp"

namespace cobot_gravity_compensation
{

class GravityCompNode : public rclcpp::Node
{
public:
  explicit GravityCompNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
  std::map<std::string, FrictionParams> load_friction_params();
  rcl_interfaces::msg::SetParametersResult on_parameter_set(const std::vector<rclcpp::Parameter> & parameters);

  // Core Logic
  std::unique_ptr<GravityCompensator> compensator_;
  std::vector<std::string> joint_names_;
  bool initialized_{false};

  // ROS 2 Communication
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr direct_torque_pub_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace cobot_gravity_compensation

#endif  // COBOT_GRAVITY_COMPENSATION__GRAVITY_COMP_NODE_HPP_
