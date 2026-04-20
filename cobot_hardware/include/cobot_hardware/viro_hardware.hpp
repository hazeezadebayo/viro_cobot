#ifndef COBOT_HARDWARE__VIRO_HARDWARE_HPP_
#define COBOT_HARDWARE__VIRO_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "cobot_hardware/visibility_control.h"
#include "cobot_hardware/viro_comm.hpp"

namespace cobot_hardware
{
class ViroHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ViroHardware)

  COBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  COBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  COBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  COBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  COBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  COBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  COBOT_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  COBOT_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  ViroComm comm_;
  std::vector<JointFeedback> hw_states_;
  std::vector<double> hw_commands_;
  WrenchFeedback hw_wrench_;
};

}  // namespace cobot_hardware

#endif  // COBOT_HARDWARE__VIRO_HARDWARE_HPP_
