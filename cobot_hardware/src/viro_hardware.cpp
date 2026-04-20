#include "cobot_hardware/viro_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cobot_hardware
{
hardware_interface::CallbackReturn ViroHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Proper memory allocation for 6 joints
  hw_states_.resize(info_.joints.size(), {0.0, 0.0, 0.0, 0.0});
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_wrench_ = {0,0,0,0,0,0};

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Verification of standard interfaces
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("ViroHardware"), "Joint '%s' has %zu command interfaces. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("ViroHardware"), "Joint '%s' command interface is '%s'. '%s' expected.", joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ViroHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ViroHardware"), "Configuring Viro Hardware Interface...");

  std::string port = info_.hardware_parameters.count("port") ? info_.hardware_parameters.at("port") : "/tmp/ttyViro";
  uint32_t baud = info_.hardware_parameters.count("baud") ? std::stoi(info_.hardware_parameters.at("baud")) : 115200;

  if (!comm_.open(port, baud))
  {
    RCLCPP_FATAL(rclcpp::get_logger("ViroHardware"), "Failed to open serial port: %s", port.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ViroHardware"), "Serial port opened successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ViroHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i].pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[i].vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_[i].torque));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "acceleration", &hw_states_[i].accel));
  }

  // Export FT Sensor Interfaces
  for (const auto& sensor : info_.sensors) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "force.x", &hw_wrench_.force_x));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "force.y", &hw_wrench_.force_y));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "force.z", &hw_wrench_.force_z));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "torque.x", &hw_wrench_.torque_x));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "torque.y", &hw_wrench_.torque_y));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "torque.z", &hw_wrench_.torque_z));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ViroHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ViroHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ViroHardware"), "Activating... Performing Handshake.");
  if (!comm_.ping())
  {
    RCLCPP_ERROR(rclcpp::get_logger("ViroHardware"), "Handshake FAILED. Hardware not responding.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Initialize command buffer with current position to prevent jumps
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  for (size_t i = 0; i < hw_states_.size(); i++) {
    hw_commands_[i] = hw_states_[i].pos;
  }

  RCLCPP_INFO(rclcpp::get_logger("ViroHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ViroHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ViroHardware"), "Deactivating... Disabling torque.");
  // Optional: send explicit disable command to motors
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ViroHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comm_.read_hardware(hw_states_, hw_wrench_))
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ViroHardware"), *rclcpp::Clock::make_shared(), 1000, "Failed to read joint data!");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ViroHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Secondary safety check: ensure commands are not NaN or garbage
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    if (std::isnan(hw_commands_[i])) {
      hw_commands_[i] = hw_states_[i].pos; // Hold last position
    }
  }

  if (!comm_.write_joints(hw_commands_, 0)) // 0: Position Mode
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ViroHardware"), *rclcpp::Clock::make_shared(), 1000, "Failed to write joint commands!");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace cobot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cobot_hardware::ViroHardware, hardware_interface::SystemInterface)
