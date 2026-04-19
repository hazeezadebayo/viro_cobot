#include <soem_ros_control/soem_hw_interface.h>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <cmath>
#include <exception>
#include <hardware_interface/handle.hpp>

//Migration from Foxy to newer versions
//https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html#migration-from-foxy-to-newer-versions

namespace soem_hardware_interface
{

  CallbackReturn SoemHWInterface::on_init(const hardware_interface::HardwareInfo & info)
  {

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    n_ = std::make_shared<rclcpp::Node>("soem_hw", rclcpp::NodeOptions().context(rclcpp::contexts::get_global_default_context()));

    std::string ethernet_name;
    n_->declare_parameter("soem_ethernet_name", "eth0");
    n_->get_parameter("soem_ethernet_name", ethernet_name);

    //record
    start_record_ = n_->create_service<std_srvs::srv::Trigger>("start_soem_hw_record", std::bind(&SoemHWInterface::start_record_cb, this, std::placeholders::_1, std::placeholders::_2));
    stop_record_ = n_->create_service<std_srvs::srv::Trigger>("stop_soem_hw_record", std::bind(&SoemHWInterface::stop_record_cb, this, std::placeholders::_1, std::placeholders::_2));

    try {
      ethercat_driver_ = new soem_ethercat_driver::EtherCatDriver(ethernet_name, n_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(n_->get_logger(), "Failed to initialize EtherCAT driver: %s", e.what());
      return CallbackReturn::FAILURE;
    }

    axis_infos_.clear();
    for (size_t j = 0; j < ethercat_driver_->getEtherCATModuleClientNumber(); j++)
    {
      AxisInfo axis_info_tmp;
      axis_info_tmp.client_ptr = ethercat_driver_->getEtherCATModuleClientPtr(j);
      axis_info_tmp.name = ethercat_driver_->getJointName(j);
      axis_info_tmp.axis_position_factor = ethercat_driver_->getAxisPositionFactor(j);
      axis_info_tmp.count_zero = ethercat_driver_->getCountZero(j);
      axis_info_tmp.axis_torque_factor = ethercat_driver_->getAxisTorqueFactor(j);

      axis_info_tmp.count_rad_factor = axis_info_tmp.axis_position_factor / (2 * M_PI);
      axis_info_tmp.count_Nm_factor = axis_info_tmp.axis_torque_factor / 1000.0;
      axis_infos_.push_back(axis_info_tmp);
    }

    for (size_t i = 0; i < axis_infos_.size(); i++)
    {
      int32_t pos_count = axis_infos_[i].client_ptr->get_position_actual();

      double position_tmp = (pos_count - axis_infos_[i].count_zero) / axis_infos_[i].count_rad_factor;
      if (position_tmp >= M_PI)
      {
        axis_infos_[i].count_zero += axis_infos_[i].count_rad_factor * 2 * M_PI;
      }
      else if (position_tmp < -1 * M_PI)
      {
        axis_infos_[i].count_zero -= axis_infos_[i].count_rad_factor * 2 * M_PI;
      }
      axis_infos_[i].position = -1 * (pos_count - axis_infos_[i].count_zero) / axis_infos_[i].count_rad_factor;
    }

    return CallbackReturn::SUCCESS;
  }




  std::vector<hardware_interface::StateInterface> SoemHWInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < axis_infos_.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(axis_infos_[i].name, hardware_interface::HW_IF_POSITION, &axis_infos_[i].position));

      state_interfaces.emplace_back(hardware_interface::StateInterface(axis_infos_[i].name, hardware_interface::HW_IF_VELOCITY, &axis_infos_[i].velocity));

      state_interfaces.emplace_back(hardware_interface::StateInterface(axis_infos_[i].name, hardware_interface::HW_IF_EFFORT, &axis_infos_[i].effort));
    }
    return state_interfaces;
  }




  std::vector<hardware_interface::CommandInterface> SoemHWInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(axis_infos_.size());
    for (unsigned int i = 0; i < axis_infos_.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(axis_infos_[i].name, hardware_interface::HW_IF_POSITION, &axis_infos_[i].position_cmd));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(axis_infos_[i].name, hardware_interface::HW_IF_VELOCITY, &axis_infos_[i].velocity_cmd));
    }

    return command_interfaces;
  }




  hardware_interface::return_type SoemHWInterface::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
  {
    auto cur_interface = [](const std::string &interface)
    {
      return interface.find(hardware_interface::HW_IF_POSITION) != std::string::npos;
    };
    int64_t num_stop_cur_interfaces =
        std::count_if(stop_interfaces.begin(), stop_interfaces.end(), cur_interface);
    if (num_stop_cur_interfaces == 6)
    {
      pos_interface_claimed = false;
    }
    else if (num_stop_cur_interfaces != 0)
    {
      RCLCPP_FATAL(n_->get_logger(), "Expected %d pos interfaces to stop, but got %ld instead.", 6, num_stop_cur_interfaces);
      // RCLCPP_FATAL(n_->get_logger(), "Expected %d pos interfaces to stop, but god %d instead.", 6, num_stop_cur_interfaces);
      std::string error_string = "Invalid number of pos interfaces to stop,Expected";
      error_string += std::to_string(6);
      throw std::invalid_argument(error_string);
    }

    int64_t num_start_cur_interfaces =
        std::count_if(start_interfaces.begin(), start_interfaces.end(), cur_interface);
    if (num_start_cur_interfaces == 6)
    {
      pos_interface_claimed = true;
    }
    else if (num_start_cur_interfaces != 0)
    {
      RCLCPP_FATAL(n_->get_logger(), "Expected %d pos interfaces to start, but got %ld instead.", 6, num_start_cur_interfaces);
      // RCLCPP_FATAL(n_->get_logger(), "Expected %d pos interfaces to start, but god %d instead.", 6, num_start_cur_interfaces);
      std::string error_string = "Invalid manmagerf of pos interfaces top stop. Execeptd";
      error_string += std::to_string(6);
      throw std::invalid_argument(error_string);
    }
    return hardware_interface::return_type::OK;
  }





  hardware_interface::return_type SoemHWInterface::perform_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &)
  {
    if (pos_interface_claimed && !pos_interface_running)
    {
      pos_interface_running = true;
    }
    else if (pos_interface_running && !pos_interface_claimed)
    {
      pos_interface_running = false;
    }
    return hardware_interface::return_type::OK;
  }





  CallbackReturn SoemHWInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    // initialize commands
    // command and state should be equal when starting
    for (size_t i = 0; i < axis_infos_.size(); i++)
    {
      axis_infos_[i].position_cmd = axis_infos_[i].position;
    }

    RCLCPP_INFO(n_->get_logger(), "Started");
    return CallbackReturn::SUCCESS;
  }





  CallbackReturn SoemHWInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(n_->get_logger(), "trying to Stop");

    if (ethercat_driver_ != NULL)
    {
      delete ethercat_driver_;
    }

    RCLCPP_INFO(n_->get_logger(), "Stopped");
    return CallbackReturn::SUCCESS;
  }








  // humble:
  hardware_interface::return_type SoemHWInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    rclcpp::spin_some(n_);
    for (size_t i = 0; i < axis_infos_.size(); i++)
    {
      int32_t pos_count = axis_infos_[i].client_ptr->get_position_actual() - axis_infos_[i].count_zero;
      int32_t vel_count = axis_infos_[i].client_ptr->get_velocity_actual();
      int16_t trq_count = axis_infos_[i].client_ptr->get_torque_actual();

      axis_infos_[i].position = -1 * pos_count / axis_infos_[i].count_rad_factor;
      axis_infos_[i].velocity = -1 * vel_count / axis_infos_[i].count_rad_factor;
      axis_infos_[i].effort = -1 * trq_count / axis_infos_[i].count_Nm_factor / 1000.0;
    }

    return hardware_interface::return_type::OK;;
  }


  // humble:
  hardware_interface::return_type SoemHWInterface::write(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
  {

    for (size_t i = 0; i < axis_infos_.size(); i++)
    {
      if (!axis_infos_[i].client_ptr->in_position_mode())
      {
        axis_infos_[i].position_cmd = axis_infos_[i].position;
      }
      double position_cmd_count = -1 * axis_infos_[i].position_cmd * axis_infos_[i].count_rad_factor + axis_infos_[i].count_zero;
      axis_infos_[i].client_ptr->set_target_position(int32_t(position_cmd_count));
    }

    if (record == true)
    {
      rclcpp::Duration period = time - record_start_time;
      step_data.clear();
      step_data.push_back(period.nanoseconds()/1'000'000.0); //ms
      step_data.push_back(period.nanoseconds());//ns
      for (size_t i = 0; i < axis_infos_.size(); i++)
      {
        step_data.push_back(axis_infos_[i].position_cmd);
      }
      data_buffer.push_back(step_data);
    }

    return hardware_interface::return_type::OK;
  }








  void SoemHWInterface::start_record_cb(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, 
      const std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
      record = true;
      record_start_time = n_->get_clock()->now();
      data_buffer.clear();

      resp->success = true;
      resp->message = "Record started.";
      std::cout << "Record started." << std::endl;
  }


  void SoemHWInterface::stop_record_cb(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, 
      const std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
      record = false;
      std::ofstream file_csv;
      file_csv.open(record_file_path + "soem_hw_record.csv");
      //Header
      file_csv << "Index,Time[ms],Period[ns],joint_1[rad],joint_2[rad],joint_3[rad],joint_4[rad],joint_5[rad],joint_6[rad]"<< std::endl;
      //Data
      for (size_t i = 0; i < data_buffer.size(); i++)
      {
          file_csv << i+1 << ","<< std::fixed <<std::setprecision(6) << data_buffer[i][0];
          for (size_t j = 1; j < data_buffer[i].size(); j++)
              {
                  file_csv << "," << data_buffer[i][j];
              }
          file_csv << std::endl;
      }
      file_csv.close();
      data_buffer.clear();
      resp->success = true;
      resp->message = "Record stopped.";
      std::cout << "Record stopped." << std::endl;
  }







}





#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(soem_hardware_interface::SoemHWInterface, hardware_interface::SystemInterface)