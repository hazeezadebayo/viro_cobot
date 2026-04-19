#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <soem_ethercat_driver/ethercat_driver.h>
// #include <urdf/model.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <string>
#include <memory>
#include <vector>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
// #include <controller_manager/controller_manager.hpp>
#include <std_msgs/msg/float64.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>



using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace soem_hardware_interface
{

typedef struct{
    soem_ethercat_driver::EtherCatModuleClient* client_ptr;
    
    std::string name;
    double count_rad_factor;
    double count_rad_per_s_factor;
    double count_Nm_factor;
    int32_t count_zero;

    double axis_position_factor;
    double axis_torque_factor;

    double position;
    double velocity;
    double effort;

    double position_cmd;
    double velocity_cmd;
    double effort_cmd;
}AxisInfo;

class SoemHWInterface : public hardware_interface::SystemInterface
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(SoemHWInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // humble
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;


private:
 
  soem_ethercat_driver::EtherCatDriver* ethercat_driver_;
  std::vector<AxisInfo> axis_infos_;

  rclcpp::Node::SharedPtr n_;
  
  bool pos_interface_claimed = false;
  bool pos_interface_running = false;
  bool initialized_ = false;

  //record interface
  bool record = false; // double record_time = 5; //in sec
  rclcpp::Time record_start_time;
  std::vector<double> step_data;
  std::vector<std::vector<double>> data_buffer;

  std::string record_file_path ="/home/hazeezadebayo/";

  void start_record_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_record_;

  void stop_record_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_record_;


};
} 
