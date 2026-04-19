/*
Created on Mon Sep 17 10:22:24 2018

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#ifndef SOEM_ETHERCAT_DRIVER_H
#define SOEM_ETHERCAT_DRIVER_H

#include <soem_ethercat_driver/ethercat_module_client.h>
#include <std_srvs/srv/trigger.hpp>

namespace soem_ethercat_driver
{

  class EtherCatDriver
  {
  public:
    EtherCatDriver(const std::string &ifname, const rclcpp::Node::SharedPtr& node);
    ~EtherCatDriver();

    std::shared_ptr<rclcpp::Node> root_nh_, ed_nh_;

    bool getEnableState();
    bool getFaultState();
    void updateStatus();

    size_t getEtherCATModuleClientNumber();
    EtherCatModuleClient *getEtherCATModuleClientPtr(size_t n);
    std::string getJointName(size_t n);
    double getAxisPositionFactor(size_t n);
    double getAxisTorqueFactor(size_t n);
    int32_t getCountZero(size_t n);

    void enableRobot_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_robot_;
  
    void disableRobot_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_robot_;

    void clearFault_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_;
    
  private:
    std::vector<EtherCatModuleClient *> ethercat_clients_;
    std::vector<int64_t> slave_no_;
    std::vector<std::string> joint_names_;
    std::vector<double> axis_position_factors_;
    std::vector<double> axis_torque_factors_;
    std::vector<int64_t> count_zeros_;

    EtherCatManager *manager_;

    std_msgs::msg::Bool enable_state_msg_;
    std_msgs::msg::Bool fault_state_msg_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_state_pub_;

    rclcpp::TimerBase::SharedPtr status_timer_;

    std::vector<double> count_rad_factors_;
  };

}

#endif
