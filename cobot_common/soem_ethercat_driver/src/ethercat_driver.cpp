/*
Created on Mon Sep 17 10:31:06 2018

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

#include <soem_ethercat_driver/ethercat_driver.h>

namespace soem_ethercat_driver
{

    EtherCatDriver::EtherCatDriver(const std::string &ifname, const rclcpp::Node::SharedPtr &node)
        : ed_nh_(node)
    {
        // Initialize slave_no_
        ed_nh_->declare_parameter("slave_no", std::vector<int64_t>()); // BUG
        ed_nh_->get_parameter("slave_no", slave_no_);

        if (slave_no_.size() == 0)
        {
            RCLCPP_ERROR(ed_nh_->get_logger(), "slave_no is empty please check the parameters");
            exit(0);
        }

        // Initialize joint_names_
        ed_nh_->declare_parameter("joint_names", std::vector<std::string>({"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}));
        ed_nh_->get_parameter<std::vector<std::string>>("joint_names", joint_names_);

        if (joint_names_.size() != slave_no_.size())
        {
            RCLCPP_ERROR(ed_nh_->get_logger(), "the number of joint names is %lu, it should be %lu", joint_names_.size(), slave_no_.size());
            exit(0);
        }

        // Initialize axis_position_factors_
        ed_nh_->declare_parameter("axis_position_factors", std::vector<double>());
        ed_nh_->get_parameter<std::vector<double>>("axis_position_factors", axis_position_factors_);

        if (axis_position_factors_.size() != slave_no_.size())
        {
            RCLCPP_ERROR(ed_nh_->get_logger(), "the number of axis position factors is %lu, it should be %lu", axis_position_factors_.size(), slave_no_.size());
            exit(0);
        }

        // Check the values of axis position factors
        for (size_t i = 0; i < axis_position_factors_.size(); i++)
        {
            if (axis_position_factors_[i] < 1)
            {
                RCLCPP_ERROR(ed_nh_->get_logger(), "axis_position_factors[%lu] is too small", i);
                exit(0);
            }
        }

        // Initialize axis_torque_factors_
        ed_nh_->declare_parameter("axis_torque_factors", std::vector<double>());
        ed_nh_->get_parameter("axis_torque_factors", axis_torque_factors_);

        if (axis_torque_factors_.size() != slave_no_.size())
        {
            RCLCPP_ERROR(ed_nh_->get_logger(), "the number of axis torque factors is %lu, it should be %lu", axis_torque_factors_.size(), slave_no_.size());
            exit(0);
        }

        // Check the values of axis torque factors
        for (size_t i = 0; i < axis_torque_factors_.size(); i++)
        {
            if (axis_torque_factors_[i] < 1)
            {
                RCLCPP_ERROR(ed_nh_->get_logger(), "axis_torque_factors[%lu] is too small", i);
                exit(0);
            }
        }

        ed_nh_->declare_parameter("count_zeros", std::vector<int64_t>()); // BUG
        ed_nh_->get_parameter("count_zeros", count_zeros_);

        if (count_zeros_.size() != slave_no_.size())
        {
            RCLCPP_ERROR(ed_nh_->get_logger(), "the number of count_zeros is %lu, it should be %lu", count_zeros_.size(), slave_no_.size());
            exit(0);
        }

        // Initialize count_rad_factors
        count_rad_factors_.resize(count_zeros_.size());
        for (size_t i = 0; i < count_rad_factors_.size(); i++)
        {
            count_rad_factors_[i] = axis_position_factors_[i] / (2 * M_PI);
        }

        // Initialize manager_
        manager_ = new soem_ethercat_driver::EtherCatManager(ifname);

        // Initialize ethercat_client_
        ethercat_clients_.clear();
        ethercat_clients_.resize(slave_no_.size());
        for (size_t i = 0; i < slave_no_.size(); i++)
        {
            ethercat_clients_[i] = new EtherCatModuleClient(manager_, slave_no_[i]);
        }

        // Initialize ROS service server
        enable_robot_ = ed_nh_->create_service<std_srvs::srv::Trigger>("/enable_robot", std::bind(&EtherCatDriver::enableRobot_cb, this, std::placeholders::_1, std::placeholders::_2));
        disable_robot_ = ed_nh_->create_service<std_srvs::srv::Trigger>("/disable_robot", std::bind(&EtherCatDriver::disableRobot_cb, this, std::placeholders::_1, std::placeholders::_2));
        clear_fault_ = ed_nh_->create_service<std_srvs::srv::Trigger>("/clear_fault", std::bind(&EtherCatDriver::clearFault_cb, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize ROS publisher
        enable_state_pub_ = ed_nh_->create_publisher<std_msgs::msg::Bool>("enable_state", 1);
        fault_state_pub_ = ed_nh_->create_publisher<std_msgs::msg::Bool>("fault_state", 1);

        // Initialize ROS timer
        auto timeout = std::chrono::duration<double, std::milli>(100);
        status_timer_ = ed_nh_->create_wall_timer(timeout, std::bind(&EtherCatDriver::updateStatus, this));
        rclcpp::spin_some(ed_nh_);
    }

    EtherCatDriver::~EtherCatDriver()
    {
        for (size_t i = 0; i < ethercat_clients_.size(); i++)
        {
            if (ethercat_clients_[i] != NULL)
            {
                delete ethercat_clients_[i];
            }
        }

        if (manager_ != NULL)
        {
            delete manager_;
        }
    }

    // true: enabled; false: disabled
    bool EtherCatDriver::getEnableState()
    {
        bool enable_flag_tmp = true;
        for (size_t i = 0; i < ethercat_clients_.size(); i++)
        {
            enable_flag_tmp = enable_flag_tmp && ethercat_clients_[i]->is_enabled();
        }
        return enable_flag_tmp;
    }

    // true: there is a fault; false: there is no fault
    bool EtherCatDriver::getFaultState()
    {
        bool fault_flag_tmp = false;
        for (size_t i = 0; i < ethercat_clients_.size(); i++)
        {
            fault_flag_tmp = fault_flag_tmp || ethercat_clients_[i]->has_fault();
        }
        return fault_flag_tmp;
    }

    void EtherCatDriver::updateStatus()
    {
        enable_state_msg_.data = getEnableState();
        fault_state_msg_.data = getFaultState();

        enable_state_pub_->publish(enable_state_msg_);
        fault_state_pub_->publish(fault_state_msg_);
    }

    size_t EtherCatDriver::getEtherCATModuleClientNumber()
    {
        return ethercat_clients_.size();
    }

    EtherCatModuleClient *EtherCatDriver::getEtherCATModuleClientPtr(size_t n)
    {
        return ethercat_clients_[n];
    }

    std::string EtherCatDriver::getJointName(size_t n)
    {
        return joint_names_[n];
    }

    double EtherCatDriver::getAxisPositionFactor(size_t n)
    {
        return axis_position_factors_[n];
    }

    double EtherCatDriver::getAxisTorqueFactor(size_t n)
    {
        return axis_torque_factors_[n];
    }

    int32_t EtherCatDriver::getCountZero(size_t n)
    {
        return count_zeros_[n];
    }


    void EtherCatDriver::enableRobot_cb(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, 
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (getFaultState())
        {
            response->success = false;
            response->message = "Please clear fault first";
            return;
        }

        std::vector<pthread_t> tids;
        tids.resize(ethercat_clients_.size());

        std::vector<int> threads;
        threads.resize(ethercat_clients_.size());

        for (size_t i = 0; i < ethercat_clients_.size(); i++)
        {
            threads[i] = pthread_create(&tids[i], NULL, ethercat_clients_[i]->enable, (void *)ethercat_clients_[i]);
            usleep(300'000); // Don't release all the brakes at the same time
        }

        for (size_t i = 0; i < ethercat_clients_.size(); i++)
        {
            pthread_join(tids[i], NULL);
        }

        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while (rclcpp::ok())
        {
            if (getEnableState())
            {
                response->success = true;
                response->message = "Robot is enabled";
                return;
            }
            if (tick.tv_sec * 1e+9 + tick.tv_nsec - before.tv_sec * 1e+9 - before.tv_nsec >= 6e+9)
            {
                response->success = false;
                response->message = "Robot is not enabled";
                return;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        return;
    }



    void EtherCatDriver::disableRobot_cb(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, 
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::vector<pthread_t> tids;
        tids.resize(ethercat_clients_.size());

        std::vector<int> threads;
        threads.resize(ethercat_clients_.size());

        for (size_t i = 0; i < ethercat_clients_.size(); i++)
        {
            threads[i] = pthread_create(&tids[i], NULL, ethercat_clients_[i]->disable, (void *)ethercat_clients_[i]);
        }

        for (size_t i = 0; i < ethercat_clients_.size(); i++)
        {
            pthread_join(tids[i], NULL);
        }

        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while (rclcpp::ok())
        {
            if (!getEnableState())
            {
                response->success = true;
                response->message = "Robot is disabled";
                return;
            }
            if (tick.tv_sec * 1e+9 + tick.tv_nsec - before.tv_sec * 1e+9 - before.tv_nsec >= 6e+9)
            {
                response->success = false;
                response->message = "Robot is not disabled";
                return;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        return;
    }


    void EtherCatDriver::clearFault_cb(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, 
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        for (size_t i = 0; i < ethercat_clients_.size(); i++)
        {
            ethercat_clients_[i]->clear_fault();
        }

        struct timespec before, tick;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_gettime(CLOCK_REALTIME, &tick);
        while (rclcpp::ok())
        {
            if (!getFaultState())
            {
                response->success = true;
                response->message = "Faults are cleared";
                return;
            }
            if (tick.tv_sec * 1e+9 + tick.tv_nsec - before.tv_sec * 1e+9 - before.tv_nsec >= 6e+9)
            {
                response->success = false;
                response->message = "There are still Faults";
                return;
            }
            usleep(100000);
            clock_gettime(CLOCK_REALTIME, &tick);
        }
        return;
    }

} // end namespace
