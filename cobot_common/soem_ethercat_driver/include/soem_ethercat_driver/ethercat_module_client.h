/*
Created on Mon Sep 17 09:42:14 2018

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

#ifndef ELFIN2_ETHERCAT_MODULE_CLIENT_H_H
#define ELFIN2_ETHERCAT_MODULE_CLIENT_H_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vector>
#include <soem_ethercat_driver/ethercat_manager.h>

#include <pthread.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <chrono>

namespace soem_ethercat_driver {

class EtherCatModuleClient
{
private:

    enum class State402 {
      Unknown, NotReadyToSwitchOn, SwitchOnDisabled, ReadyToSwitchOn,
      SwitchedOn, OperationEnabled, QuickStopActive, FaultReactionActive, Fault,
    };

    struct SwitchCommand {
      static constexpr uint16_t Shutdown = 0x6;
      static constexpr uint16_t SwitchOn = 0x7;
      static constexpr uint16_t DisableVoltage = 0x0;
      static constexpr uint16_t QuickStop = 0x2;
      static constexpr uint16_t DisableOperation = 0x7;
      static constexpr uint16_t EnableOperation = 0xf;
      static constexpr uint16_t FaultReset = 0x80;
    };

    template <typename T>
    struct PdoMember {
      size_t offset;
    };

    static State402 extract_state(uint16_t statusword);

    EtherCatManager* manager_;
    int slave_no_;
  
    // List of PDO Mappings. Types and byte offsets must match with
    // EtherCatManager::zeroerr_pdo_config_hook() in elfin_ethercat_manager.cpp

    inline static const PdoMember<int32_t> PositionActual {0};
    inline static const PdoMember<int32_t> DigitalInputs {4};
    inline static const PdoMember<uint16_t> Statusword {8};
    inline static const PdoMember<int16_t> TorqueActual {10};
    inline static const PdoMember<int32_t> VelocityActual {12};
    inline static const PdoMember<uint16_t> ErrorCode {16};
    inline static const PdoMember<uint8_t> ModeDisplay {18};

    inline static const PdoMember<int32_t> TargetPosition {0};
    inline static const PdoMember<int32_t> DigitalOutputs {4};
    inline static const PdoMember<int16_t> TargetTorque {8};
    inline static const PdoMember<int16_t> TorqueOffset {10};
    inline static const PdoMember<uint16_t> Controlword {12};


public:

    EtherCatModuleClient(EtherCatManager* manager, int slave_no);
    ~EtherCatModuleClient() = default;

    template <typename T>
    T read_input(PdoMember<T> member)
    {
      T result;
      manager_->read_input(slave_no_, member.offset, &result, sizeof(T));
      return result;
    }

    template <typename T>
    T read_output(PdoMember<T> member)
    {
      T result;
      manager_->read_output(slave_no_, member.offset, &result, sizeof(T));
      return result;
    }

    template <typename T>
    void write_output(PdoMember<T> member, T value)
    {
      manager_->write_output(slave_no_, member.offset, &value, sizeof(T));
    }

    // Process Image Setters & Getters
    int32_t get_position_actual();
    void set_target_position(int32_t pos_cnt);
    int32_t get_target_position();
    int32_t get_velocity_actual();
    int16_t get_torque_actual();
    void set_target_torque(int16_t trq_cnt);

    bool is_enabled();
    static void* enable(void *threadarg);
    static void* disable(void *threadarg);
    bool has_fault();
    void clear_fault();
    bool in_position_mode();
    bool in_torque_mode();
    void switch_to_position_mode();
    void switch_to_torque_mode();
};

} // namespace soem_ethercat_driver

#endif
