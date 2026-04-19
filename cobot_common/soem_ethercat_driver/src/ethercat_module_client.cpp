/*
Created on Mon Sep 17 10:02:30 2018

@author: Cong Liu, Burb

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
// author: Cong Liu, Burb

#include <soem_ethercat_driver/ethercat_module_client.h>

namespace soem_ethercat_driver
{

EtherCatModuleClient::State402 EtherCatModuleClient::extract_state(uint16_t statusword)
{
    constexpr uint16_t Mask_6_Bit  {0x6f};
    switch (statusword & Mask_6_Bit) {
        case 0x21: return State402::ReadyToSwitchOn;
        case 0x23: return State402::SwitchedOn;
        case 0x27: return State402::OperationEnabled;
        case 0x07: return State402::QuickStopActive;
    }

    constexpr uint16_t Mask_5_Bit {0x4f};
    switch (statusword & Mask_5_Bit) {
        case 0x00: return State402::NotReadyToSwitchOn;
        case 0x40: return State402::SwitchOnDisabled;
        case 0x0f: return State402::FaultReactionActive;
        case 0x08: return State402::Fault;
    }

    return State402::Unknown;
}


EtherCatModuleClient::EtherCatModuleClient(EtherCatManager *manager, int slave_no)
: manager_(manager), slave_no_(slave_no)
{
    write_output(Controlword, SwitchCommand::DisableVoltage);
    manager_->writeSDO(slave_no_, 0x6060, 0x00, int8_t(0x8)); // CSP Mode
}

int32_t EtherCatModuleClient::get_position_actual()
{
    return read_input(PositionActual);
}

void EtherCatModuleClient::set_target_position(int32_t pos_cnt)
{
    write_output(TargetPosition, pos_cnt);
}

int32_t EtherCatModuleClient::get_target_position()
{
    return read_output(TargetPosition);
}

int32_t EtherCatModuleClient::get_velocity_actual()
{
    return read_input(VelocityActual);
}

int16_t EtherCatModuleClient::get_torque_actual()
{
    return read_input(TorqueActual);
}

void EtherCatModuleClient::set_target_torque(int16_t trq_cnt)
{
    write_output(TargetTorque, trq_cnt);
}

bool EtherCatModuleClient::is_enabled()
{
    uint16_t raw_state = read_input(Statusword);
    return extract_state(raw_state) == State402::OperationEnabled;
}

void* EtherCatModuleClient::enable(void* threadarg)
{
    EtherCatModuleClient *pthis=(EtherCatModuleClient *)threadarg;

    if (pthis->read_input(ErrorCode) == 0x2000) {
        if(pthis->has_fault()) {
            pthis->clear_fault();
        }
    }

    if (pthis->has_fault()) {
        pthis->clear_fault();
    }

    if (pthis->has_fault() || pthis->is_enabled()) {
        std::cout<<"enable in slave "<<pthis->slave_no_<<"failed, the reason might be there is a fault or the motor is enabled"<<std::endl;

        return nullptr;
    }

    // SwitchedOnDisabled -> ReadyToSwitchOn Transition
    pthis->write_output(TargetPosition, pthis->read_input(PositionActual));
    usleep(100000);
    pthis->write_output(Controlword, SwitchCommand::Shutdown);
    pthis->manager_->writeSDO(pthis->slave_no_, 0x6060, 0x00, int8_t(0x8)); // CSP Mode
    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(rclcpp::ok())
    {
        if (extract_state(pthis->read_input(Statusword)) == State402::ReadyToSwitchOn)
        {
            break;
        }
        if (tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
        {
            std::cout<<"Slave"<<pthis->slave_no_<<": SwitchedOnDisabled -> ReadyToSwitchOn transition failed"<<std::endl;
            // ROS_WARN("Statusword: 0x%x", pthis->read_input(Statusword)); TODO:fix log ibr
            return (void *)0;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }


    // ReadyToSwitchOn -> SwitchedOn Transition
    pthis->write_output(Controlword, SwitchCommand::SwitchOn);
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(rclcpp::ok())
    {
        if (extract_state(pthis->read_input(Statusword)) == State402::SwitchedOn) {
            break;
        }
        if (tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9) {
            std::cout<<"Slave"<<pthis->slave_no_<<": ReadyToSwitchOn -> SwitchedOn transition failed"<<std::endl;
            // ROS_WARN("Statusword: 0x%x", pthis->read_input(Statusword)); TODO:fix log ibr
            return (void *)0;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
    std::cout<<"Slave"<<pthis->slave_no_<<": Switched On"<<std::endl;

    // SwitchedOn -> OperationEnabled Transition
    pthis->write_output(Controlword, SwitchCommand::EnableOperation);
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(rclcpp::ok())
    {
        if (extract_state(pthis->read_input(Statusword)) == State402::OperationEnabled) {
            break;
        }
        if (tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9) {
            std::cout<<"Slave"<<pthis->slave_no_<<": SwitchedOn -> OperationEnabled transition failed"<<std::endl;
            // ROS_WARN("Statusword: 0x%x", pthis->read_input(Statusword)); TODO:fix log ibr
            // auto error_code = pthis->read_input(ErrorCode);
            // ROS_ERROR("Error code from 0x603f: 0x%x", error_code); 
            return nullptr;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
    std::cout<<"Slave"<<pthis->slave_no_<<": Operation Enabled"<<std::endl;

    pthis->write_output(Controlword, uint16_t(0x1f));
    usleep(100000);
}

void* EtherCatModuleClient::disable(void *threadarg)
{
    EtherCatModuleClient *pthis = (EtherCatModuleClient *)threadarg;
    if (pthis->is_enabled()) {
        pthis->write_output(Controlword, SwitchCommand::Shutdown);
    }
    return nullptr;
}

bool EtherCatModuleClient::has_fault()
{
    return (read_input(Statusword) & 0x08) == 0x08;
}

void EtherCatModuleClient::clear_fault()
{
    write_output(Controlword, uint16_t(SwitchCommand::Shutdown + SwitchCommand::FaultReset));
    usleep(20000);
    write_output(Controlword, SwitchCommand::Shutdown);
    usleep(20000);
}

bool EtherCatModuleClient::in_position_mode()
{
    return is_enabled() && (read_input(ModeDisplay) == 0x8);
}

bool EtherCatModuleClient::in_torque_mode()
{
    return is_enabled() && (read_input(ModeDisplay)) == 0xa;
}

void EtherCatModuleClient::switch_to_position_mode()
{
    manager_->writeSDO(slave_no_, 0x6060, 0x00, int8_t(0x8));
}

void EtherCatModuleClient::switch_to_torque_mode()
{
     manager_->writeSDO(slave_no_, 0x6060, 0x00, int8_t(0xa));
}
} // namespace soem_ethercat_driver

