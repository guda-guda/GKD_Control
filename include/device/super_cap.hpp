#pragma once
#include "can.hpp"
#include "deviece_base.hpp"
#include "robot.hpp"

namespace Device
{
    class Super_Cap : Device::DeviceBase
    {
       private:
        IO::Can_interface* can;
        Types::ReceivePacket_Super_Cap supercap_info;

       public:
        void init(const std::string& can_name, const std::shared_ptr<Robot::Robot_set>& robot);

        void unpack(const can_frame& frame);
        void set(bool enable, uint16_t power_limit);
    };
}  // namespace Device
