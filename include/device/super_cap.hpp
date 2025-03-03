#pragma once
#include "can.hpp"
#include "deviece_base.hpp"

namespace Device 
{
    class Super_Cap : Device::DeviceBase
    {
        private:
        IO::Can_interface* can;
        public:
        void init(const std::string& can_name);
        
        void unpack(const can_frame& frame);
        void set(bool enable, uint16_t power_limit);
    };
}