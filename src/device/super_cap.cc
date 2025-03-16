#include "device/super_cap.hpp"

namespace Device
{
    void Super_Cap::init(const std::string& can_name, const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;
        can = IO::io<CAN>[can_name];
        can->register_callback_key(0x51, std::bind(&Super_Cap::unpack, this, std::placeholders::_1));
    }

    void Super_Cap::unpack(const can_frame& frame) {
        Types::ReceivePacket_Super_Cap info;
        std::memcpy(&info, frame.data, 8);
        LOG_INFO(
            "errorCode %d\tchassisPower %f\tchassisPowerlimit %d\tcapEnergy %d\n",
            info.errorCode,
            info.chassisPower,
            (int)info.chassisPowerlimit,
            (int)info.capEnergy);
    }

    void Super_Cap::set(bool enable, uint16_t power_limit) {
        can_frame send{};
        if (enable)
            send.data[0] = 0x80;
        send.data[1] = power_limit >> 8;
        send.data[2] = power_limit & 0xff;

        can->send(send);
    }
}  // namespace Device
