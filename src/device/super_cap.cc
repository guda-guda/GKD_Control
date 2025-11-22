#include "device/super_cap.hpp"

#include "macro_helpers.hpp"
#include "power_controller.hpp"

namespace Device
{
    void Super_Cap::init(
        const std::string& can_name,
        const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;
        can = IO::io<CAN>[can_name];
        can->register_callback_key(
            0x51, std::bind(&Super_Cap::unpack, this, std::placeholders::_1));
    }

    void Super_Cap::unpack(const can_frame& frame) {
        if (frame.can_id != 0x051) {
        return;
        }
        static int delta = 0;
        delta++;
        
        //取消等级机制，使用固定功率限制
        // 注意：步兵有两种底盘类型(功率优先:90W, 血量优先:75W)
        uint16_t power_limit = MUXDEF(
            CONFIG_HERO,
            Power::HeroChassisPowerLimit * 0.9,      // 英雄固定100W * 0.9 = 90W
            MUXDEF(
                CONFIG_INFANTRY,
                // 这里暂时使用血量优先模式
                Power::InfantryChassisPowerLimit_HPFirst * 0.9,  // 血量优先: 75W * 0.9 = 67.5W
                //Power::InfantryChassisPowerLimit_PowerFirst * 0.9,  // 功率优先: 90W * 0.9 = 81W
                Power::SentryChassisPowerLimit * 0.9));  // 哨兵固定100W * 0.9 = 90W

        if (delta >= 500) {
            //set(true, power_limit);
            delta = 0;
        }

        std::memcpy(&robot_set->super_cap_info, frame.data, 8);
        
        /*LOG_INFO(
            "\n----------------Original-8-bytes-------------\n"
            "SC RX raw: id=0x%03X [%d]\nframe.data:[0-3]:%02X %02X %02X %02X\nframe.data:[4-7]:%02X %02X %02X %02X\n"
            "--------------------\n",
             frame.can_id,
             frame.can_dlc,
             frame.data[0], frame.data[1], frame.data[2], frame.data[3],
             frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

        // 2) 你现在的解读结果
        LOG_INFO(
            "\n-----------------Parsed-Data------------------\n"
            "SC parsed: errCode=%u  chassisPower=%f \n chassisPowerlimit(raw)=%u  capEnergy=%u\n"
            "---------------------\n",
             robot_set->super_cap_info.errorCode,
             robot_set->super_cap_info.chassisPower,
             (unsigned)robot_set->super_cap_info.chassisPowerlimit,
             (unsigned)robot_set->super_cap_info.capEnergy);

        // 3) 手动再解一次 limit（小端/大端两种），方便对照
        uint16_t limit_le = (uint16_t)frame.data[5] | (uint16_t(frame.data[6]) << 8);
        uint16_t limit_be = (uint16_t)frame.data[6] | (uint16_t(frame.data[5]) << 8);

        LOG_INFO(
            "\n------------------Manual-Decode-------------------\n"
            "SC limit decode: LE=%u  BE=%u\n"
            "----------------------\n", 
            (unsigned)limit_le, (unsigned)limit_be);*/

        // LOG_INFO(
        //     "errorCode %d\tchassisPower %f\tchassisPowerlimit %d\t %d power limit %d\n",
        //     robot_set->super_cap_info.errorCode,
        //     robot_set->super_cap_info.chassisPower,
        //     (int)robot_set->super_cap_info.chassisPowerlimit,
        //     (int)robot_set->super_cap_info.capEnergy,
        //     power_limit);
    }

    void Super_Cap::set(bool enable, uint16_t power_limit) {
        can_frame send{};
        uint16_t chassis_power_buffer =
            robot_set->referee_info.power_heat_data.chassis_power_buffer;
        send.can_id = 0x061;
        send.can_dlc = 8;
        if (enable)
            send.data[0] = 0x01;
        send.data[1] = power_limit & 0xff;
        send.data[2] = power_limit >> 8;
        send.data[3] = 50 & 0xff;
        send.data[4] = 50 >> 8;

        can->send(send);
    }
}  // namespace Device