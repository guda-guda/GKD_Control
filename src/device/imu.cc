#include "device/imu.hpp"

#include "hardware.hpp"
#include "iostream"

namespace Device
{
    void IMU::unpack(const Types::ReceivePacket &pkg) {
        robot_set->ins_yaw = UserLib::rad_format(pkg.yaw * (M_PIf / 180));
        robot_set->ins_pitch = UserLib::rad_format(pkg.pitch * (M_PIf / 180));
        robot_set->ins_roll = UserLib::rad_format(pkg.roll * (M_PIf / 180));
        robot_set->ins_yaw_v = (pkg.yaw_v * (M_PIf / 180)) / 1000;
        robot_set->ins_pitch_v = (pkg.pitch_v * (M_PIf / 180)) / 1000;
        robot_set->ins_roll_v = (pkg.roll_v * (M_PIf / 180)) / 1000;
        update_time();
    }

    void IMU::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        Robot::hardware->register_callback<SER1>([&](const Types::ReceivePacket &rp) {
            // LOG_INFO(
            //     "%f, %f, %f %f %f %f\n",
            //     rp.yaw,
            //     rp.pitch,
            //     rp.roll,
            //     (rp.yaw_v * (M_PIf / 180)) / 1000,
            //     (rp.pitch_v * (M_PIf / 180)) / 1000,
            //     (rp.roll_v * (M_PIf / 180)) / 1000);
            unpack(rp);

            Robot::SendVisionControl svp;
            svp.header = 0xA6;
            svp.yaw = robot_set->ins_yaw;
            svp.pitch = robot_set->ins_pitch;
            svp.roll = robot_set->ins_roll;
            Robot::hardware->send<SOCKET>(svp);
        });
    }
}  // namespace Device
