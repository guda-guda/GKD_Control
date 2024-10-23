#include "device/cv_controller.hpp"

#include "hardware.hpp"
#include "utils.hpp"

namespace Device
{
    void Cv_controller::init(const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;
        Robot::hardware->register_callback<SOCKET, Robot::ReceiveGimbalPacket>(
            [&](const Robot::ReceiveGimbalPacket& pkg) { unpack(pkg); });
    }

    [[noreturn]] void Cv_controller::task() {
        while (true) {
            Robot::SendGimbalPacket sgp;
            sgp.detect_color = 1;
            sgp.reserved = 0;
            sgp.reset_tracker = false;
            sgp.header = 0x5A;
            sgp.yaw = 0.f;
            sgp.pitch = robot_set->gyro1_ins_pitch;
            sgp.roll = robot_set->gyro1_ins_roll;
            sgp.aim_x = robot_set->aimx;
            sgp.aim_y = robot_set->aimy;
            sgp.aim_z = robot_set->aimz;
            Robot::hardware->send<SOCKET>(sgp);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void Cv_controller::unpack(const Robot::ReceiveGimbalPacket& pkg) {
        LOG_INFO("cv unpacking %d %d\n", pkg.sd_yaw, pkg.sd_pitch);
        float y = -(float)pkg.sd_yaw / 32767, p = -(float)pkg.sd_pitch / 62767;
        if (y < 1.5 && y > -1.5) {
            robot_set->gimbal1_yaw_set = y;
            robot_set->gimbal2_yaw_set = y;
        }
        if (p < 0.15 && p > -0.47) {
            robot_set->gimbal1_pitch_set = p;
            robot_set->gimbal2_pitch_set = p;
        }
        // if (pkg.x != 0) {
        //     auto solver_successful = bullet_solver_.solve(
        //         { pkg.x, pkg.y, pkg.z },
        //         { pkg.vx, pkg.vy, pkg.vz },
        //         17,
        //         pkg.yaw,
        //         pkg.v_yaw,
        //         pkg.r1,
        //         pkg.r2,
        //         pkg.dz,
        //         pkg.armors_num);
        //
        //     if (solver_successful) {
        //         robot_set->gimbal1_yaw_set = (fp32)bullet_solver_.getYaw();
        //         robot_set->gimbal1_pitch_set = (fp32)bullet_solver_.getPitch();
        //         robot_set->aimx = (fp32)bullet_solver_.target_pos_.x;
        //         robot_set->aimy = (fp32)bullet_solver_.target_pos_.y;
        //         robot_set->aimz = (fp32)bullet_solver_.target_pos_.z;
        //     }
        // }

        // if (pkg.tracking) {
        //     robot_set->is_aiming = true;
        // } else {
        //     robot_set->is_aiming = false;
        // }
        update_time();
    }

}  // namespace Device
