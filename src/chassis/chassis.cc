#include "chassis.hpp"

#include "config.hpp"
#include "user_lib.hpp"

namespace Chassis
{
    Chassis::Chassis() : chassis_angle_pid(nullptr) {
        fp32 fuckme = 0.f;
        motors.reserve(4);
        for (int i = 1; i <= 4; i++) {
            motors.push_back(Hardware::M3508("CAN_CHASSIS", i));
        }
    }

    void Chassis::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        chassis_angle_pid = new Pid::PidRad(Config::CHASSIS_FOLLOW_GIMBAL_PID_CONFIG, robot_set->gimbal3_yaw_relative);

        for (auto &m : motors) {
            m.setCtrl(Pid::PidPosition(Config::M3508_SPEED_PID_CONFIG, m.angular_velocity));
            m.enable();
        }
    }

    [[noreturn]] void Chassis::task() {
        while (true) {
            decomposition_speed();
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                for (auto &m : motors) {
                    m.set(0.f);
                }
            } else {
                fp32 max_speed = 0.f;
                for (int i = 0; i < 4; i++) {
                    max_speed = std::max(max_speed, fabsf(wheel_speed[i]));
                }
                if (max_speed > max_wheel_speed) {
                    fp32 speed_rate = max_wheel_speed / max_speed;
                    for (int i = 0; i < 4; i++) {
                        wheel_speed[i] *= speed_rate;
                    }
                }
                for (int i = 0; i < 4; i++) {
                    motors[i].set(wheel_speed[i]);
                }
            }
            UserLib::sleep_ms(Config::CHASSIS_CONTROL_TIME);
        }
    }

    void Chassis::decomposition_speed() {
        if (robot_set->mode != Types::ROBOT_MODE::ROBOT_NO_FORCE) {
            fp32 sin_yaw, cos_yaw;
            sincosf(robot_set->gimbal3_yaw_relative, &sin_yaw, &cos_yaw);
            vx_set = cos_yaw * robot_set->vx_set - sin_yaw * robot_set->vy_set;
            vy_set = sin_yaw * robot_set->vx_set + cos_yaw * robot_set->vy_set;

            if (robot_set->wz_set == 0.f) {
                chassis_angle_pid->set(0.f);
                wz_set = chassis_angle_pid->out;
            } else {
                wz_set = robot_set->wz_set;
            }
        }

        wheel_speed[0] = -vx_set + vy_set + wz_set;
        wheel_speed[1] = vx_set + vy_set + wz_set;
        wheel_speed[2] = vx_set - vy_set + wz_set;
        wheel_speed[3] = -vx_set - vy_set + wz_set;
    }
}  // namespace Chassis
