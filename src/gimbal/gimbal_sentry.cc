#include "gimbal/gimbal_sentry.hpp"

#include CONFIGHPP
#include "user_lib.hpp"
#include <future>

namespace Gimbal
{
    GimbalSentry::GimbalSentry(const std::pair<GimbalConfig,GimbalConfig>& config) : imu("/dev/IMU_BIG_YAW"), yaw_motor("CAN_BULLET", 1), gimbal_left(config.first), gimbal_right(config.second) {
    }

    void GimbalSentry::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;

        yaw_absolute_pid = Pid::PidRad(Config::GIMBAL_9025_YAW_ABSOLUTE_PID_CONFIG, imu.yaw) >> Pid::Invert(-1);
        yaw_relative_pid = Pid::PidRad(Config::GIMBAL_9025_YAW_RELATIVE_PID_CONFIG, yaw_relative);
        yaw_relative_with_two_head_pid =
            Pid::PidRad(Config::GIMBAL_9025_YAW_RELATIVE_PID_CONFIG, yaw_relative_with_two_head) >> Pid::Invert(-1);

        yaw_motor.setCtrl(Pid::PidPosition(Config::YAW_9025_SPEED_PID_CONFIG, imu.yaw_rate));
        yaw_set = &robot_set->gimbal_sentry_yaw_set;

        imu.enable();
        yaw_motor.enable();

        gimbal_left.init(robot);
        gimbal_right.init(robot);
    }

    void GimbalSentry::init_task() {
        std::future<void> future_left = std::async(&GimbalT::init_task, &gimbal_left);
        std::future<void> future_right = std::async(&GimbalT::init_task, &gimbal_right);
        while (!inited) {
            update_data();
            0.f >> yaw_relative_pid >> yaw_motor;
            //  LOG_INFO("big yaw %f %f\n", yaw_motor_speed,yaw_relative);
            if (fabs(yaw_relative) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            } else {
                *yaw_set = imu.yaw;
                init_stop_times = 0;
            }
            // inited = init_stop_times >= Config::GIMBAL_INIT_STOP_TIME;
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
        future_left.get();
        future_right.get();
    }

    [[noreturn]] void GimbalSentry::task() {
        std::thread thread_left{&GimbalT::task, &gimbal_left};
        std::thread thread_right{&GimbalT::task, &gimbal_right};
        while (true) {
            update_data();
            switch (robot_set->mode) {
                case Types::ROBOT_MODE::ROBOT_NO_FORCE: 0 >> yaw_motor; continue;
                case Types::ROBOT_MODE::ROBOT_FINISH_INIT:
                case Types::ROBOT_MODE::ROBOT_IDLE:
                case Types::ROBOT_MODE::ROBOT_SEARCH: *yaw_set >> yaw_absolute_pid >> yaw_motor; continue;
                default: 0.f >> yaw_relative_with_two_head_pid >> yaw_motor;
            };
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    void GimbalSentry::update_data() {
        yaw_motor_speed = Config::RPM_TO_RAD_S * (fp32)yaw_motor.motor_measure.speed_rpm;
        yaw_relative = UserLib::rad_format(
            Config::M9025_ECD_TO_RAD * ((fp32)yaw_motor.motor_measure.ecd - Config::GIMBAL3_YAW_OFFSET_ECD));
        yaw_relative_with_two_head = robot_set->gimbalT_1_yaw_reletive + robot_set->gimbalT_2_yaw_reletive;
        robot_set->gimbal_sentry_yaw_reletive = yaw_relative;
        robot_set->gimbal_sentry_yaw = imu.yaw;
    }
}  // namespace Gimbal
