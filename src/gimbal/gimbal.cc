#include "gimbal.hpp"

namespace Gimbal
{

    Gimbal::Gimbal()
        : yaw_motor(Config::YAW_SPEED_PID_CONFIG),
          pitch_motor(Config::PITCH_SPEED_PID_CONFIG),
          yaw_absolute_pid(Config::GIMBAL_YAW_ABSOLUTE_PID_CONFIG),
          pitch_absolute_pid(Config::GIMBAL_PITCH_ABSOLUTE_PID_CONFIG),
          yaw_relative_pid(Config::GIMBAL_YAW_RELATIVE_PID_CONFIG),
          pitch_relative_pid(Config::GIMBAL_PITCH_RELATIVE_PID_CONFIG) {
    }

    void Gimbal::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;

        Robot::hardware->register_callback<CAN1>(0x206, [&](const auto &frame) { yaw_motor.unpack(frame); });
        Robot::hardware->register_callback<CAN1>(0x205, [&](const auto &frame) { pitch_motor.unpack(frame); });
    }

    void Gimbal::init_task() {
        while (!inited) {
            update_data();
            init_yaw_set = 0;
            init_pitch_set = 0;

            yaw_relative_pid.calc(robot_set->yaw_relative, init_yaw_set);
            yaw_motor.speed_set = yaw_relative_pid.out;
            yaw_motor.pid_ctrler.calc(-yaw_gyro, yaw_motor.speed_set);
            yaw_motor.give_current = (int16_t)yaw_motor.pid_ctrler.out;

            pitch_absolute_pid.calc(robot_set->ins_pitch, init_pitch_set);
            pitch_motor.speed_set = pitch_absolute_pid.out;
            pitch_motor.pid_ctrler.calc(-pitch_gyro, pitch_motor.speed_set);
            pitch_motor.give_current = (int16_t)-pitch_motor.pid_ctrler.out;

            LOG_INFO(
                "%f, %f, %f, %u, %u\n",
                robot_set->ins_pitch,
                pitch_motor.speed_set,
                -pitch_gyro,
                yaw_motor.motor_measure.ecd,
                pitch_motor.motor_measure.ecd);

            if (fabs(robot_set->yaw_relative) < Config::GIMBAL_INIT_EXP &&
                fabs(robot_set->ins_pitch - init_pitch_set) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            } else {
                robot_set->yaw_set = 0;
                robot_set->pitch_set = robot_set->ins_pitch;
                init_stop_times = 0;
            }
            inited = init_stop_times >= Config::GIMBAL_INIT_STOP_TIME;
            Robot::hardware->send<CAN1>(Hardware::get_frame(0x1FF, pitch_motor, yaw_motor));
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    [[noreturn]] void Gimbal::task() {
        while (true) {
            update_data();
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                yaw_motor.give_current = 0.f;
                pitch_motor.give_current = 0.f;
            } else if (robot_set->mode == Types::ROBOT_MODE::ROBOT_FINISH_INIT) {
                static float delta = 0, delta1 = M_PIf / 5;
                float y = .0, p = .0;
                y = robot_set->yaw_set - delta;
                // 仰角9.57 俯角27.42
                p = robot_set->pitch_set + (std::sin(delta1) * 0.10550000000000002 - 0.09233333333333336) * M_PIf;
                delta += 0.002;
                delta1 += 0.008;

                if (y < robot_set->yaw_set - M_PIf)
                    robot_set->mode = Types::ROBOT_MODE::ROBOT_FOLLOW_GIMBAL;

                LOG_INFO("%f %f %f\n", p, robot_set->ins_pitch, std::sin(delta));

                yaw_relative_pid.calc(robot_set->yaw_relative, y);
                yaw_motor.speed_set = yaw_relative_pid.out;
                yaw_motor.pid_ctrler.calc(-yaw_gyro, yaw_motor.speed_set);
                yaw_motor.give_current = (int16_t)yaw_motor.pid_ctrler.out;

                pitch_absolute_pid.calc(robot_set->ins_pitch, p);
                pitch_motor.speed_set = pitch_absolute_pid.out;
                pitch_motor.pid_ctrler.calc(-pitch_gyro, pitch_motor.speed_set);
                pitch_motor.give_current = (int16_t)-pitch_motor.pid_ctrler.out;

            } else {
                yaw_relative_pid.calc(robot_set->yaw_relative, init_yaw_set);
                yaw_motor.speed_set = yaw_relative_pid.out;
                yaw_motor.pid_ctrler.calc(-yaw_gyro, yaw_motor.speed_set);
                yaw_motor.give_current = (int16_t)yaw_motor.pid_ctrler.out;

                pitch_absolute_pid.calc(robot_set->ins_pitch, init_pitch_set);
                pitch_motor.speed_set = pitch_absolute_pid.out;
                pitch_motor.pid_ctrler.calc(-pitch_gyro, pitch_motor.speed_set);
                pitch_motor.give_current = (int16_t)-pitch_motor.pid_ctrler.out;
            }
            Robot::hardware->send<CAN1>(Hardware::get_frame(0x1FF, pitch_motor, yaw_motor));
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    void Gimbal::update_data() {
        yaw_motor.speed = Config::RPM_TO_RAD_S * (fp32)yaw_motor.motor_measure.speed_rpm;
        pitch_motor.speed = Config::RPM_TO_RAD_S * (fp32)pitch_motor.motor_measure.speed_rpm;

        robot_set->yaw_relative = UserLib::rad_format(
            Config::M6020_ECD_TO_RAD * ((fp32)yaw_motor.motor_measure.ecd - Config::GIMBAL_YAW_OFFSET_ECD));
        robot_set->pitch_relative = UserLib::rad_format(
            Config::M6020_ECD_TO_RAD * -((fp32)pitch_motor.motor_measure.ecd - Config::GIMBAL_PITCH_OFFSET_ECD));

        yaw_gyro = std::sin(robot_set->pitch_relative) * robot_set->ins_roll_v -
                   std::cos(robot_set->pitch_relative) * robot_set->ins_yaw_v;
        pitch_gyro = robot_set->ins_pitch_v;
    }

}  // namespace Gimbal
