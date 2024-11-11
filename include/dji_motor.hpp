//#ifndef __MOTOR__
//#define __MOTOR__

//#include <linux/can.h>

//#include "can.hpp"
//#include "device/deviece_base.hpp"
//#include "pid_controller.hpp"
//#include "types.hpp"

//namespace Hardware
//{

//    struct motor_can_message
//    {
//        uint16_t ecd = 0;
//        int16_t speed_rpm = 0;
//        int16_t given_current = 0;
//        uint8_t temperate = 0;
//        int16_t last_ecd = 0;
//    };

//    class Motor : public Device::DeviceBase
//    {
//       public:
//        explicit Motor(const Pid::Pid_config &config) : pid_ctrler(config) {};
//        void unpack(const can_frame &frame);
//        ~Motor() = default;

//       private:
//       public:
//        Pid::Pid_position pid_ctrler;
//        // TODO: can bus api related
//        motor_can_message motor_measure;

//        fp32 accel = 0.f;
//        fp32 speed = 0.f;
//        fp32 speed_set = 0.f;

//        int16_t give_current = 0;

//       private:
//    };

//    class Motor_9025 : public Device::DeviceBase
//    {
//       public:
//        explicit Motor_9025(const Pid::Pid_config &config) : pid_ctrler(config) {};
//        ~Motor_9025() = default;
//        void unpack(const can_frame &frame);

//       private:
//       public:
//        Pid::Pid_position pid_ctrler;
//        // TODO: can bus api related
//        motor_can_message motor_measure;

//        fp32 accel = 0.f;
//        fp32 speed = 0.f;
//        fp32 speed_set = 0.f;

//        int16_t give_current = 0;

//       private:
//    };

//    can_frame get_frame(canid_t can_id, const std::vector<Motor> &mots);

//    can_frame get_frame(canid_t can_id, const Motor_9025 &mots);

//    template<typename... Args>
//    can_frame get_frame(canid_t can_id, const Motor &head, Args &&...args) {
//        can_frame frame{};
//        frame.can_id = can_id;
//        frame.can_dlc = 8;
//        std::vector<const Motor *> mot_list = { &head, &args... };
//        for (int i = 0; i < mot_list.size() && i < 4; i++) {
//            frame.data[i << 1] = (mot_list[i]->give_current >> 8);
//            frame.data[i << 1 | 1] = (mot_list[i]->give_current & 0xFF);
//        }
//        return frame;
//    }
//}  // namespace Hardware

//#endif
#pragma once

#include <cmath>
#include <linux/can.h>

#include "actuator.hpp"
#include "can.hpp"
#include "types.hpp"

namespace Hardware {
    enum class DJIMotorCanID {
        ID1FF = 0,
        ID200 = 1,
        ID2FF = 2,
        ID_NULL = 3
    };

    class DJIMotor : public Actuator {
    public:
        struct Message {
            uint16_t ecd = 0;
            int16_t speed_rpm = 0;
            int16_t given_current = 0;
            uint8_t temperate = 0;

            void unpack(const can_frame &frame);
        };

        struct Can_info {
            std::string can_name_;
            DJIMotorCanID can_id_ = DJIMotorCanID::ID_NULL;
            int data_bias = 0;
            int callback_flag = 0;
        };

        Can_info can_info;
        std::string motor_name_;
        Message motor_measure_{};
        int motor_id_ = 0;
        bool motor_enabled_ = false;
        int16_t give_current = 0;

        DJIMotor() = default;

        virtual void unpack(const can_frame &frame);

        void set(float x) override;

        void enable();

        DJIMotor(const DJIMotor& other);
    };

    class M3508 final : public DJIMotor {
    public:
        constexpr static fp32 RPM_TO_RAD_S = 2.f * M_PIf / 60.f;
        constexpr static fp32 M6020_ECD_TO_RAD = 2.f * M_PIf / 8192.f;
        float angular_velocity = 0.f;
        float angle = 0.f;

        ~M3508() override = default;

        M3508(const std::string &can_name, int motor_id);

        void unpack(const can_frame &frame) override;
    };

    class M6020 final : public DJIMotor {
    public:
        constexpr static fp32 RPM_TO_RAD_S = 2.f * M_PIf / 60.f;
        constexpr static fp32 M6020_ECD_TO_RAD = 2.f * M_PIf / 8192.f;
        float angular_velocity = 0.f;
        float angle = 0.f;

        ~M6020() override = default;

        M6020(const std::string &can_name, int motor_id);

        void unpack(const can_frame &frame) override;
    };

    class M2006 final : public DJIMotor {
    public:

        ~M2006() override = default;

        M2006(const std::string &can_name, int motor_id);

        void unpack(const can_frame &frame) override;
    };

    namespace DJIMotorManager {
        struct CanBlock {
            IO::Can_interface *can_ = nullptr;
            std::vector<DJIMotor *> motors_;
        };

        extern void register_motor(DJIMotor &motor);

        extern void start();
    }
}
