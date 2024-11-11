#include "robot_controller.hpp"

#include "io.hpp"

namespace Robot
{
    Robot_ctrl::Robot_ctrl() {
        can0 = new IO::Can_interface("CAN_CHASSIS");
        can1 = new IO::Can_interface("CAN_LEFT_HEAD");
        can2 = new IO::Can_interface("CAN_RIGHT_HEAD");
        robot_set = std::make_shared<Robot_set>();
    }

    Robot_ctrl::~Robot_ctrl() {
        delete ser1;
        delete ser2;
        delete ser3;
        delete socket_intrf;
    }

    void Robot_ctrl::start_init() {
        // NOTE: register motors here
        imu.init(robot_set);
        // cv_controller_.init(robot_set);
        // chassis.init(robot_set);
        gimbal.init(robot_set);
        // gimbal_l.init(robot_set);
        // gimbal_big_yaw.init(robot_set);
        //  shoot.init(robot_set);

        // start DJIMotorManager thread
        Hardware::DJIMotorManager::start();

        while (imu.offline()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        gimbal_init_thread = std::make_unique<std::thread>(&Gimbal::Gimbal::init_task, &gimbal);
        // gimbal_l_init_thread = std::make_unique<std::thread>(&Gimbal::Gimbal_L::init_task, &gimbal_l);
        // gimbal_big_yaw_init_thread = std::make_unique<std::thread>(&Gimbal::Gimbal_big_yaw::init_task,
        // &gimbal_big_yaw);
    }

    void Robot_ctrl::init_join() const {
        if (gimbal_init_thread != nullptr) {
            gimbal_init_thread->join();
        }
        // if (gimbal_l_init_thread != nullptr) {
        //     gimbal_l_init_thread->join();
        // }
        // if (gimbal_big_yaw_init_thread != nullptr) {
        //     gimbal_big_yaw_init_thread->join();
        // }
    }

    void Robot_ctrl::start() {
        chassis_thread = std::make_unique<std::thread>(&Chassis::Chassis::task, &chassis);
        gimbal_thread = std::make_unique<std::thread>(&Gimbal::Gimbal::task, &gimbal);
        gimbal_l_thread = std::make_unique<std::thread>(&Gimbal::Gimbal_L::task, &gimbal_l);
        gimbal_big_yaw_thread = std::make_unique<std::thread>(&Gimbal::Gimbal_big_yaw::task, &gimbal_big_yaw);
        // shoot_thread = std::make_unique<std::thread>(&Shoot::Shoot::task, &shoot);

        vision_thread = std::make_unique<std::thread>(&Device::Cv_controller::task, &cv_controller_);
    }

    void Robot_ctrl::join() const {
        if (chassis_thread != nullptr) {
            chassis_thread->join();
        }
        if (gimbal_thread != nullptr) {
            gimbal_thread->join();
        }
        if (gimbal_l_thread != nullptr) {
            gimbal_l_thread->join();
        }
        if (gimbal_big_yaw_thread != nullptr) {
            gimbal_big_yaw_thread->join();
        }
    }

    void Robot_ctrl::load_hardware() {
        socket_intrf = new IO::Server_socket_interface("CONTROLLER_SOCKET");
        try {
            ser1 = new IO::Serial_interface<Types::ReceivePacket>("/dev/IMU_RIGHT", 115200, 2000);
        } catch (serial::IOException &ex) {
            LOG_ERR("there's no such serial device called IMU_RIGHT \n");
        }
        try {
            ser2 = new IO::Serial_interface<Types::ReceivePacket>("/dev/IMU_LEFT", 115200, 2000);
        } catch (serial::IOException &ex) {
            LOG_ERR("there's no such serial device called IMU_LEFT \n");
        }
        try {
            ser3 = new IO::Serial_interface<Types::ReceivePacket>("/dev/IMU_BIG_YAW", 115200, 2000);
        } catch (serial::IOException &ex) {
            LOG_ERR("there's no such serial device called IMU_BIG_YAW \n");
        }

        IO::io<CAN>.insert(*can0);
        IO::io<CAN>.insert(*can1);
        IO::io<CAN>.insert(*can2);
        IO::io<SERIAL>.insert(*ser1);
        IO::io<SERIAL>.insert(*ser2);
        IO::io<SERIAL>.insert(*ser3);
        IO::io<SOCKET>.insert(*socket_intrf);
    }
};  // namespace Robot
