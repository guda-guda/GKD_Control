#include "robot_controller.hpp"
#include CONFIGHPP
#include "io.hpp"

namespace Robot
{

    Robot_ctrl::Robot_ctrl() : rc_controller("/dev/IMU_BIG_YAW"), gimbal(Config::gimbal_config), chassis(Config::chassis_config) {
        robot_set = std::make_shared<Robot_set>();
    }

    Robot_ctrl::~Robot_ctrl() = default;

    void Robot_ctrl::start_init() {
        // NOTE: register motors here

        // cv_controller_.init(robot_set);
        rc_controller.enable();
        //  shoot.init(robot_set);

        chassis.init(robot_set);
        gimbal.init(robot_set);

        // start DJIMotorManager thread
        Hardware::DJIMotorManager::start();

        threads.emplace_back(&Config::GimbalType::init_task, &gimbal);

    }

    void Robot_ctrl::init_join() {
        threads.clear();
    }

    void Robot_ctrl::start() {
        // chassis_thread = std::make_unique<std::thread>(&Chassis::Chassis::task, &chassis);
        threads.emplace_back(&Config::GimbalType::task, &gimbal);

        // shoot_thread = std::make_unique<std::thread>(&Shoot::Shoot::task, &shoot);
        // vision_thread = std::make_unique<std::thread>(&Device::Cv_controller::task, &cv_controller_);
    }

    void Robot_ctrl::join() {
        threads.clear();
    }

    void Robot_ctrl::load_hardware() {
        for (auto& name : Config::CanInitList) {
            IO::io<CAN>.insert(name);
        }
        for (auto& [name, baud_rate, simple_timeout] : Config::SerialInitList) {
            IO::io<SERIAL>.insert(name, baud_rate, simple_timeout);
        }
        // for(auto & name : Config :: SocketInitList) {
        //     IO::io<SOCKET>.insert(name);
        // }
    }
};  // namespace Robot
