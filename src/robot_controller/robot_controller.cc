#include "robot_controller.hpp"
#include CONFIGHPP
#include "io.hpp"

template<typename Ptr, typename Config>
void create_ptr_config(Ptr& ptr, Config&& config)
{
    if (config)
        ptr = new typename std::remove_pointer_t<Ptr>(*config);
    else
        ptr = nullptr;
}

namespace Robot
{

    Robot_ctrl::Robot_ctrl() : rc_controller("/dev/IMU_BIG_YAW") {
        robot_set = std::make_shared<Robot_set>();
        create_ptr_config(chassis, Config::chassis_config);
        create_ptr_config(gimbal, Config::gimbal_right_config);
        create_ptr_config(gimbal_l, Config::gimbal_left_config);

#ifdef SENTRY
        gimbal_big_yaw = new Gimbal::GimbalSentry{};
#endif
        shoot = new Shoot::Shoot{};
        cv_controller_ = new Device::Cv_controller{};
    }

    Robot_ctrl::~Robot_ctrl() = default;

    void Robot_ctrl::start_init() {
        // NOTE: register motors here

        // cv_controller_.init(robot_set);
        rc_controller.enable();
        //  shoot.init(robot_set);

        threads.emplace_back(&Chassis::Chassis::init, chassis, robot_set);
        threads.emplace_back(&Gimbal::GimbalT::init, gimbal, robot_set);
        threads.emplace_back(&Gimbal::GimbalT::init, gimbal_l, robot_set);
        threads.emplace_back(&Gimbal::GimbalSentry::init, gimbal_big_yaw, robot_set);

        threads.join();

        // start DJIMotorManager thread
        Hardware::DJIMotorManager::start();
        LOG_INFO("DJIMotorManager start");
        
        threads.emplace_back(&Gimbal::GimbalT::init_task, gimbal);
        threads.emplace_back(&Gimbal::GimbalT::init_task, gimbal_l);
        threads.emplace_back(&Gimbal::GimbalSentry::init_task, gimbal_big_yaw);

    }

    void Robot_ctrl::init_join() {
        threads.join();
    }

    void Robot_ctrl::start() {
        // chassis_thread = std::make_unique<std::thread>(&Chassis::Chassis::task, &chassis);
        threads.emplace_back(&Gimbal::GimbalT::task, gimbal);
        threads.emplace_back(&Gimbal::GimbalT::task, gimbal_l);
        threads.emplace_back(&Gimbal::GimbalSentry::task, gimbal_big_yaw);

        // shoot_thread = std::make_unique<std::thread>(&Shoot::Shoot::task, &shoot);
        // vision_thread = std::make_unique<std::thread>(&Device::Cv_controller::task, &cv_controller_);
    }

    void Robot_ctrl::join() {
        threads.join();
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
