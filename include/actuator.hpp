#pragma once

#include "controller.hpp"

class Actuator
{
   public:
    ControllerList controller;
    Actuator(const Actuator &other) = delete;

    Actuator() = default;

    bool operator=(const Actuator &other) = delete;
    virtual void set(float x) = 0;  //统一控制接口
    virtual ~Actuator() = default;

    template<ControllerBase Ctrl>
    //设置控制器
    void setCtrl(Ctrl &&ctrl) {
        controller = std::forward<Ctrl>(ctrl);
    }

    //在控制器前添加一个控制器
    template<ControllerBase Ctrl>
    void pushFrontCtrl(Ctrl &&ctrl) {
        controller = std::forward<Ctrl>(ctrl) >> std::move(controller);
    }

    //在控制器后添加一个控制器
    template<ControllerBase Ctrl>
    void pushBackCtrl(Ctrl &&ctrl) {
        controller = std::move(controller) >> std::forward<Ctrl>(ctrl);
    }
};

template<typename T>
concept ActuatorBase = std::is_base_of_v<Actuator, T>;

// template<ControllerBase Ctrl, ActuatorBase Act>
// Act operator>>(Ctrl &&ctrl, Act &&act) {
//     Act res = act;
//     res.controller = std::forward<Ctrl>(ctrl) >> std::forward<Act>(act.controller);
//     return res;
// }

template<ActuatorBase Act>
void operator>>(float x, Act &act) {
    act.set(x);
}
