#include <algorithm>
#include <fstream>
#include <iostream>
#include <thread>

#include "logger.hpp"
#include "macro_helpers.hpp"
#include "pid_controller.hpp"
#include "power_controller.hpp"
#include "robot_type_config.hpp"
#include "utils.hpp"

namespace Power {

PowerStatus powerStatus;
static bool  isCapEnergyOut             = false;
static float MIN_MAXPOWER_CONFIGURED    = 30.0f;

static inline bool floatEqual(float a, float b) {
    return fabs(a - b) < 1e-5f;
}

static inline float rpm2av(float rpm) {
    return rpm * (float)M_PI / 30.0f;
}

static inline float av2rpm(float av) {
    return av * 30.0f / (float)M_PI;
}

static inline void setErrorFlag(uint8_t &curFlag, Manager::ErrorFlags setFlag) {
    curFlag |= static_cast<uint8_t>(setFlag);
}

static inline void clearErrorFlag(uint8_t &curFlag, Manager::ErrorFlags clearFlag) {
    curFlag &= (~static_cast<uint8_t>(clearFlag));
}

static inline bool isFlagged(uint8_t &curFlag, Manager::ErrorFlags flag) {
    return (curFlag & static_cast<uint8_t>(flag)) != 0;
}

Manager::Manager(std::deque<Hardware::DJIMotor> &motors_,
                 const Division                 division_,
                 RLSEnabled                     rlsEnabled_,
                 const float                    k1_,
                 const float                    k2_,
                 const float                    k3_,
                 const float                    lambda_)
    : rlsEnabled(Manager::RLSEnabled::Enable),
      error(0UL),
      motors(motors_),
      division(division_),
      powerBuff(0.0f),
      fullBuffSet(0.0f),
      baseBuffSet(0.0f),
      fullMaxPower(0.0f),
      baseMaxPower(0.0f),
      powerUpperLimit(0.0f),
      refereeMaxPower(0.0f),
      userConfiguredMaxPower(0.0f),
      callback(nullptr),
      k1(k1_),
      k2(k2_),
      k3(k3_),
      lastUpdateTick(0),
      rls(1e-5f, 0.99999f) {
    configASSERT(k1_ >= 0);
    configASSERT(k2_ >= 0);
    configASSERT(k3_ >= 0);

    float initParams[2] = { k1_, k2_ };
    rls.setParamVector(Math::Matrixf<2, 1>(initParams));
}

static bool isInitialized;
uint8_t     xPowerTaskStack[1024];

void Manager::setMaxPowerConfigured(float maxPower) {
    userConfiguredMaxPower = std::clamp(maxPower, MIN_MAXPOWER_CONFIGURED, powerUpperLimit);
}

void Manager::setMode(uint8_t mode) {
    setMaxPowerConfigured(mode == 1 ? powerUpperLimit : powerLowerLimit);
}

std::array<float, 4> Manager::getControlledOutput(PowerObj *objs[4]) {
    std::array<float, 4> newTorqueCurrent{};
    std::array<bool, 4>  motorOnline{};

    for (int i = 0; i < 4; i++) {
        motorOnline[i] = isMotorOnline(i);
    }

    float torqueConst = 0.3f * (187.0f / 3591.0f);
    float k0 = torqueConst * 20.0f / 16384.0f;  // torque current rate: Nm/Output

    float sumCmdPower = 0.0f;
    std::array<float, 4> cmdPower{};

    float sumError = 0.0f;
    std::array<float, 4> error{};

    //按照现有逻辑，userConfiguredMaxPower永远大于baseMaxPower，所以maxPower恒等于baseMaxPower
    float maxPower = std::clamp(userConfiguredMaxPower, fullMaxPower, baseMaxPower);

    float allocatablePower  = maxPower;
    float sumPowerRequired  = 0.0f;
    static float newCmdPower;

    for (int i = 0; i < 4; i++) {
        PowerObj *p = objs[i];

        if (!motorOnline[i]) {
            cmdPower[i] = 0.0f;
            error[i]    = 0.0f;
            continue;
        }

        cmdPower[i] = p->pidOutput * k0 * p->curAv +
                      fabs(p->curAv) * k1 +
                      p->pidOutput * k0 * p->pidOutput * k0 * k2 +
                      k3 / 4.0f;

        sumCmdPower += cmdPower[i];
        error[i]     = fabs(p->setAv - p->curAv);

        if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
            allocatablePower += -cmdPower[i];
        } else {
            sumError        += error[i];
            sumPowerRequired += cmdPower[i];
        }
    }

    powerStatus.maxPowerLimited          = maxPower;
    powerStatus.sumPowerCmd_before_clamp = sumCmdPower;

    if (sumCmdPower > maxPower) {
        float errorConfidence = 0.0f;

        if (sumError > error_powerDistribution_set) {
            errorConfidence = 1.0f;
        } else if (sumError > prop_powerDistribution_set) {
            errorConfidence = std::clamp(
                (sumError - prop_powerDistribution_set) /
                    (error_powerDistribution_set - prop_powerDistribution_set),
                0.0f,
                1.0f);
        }

        for (int i = 0; i < 4; i++) {
            PowerObj *p = objs[i];

            if (!motorOnline[i]) {
                newTorqueCurrent[i] = 0.0f;
                continue;
            }

            if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
                newTorqueCurrent[i] = p->pidOutput;
                continue;
            }

            float powerWeight_Error = fabs(p->setAv - p->curAv) / sumError;
            float powerWeight_Prop  = cmdPower[i] / sumPowerRequired;
            float powerWeight       = errorConfidence * powerWeight_Error +
                                (1.0f - errorConfidence) * powerWeight_Prop;

            float delta = p->curAv * p->curAv -
                          4.0f * k2 *
                              (k1 * fabs(p->curAv) + k3 / 4.0f -
                               powerWeight * allocatablePower);

            if (floatEqual(delta, 0.0f))// repeat roots 
            {
                newTorqueCurrent[i] = -p->curAv / (2.0f * k2) / k0;
            } else if (delta > 0.0f)// two real roots 
            {
                newTorqueCurrent[i] = p->pidOutput > 0.0f
                                           ? (-p->curAv + sqrtf(delta)) / (2.0f * k2) / k0
                                           : (-p->curAv - sqrtf(delta)) / (2.0f * k2) / k0;
            } else// complex roots 
            {
                newTorqueCurrent[i] = -p->curAv / (2.0f * k2) / k0;
            }

            newTorqueCurrent[i] =
                std::clamp(newTorqueCurrent[i], -p->pidMaxOutput, p->pidMaxOutput);
        }
    } else {
        for (int i = 0; i < 4; i++) {
            if (!motorOnline[i]) {
                newTorqueCurrent[i] = 0.0f;
                continue;
            }
            newTorqueCurrent[i] = objs[i]->pidOutput;
        }
    }

    float newCmdPower = 0.0f;
    for (int i = 0; i < 4; i++) {
        PowerObj *p = objs[i];
        newCmdPower += newTorqueCurrent[i] * k0 * p->curAv +
                       fabs(p->curAv) * k1 +
                       newTorqueCurrent[i] * k0 * newTorqueCurrent[i] * k0 * k2 +
                       k3 / 4.0f;
    }

    LOG_INFO(
        "\n---------------------PowerLimit LOG_INFO----------------------\n"
        "sumPower: %f,\nNewCMDPower: %f,\nMax Power: %f,\nmeasuredPower: %f,\n"
        "capEnergy: %d,\nchassisPowerlimit: %f\n"
        "-------------------------------\n",
        sumPowerRequired,
        newCmdPower,
        maxPower,
        robot_set->super_cap_info.rx.chassisPower,
        robot_set->super_cap_info.rx.capEnergy,
        refereeMaxPower);

    return newTorqueCurrent;// 直接返回 std::array
}

[[noreturn]] void Manager::powerDaemon() {
    static Math::Matrixf<2, 1> samples;
    static Math::Matrixf<2, 1> params;
    static float                effectivePower = 0.0f;

    // 能量环软启动相关状态，防止 maxPower 剧烈跳变
    static bool  energyLoopInitialized = false;
    static float smoothBaseBuffSet     = capBaseBuffSet;
    static float smoothFullBuffSet     = capFullBuffSet;
    static float pdEffectLimit         = 0.0f;

    isInitialized = true;

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    lastUpdateTick = clock();

    while (true) {
        setMode(1);//似乎没有任何作用，TODO

        bool  capOnline    = isCapOnline();//fallback add
        float torqueConst  = 0.3f * (187.0f / 3591.0f);
        float k0           = torqueConst * 20.0f / 16384.0f;// torque current rate of the motor, defined as Nm/Output
        size_t now         = clock();

        isCapEnergyOut     = false;
        estimatedCapEnergy = robot_set->super_cap_info.rx.capEnergy / 255.0f * 2100.0f;

        // 估计功率（根据模型）
        effectivePower = 0.0f;
        samples[0][0]  = 0.0f;
        samples[1][0]  = 0.0f;

        for (int i = 0; i < 4; i++) {
            //LOG_INFO("%d", motors[i].motor_measure_.given_current);
            effectivePower += motors[i].motor_measure_.given_current * k0 *
                              rpm2av(motors[i].motor_measure_.speed_rpm);
            samples[0][0] += fabsf(rpm2av(motors[i].motor_measure_.speed_rpm));
            samples[1][0] += motors[i].motor_measure_.given_current * k0 *
                             motors[i].motor_measure_.given_current * k0;
        }

        estimatedPower = k1 * samples[0][0] +
                         k2 * samples[1][0] +
                         effectivePower +
                         k3;

        // 根据电容在线状态选择能量反馈来源
        if (capOnline) {
            uint8_t cap = robot_set->super_cap_info.rx.capEnergy;
            powerBuff   = sqrtf(static_cast<float>(cap));

            if (!energyLoopInitialized) {
                energyLoopInitialized = true;
                smoothBaseBuffSet     = static_cast<float>(cap);
                smoothFullBuffSet     = static_cast<float>(cap);
            }

            measuredPower = robot_set->super_cap_info.rx.chassisPower;

            constexpr float alpha = 0.01f;
            if (measuredPower > 5.0f) {
                smoothBaseBuffSet = (1.0f - alpha) * smoothBaseBuffSet + alpha * capBaseBuffSet;
                smoothFullBuffSet = (1.0f - alpha) * smoothFullBuffSet + alpha * capFullBuffSet;
            }

            fullBuffSet = smoothFullBuffSet;
            baseBuffSet = smoothBaseBuffSet;

            uint16_t power_limit = MUXDEF(
                CONFIG_HERO,
                Power::HeroChassisPowerLimit * 0.9,
                MUXDEF(
                    CONFIG_INFANTRY,
                     // 步兵这里暂时使用血量优先模式
                    Power::InfantryChassisPowerLimit_HPFirst * 0.9,
                    Power::SentryChassisPowerLimit * 0.9));

            refereeMaxPower         = power_limit;
            powerUpperLimit         = refereeMaxPower + MAX_CAP_POWER_OUT;
            powerLowerLimit         = 50.0f;
            MIN_MAXPOWER_CONFIGURED = powerLowerLimit * 0.8f;

            // 对 PID 输出增加软上限，避免 maxPower 剧烈跳变
            constexpr float pdEffectLimitMax  = 60.0f;  // PID 调整最多 ±60W
            constexpr float pdEffectLimitRamp = 0.1f;   // 每次循环变化 0.1W

            if (measuredPower < 5.0f) {
                baseMaxPower = refereeMaxPower;
                fullMaxPower = refereeMaxPower;

                // 慢慢“冷却” PID 影响
                pdEffectLimit = std::max(pdEffectLimit - pdEffectLimitRamp, 0.0f);
            } else {
                // 有功率输出：启用能量环，并逐步开放 PID 影响力
                pdEffectLimit = std::min(pdEffectLimit + pdEffectLimitRamp, pdEffectLimitMax);

                powerPD_base.set(sqrtf(baseBuffSet));
                powerPD_full.set(sqrtf(fullBuffSet));

                float baseOut = powerPD_base.out;
                float fullOut = powerPD_full.out;

                baseOut = std::clamp(baseOut, -pdEffectLimit, pdEffectLimit);
                fullOut = std::clamp(fullOut, -pdEffectLimit, pdEffectLimit);

                baseMaxPower = std::clamp(refereeMaxPower - baseOut,
                                          MIN_MAXPOWER_CONFIGURED,
                                          powerUpperLimit);
                fullMaxPower = std::clamp(refereeMaxPower - fullOut,
                                          MIN_MAXPOWER_CONFIGURED,
                                          powerUpperLimit);
            }
        } else {
            // 超电离线：停止使用能量环，使用估计功率和保守功率上限
            powerBuff   = 0.0f;
            baseBuffSet = 0.0f;
            fullBuffSet = 0.0f;

            measuredPower = estimatedPower;

            baseMaxPower = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD;
            fullMaxPower = CAP_OFFLINE_ENERGY_TARGET_POWER;

            energyLoopInitialized = false;
            pdEffectLimit         = 0.0f;
        }

    // NOTE: log k1 k2 k3
        // LOG_INFO(
        //     "%f %f %f %f %f %f\n", measuredPower, effectivePower, estimatedPower, k1, k2,
        //     k3);

    // NOTE: log PIDs
        // LOG_INFO(
        //     "%f, %f, %f, %f, %f %d\n",
        //    sqrtf(baseBuffSet),
        //     powerBuff,
        //     refereeMaxPower,
        //     powerPD_base.out,
        //     baseMaxPower,
        //     robot_set->super_cap_info.rx.capEnergy);

    //NOTE: log super_cat_info
        //LOG_INFO(
        //    "CapEnergy: %d , ChassisPower: %f , ChassisPowerlimit: %d\n",
        //    robot_set->super_cap_info.rx.capEnergy,
        //    robot_set->super_cap_info.rx.chassisPower,
        //    robot_set->super_cap_info.rx.chassisPowerlimit);

    // NOTE: for dumping log and draw purpose
        // printf("%f, %f\n", baseMaxPower, fullMaxPower);
        // outputFile << refereeMaxPower << ", " << baseMaxPower << "\n" << std::flush;
        //outputFile << baseMaxPower << ", " << fullMaxPower << "\n" << std::flush;

        powerStatus.userConfiguredMaxPower = userConfiguredMaxPower;
        powerStatus.effectivePower         = effectivePower;
        powerStatus.powerLoss              = measuredPower - effectivePower;
        powerStatus.efficiency             = std::clamp(
            effectivePower / measuredPower, 0.0f, 1.0f);
        powerStatus.estimatedCapEnergy =
            static_cast<uint8_t>(estimatedCapEnergy / 2100.0f * 255.0f);
        powerStatus.error = static_cast<Manager::ErrorFlags>(error);

        // RLS 参数更新：只在 CAP 在线且功率足够大时更新
        if (fabs(measuredPower) > 5.0f &&
            capOnline &&
            rlsEnabled == Manager::RLSEnabled::Enable) {
            params = rls.update(samples, measuredPower - effectivePower - k3);
            k1     = fmax(params[0][0], 1e-5f);
            k2     = fmax(params[1][0], 1e-5f);
        }

        lastUpdateTick = now;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Manager::init(const std::shared_ptr<Robot::Robot_set> &robot) {
    if (isInitialized)
        return;

    robot_set                = robot;
    MIN_MAXPOWER_CONFIGURED  = 30.0f;
    powerUpperLimit          = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD;
    powerPD_base             = Pid::PidPosition(powerPD_base_pid_config, powerBuff);
    powerPD_full             = Pid::PidPosition(powerPD_full_pid_config, powerBuff);
}

//fallback add begin
bool Manager::isCapOnline() const {
    using Clock = std::chrono::steady_clock;

    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      Clock::now().time_since_epoch())
                      .count();
    auto duration = now_ms - robot_set->super_cap_info.last_update_ms;

    if (duration > CAP_OFFLINE_TIME_MS ||
        robot_set->super_cap_info.rx.errorCode != 0) {
        return false;
    }
    return true;
}

bool Manager::isMotorOnline(int idx) const {
    if (idx < 0 || idx >= static_cast<int>(motors.size())) {
        return false;
    }

    static uint16_t lastEcd[4]   = { 0 };
    static int16_t  lastSpeed[4] = { 0 };
    static int      offlineCounter[4] = { 0 };

    bool cmdActive = abs(motors[idx].motor_measure_.given_current) >
                     MOTOR_CURRENT_ACTIVE_THRESHOLD;
    bool unchanged = (motors[idx].motor_measure_.ecd == lastEcd[idx]) &&
                     (motors[idx].motor_measure_.speed_rpm == lastSpeed[idx]);

    lastEcd[idx]   = motors[idx].motor_measure_.ecd;
    lastSpeed[idx] = motors[idx].motor_measure_.speed_rpm;

    if (cmdActive && unchanged) {
        offlineCounter[idx]++;
        if (offlineCounter[idx] >= MOTOR_OFFLINE_THRESHOLD) {
            return false;
        }
    } else if (!cmdActive) {
        offlineCounter[idx] = std::max(offlineCounter[idx] - 1, 0);
    } else {
        offlineCounter[idx] = 0;
    }

    return true;
}

// TODO: 当前未在其它模块使用 error flags，预留备用
void Manager::updateErrorFlags() {
    if (!isCapOnline()) {
        setErrorFlag(error, Manager::ErrorFlags::CAPDisConnect);
    } else {
        clearErrorFlag(error, Manager::ErrorFlags::CAPDisConnect);
    }

    bool anyMotorOff = false;
    for (int i = 0; i < 4; i++) {
        if (!isMotorOnline(i)) {
            anyMotorOff = true;
            break;
        }
    }

    if (anyMotorOff) {
        setErrorFlag(error, Manager::ErrorFlags::MotorDisconnect);
    } else {
        clearErrorFlag(error, Manager::ErrorFlags::MotorDisconnect);
    }
}//fallback add end
} // namespace Power
