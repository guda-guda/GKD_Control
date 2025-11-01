#ifndef __ROBOT__
#define __ROBOT__
#include "types.hpp"

namespace Robot
{
    // robot set header = 0xEA;
    //单写多读
    struct Robot_set
    {
        uint8_t header;                     // 数据包头
        /** chassis_control **/
        fp32 vx_set = 0.f;                  // 期望机器人在全局坐标系下的x轴速度
        fp32 vy_set = 0.f;                  // 期望机器人在全局坐标系下的y轴速度
        fp32 wz_set = 0.f;                  // 期望机器人绕z轴旋转速度，即角速度指令
        bool spin_state = false;            // 机器人旋转状态，true表示正在旋转，false表示不旋转

        /** gimbal_control **/
        fp32 gimbal1_yaw_set = 0.f;         // 期望云台1的绝对yaw角度
        fp32 gimbal1_yaw_offset = 0.f;      // 云台1的yaw角度偏移，用于校准
        fp32 gimbal1_pitch_set = 0.f;       // 期望云台1的绝对pitch角度

        fp32 gimbal2_yaw_set = 0.f;       // 期望云台2的绝对yaw角度
        fp32 gimbal2_yaw_offset = 0.f;    // 云台2的yaw角度偏移，用于校准
        fp32 gimbal2_pitch_set = 0.f;     // 期望云台2的绝对pitch角度

        fp32 gimbal3_yaw_set = 0.f;       // 期望云台3的绝对yaw角度
        fp32 gimbal3_yaw_offset = 0.f;    // 云台3的yaw角度偏移，用于校准
        fp32 gimbal3_pitch_set = 0.f;     // 期望云台3的绝对pitch角度

        /** shoot_control **/
        bool friction_open = false;      //摩擦轮开关
        bool friction_real_state =          
            false;  // friction's real state (motor linear speed < 0.5 ? false : true)
        bool cv_fire = false;             //视觉触发射击
        int shoot_open = 0;               //射击权限，按位表示各个云台的射击权限

        /** other **/
        fp32 gimbalT_1_yaw_set = 0.f;       // 期望云台1的绝对yaw角度
        fp32 gimbalT_1_pitch_set = 0.f;     // 期望云台1的绝对pitch角度
        fp32 gimbalT_1_yaw_reletive = 0.f;  // 云台1相对机器人前进方向的yaw角度    

        // only sentry needs gimbalT_2
        fp32 gimbalT_2_yaw_set = 0.f;       // 期望云台2的绝对yaw角度
        fp32 gimbalT_2_pitch_set = 0.f;     // 期望云台2的绝对pitch角度
        fp32 gimbalT_2_yaw_reletive = 0.f;  // 云台2相对机器人前进方向的yaw角度

        fp32 gimbal_sentry_yaw_set = 0.f;   // 期望哨兵云台的绝对yaw角度
        fp32 gimbal_sentry_yaw = 0.f;       // 哨兵云台的yaw角度
        fp32 gimbal_sentry_yaw_reletive = 0.f;// 哨兵云台相对机器人前进方向的yaw角度

        fp32 aimx;                          // 目标坐标x，由视觉提供
        fp32 aimy;                          // 目标坐标y，由视觉提供
        fp32 aimz;                          // 目标坐标z，由视觉提供
        bool is_aiming = false;             // 是否有目标，由视觉提供
        uint8_t inited = 0;                // 初始化状态

        uint8_t sentry_follow_gimbal = 0;   // 哨兵云台是否跟随主云台

        bool auto_aim_status = false;       // 是否处于自瞄状态

        Types::ROBOT_MODE mode = Types::ROBOT_MODE::ROBOT_NO_FORCE;         // 机器人当前工作模式
        Types::ROBOT_MODE last_mode = Types::ROBOT_MODE::ROBOT_NO_FORCE;    // 上一个机器人工作模式

        Types::ReceivePacket_Super_Cap super_cap_info;                      // 超级电容信息
        Types::Referee_info referee_info;                                   // 裁判系统信息

        void set_mode(Types::ROBOT_MODE set_mode) {                         // 设置机器人工作模式
            this->last_mode = this->mode;
            this->mode = set_mode;
        }

        bool mode_changed() {                                               // 检测机器人工作模式是否改变   
            if (this->last_mode != this->mode) {
                this->last_mode = this->mode;
                return true;
            }
            return false;
        }

        void sync_head() {
        }
    };

    // send gimbal package header = 0x5A;
    struct ReceiveGimbalPacket
    {
        /*      自瞄部分     */
        uint8_t header;
        bool tracking : 1;
        uint8_t id : 3;
        uint8_t armors_num : 3;
        uint8_t reserved : 1;
        int16_t sd_yaw;
        int16_t sd_pitch;
        int16_t sd_vx;
        int16_t sd_vy;
        uint8_t sd_a;
        uint8_t sd_b;
        int8_t sd_mx;
        int8_t sd_my;
        uint8_t sd_x;
        uint8_t sd_y;
        uint8_t sd_ltb;
        uint8_t sd_rtb;
        float x;
        float y;
        float z;
        float yaw;
        float vx;
        float vy;
        float vz;
        float v_yaw;
        float r1;
        float r2;
        float dz;
        /*  决策部分   */

    } __attribute__((packed));

    struct SendGimbalPacket
    {
        uint8_t header = 0x5A;
        uint8_t detect_color : 1;  // 0-red 1-blue
        bool reset_tracker : 1;
        uint8_t reserved : 6;
        float roll;
        float pitch;
        float yaw;
        float aim_x;
        float aim_y;
        float aim_z;
        uint16_t checksum = 0;
    } __attribute__((packed));

    // send twist package heaedr = 0x6A;
    struct Auto_aim_control
    {
        uint8_t header;
        float yaw_set;
        float pitch_set;
        bool fire;

        Types::ROBOT_MODE mode = Types::ROBOT_MODE::ROBOT_NO_FORCE;
    } __attribute__((packed));
 
    struct SendAutoAimInfo
    {
        uint8_t header;
        float yaw;
        float pitch;
        bool red;
    } __attribute__((packed));

    // not used currently
    struct SendVisionControl
    {
        uint8_t header = 0xA6;
        float roll;
        float pitch;
        float yaw;
    } __attribute__((packed));

    struct ReceiveNavigationInfo
    {
        uint8_t header;
        float vx;
        float vy;
    } __attribute__((packed));

    struct SendNavigationInfo
    {
        uint8_t header;
        float yaw;
        float pitch;
        float hp;
        bool start;
    } __attribute__((packed));

}  // namespace Robot

#endif
