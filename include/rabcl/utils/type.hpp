#ifndef RABCL_TYPE_HPP
#define RABCL_TYPE_HPP

#include <cstdint>

namespace rabcl
{
enum UART_ID
{
    UART_CHASSIS_X = 1,
    UART_CHASSIS_Y,
    UART_CHASSIS_Z,
    UART_YAW,
    UART_PITCH,
    UART_MODES
};

enum CAN_ID
{
    CAN_CHASSIS_X_Y = 71,
    CAN_CHASSIS_Z_YAW,
    CAN_PITCH_MODES,
    CAN_FRONT_RIGHT,
    CAN_FRONT_LEFT,
    CAN_BACK_RIGHT,
    CAN_BACK_LEFT,
    CAN_YAW
};

enum MODE_ID
{
    LOAD,
    FIRE,
    SPEED,
    CHASSIS
};

class Type
{
public:
    float chassis_vel_x_;
    float chassis_vel_y_;
    float chassis_vel_z_;
    float yaw_vel_;
    float pitch_vel_;
    uint8_t load_mode_; // 0:Stop, 1:Forward, 2:Reverse
    uint8_t fire_mode_; // 0:Stop, 1:Low, 2:High
    uint8_t speed_mode_; // 0:Low, 1:High
    uint8_t chassis_mode_; // 0:Nomal, 1:InfiniteRot

    // ちびMDのCAN通信用
    // float front_right_cmd_;
    // float front_right_act_;
    // float front_left_cmd_;
    // float front_left_act_;
    // float back_right_cmd_;
    // float back_right_act_;
    // float back_left_cmd_;
    // float back_left_act_;
    // float yaw_cmd_;
    // float yaw_act_;
};
} // namespace rabcl

#endif
