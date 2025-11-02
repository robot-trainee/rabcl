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
    CAN_CHASSIS_X_Y = 0x711,
    CAN_CHASSIS_Z_YAW,
    CAN_PITCH_MODES,
    CAN_YAW_CMD_ACT,
    CAN_CHASSIS_FRONT_CMD,
    CAN_CHASSIS_BACK_CMD,
    CAN_CHASSIS_FRONT_ACT,
    CAN_CHASSIS_BACK_ACT,
};

enum MODE_ID
{
    LOAD,
    FIRE,
    SPEED,
    CHASSIS
};

class Info
{
public:
    // reference
    float chassis_vel_x_;
    float chassis_vel_y_;
    float chassis_vel_z_;
    float yaw_pos_;
    float pitch_pos_;
    uint8_t load_mode_; // 0:Stop, 1:Forward, 2:Reverse
    uint8_t fire_mode_; // 0:Stop, 1:Low, 2:High
    uint8_t speed_mode_; // 0:Low, 1:High
    uint8_t chassis_mode_; // 0:Nomal, 1:InfiniteRot

    // command
    float yaw_pos_cmd_;
    float front_right_vel_cmd_;
    float front_left_vel_cmd_;
    float back_right_vel_cmd_;
    float back_left_vel_cmd_;

    // actual
    float yaw_pos_act_;
    float front_right_vel_act_;
    float front_left_vel_act_;
    float back_right_vel_act_;
    float back_left_vel_act_;
};
} // namespace rabcl

#endif
