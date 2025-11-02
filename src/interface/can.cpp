#include "rabcl/interface/can.hpp"

namespace rabcl
{
Can::Can()
{
    // NOP
}

Can::~Can()
{
    // NOP
}

bool Can::UpdateData(uint32_t idx, const uint8_t can_data[8], Info& data)
{
    union {
        float f;
        int32_t ui;
    } buf;
    if (idx == (uint32_t)CAN_ID::CAN_CHASSIS_X_Y)
    {
        buf.ui = (int32_t) (
            (((int32_t)can_data[0] << 24) & 0xFF000000)
          | (((int32_t)can_data[1] << 16) & 0x00FF0000)
          | (((int32_t)can_data[2] <<  8) & 0x0000FF00)
          | (((int32_t)can_data[3] <<  0) & 0x000000FF)
        );
        data.chassis_vel_x_ = buf.f;
        buf.ui = (int32_t) (
              (((int32_t)can_data[4] << 24) & 0xFF000000)
            | (((int32_t)can_data[5] << 16) & 0x00FF0000)
            | (((int32_t)can_data[6] <<  8) & 0x0000FF00)
            | (((int32_t)can_data[7] <<  0) & 0x000000FF)
        );
        data.chassis_vel_y_ = buf.f;
    }
    else if (idx == (uint32_t)CAN_ID::CAN_CHASSIS_Z_YAW)
    {
        buf.ui = (int32_t) (
            (((int32_t)can_data[0] << 24) & 0xFF000000)
          | (((int32_t)can_data[1] << 16) & 0x00FF0000)
          | (((int32_t)can_data[2] <<  8) & 0x0000FF00)
          | (((int32_t)can_data[3] <<  0) & 0x000000FF)
        );
        data.chassis_vel_z_ = buf.f;
        buf.ui = (int32_t) (
              (((int32_t)can_data[4] << 24) & 0xFF000000)
            | (((int32_t)can_data[5] << 16) & 0x00FF0000)
            | (((int32_t)can_data[6] <<  8) & 0x0000FF00)
            | (((int32_t)can_data[7] <<  0) & 0x000000FF)
        );
        data.yaw_pos_ = buf.f;
    }
    else if (idx == (uint32_t)CAN_ID::CAN_PITCH_MODES)
    {
        buf.ui = (int32_t) (
            (((int32_t)can_data[0] << 24) & 0xFF000000)
          | (((int32_t)can_data[1] << 16) & 0x00FF0000)
          | (((int32_t)can_data[2] <<  8) & 0x0000FF00)
          | (((int32_t)can_data[3] <<  0) & 0x000000FF)
        );
        data.pitch_pos_ = buf.f;
        data.load_mode_ = can_data[4 + (int)MODE_ID::LOAD];
        data.fire_mode_ = can_data[4 + (int)MODE_ID::FIRE];
        data.speed_mode_ = can_data[4 + (int)MODE_ID::SPEED];
        data.chassis_mode_ = can_data[4 + (int)MODE_ID::CHASSIS];
    }
    else
    {
        return false;
    }
    return true;
}

void Can::Prepare2FloatData(float in_1, float in_2, uint8_t can_data[8])
{
    union {
        float f;
        int32_t ui;
    } buf;
    buf.f = in_1;
    can_data[0] = (uint8_t)((buf.ui & 0xFF000000) >> 24);
    can_data[1] = (uint8_t)((buf.ui & 0x00FF0000) >> 16);
    can_data[2] = (uint8_t)((buf.ui & 0x0000FF00) >> 8);
    can_data[3] = (uint8_t)((buf.ui & 0x000000FF) >> 0);
    buf.f = in_2;
    can_data[4] = (uint8_t)((buf.ui & 0xFF000000) >> 24);
    can_data[5] = (uint8_t)((buf.ui & 0x00FF0000) >> 16);
    can_data[6] = (uint8_t)((buf.ui & 0x0000FF00) >> 8);
    can_data[7] = (uint8_t)((buf.ui & 0x000000FF) >> 0);
}

void Can::Prepare1Float4IntData(float in_1, const uint8_t in_2[4], uint8_t can_data[8])
{
    union {
        float f;
        int32_t ui;
    } buf;
    buf.f = in_1;
    can_data[0] = (uint8_t)((buf.ui & 0xFF000000) >> 24);
    can_data[1] = (uint8_t)((buf.ui & 0x00FF0000) >> 16);
    can_data[2] = (uint8_t)((buf.ui & 0x0000FF00) >> 8);
    can_data[3] = (uint8_t)((buf.ui & 0x000000FF) >> 0);
    can_data[4] = in_2[0];
    can_data[5] = in_2[1];
    can_data[6] = in_2[2];
    can_data[7] = in_2[3];
}
} // namespace rabcl
