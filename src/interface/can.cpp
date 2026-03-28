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

bool Can::UpdateData(uint32_t idx, const uint8_t can_data[8], Info & data)
{
  union {
    float f;
    int32_t ui;
  } buf;
  if (idx == static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_X_Y)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF)
    );
    data.chassis_vel_x_ = buf.f;
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[4]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[5]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[6]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[7]) << 0) & 0x000000FF)
    );
    data.chassis_vel_y_ = buf.f;
  } else if (idx == static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_Z_YAW)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF)
    );
    data.chassis_vel_z_ = buf.f;
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[4]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[5]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[6]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[7]) << 0) & 0x000000FF)
    );
    data.yaw_pos_ = buf.f;
  } else if (idx == static_cast<uint32_t>(CAN_ID::CAN_PITCH_MODES)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF)
    );
    data.pitch_pos_ = buf.f;
    data.load_mode_ = can_data[4 + static_cast<int>(MODE_ID::LOAD)];
    data.fire_mode_ = can_data[4 + static_cast<int>(MODE_ID::FIRE)];
    data.speed_mode_ = can_data[4 + static_cast<int>(MODE_ID::SPEED)];
    data.chassis_mode_ = can_data[4 + static_cast<int>(MODE_ID::CHASSIS)];
  } else if (idx == static_cast<uint32_t>(CAN_ID::CAN_YAW_CMD_ACT)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF)
    );
    data.yaw_pos_cmd_ = buf.f;
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[4]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[5]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[6]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[7]) << 0) & 0x000000FF)
    );
    data.yaw_pos_act_ = buf.f;
  } else if (idx == static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_FRONT_CMD)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF)
    );
    data.front_right_vel_cmd_ = buf.f;
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[4]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[5]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[6]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[7]) << 0) & 0x000000FF)
    );
    data.front_left_vel_cmd_ = buf.f;
  } else if (idx == static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_BACK_CMD)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF)
    );
    data.back_right_vel_cmd_ = buf.f;
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[4]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[5]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[6]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[7]) << 0) & 0x000000FF)
    );
    data.back_left_vel_cmd_ = buf.f;
  } else if (idx == static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_FRONT_ACT)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF)
    );
    data.front_right_vel_act_ = buf.f;
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[4]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[5]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[6]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[7]) << 0) & 0x000000FF)
    );
    data.front_left_vel_act_ = buf.f;
  } else if (idx == static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_BACK_ACT)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF)
    );
    data.back_right_vel_act_ = buf.f;
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[4]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[5]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[6]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[7]) << 0) & 0x000000FF)
    );
    data.back_left_vel_act_ = buf.f;
  } else if (idx == static_cast<uint32_t>(CAN_ID::PITCH_RX)) {
    data.command_byte_ = can_data[0];
    data.temperature_ = can_data[1];
    buf.ui = static_cast<int16_t>(
      ((static_cast<int16_t>(can_data[2]) << 8) & 0xFF00) |
      ((static_cast<int16_t>(can_data[3]) << 0) & 0x00FF)
    );
    data.torque_ = buf.f;
    buf.ui = static_cast<int16_t>(
      ((static_cast<int16_t>(can_data[4]) << 8) & 0xFF00) |
      ((static_cast<int16_t>(can_data[5]) << 0) & 0x00FF)
    );
    data.speed_ = buf.f;
    buf.ui = static_cast<int16_t>(
      ((static_cast<int16_t>(can_data[6]) << 8) & 0xFF00) |
      ((static_cast<int16_t>(can_data[7]) << 0) & 0x00FF)
    );
    data.position_ = buf.f;
  } else {
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
  can_data[0] = static_cast<uint8_t>((buf.ui & 0xFF000000) >> 24);
  can_data[1] = static_cast<uint8_t>((buf.ui & 0x00FF0000) >> 16);
  can_data[2] = static_cast<uint8_t>((buf.ui & 0x0000FF00) >> 8);
  can_data[3] = static_cast<uint8_t>((buf.ui & 0x000000FF) >> 0);
  buf.f = in_2;
  can_data[4] = static_cast<uint8_t>((buf.ui & 0xFF000000) >> 24);
  can_data[5] = static_cast<uint8_t>((buf.ui & 0x00FF0000) >> 16);
  can_data[6] = static_cast<uint8_t>((buf.ui & 0x0000FF00) >> 8);
  can_data[7] = static_cast<uint8_t>((buf.ui & 0x000000FF) >> 0);
}

void Can::Prepare1Float4IntData(float in_1, const uint8_t in_2[4], uint8_t can_data[8])
{
  union {
    float f;
    int32_t ui;
  } buf;
  buf.f = in_1;
  can_data[0] = static_cast<uint8_t>((buf.ui & 0xFF000000) >> 24);
  can_data[1] = static_cast<uint8_t>((buf.ui & 0x00FF0000) >> 16);
  can_data[2] = static_cast<uint8_t>((buf.ui & 0x0000FF00) >> 8);
  can_data[3] = static_cast<uint8_t>((buf.ui & 0x000000FF) >> 0);
  can_data[4] = in_2[0];
  can_data[5] = in_2[1];
  can_data[6] = in_2[2];
  can_data[7] = in_2[3];
}

void Can::PrepareLKMotorPositionCmd(int32_t pos, uint16_t max_speed, uint8_t can_data[8])
{
  can_data[0] = 0xA4;
  can_data[1] = 0x00;
  can_data[2] = static_cast<uint8_t>( max_speed & 0xFF);
  can_data[3] = static_cast<uint8_t>((max_speed >> 8 ) & 0xFF);
  can_data[4] = static_cast<uint8_t>( pos & 0xFF);
  can_data[5] = static_cast<uint8_t>((pos >> 8 ) & 0xFF);
  can_data[6] = static_cast<uint8_t>((pos >> 16) & 0xFF);
  can_data[7] = static_cast<uint8_t>((pos >> 24) & 0xFF);
}
}  // namespace rabcl
