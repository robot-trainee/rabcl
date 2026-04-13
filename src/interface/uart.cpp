#include "rabcl/interface/uart.hpp"

namespace rabcl
{
Uart::Uart()
: rx_state_(RX_PACKET), fail_count_(0)
{
  for (int i = 0; i < REFERENCE_PACKET_SIZE; i++) {
    reference_receive_buffer_[i] = 0;
    reference_transmit_buffer_[i] = 0;
  }
  for (int i = 0; i < FEEDBACK_PACKET_SIZE; i++) {
    feedback_transmit_buffer_[i] = 0;
    feedback_receive_buffer_[i] = 0;
  }
  scan_buf_[0] = 0;
}

Uart::~Uart()
{
  // NOP
}

uint8_t Uart::CalcCrc8(const uint8_t * buf, uint8_t offset, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[offset + i];
  }
  return crc;
}

void Uart::EncodeFloat(uint8_t * buf, float val)
{
  union {
    float f;
    int32_t ui;
  } conv;
  conv.f = val;
  buf[0] = static_cast<uint8_t>((conv.ui >> 24) & 0xFF);
  buf[1] = static_cast<uint8_t>((conv.ui >> 16) & 0xFF);
  buf[2] = static_cast<uint8_t>((conv.ui >> 8) & 0xFF);
  buf[3] = static_cast<uint8_t>((conv.ui >> 0) & 0xFF);
}

float Uart::DecodeFloat(const uint8_t * buf)
{
  union {
    float f;
    int32_t ui;
  } conv;
  conv.ui = (static_cast<int32_t>(buf[0]) << 24) |
    (static_cast<int32_t>(buf[1]) << 16) |
    (static_cast<int32_t>(buf[2]) << 8) |
    (static_cast<int32_t>(buf[3]));
  return conv.f;
}

// ============================================================
// Reference packet (ROS2 → STM32)
// ============================================================

bool Uart::UpdateReferenceData(Info & data)
{
  if (reference_receive_buffer_[0] != REFERENCE_HEADER_0 ||
    reference_receive_buffer_[1] != REFERENCE_HEADER_1)
  {
    return false;
  }

  uint8_t crc = CalcCrc8(reference_receive_buffer_, 2, 24);
  if (crc != reference_receive_buffer_[26]) {
    return false;
  }

  data.chassis_vel_x_ = DecodeFloat(&reference_receive_buffer_[2]);
  data.chassis_vel_y_ = DecodeFloat(&reference_receive_buffer_[6]);
  data.chassis_vel_z_ = DecodeFloat(&reference_receive_buffer_[10]);
  data.yaw_pos_ = DecodeFloat(&reference_receive_buffer_[14]);
  data.pitch_pos_ = DecodeFloat(&reference_receive_buffer_[18]);

  data.load_mode_ = reference_receive_buffer_[22];
  data.fire_mode_ = reference_receive_buffer_[23];
  data.speed_mode_ = reference_receive_buffer_[24];
  data.chassis_mode_ = reference_receive_buffer_[25];

  return true;
}

void Uart::PrepareReferencePacket(const Info & data)
{
  reference_transmit_buffer_[0] = REFERENCE_HEADER_0;
  reference_transmit_buffer_[1] = REFERENCE_HEADER_1;

  EncodeFloat(&reference_transmit_buffer_[2], data.chassis_vel_x_);
  EncodeFloat(&reference_transmit_buffer_[6], data.chassis_vel_y_);
  EncodeFloat(&reference_transmit_buffer_[10], data.chassis_vel_z_);
  EncodeFloat(&reference_transmit_buffer_[14], data.yaw_pos_);
  EncodeFloat(&reference_transmit_buffer_[18], data.pitch_pos_);

  reference_transmit_buffer_[22] = data.load_mode_;
  reference_transmit_buffer_[23] = data.fire_mode_;
  reference_transmit_buffer_[24] = data.speed_mode_;
  reference_transmit_buffer_[25] = data.chassis_mode_;

  reference_transmit_buffer_[26] = CalcCrc8(reference_transmit_buffer_, 2, 24);
  reference_transmit_buffer_[27] = 0x00;
}

Uart::RxResult Uart::HandleRxComplete(Info & data)
{
  RxResult result = {false, nullptr, 0};

  switch (rx_state_) {
    case RX_PACKET:
      if (UpdateReferenceData(data)) {
        fail_count_ = 0;
        result.data_updated = true;
        result.next_rx_buf = reference_receive_buffer_;
        result.next_rx_size = REFERENCE_PACKET_SIZE;
      } else {
        fail_count_++;
        if (fail_count_ >= RESYNC_THRESHOLD) {
          rx_state_ = RX_SCAN_H0;
          result.next_rx_buf = scan_buf_;
          result.next_rx_size = 1;
        } else {
          result.next_rx_buf = reference_receive_buffer_;
          result.next_rx_size = REFERENCE_PACKET_SIZE;
        }
      }
      break;

    case RX_SCAN_H0:
      if (scan_buf_[0] == REFERENCE_HEADER_0) {
        rx_state_ = RX_SCAN_H1;
      }
      result.next_rx_buf = scan_buf_;
      result.next_rx_size = 1;
      break;

    case RX_SCAN_H1:
      if (scan_buf_[0] == REFERENCE_HEADER_1) {
        reference_receive_buffer_[0] = REFERENCE_HEADER_0;
        reference_receive_buffer_[1] = REFERENCE_HEADER_1;
        rx_state_ = RX_SCAN_REMAIN;
        result.next_rx_buf = &reference_receive_buffer_[2];
        result.next_rx_size = REFERENCE_PACKET_SIZE - 2;
      } else {
        rx_state_ = (scan_buf_[0] == REFERENCE_HEADER_0) ? RX_SCAN_H1 : RX_SCAN_H0;
        result.next_rx_buf = scan_buf_;
        result.next_rx_size = 1;
      }
      break;

    case RX_SCAN_REMAIN:
      if (UpdateReferenceData(data)) {
        fail_count_ = 0;
        result.data_updated = true;
      }
      rx_state_ = RX_PACKET;
      result.next_rx_buf = reference_receive_buffer_;
      result.next_rx_size = REFERENCE_PACKET_SIZE;
      break;
  }

  return result;
}

Uart::RxResult Uart::HandleRxError()
{
  rx_state_ = RX_SCAN_H0;
  fail_count_ = 0;
  return {false, scan_buf_, 1};
}

// ============================================================
// Feedback packet (STM32 → ROS2)
// ============================================================

void Uart::PrepareFeedbackPacket(const Info & data)
{
  feedback_transmit_buffer_[0] = FEEDBACK_HEADER_0;
  feedback_transmit_buffer_[1] = FEEDBACK_HEADER_1;

  uint8_t offset = 2;

  // IMU (9 floats) [2..37]
  EncodeFloat(&feedback_transmit_buffer_[offset], data.imu_.acc_x_);       offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.imu_.acc_y_);       offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.imu_.acc_z_);       offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.imu_.gyro_x_);      offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.imu_.gyro_y_);      offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.imu_.gyro_z_);      offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.imu_.euler_heading_); offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.imu_.euler_roll_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.imu_.euler_pitch_); offset += 4;

  // YAW (3 floats) [38..49]
  EncodeFloat(&feedback_transmit_buffer_[offset], data.yaw_act_.position_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.yaw_act_.velocity_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.yaw_act_.current_);   offset += 4;

  // PITCH (3 floats) [50..61]
  EncodeFloat(&feedback_transmit_buffer_[offset], data.pitch_act_.position_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.pitch_act_.velocity_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.pitch_act_.current_);   offset += 4;

  // Chassis FR (3 floats) [62..73]
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_fr_act_.position_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_fr_act_.velocity_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_fr_act_.torque_);    offset += 4;

  // Chassis FL (3 floats) [74..85]
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_fl_act_.position_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_fl_act_.velocity_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_fl_act_.torque_);    offset += 4;

  // Chassis BR (3 floats) [86..97]
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_br_act_.position_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_br_act_.velocity_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_br_act_.torque_);    offset += 4;

  // Chassis BL (3 floats) [98..109]
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_bl_act_.position_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_bl_act_.velocity_);  offset += 4;
  EncodeFloat(&feedback_transmit_buffer_[offset], data.chassis_bl_act_.torque_);    offset += 4;

  // CRC8 [110]
  feedback_transmit_buffer_[110] = CalcCrc8(feedback_transmit_buffer_, 2, 108);

  // Padding [111]
  feedback_transmit_buffer_[111] = 0x00;
}

bool Uart::UpdateFeedbackData(Info & data)
{
  if (feedback_receive_buffer_[0] != FEEDBACK_HEADER_0 ||
    feedback_receive_buffer_[1] != FEEDBACK_HEADER_1)
  {
    return false;
  }

  uint8_t crc = CalcCrc8(feedback_receive_buffer_, 2, 108);
  if (crc != feedback_receive_buffer_[110]) {
    return false;
  }

  uint8_t offset = 2;

  // IMU
  data.imu_.acc_x_ = DecodeFloat(&feedback_receive_buffer_[offset]);         offset += 4;
  data.imu_.acc_y_ = DecodeFloat(&feedback_receive_buffer_[offset]);         offset += 4;
  data.imu_.acc_z_ = DecodeFloat(&feedback_receive_buffer_[offset]);         offset += 4;
  data.imu_.gyro_x_ = DecodeFloat(&feedback_receive_buffer_[offset]);        offset += 4;
  data.imu_.gyro_y_ = DecodeFloat(&feedback_receive_buffer_[offset]);        offset += 4;
  data.imu_.gyro_z_ = DecodeFloat(&feedback_receive_buffer_[offset]);        offset += 4;
  data.imu_.euler_heading_ = DecodeFloat(&feedback_receive_buffer_[offset]); offset += 4;
  data.imu_.euler_roll_ = DecodeFloat(&feedback_receive_buffer_[offset]);    offset += 4;
  data.imu_.euler_pitch_ = DecodeFloat(&feedback_receive_buffer_[offset]);   offset += 4;

  // YAW
  data.yaw_act_.position_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.yaw_act_.velocity_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.yaw_act_.current_ = DecodeFloat(&feedback_receive_buffer_[offset]);   offset += 4;

  // PITCH
  data.pitch_act_.position_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.pitch_act_.velocity_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.pitch_act_.current_ = DecodeFloat(&feedback_receive_buffer_[offset]);   offset += 4;

  // Chassis FR
  data.chassis_fr_act_.position_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.chassis_fr_act_.velocity_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.chassis_fr_act_.torque_ = DecodeFloat(&feedback_receive_buffer_[offset]);    offset += 4;

  // Chassis FL
  data.chassis_fl_act_.position_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.chassis_fl_act_.velocity_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.chassis_fl_act_.torque_ = DecodeFloat(&feedback_receive_buffer_[offset]);    offset += 4;

  // Chassis BR
  data.chassis_br_act_.position_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.chassis_br_act_.velocity_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.chassis_br_act_.torque_ = DecodeFloat(&feedback_receive_buffer_[offset]);    offset += 4;

  // Chassis BL
  data.chassis_bl_act_.position_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.chassis_bl_act_.velocity_ = DecodeFloat(&feedback_receive_buffer_[offset]);  offset += 4;
  data.chassis_bl_act_.torque_ = DecodeFloat(&feedback_receive_buffer_[offset]);    offset += 4;

  return true;
}

}  // namespace rabcl
