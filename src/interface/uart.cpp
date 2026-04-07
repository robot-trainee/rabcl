#include "rabcl/interface/uart.hpp"

namespace rabcl
{
Uart::Uart()
{
  for (int i = 0; i < PACKET_SIZE; i++) {
    uart_receive_buffer_[i] = 0;
    uart_transmit_buffer_[i] = 0;
  }
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

bool Uart::UpdateData(Info & data)
{
  // header check
  if (uart_receive_buffer_[0] != HEADER_0 || uart_receive_buffer_[1] != HEADER_1) {
    return false;
  }

  // CRC check (payload bytes[2..25])
  uint8_t crc = CalcCrc8(uart_receive_buffer_, 2, 24);
  if (crc != uart_receive_buffer_[26]) {
    return false;
  }

  // parse payload
  union {
    float f;
    int32_t ui;
  } buf;

  // chassis_vel_x [2..5]
  buf.ui = (static_cast<int32_t>(uart_receive_buffer_[2]) << 24) |
    (static_cast<int32_t>(uart_receive_buffer_[3]) << 16) |
    (static_cast<int32_t>(uart_receive_buffer_[4]) << 8) |
    (static_cast<int32_t>(uart_receive_buffer_[5]));
  data.chassis_vel_x_ = buf.f;

  // chassis_vel_y [6..9]
  buf.ui = (static_cast<int32_t>(uart_receive_buffer_[6]) << 24) |
    (static_cast<int32_t>(uart_receive_buffer_[7]) << 16) |
    (static_cast<int32_t>(uart_receive_buffer_[8]) << 8) |
    (static_cast<int32_t>(uart_receive_buffer_[9]));
  data.chassis_vel_y_ = buf.f;

  // chassis_vel_z [10..13]
  buf.ui = (static_cast<int32_t>(uart_receive_buffer_[10]) << 24) |
    (static_cast<int32_t>(uart_receive_buffer_[11]) << 16) |
    (static_cast<int32_t>(uart_receive_buffer_[12]) << 8) |
    (static_cast<int32_t>(uart_receive_buffer_[13]));
  data.chassis_vel_z_ = buf.f;

  // yaw_pos [14..17]
  buf.ui = (static_cast<int32_t>(uart_receive_buffer_[14]) << 24) |
    (static_cast<int32_t>(uart_receive_buffer_[15]) << 16) |
    (static_cast<int32_t>(uart_receive_buffer_[16]) << 8) |
    (static_cast<int32_t>(uart_receive_buffer_[17]));
  data.yaw_pos_ = buf.f;

  // pitch_pos [18..21]
  buf.ui = (static_cast<int32_t>(uart_receive_buffer_[18]) << 24) |
    (static_cast<int32_t>(uart_receive_buffer_[19]) << 16) |
    (static_cast<int32_t>(uart_receive_buffer_[20]) << 8) |
    (static_cast<int32_t>(uart_receive_buffer_[21]));
  data.pitch_pos_ = buf.f;

  // modes [22..25]
  data.load_mode_ = uart_receive_buffer_[22];
  data.fire_mode_ = uart_receive_buffer_[23];
  data.speed_mode_ = uart_receive_buffer_[24];
  data.chassis_mode_ = uart_receive_buffer_[25];

  return true;
}

void Uart::PreparePacket(const Info & data)
{
  union {
    float f;
    int32_t ui;
  } buf;

  // header
  uart_transmit_buffer_[0] = HEADER_0;
  uart_transmit_buffer_[1] = HEADER_1;

  // chassis_vel_x [2..5]
  buf.f = data.chassis_vel_x_;
  uart_transmit_buffer_[2] = static_cast<uint8_t>((buf.ui >> 24) & 0xFF);
  uart_transmit_buffer_[3] = static_cast<uint8_t>((buf.ui >> 16) & 0xFF);
  uart_transmit_buffer_[4] = static_cast<uint8_t>((buf.ui >> 8) & 0xFF);
  uart_transmit_buffer_[5] = static_cast<uint8_t>((buf.ui >> 0) & 0xFF);

  // chassis_vel_y [6..9]
  buf.f = data.chassis_vel_y_;
  uart_transmit_buffer_[6] = static_cast<uint8_t>((buf.ui >> 24) & 0xFF);
  uart_transmit_buffer_[7] = static_cast<uint8_t>((buf.ui >> 16) & 0xFF);
  uart_transmit_buffer_[8] = static_cast<uint8_t>((buf.ui >> 8) & 0xFF);
  uart_transmit_buffer_[9] = static_cast<uint8_t>((buf.ui >> 0) & 0xFF);

  // chassis_vel_z [10..13]
  buf.f = data.chassis_vel_z_;
  uart_transmit_buffer_[10] = static_cast<uint8_t>((buf.ui >> 24) & 0xFF);
  uart_transmit_buffer_[11] = static_cast<uint8_t>((buf.ui >> 16) & 0xFF);
  uart_transmit_buffer_[12] = static_cast<uint8_t>((buf.ui >> 8) & 0xFF);
  uart_transmit_buffer_[13] = static_cast<uint8_t>((buf.ui >> 0) & 0xFF);

  // yaw_pos [14..17]
  buf.f = data.yaw_pos_;
  uart_transmit_buffer_[14] = static_cast<uint8_t>((buf.ui >> 24) & 0xFF);
  uart_transmit_buffer_[15] = static_cast<uint8_t>((buf.ui >> 16) & 0xFF);
  uart_transmit_buffer_[16] = static_cast<uint8_t>((buf.ui >> 8) & 0xFF);
  uart_transmit_buffer_[17] = static_cast<uint8_t>((buf.ui >> 0) & 0xFF);

  // pitch_pos [18..21]
  buf.f = data.pitch_pos_;
  uart_transmit_buffer_[18] = static_cast<uint8_t>((buf.ui >> 24) & 0xFF);
  uart_transmit_buffer_[19] = static_cast<uint8_t>((buf.ui >> 16) & 0xFF);
  uart_transmit_buffer_[20] = static_cast<uint8_t>((buf.ui >> 8) & 0xFF);
  uart_transmit_buffer_[21] = static_cast<uint8_t>((buf.ui >> 0) & 0xFF);

  // modes [22..25]
  uart_transmit_buffer_[22] = data.load_mode_;
  uart_transmit_buffer_[23] = data.fire_mode_;
  uart_transmit_buffer_[24] = data.speed_mode_;
  uart_transmit_buffer_[25] = data.chassis_mode_;

  // CRC8 (payload bytes[2..25])
  uart_transmit_buffer_[26] = CalcCrc8(uart_transmit_buffer_, 2, 24);

  // padding
  uart_transmit_buffer_[27] = 0x00;
}
}  // namespace rabcl
