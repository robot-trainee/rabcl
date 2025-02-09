#include "rabcl/interface/uart.hpp"

namespace rabcl
{
Uart::Uart()
{
    // NOP
}

Uart::~Uart()
{
    // NOP
}

bool Uart::UpdateData(uint8_t receive_buf[], Type& data)
{
    if (receive_buf[0] == 0xFF && receive_buf[6] == 0xFF)
    {
      int idx = (int)receive_buf[1];
      union {
          float f;
          int32_t ui;
      } buf;
      buf.ui = (int32_t) (
            (((int32_t)receive_buf[2] << 24) & 0xFF000000)
          | (((int32_t)receive_buf[3] << 16) & 0x00FF0000)
          | (((int32_t)receive_buf[4] <<  8) & 0x0000FF00)
          | (((int32_t)receive_buf[5] <<  0) & 0x000000FF)
      );

      if (idx == UART_ID::UART_CHASSIS_X)
      {
        data.chassis_vel_x_ = buf.f;
      }
      else if (idx == UART_ID::UART_CHASSIS_Y)
      {
        data.chassis_vel_y_ = buf.f;
      }
      else if (idx == UART_ID::UART_CHASSIS_Z)
      {
        data.chassis_vel_z_ = buf.f;
      }
      else if (idx == UART_ID::UART_YAW)
      {
        data.yaw_vel_ = buf.f;
      }
      else if (idx == UART_ID::UART_PITCH)
      {
        data.pitch_vel_ = buf.f;
      }
      else if (idx == UART_ID::UART_MODES)
      {
        data.load_mode_ = receive_buf[2 + (int)MODE_ID::LOAD];
        data.fire_mode_ = receive_buf[2 + (int)MODE_ID::FIRE];
        data.speed_mode_ = receive_buf[2 + (int)MODE_ID::SPEED];
        data.chassis_mode_ = receive_buf[2 + (int)MODE_ID::CHASSIS];
      }
      else
      {
        return false;
      }
      return true;
    }
    else
    {
        return false;
    }
}
} // namespace rabcl
