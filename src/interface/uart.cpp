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

bool Uart::UpdateData(Info& data)
{
    if (uart_receive_buffer_[0] == 0xFF && uart_receive_buffer_[6] == 0xFF)
    {
        int idx = (int)uart_receive_buffer_[1];
        union {
            float f;
            int32_t ui;
        } buf;
        buf.ui = (int32_t) (
            (((int32_t)uart_receive_buffer_[2] << 24) & 0xFF000000)
          | (((int32_t)uart_receive_buffer_[3] << 16) & 0x00FF0000)
          | (((int32_t)uart_receive_buffer_[4] <<  8) & 0x0000FF00)
          | (((int32_t)uart_receive_buffer_[5] <<  0) & 0x000000FF)
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
            data.load_mode_ = uart_receive_buffer_[2 + (int)MODE_ID::LOAD];
            data.fire_mode_ = uart_receive_buffer_[2 + (int)MODE_ID::FIRE];
            data.speed_mode_ = uart_receive_buffer_[2 + (int)MODE_ID::SPEED];
            data.chassis_mode_ = uart_receive_buffer_[2 + (int)MODE_ID::CHASSIS];
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

void Uart::PrepareFloatData(const uint8_t idx, const float data)
{
    // ---header
    uart_transmit_buffer_[0] = 0xFF;
    // ---index
    uart_transmit_buffer_[1] = idx;
    // ---data
    union {
        float f;
        int32_t ui;
    } buf;
    buf.f = data;
    uart_transmit_buffer_[2] = (uint8_t)((buf.ui & 0xFF000000) >> 24);
    uart_transmit_buffer_[3] = (uint8_t)((buf.ui & 0x00FF0000) >> 16);
    uart_transmit_buffer_[4] = (uint8_t)((buf.ui & 0x0000FF00) >> 8);
    uart_transmit_buffer_[5] = (uint8_t)((buf.ui & 0x000000FF) >> 0);
    // ---end
    uart_transmit_buffer_[6] = 0xFF;
    uart_transmit_buffer_[7] = 0x00;
}

void Uart::Prepare4IntData(const uint8_t idx, const uint8_t data[4])
{
    // ---header
    uart_transmit_buffer_[0] = 0xFF;
    // ---index
    uart_transmit_buffer_[1] = idx;
    // ---data
    uart_transmit_buffer_[2 + (int)MODE_ID::LOAD] = data[0];
    uart_transmit_buffer_[2 + (int)MODE_ID::FIRE] = data[1];
    uart_transmit_buffer_[2 + (int)MODE_ID::SPEED] = data[2];
    uart_transmit_buffer_[2 + (int)MODE_ID::CHASSIS] = data[3];
    // ---end
    uart_transmit_buffer_[6] = 0xFF;
    uart_transmit_buffer_[7] = 0x00;
}
} // namespace rabcl
