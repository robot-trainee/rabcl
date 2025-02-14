#ifndef RABCL_UART_HPP
#define RABCL_UART_HPP

#include "rabcl/utils/type.hpp"

#include <cstdint>
#include <map>

namespace rabcl
{
class Uart
{
private:
    static constexpr int ID_CHASSIS_VEL_X = 1;
    static constexpr int ID_CHASSIS_VEL_Y = 2;
    static constexpr int ID_CHASSIS_VEL_Z = 3;
    static constexpr int ID_YAW_VEL = 4;
    static constexpr int ID_PITCH_VEL = 5;
    static constexpr int ID_MODES = 6;

public:
    Uart();
    ~Uart();
    bool UpdateData(Info& data);
    void PrepareFloatData(const uint8_t idx, const float data);
    void Prepare4IntData(const uint8_t idx, const uint8_t data[4]);

public:
    uint8_t uart_receive_buffer_[8];
    uint8_t uart_transmit_buffer_[8];
};
} // namespace rabcl

#endif
