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
    bool UpdateData(uint8_t receive_buf[], Type& data);

public:
    uint8_t uart_receive_buffer_[8];
};
} // namespace rabcl

#endif
