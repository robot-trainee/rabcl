#ifndef RABCL_UART_HPP
#define RABCL_UART_HPP

#include "rabcl/utils/type.hpp"

#include <cstdint>

namespace rabcl
{
class Uart
{
public:
    Uart();
    ~Uart();
    bool UpdateData(Info& data);
    void PrepareFloatData(uint8_t idx, float data);
    void Prepare4IntData(uint8_t idx, const uint8_t data[4]);

public:
    uint8_t uart_receive_buffer_[8];
    uint8_t uart_transmit_buffer_[8];
};
} // namespace rabcl

#endif
