#ifndef RABCL__INTERFACE__UART_HPP_
#define RABCL__INTERFACE__UART_HPP_

#include <cstdint>

#include "rabcl/utils/type.hpp"

namespace rabcl
{
class Uart
{
public:
  static constexpr uint8_t PACKET_SIZE = 28;
  static constexpr uint8_t HEADER_0 = 0xA5;
  static constexpr uint8_t HEADER_1 = 0x5A;

  Uart();
  ~Uart();
  bool UpdateData(Info & data);
  void PreparePacket(const Info & data);
  static uint8_t CalcCrc8(const uint8_t * buf, uint8_t offset, uint8_t len);

  uint8_t uart_receive_buffer_[PACKET_SIZE];
  uint8_t uart_transmit_buffer_[PACKET_SIZE];
};
}  // namespace rabcl

#endif  // RABCL__INTERFACE__UART_HPP_
