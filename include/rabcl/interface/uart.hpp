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
  static constexpr uint8_t RESYNC_THRESHOLD = 3;

  struct RxResult
  {
    bool data_updated;
    uint8_t * next_rx_buf;
    uint16_t next_rx_size;
  };

  Uart();
  ~Uart();
  bool UpdateData(Info & data);
  void PreparePacket(const Info & data);
  static uint8_t CalcCrc8(const uint8_t * buf, uint8_t offset, uint8_t len);

  RxResult HandleRxComplete(Info & data);
  RxResult HandleRxError();

  uint8_t uart_receive_buffer_[PACKET_SIZE];
  uint8_t uart_transmit_buffer_[PACKET_SIZE];

private:
  enum RxState { RX_PACKET, RX_SCAN_H0, RX_SCAN_H1, RX_SCAN_REMAIN };
  volatile RxState rx_state_;
  uint8_t fail_count_;
  uint8_t scan_buf_[1];
};
}  // namespace rabcl

#endif  // RABCL__INTERFACE__UART_HPP_
