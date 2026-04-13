#ifndef RABCL__INTERFACE__UART_HPP_
#define RABCL__INTERFACE__UART_HPP_

#include <cstdint>

#include "rabcl/utils/type.hpp"

namespace rabcl
{
class Uart
{
public:
  // --- Reference (ROS2 → STM32 command) ---
  static constexpr uint8_t REFERENCE_PACKET_SIZE = 28;
  static constexpr uint8_t REFERENCE_HEADER_0 = 0xA5;
  static constexpr uint8_t REFERENCE_HEADER_1 = 0x5A;
  static constexpr uint8_t RESYNC_THRESHOLD = 3;

  // --- Feedback (STM32 → ROS2 sensor/motor) ---
  static constexpr uint8_t FEEDBACK_PACKET_SIZE = 112;
  static constexpr uint8_t FEEDBACK_HEADER_0 = 0xB5;
  static constexpr uint8_t FEEDBACK_HEADER_1 = 0x5B;

  struct RxResult
  {
    bool data_updated;
    uint8_t * next_rx_buf;
    uint16_t next_rx_size;
  };

  Uart();
  ~Uart();

  // Reference packet
  bool UpdateReferenceData(Info & data);
  void PrepareReferencePacket(const Info & data);

  // Feedback packet
  void PrepareFeedbackPacket(const Info & data);
  bool UpdateFeedbackData(Info & data);

  // Common
  static uint8_t CalcCrc8(const uint8_t * buf, uint8_t offset, uint8_t len);

  // Reference resync
  RxResult HandleRxComplete(Info & data);
  RxResult HandleRxError();

  // Reference buffers
  uint8_t reference_receive_buffer_[REFERENCE_PACKET_SIZE];
  uint8_t reference_transmit_buffer_[REFERENCE_PACKET_SIZE];

  // Feedback buffers
  uint8_t feedback_transmit_buffer_[FEEDBACK_PACKET_SIZE];
  uint8_t feedback_receive_buffer_[FEEDBACK_PACKET_SIZE];

private:
  static void EncodeFloat(uint8_t * buf, float val);
  static float DecodeFloat(const uint8_t * buf);

  enum RxState { RX_PACKET, RX_SCAN_H0, RX_SCAN_H1, RX_SCAN_REMAIN };
  volatile RxState rx_state_;
  uint8_t fail_count_;
  uint8_t scan_buf_[1];
};
}  // namespace rabcl

#endif  // RABCL__INTERFACE__UART_HPP_
