#include <gtest/gtest.h>
#include <cstring>
#include "rabcl/interface/uart.hpp"
#include "rabcl/utils/type.hpp"

namespace rabcl
{

class UartTest : public ::testing::Test
{
protected:
  Uart uart_;

  // Info を設定して PreparePacket → 受信バッファにコピーするヘルパー
  void PrepareAndCopyToReceive(const Info & info)
  {
    uart_.PreparePacket(info);
    std::memcpy(uart_.uart_receive_buffer_, uart_.uart_transmit_buffer_, Uart::PACKET_SIZE);
  }
};

TEST_F(UartTest, PacketSizeIs28)
{
  EXPECT_EQ(Uart::PACKET_SIZE, 28);
}

TEST_F(UartTest, UpdateDataInvalidHeader0)
{
  Info info{};
  PrepareAndCopyToReceive(info);
  uart_.uart_receive_buffer_[0] = 0x00;
  Info data;
  EXPECT_FALSE(uart_.UpdateData(data));
}

TEST_F(UartTest, UpdateDataInvalidHeader1)
{
  Info info{};
  PrepareAndCopyToReceive(info);
  uart_.uart_receive_buffer_[1] = 0x00;
  Info data;
  EXPECT_FALSE(uart_.UpdateData(data));
}

TEST_F(UartTest, UpdateDataInvalidCrc)
{
  Info info{};
  PrepareAndCopyToReceive(info);
  uart_.uart_receive_buffer_[26] ^= 0xFF;  // CRC を壊す
  Info data;
  EXPECT_FALSE(uart_.UpdateData(data));
}

TEST_F(UartTest, RoundTripAllFields)
{
  Info src{};
  src.chassis_vel_x_ = 1.5f;
  src.chassis_vel_y_ = -2.3f;
  src.chassis_vel_z_ = 0.7f;
  src.yaw_pos_ = 3.14f;
  src.pitch_pos_ = -0.5f;
  src.load_mode_ = 1;
  src.fire_mode_ = 2;
  src.speed_mode_ = 0;
  src.chassis_mode_ = 1;

  PrepareAndCopyToReceive(src);

  Info dst{};
  EXPECT_TRUE(uart_.UpdateData(dst));
  EXPECT_FLOAT_EQ(dst.chassis_vel_x_, 1.5f);
  EXPECT_FLOAT_EQ(dst.chassis_vel_y_, -2.3f);
  EXPECT_FLOAT_EQ(dst.chassis_vel_z_, 0.7f);
  EXPECT_FLOAT_EQ(dst.yaw_pos_, 3.14f);
  EXPECT_FLOAT_EQ(dst.pitch_pos_, -0.5f);
  EXPECT_EQ(dst.load_mode_, 1);
  EXPECT_EQ(dst.fire_mode_, 2);
  EXPECT_EQ(dst.speed_mode_, 0);
  EXPECT_EQ(dst.chassis_mode_, 1);
}

TEST_F(UartTest, RoundTripZeroValues)
{
  Info src{};
  PrepareAndCopyToReceive(src);
  Info dst{};
  dst.chassis_vel_x_ = 999.0f;  // 上書きされることを確認
  EXPECT_TRUE(uart_.UpdateData(dst));
  EXPECT_FLOAT_EQ(dst.chassis_vel_x_, 0.0f);
  EXPECT_FLOAT_EQ(dst.yaw_pos_, 0.0f);
  EXPECT_EQ(dst.load_mode_, 0);
}

TEST_F(UartTest, PreparePacketHeader)
{
  Info info{};
  uart_.PreparePacket(info);
  EXPECT_EQ(uart_.uart_transmit_buffer_[0], Uart::HEADER_0);
  EXPECT_EQ(uart_.uart_transmit_buffer_[1], Uart::HEADER_1);
  EXPECT_EQ(uart_.uart_transmit_buffer_[27], 0x00);
}

TEST_F(UartTest, CalcCrc8)
{
  uint8_t data[4] = {0x01, 0x02, 0x04, 0x08};
  EXPECT_EQ(Uart::CalcCrc8(data, 0, 4), 0x01 ^ 0x02 ^ 0x04 ^ 0x08);
}

TEST_F(UartTest, CalcCrc8WithOffset)
{
  uint8_t data[6] = {0xAA, 0xBB, 0x01, 0x02, 0x04, 0x08};
  EXPECT_EQ(Uart::CalcCrc8(data, 2, 4), 0x01 ^ 0x02 ^ 0x04 ^ 0x08);
}

TEST_F(UartTest, CorruptedPayloadDetected)
{
  Info src{};
  src.chassis_vel_x_ = 1.0f;
  PrepareAndCopyToReceive(src);

  // ペイロードの1バイトを壊す
  uart_.uart_receive_buffer_[3] ^= 0x01;

  Info dst{};
  EXPECT_FALSE(uart_.UpdateData(dst));
}

// --- HandleRxComplete / HandleRxError tests ---

TEST_F(UartTest, HandleRxCompleteNormalPacket)
{
  Info src{};
  src.chassis_vel_x_ = 1.0f;
  PrepareAndCopyToReceive(src);
  Info dst{};
  auto result = uart_.HandleRxComplete(dst);
  EXPECT_TRUE(result.data_updated);
  EXPECT_FLOAT_EQ(dst.chassis_vel_x_, 1.0f);
  EXPECT_EQ(result.next_rx_size, Uart::PACKET_SIZE);
}

TEST_F(UartTest, HandleRxCompleteEntersResyncAfterThreshold)
{
  // send invalid packets to trigger resync
  Info dst{};
  uart_.uart_receive_buffer_[0] = 0x00;  // bad header
  for (uint8_t i = 0; i < Uart::RESYNC_THRESHOLD; i++) {
    auto result = uart_.HandleRxComplete(dst);
    EXPECT_FALSE(result.data_updated);
  }
  // after threshold, should be in scan mode (1 byte receive)
  auto result = uart_.HandleRxComplete(dst);
  EXPECT_EQ(result.next_rx_size, 1);
}

TEST_F(UartTest, HandleRxCompleteResyncFindsHeader)
{
  Info src{};
  src.chassis_vel_x_ = 2.5f;
  uart_.PreparePacket(src);

  // force into scan mode
  Info dst{};
  uart_.uart_receive_buffer_[0] = 0x00;
  Uart::RxResult r{};
  for (uint8_t i = 0; i < Uart::RESYNC_THRESHOLD + 1; i++) {
    r = uart_.HandleRxComplete(dst);
  }
  // now in RX_SCAN_H0 — r.next_rx_buf points to scan_buf_
  EXPECT_EQ(r.next_rx_size, 1);

  // simulate DMA writing HEADER_0 into the returned buffer
  r.next_rx_buf[0] = Uart::HEADER_0;
  auto r1 = uart_.HandleRxComplete(dst);  // scan_h0 → finds 0xA5 → scan_h1
  EXPECT_FALSE(r1.data_updated);
  EXPECT_EQ(r1.next_rx_size, 1);

  // simulate DMA writing HEADER_1
  r1.next_rx_buf[0] = Uart::HEADER_1;
  auto r2 = uart_.HandleRxComplete(dst);  // scan_h1 → finds 0x5A → scan_remain
  EXPECT_FALSE(r2.data_updated);
  EXPECT_EQ(r2.next_rx_size, Uart::PACKET_SIZE - 2);

  // simulate DMA writing remaining payload into &uart_receive_buffer_[2]
  std::memcpy(r2.next_rx_buf, &uart_.uart_transmit_buffer_[2], Uart::PACKET_SIZE - 2);
  auto r3 = uart_.HandleRxComplete(dst);  // scan_remain → parse success → back to packet mode
  EXPECT_TRUE(r3.data_updated);
  EXPECT_FLOAT_EQ(dst.chassis_vel_x_, 2.5f);
  EXPECT_EQ(r3.next_rx_size, Uart::PACKET_SIZE);
}

TEST_F(UartTest, HandleRxErrorEntersScanMode)
{
  auto result = uart_.HandleRxError();
  EXPECT_FALSE(result.data_updated);
  EXPECT_EQ(result.next_rx_size, 1);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
