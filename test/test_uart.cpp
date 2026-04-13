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

  void PrepareAndCopyToReceive(const Info & info)
  {
    uart_.PrepareReferencePacket(info);
    std::memcpy(
      uart_.reference_receive_buffer_, uart_.reference_transmit_buffer_,
      Uart::REFERENCE_PACKET_SIZE);
  }

  void PrepareAndCopyFeedback(const Info & info)
  {
    uart_.PrepareFeedbackPacket(info);
    std::memcpy(
      uart_.feedback_receive_buffer_, uart_.feedback_transmit_buffer_,
      Uart::FEEDBACK_PACKET_SIZE);
  }
};

// ============================================================
// Reference packet tests
// ============================================================

TEST_F(UartTest, ReferencePacketSizeIs28)
{
  EXPECT_EQ(Uart::REFERENCE_PACKET_SIZE, 28);
}

TEST_F(UartTest, UpdateReferenceDataInvalidHeader0)
{
  Info info{};
  PrepareAndCopyToReceive(info);
  uart_.reference_receive_buffer_[0] = 0x00;
  Info data;
  EXPECT_FALSE(uart_.UpdateReferenceData(data));
}

TEST_F(UartTest, UpdateReferenceDataInvalidHeader1)
{
  Info info{};
  PrepareAndCopyToReceive(info);
  uart_.reference_receive_buffer_[1] = 0x00;
  Info data;
  EXPECT_FALSE(uart_.UpdateReferenceData(data));
}

TEST_F(UartTest, UpdateReferenceDataInvalidCrc)
{
  Info info{};
  PrepareAndCopyToReceive(info);
  uart_.reference_receive_buffer_[26] ^= 0xFF;
  Info data;
  EXPECT_FALSE(uart_.UpdateReferenceData(data));
}

TEST_F(UartTest, ReferenceRoundTripAllFields)
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
  EXPECT_TRUE(uart_.UpdateReferenceData(dst));
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

TEST_F(UartTest, ReferenceRoundTripZeroValues)
{
  Info src{};
  PrepareAndCopyToReceive(src);
  Info dst{};
  dst.chassis_vel_x_ = 999.0f;
  EXPECT_TRUE(uart_.UpdateReferenceData(dst));
  EXPECT_FLOAT_EQ(dst.chassis_vel_x_, 0.0f);
  EXPECT_FLOAT_EQ(dst.yaw_pos_, 0.0f);
  EXPECT_EQ(dst.load_mode_, 0);
}

TEST_F(UartTest, PrepareReferencePacketHeader)
{
  Info info{};
  uart_.PrepareReferencePacket(info);
  EXPECT_EQ(uart_.reference_transmit_buffer_[0], Uart::REFERENCE_HEADER_0);
  EXPECT_EQ(uart_.reference_transmit_buffer_[1], Uart::REFERENCE_HEADER_1);
  EXPECT_EQ(uart_.reference_transmit_buffer_[27], 0x00);
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

TEST_F(UartTest, ReferenceCorruptedPayloadDetected)
{
  Info src{};
  src.chassis_vel_x_ = 1.0f;
  PrepareAndCopyToReceive(src);
  uart_.reference_receive_buffer_[3] ^= 0x01;
  Info dst{};
  EXPECT_FALSE(uart_.UpdateReferenceData(dst));
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
  EXPECT_EQ(result.next_rx_size, Uart::REFERENCE_PACKET_SIZE);
}

TEST_F(UartTest, HandleRxCompleteEntersResyncAfterThreshold)
{
  Info dst{};
  uart_.reference_receive_buffer_[0] = 0x00;
  for (uint8_t i = 0; i < Uart::RESYNC_THRESHOLD; i++) {
    auto result = uart_.HandleRxComplete(dst);
    EXPECT_FALSE(result.data_updated);
  }
  auto result = uart_.HandleRxComplete(dst);
  EXPECT_EQ(result.next_rx_size, 1);
}

TEST_F(UartTest, HandleRxCompleteResyncFindsHeader)
{
  Info src{};
  src.chassis_vel_x_ = 2.5f;
  uart_.PrepareReferencePacket(src);

  Info dst{};
  uart_.reference_receive_buffer_[0] = 0x00;
  Uart::RxResult r{};
  for (uint8_t i = 0; i < Uart::RESYNC_THRESHOLD + 1; i++) {
    r = uart_.HandleRxComplete(dst);
  }
  EXPECT_EQ(r.next_rx_size, 1);

  r.next_rx_buf[0] = Uart::REFERENCE_HEADER_0;
  auto r1 = uart_.HandleRxComplete(dst);
  EXPECT_FALSE(r1.data_updated);
  EXPECT_EQ(r1.next_rx_size, 1);

  r1.next_rx_buf[0] = Uart::REFERENCE_HEADER_1;
  auto r2 = uart_.HandleRxComplete(dst);
  EXPECT_FALSE(r2.data_updated);
  EXPECT_EQ(r2.next_rx_size, Uart::REFERENCE_PACKET_SIZE - 2);

  std::memcpy(r2.next_rx_buf, &uart_.reference_transmit_buffer_[2],
    Uart::REFERENCE_PACKET_SIZE - 2);
  auto r3 = uart_.HandleRxComplete(dst);
  EXPECT_TRUE(r3.data_updated);
  EXPECT_FLOAT_EQ(dst.chassis_vel_x_, 2.5f);
  EXPECT_EQ(r3.next_rx_size, Uart::REFERENCE_PACKET_SIZE);
}

TEST_F(UartTest, HandleRxErrorEntersScanMode)
{
  auto result = uart_.HandleRxError();
  EXPECT_FALSE(result.data_updated);
  EXPECT_EQ(result.next_rx_size, 1);
}

// ============================================================
// Feedback packet tests
// ============================================================

TEST_F(UartTest, FeedbackPacketSizeIs112)
{
  EXPECT_EQ(Uart::FEEDBACK_PACKET_SIZE, 112);
}

TEST_F(UartTest, PrepareFeedbackPacketHeader)
{
  Info info{};
  uart_.PrepareFeedbackPacket(info);
  EXPECT_EQ(uart_.feedback_transmit_buffer_[0], Uart::FEEDBACK_HEADER_0);
  EXPECT_EQ(uart_.feedback_transmit_buffer_[1], Uart::FEEDBACK_HEADER_1);
  EXPECT_EQ(uart_.feedback_transmit_buffer_[111], 0x00);
}

TEST_F(UartTest, FeedbackRoundTripImu)
{
  Info src{};
  src.imu_.acc_x_ = 1.1f;
  src.imu_.acc_y_ = -2.2f;
  src.imu_.acc_z_ = 9.81f;
  src.imu_.gyro_x_ = 0.01f;
  src.imu_.gyro_y_ = -0.02f;
  src.imu_.gyro_z_ = 0.03f;
  src.imu_.euler_heading_ = 1.57f;
  src.imu_.euler_roll_ = 0.1f;
  src.imu_.euler_pitch_ = -0.2f;

  PrepareAndCopyFeedback(src);

  Info dst{};
  EXPECT_TRUE(uart_.UpdateFeedbackData(dst));
  EXPECT_FLOAT_EQ(dst.imu_.acc_x_, 1.1f);
  EXPECT_FLOAT_EQ(dst.imu_.acc_y_, -2.2f);
  EXPECT_FLOAT_EQ(dst.imu_.acc_z_, 9.81f);
  EXPECT_FLOAT_EQ(dst.imu_.gyro_x_, 0.01f);
  EXPECT_FLOAT_EQ(dst.imu_.gyro_y_, -0.02f);
  EXPECT_FLOAT_EQ(dst.imu_.gyro_z_, 0.03f);
  EXPECT_FLOAT_EQ(dst.imu_.euler_heading_, 1.57f);
  EXPECT_FLOAT_EQ(dst.imu_.euler_roll_, 0.1f);
  EXPECT_FLOAT_EQ(dst.imu_.euler_pitch_, -0.2f);
}

TEST_F(UartTest, FeedbackRoundTripMotors)
{
  Info src{};
  src.yaw_act_.position_ = 1.0f;
  src.yaw_act_.velocity_ = 2.0f;
  src.yaw_act_.current_ = 0.5f;
  src.pitch_act_.position_ = -1.0f;
  src.pitch_act_.velocity_ = -2.0f;
  src.pitch_act_.current_ = 0.3f;
  src.chassis_fr_act_.position_ = 10.0f;
  src.chassis_fr_act_.velocity_ = 20.0f;
  src.chassis_fr_act_.torque_ = 1.5f;
  src.chassis_fl_act_.position_ = 11.0f;
  src.chassis_fl_act_.velocity_ = 21.0f;
  src.chassis_fl_act_.torque_ = 1.6f;
  src.chassis_br_act_.position_ = 12.0f;
  src.chassis_br_act_.velocity_ = 22.0f;
  src.chassis_br_act_.torque_ = 1.7f;
  src.chassis_bl_act_.position_ = 13.0f;
  src.chassis_bl_act_.velocity_ = 23.0f;
  src.chassis_bl_act_.torque_ = 1.8f;

  PrepareAndCopyFeedback(src);

  Info dst{};
  EXPECT_TRUE(uart_.UpdateFeedbackData(dst));

  EXPECT_FLOAT_EQ(dst.yaw_act_.position_, 1.0f);
  EXPECT_FLOAT_EQ(dst.yaw_act_.velocity_, 2.0f);
  EXPECT_FLOAT_EQ(dst.yaw_act_.current_, 0.5f);
  EXPECT_FLOAT_EQ(dst.pitch_act_.position_, -1.0f);
  EXPECT_FLOAT_EQ(dst.pitch_act_.velocity_, -2.0f);
  EXPECT_FLOAT_EQ(dst.pitch_act_.current_, 0.3f);
  EXPECT_FLOAT_EQ(dst.chassis_fr_act_.position_, 10.0f);
  EXPECT_FLOAT_EQ(dst.chassis_fr_act_.velocity_, 20.0f);
  EXPECT_FLOAT_EQ(dst.chassis_fr_act_.torque_, 1.5f);
  EXPECT_FLOAT_EQ(dst.chassis_fl_act_.position_, 11.0f);
  EXPECT_FLOAT_EQ(dst.chassis_fl_act_.velocity_, 21.0f);
  EXPECT_FLOAT_EQ(dst.chassis_fl_act_.torque_, 1.6f);
  EXPECT_FLOAT_EQ(dst.chassis_br_act_.position_, 12.0f);
  EXPECT_FLOAT_EQ(dst.chassis_br_act_.velocity_, 22.0f);
  EXPECT_FLOAT_EQ(dst.chassis_br_act_.torque_, 1.7f);
  EXPECT_FLOAT_EQ(dst.chassis_bl_act_.position_, 13.0f);
  EXPECT_FLOAT_EQ(dst.chassis_bl_act_.velocity_, 23.0f);
  EXPECT_FLOAT_EQ(dst.chassis_bl_act_.torque_, 1.8f);
}

TEST_F(UartTest, FeedbackInvalidHeader)
{
  Info info{};
  PrepareAndCopyFeedback(info);
  uart_.feedback_receive_buffer_[0] = 0x00;
  Info data;
  EXPECT_FALSE(uart_.UpdateFeedbackData(data));
}

TEST_F(UartTest, FeedbackInvalidCrc)
{
  Info info{};
  PrepareAndCopyFeedback(info);
  uart_.feedback_receive_buffer_[110] ^= 0xFF;
  Info data;
  EXPECT_FALSE(uart_.UpdateFeedbackData(data));
}

TEST_F(UartTest, FeedbackCorruptedPayloadDetected)
{
  Info src{};
  src.imu_.acc_x_ = 1.0f;
  PrepareAndCopyFeedback(src);
  uart_.feedback_receive_buffer_[5] ^= 0x01;
  Info dst{};
  EXPECT_FALSE(uart_.UpdateFeedbackData(dst));
}

TEST_F(UartTest, FeedbackRoundTripZeroValues)
{
  Info src{};
  PrepareAndCopyFeedback(src);
  Info dst{};
  dst.imu_.acc_x_ = 999.0f;
  EXPECT_TRUE(uart_.UpdateFeedbackData(dst));
  EXPECT_FLOAT_EQ(dst.imu_.acc_x_, 0.0f);
  EXPECT_FLOAT_EQ(dst.yaw_act_.position_, 0.0f);
  EXPECT_FLOAT_EQ(dst.chassis_fr_act_.torque_, 0.0f);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
