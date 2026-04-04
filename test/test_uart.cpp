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

    // float を 4バイトに分解して受信バッファに書き込むヘルパー
  void SetReceiveFloat(uint8_t idx, float value)
  {
    union { float f; int32_t ui; } buf;
    buf.f = value;
    uart_.uart_receive_buffer_[0] = 0xFF;
    uart_.uart_receive_buffer_[1] = idx;
    uart_.uart_receive_buffer_[2] = static_cast<uint8_t>((buf.ui >> 24) & 0xFF);
    uart_.uart_receive_buffer_[3] = static_cast<uint8_t>((buf.ui >> 16) & 0xFF);
    uart_.uart_receive_buffer_[4] = static_cast<uint8_t>((buf.ui >> 8) & 0xFF);
    uart_.uart_receive_buffer_[5] = static_cast<uint8_t>((buf.ui >> 0) & 0xFF);
    uart_.uart_receive_buffer_[6] = 0xFF;
    uart_.uart_receive_buffer_[7] = 0x00;
  }
};

TEST_F(UartTest, UpdateDataInvalidHeader)
{
  uart_.uart_receive_buffer_[0] = 0x00;  // 不正ヘッダ
  uart_.uart_receive_buffer_[6] = 0xFF;
  Info data;
  EXPECT_FALSE(uart_.UpdateData(data));
}

TEST_F(UartTest, UpdateDataInvalidFooter)
{
  uart_.uart_receive_buffer_[0] = 0xFF;
  uart_.uart_receive_buffer_[6] = 0x00;  // 不正フッタ
  Info data;
  EXPECT_FALSE(uart_.UpdateData(data));
}

TEST_F(UartTest, UpdateDataChassisX)
{
  SetReceiveFloat(UART_ID::UART_CHASSIS_X, 1.5f);
  Info data;
  EXPECT_TRUE(uart_.UpdateData(data));
  EXPECT_FLOAT_EQ(data.chassis_vel_x_, 1.5f);
}

TEST_F(UartTest, UpdateDataChassisY)
{
  SetReceiveFloat(UART_ID::UART_CHASSIS_Y, 2.5f);
  Info data;
  EXPECT_TRUE(uart_.UpdateData(data));
  EXPECT_FLOAT_EQ(data.chassis_vel_y_, 2.5f);
}

TEST_F(UartTest, UpdateDataChassisZ)
{
  SetReceiveFloat(UART_ID::UART_CHASSIS_Z, 3.5f);
  Info data;
  EXPECT_TRUE(uart_.UpdateData(data));
  EXPECT_FLOAT_EQ(data.chassis_vel_z_, 3.5f);
}

TEST_F(UartTest, UpdateDataYaw)
{
  SetReceiveFloat(UART_ID::UART_YAW, 0.5f);
  Info data;
  EXPECT_TRUE(uart_.UpdateData(data));
  EXPECT_FLOAT_EQ(data.yaw_pos_, 0.5f);
}

TEST_F(UartTest, UpdateDataPitch)
{
  SetReceiveFloat(UART_ID::UART_PITCH, -0.3f);
  Info data;
  EXPECT_TRUE(uart_.UpdateData(data));
  EXPECT_FLOAT_EQ(data.pitch_pos_, -0.3f);
}

TEST_F(UartTest, UpdateDataModes)
{
  uart_.uart_receive_buffer_[0] = 0xFF;
  uart_.uart_receive_buffer_[1] = UART_ID::UART_MODES;
  uart_.uart_receive_buffer_[2 + static_cast<int>(MODE_ID::LOAD)] = 1;
  uart_.uart_receive_buffer_[2 + static_cast<int>(MODE_ID::FIRE)] = 2;
  uart_.uart_receive_buffer_[2 + static_cast<int>(MODE_ID::SPEED)] = 0;
  uart_.uart_receive_buffer_[2 + static_cast<int>(MODE_ID::CHASSIS)] = 1;
  uart_.uart_receive_buffer_[6] = 0xFF;
  Info data;
  EXPECT_TRUE(uart_.UpdateData(data));
  EXPECT_EQ(data.load_mode_, 1);
  EXPECT_EQ(data.fire_mode_, 2);
  EXPECT_EQ(data.speed_mode_, 0);
  EXPECT_EQ(data.chassis_mode_, 1);
}

TEST_F(UartTest, UpdateDataUnknownIdx)
{
  uart_.uart_receive_buffer_[0] = 0xFF;
  uart_.uart_receive_buffer_[1] = 0xFF;  // 不正インデックス
  uart_.uart_receive_buffer_[6] = 0xFF;
  Info data;
  EXPECT_FALSE(uart_.UpdateData(data));
}

TEST_F(UartTest, PrepareFloatData)
{
  float value = 3.14f;
  uart_.PrepareFloatData(UART_ID::UART_CHASSIS_X, value);

  EXPECT_EQ(uart_.uart_transmit_buffer_[0], 0xFF);
  EXPECT_EQ(uart_.uart_transmit_buffer_[1], UART_ID::UART_CHASSIS_X);
  EXPECT_EQ(uart_.uart_transmit_buffer_[6], 0xFF);
  EXPECT_EQ(uart_.uart_transmit_buffer_[7], 0x00);

  // 送信バッファを受信バッファにコピーして復元できるか検証
  std::memcpy(uart_.uart_receive_buffer_, uart_.uart_transmit_buffer_, 8);
  Info data;
  EXPECT_TRUE(uart_.UpdateData(data));
  EXPECT_FLOAT_EQ(data.chassis_vel_x_, value);
}

TEST_F(UartTest, Prepare4IntData)
{
  uint8_t modes[4] = {1, 2, 0, 1};
  uart_.Prepare4IntData(UART_ID::UART_MODES, modes);

  EXPECT_EQ(uart_.uart_transmit_buffer_[0], 0xFF);
  EXPECT_EQ(uart_.uart_transmit_buffer_[1], UART_ID::UART_MODES);
  EXPECT_EQ(uart_.uart_transmit_buffer_[6], 0xFF);
  EXPECT_EQ(uart_.uart_transmit_buffer_[7], 0x00);

  std::memcpy(uart_.uart_receive_buffer_, uart_.uart_transmit_buffer_, 8);
  Info data;
  EXPECT_TRUE(uart_.UpdateData(data));
  EXPECT_EQ(data.load_mode_, 1);
  EXPECT_EQ(data.fire_mode_, 2);
  EXPECT_EQ(data.speed_mode_, 0);
  EXPECT_EQ(data.chassis_mode_, 1);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
