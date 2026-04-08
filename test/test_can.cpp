#include <gtest/gtest.h>
#include "rabcl/interface/can.hpp"
#include "rabcl/utils/type.hpp"

namespace rabcl
{

class CanTest : public ::testing::Test
{
protected:
    // float を 8バイト CAN フレームに書き込むヘルパー（前半4バイト）
  static void PackFloat(float value, uint8_t * buf, int offset = 0)
  {
    union { float f; int32_t ui; } b;
    b.f = value;
    buf[offset + 0] = static_cast<uint8_t>((b.ui >> 24) & 0xFF);
    buf[offset + 1] = static_cast<uint8_t>((b.ui >> 16) & 0xFF);
    buf[offset + 2] = static_cast<uint8_t>((b.ui >> 8) & 0xFF);
    buf[offset + 3] = static_cast<uint8_t>((b.ui >> 0) & 0xFF);
  }
};

TEST_F(CanTest, UpdateDataChassisXY)
{
  uint8_t data[8] = {};
  PackFloat(1.0f, data, 0);
  PackFloat(2.0f, data, 4);
  Info info;
  EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_X_Y), data, info));
  EXPECT_FLOAT_EQ(info.chassis_vel_x_, 1.0f);
  EXPECT_FLOAT_EQ(info.chassis_vel_y_, 2.0f);
}

TEST_F(CanTest, UpdateDataChassisZYaw)
{
  uint8_t data[8] = {};
  PackFloat(3.0f, data, 0);
  PackFloat(4.0f, data, 4);
  Info info;
  EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_Z_YAW), data, info));
  EXPECT_FLOAT_EQ(info.chassis_vel_z_, 3.0f);
  EXPECT_FLOAT_EQ(info.yaw_pos_, 4.0f);
}

TEST_F(CanTest, UpdateDataPitchModes)
{
  uint8_t data[8] = {};
  PackFloat(0.5f, data, 0);
  data[4 + static_cast<int>(MODE_ID::LOAD)] = 1;
  data[4 + static_cast<int>(MODE_ID::FIRE)] = 2;
  data[4 + static_cast<int>(MODE_ID::SPEED)] = 0;
  data[4 + static_cast<int>(MODE_ID::CHASSIS)] = 1;
  Info info;
  EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_PITCH_MODES), data, info));
  EXPECT_FLOAT_EQ(info.pitch_pos_, 0.5f);
  EXPECT_EQ(info.load_mode_, 1);
  EXPECT_EQ(info.fire_mode_, 2);
  EXPECT_EQ(info.speed_mode_, 0);
  EXPECT_EQ(info.chassis_mode_, 1);
}

TEST_F(CanTest, UpdateDataPitchRx)
{
  // LK motor feedback format: [cmd, temp, curr_L, curr_H, spd_L, spd_H, pos_L, pos_H]
  uint8_t data[8] = {0xA4, 0x25, 0x64, 0x00, 0x01, 0x00, 0x00, 0x10};
  Info info;
  EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::PITCH_RX), data, info));
  EXPECT_FLOAT_EQ(info.pitch_act_.temperature_, 37.0f);  // 0x25 = 37
  EXPECT_NEAR(info.pitch_act_.current_, 1.0f, 0.01f);  // 0x0064 = 100 → 100 * 0.01 = 1.0 A
}

TEST_F(CanTest, UpdateDataUnknownIdx)
{
  uint8_t data[8] = {};
  Info info;
  EXPECT_FALSE(Can::UpdateData(0xFFFF, data, info));
}

TEST_F(CanTest, Prepare2FloatData)
{
  uint8_t out[8] = {};
  Can::Prepare2FloatData(1.0f, 2.0f, out);

  uint8_t expected[8] = {};
  PackFloat(1.0f, expected, 0);
  PackFloat(2.0f, expected, 4);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(out[i], expected[i]);
  }
}

TEST_F(CanTest, Prepare1Float4IntData)
{
  uint8_t out[8] = {};
  uint8_t modes[4] = {1, 2, 3, 4};
  Can::Prepare1Float4IntData(1.0f, modes, out);

  uint8_t expected[8] = {};
  PackFloat(1.0f, expected, 0);
  expected[4] = 1; expected[5] = 2; expected[6] = 3; expected[7] = 4;
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(out[i], expected[i]);
  }
}

TEST_F(CanTest, PrepareLKMotorTorqueCmd)
{
  uint8_t out[8] = {};
  Can::PrepareLKMotorTorqueCmd(0x0102, out);

  EXPECT_EQ(out[0], 0xA1);
  EXPECT_EQ(out[1], 0x00);
  EXPECT_EQ(out[2], 0x00);
  EXPECT_EQ(out[3], 0x00);
  EXPECT_EQ(out[4], 0x02);  // current low byte
  EXPECT_EQ(out[5], 0x01);  // current high byte
  EXPECT_EQ(out[6], 0x00);
  EXPECT_EQ(out[7], 0x00);
}

TEST_F(CanTest, PrepareLKMotorTorqueCmdNegative)
{
  uint8_t out[8] = {};
  Can::PrepareLKMotorTorqueCmd(-500, out);

  EXPECT_EQ(out[0], 0xA1);
  int16_t reconstructed = static_cast<int16_t>(
    (static_cast<uint16_t>(out[5]) << 8) | out[4]);
  EXPECT_EQ(reconstructed, -500);
}

TEST_F(CanTest, PrepareLKMotorPositionCmd)
{
  uint8_t out[8] = {};
  Can::PrepareLKMotorPositionCmd(0x00010203, 0x0405, out);

  EXPECT_EQ(out[0], 0xA4);
  EXPECT_EQ(out[1], 0x00);
  EXPECT_EQ(out[2], 0x05);  // max_speed low byte
  EXPECT_EQ(out[3], 0x04);  // max_speed high byte
  EXPECT_EQ(out[4], 0x03);  // pos byte0
  EXPECT_EQ(out[5], 0x02);  // pos byte1
  EXPECT_EQ(out[6], 0x01);  // pos byte2
  EXPECT_EQ(out[7], 0x00);  // pos byte3
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
