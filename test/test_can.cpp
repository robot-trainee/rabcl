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

TEST_F(CanTest, UpdateDataYawCmdAct)
{
    uint8_t data[8] = {};
    PackFloat(1.1f, data, 0);
    PackFloat(1.2f, data, 4);
    Info info;
    EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_YAW_CMD_ACT), data, info));
    EXPECT_FLOAT_EQ(info.yaw_pos_cmd_, 1.1f);
    EXPECT_FLOAT_EQ(info.yaw_pos_act_, 1.2f);
}

TEST_F(CanTest, UpdateDataChassisFrontCmd)
{
    uint8_t data[8] = {};
    PackFloat(5.0f, data, 0);
    PackFloat(6.0f, data, 4);
    Info info;
    EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_FRONT_CMD), data, info));
    EXPECT_FLOAT_EQ(info.front_right_vel_cmd_, 5.0f);
    EXPECT_FLOAT_EQ(info.front_left_vel_cmd_, 6.0f);
}

TEST_F(CanTest, UpdateDataChassisBackCmd)
{
    uint8_t data[8] = {};
    PackFloat(7.0f, data, 0);
    PackFloat(8.0f, data, 4);
    Info info;
    EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_BACK_CMD), data, info));
    EXPECT_FLOAT_EQ(info.back_right_vel_cmd_, 7.0f);
    EXPECT_FLOAT_EQ(info.back_left_vel_cmd_, 8.0f);
}

TEST_F(CanTest, UpdateDataChassisFrontAct)
{
    uint8_t data[8] = {};
    PackFloat(9.0f, data, 0);
    PackFloat(10.0f, data, 4);
    Info info;
    EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_FRONT_ACT), data, info));
    EXPECT_FLOAT_EQ(info.front_right_vel_act_, 9.0f);
    EXPECT_FLOAT_EQ(info.front_left_vel_act_, 10.0f);
}

TEST_F(CanTest, UpdateDataChassisBackAct)
{
    uint8_t data[8] = {};
    PackFloat(11.0f, data, 0);
    PackFloat(12.0f, data, 4);
    Info info;
    EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_BACK_ACT), data, info));
    EXPECT_FLOAT_EQ(info.back_right_vel_act_, 11.0f);
    EXPECT_FLOAT_EQ(info.back_left_vel_act_, 12.0f);
}

TEST_F(CanTest, UpdateDataPitchRx)
{
    uint8_t data[8] = {0xA0, 0x25, 0x00, 0x64, 0x01, 0x00, 0x00, 0x10};
    Info info;
    EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::PITCH_RX), data, info));
    EXPECT_EQ(info.command_byte_, 0xA0);
    EXPECT_EQ(info.temperature_, 0x25);
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
