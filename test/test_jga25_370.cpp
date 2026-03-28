#include <gtest/gtest.h>
#include "rabcl/component/jga25_370.hpp"

namespace rabcl
{

class Jga25370Test : public ::testing::Test
{
protected:
  static constexpr uint16_t kMaxOutput = 1000;
  static constexpr double kReductionRatio = 1.0;
};

TEST_F(Jga25370Test, VelModeGettersInitial)
{
    JGA25_370 motor(kMaxOutput, kReductionRatio, 1);
    EXPECT_DOUBLE_EQ(motor.GetCmdValue(), 0.0);
    EXPECT_DOUBLE_EQ(motor.GetActVel(), 0.0);
    EXPECT_DOUBLE_EQ(motor.GetActPos(), 0.0);
}

TEST_F(Jga25370Test, SetCmdValue)
{
    JGA25_370 motor(kMaxOutput, kReductionRatio, 1);
    motor.SetCmdValue(3.14);
    EXPECT_DOUBLE_EQ(motor.GetCmdValue(), 3.14);
}

TEST_F(Jga25370Test, SetEncoderCountAndUpdateEncoder)
{
    JGA25_370 motor(kMaxOutput, kReductionRatio, 1);
    motor.SetEncoderCount(10);
    motor.UpdateEncoder();
    EXPECT_NE(motor.GetActVel(), 0.0);
}

TEST_F(Jga25370Test, CalcMotorOutputVelMode)
{
    JGA25_370 motor(kMaxOutput, kReductionRatio, 1);
    motor.SetCmdValue(10.0);
    int16_t output = motor.CalcMotorOutput();
    EXPECT_NE(output, 0);
}

TEST_F(Jga25370Test, CalcMotorOutputPosMode)
{
    JGA25_370 motor(kMaxOutput, kReductionRatio, 0);
    motor.SetCmdValue(1.0);
    int16_t output = motor.CalcMotorOutput();
    EXPECT_NE(output, 0);
}

TEST_F(Jga25370Test, CalcMotorOutputInvalidMode)
{
    JGA25_370 motor(kMaxOutput, kReductionRatio, 2);
    int16_t output = motor.CalcMotorOutput();
    EXPECT_EQ(output, 0);
}

TEST_F(Jga25370Test, VelOutputClampPositive)
{
    JGA25_370 motor(100, kReductionRatio, 1);
    motor.SetCmdValue(1e6);  // 巨大な指令でクランプを誘発
    int16_t output = motor.CalcMotorOutput();
    EXPECT_LE(output, 100);
}

TEST_F(Jga25370Test, VelOutputClampNegative)
{
    JGA25_370 motor(100, kReductionRatio, 1);
    motor.SetCmdValue(-1e6);  // 負の巨大な指令でクランプを誘発
    int16_t output = motor.CalcMotorOutput();
    EXPECT_GE(output, -100);
}

TEST_F(Jga25370Test, PosOutputClampPositive)
{
    JGA25_370 motor(100, kReductionRatio, 0);
    motor.SetCmdValue(1e6);
    int16_t output = motor.CalcMotorOutput();
    EXPECT_LE(output, 100);
}

TEST_F(Jga25370Test, PosOutputClampNegative)
{
    JGA25_370 motor(100, kReductionRatio, 0);
    motor.SetCmdValue(-1e6);
    int16_t output = motor.CalcMotorOutput();
    EXPECT_GE(output, -100);
}

TEST_F(Jga25370Test, EncoderCountNegativeActPos)
{
    JGA25_370 motor(kMaxOutput, kReductionRatio, 0);
    // act_pos_ が負になるケース → act_pos_ += 2π が通ることを確認
    for (int i = 0; i < 20; ++i) {
    motor.SetEncoderCount(-10);
    }
    motor.UpdateEncoder();
    EXPECT_GE(motor.GetActPos(), 0.0);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
