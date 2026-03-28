#include <gtest/gtest.h>
#include <cmath>
#include "rabcl/component/ld_20mg.hpp"

namespace rabcl
{

TEST(Ld20mgTest, ConstructorMinPosNegativeClamp)
{
    // min_pos < 0 → 0 にクランプ
    LD_20MG servo(-1.0, 1.0, 0.0, 1.0);
    EXPECT_DOUBLE_EQ(servo.GetCmdPos(), 0.0);
}

TEST(Ld20mgTest, ConstructorMaxPosExceedClamp)
{
    // max_pos > MAX_POS * reduction_ratio → クランプ
    LD_20MG servo(0.0, 100.0, 0.0, 1.0);
    // cmd_pos は min_pos で初期化されるため 0
    EXPECT_DOUBLE_EQ(servo.GetCmdPos(), 0.0);
}

TEST(Ld20mgTest, ConstructorValidRange)
{
    LD_20MG servo(0.0, 1.0, 0.0, 1.0);
    EXPECT_DOUBLE_EQ(servo.GetCmdPos(), 0.0);
}

TEST(Ld20mgTest, UpdateNormal)
{
    LD_20MG servo(0.0, M_PI, 0.0, 1.0);
    servo.Update(1.0);
    EXPECT_DOUBLE_EQ(servo.GetCmdPos(), 1.0);
}

TEST(Ld20mgTest, UpdateClampMin)
{
    LD_20MG servo(0.0, M_PI, 0.0, 1.0);
    servo.Update(-10.0);  // min_pos(0.0) にクランプ
    EXPECT_DOUBLE_EQ(servo.GetCmdPos(), 0.0);
}

TEST(Ld20mgTest, UpdateClampMax)
{
    LD_20MG servo(0.0, M_PI, 0.0, 1.0);
    servo.Update(100.0);  // max_pos(π) にクランプ
    EXPECT_DOUBLE_EQ(servo.GetCmdPos(), M_PI);
}

TEST(Ld20mgTest, UpdateWithOffset)
{
    double offset = 0.5;
    LD_20MG servo(0.0, M_PI, offset, 1.0);
    servo.Update(1.0);
    EXPECT_DOUBLE_EQ(servo.GetCmdPos(), 1.0 + offset);
}

TEST(Ld20mgTest, CalcMotorOutput)
{
    LD_20MG servo(0.0, M_PI, 0.0, 1.0);
    servo.Update(0.0);
    int16_t output = servo.CalcMotorOutput();
    // MIN_PULSE_WIDTH / CONTROL_RESOLUTION = 500 / 2 = 250
    EXPECT_EQ(output, 250);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
