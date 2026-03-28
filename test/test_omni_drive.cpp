#include <gtest/gtest.h>
#include <cmath>
#include "rabcl/controller/omni_drive.hpp"

namespace rabcl
{

class OmniDriveTest : public ::testing::Test
{
protected:
  static constexpr double kWheelD = 0.06;
  static constexpr double kBodyD = 0.28;
  OmniDrive omni_{kWheelD, kBodyD};
};

TEST_F(OmniDriveTest, ZeroInput)
{
    double fr, fl, br, bl;
    omni_.CalcVel(0.0, 0.0, 0.0, fr, fl, br, bl);
    EXPECT_DOUBLE_EQ(fr, 0.0);
    EXPECT_DOUBLE_EQ(fl, 0.0);
    EXPECT_DOUBLE_EQ(br, 0.0);
    EXPECT_DOUBLE_EQ(bl, 0.0);
}

TEST_F(OmniDriveTest, PureRotation)
{
    double fr, fl, br, bl;
    double vz = 1.0;
    omni_.CalcVel(0.0, 0.0, vz, fr, fl, br, bl);
    double expected = -vz * kBodyD / kWheelD;
    EXPECT_DOUBLE_EQ(fr, expected);
    EXPECT_DOUBLE_EQ(fl, expected);
    EXPECT_DOUBLE_EQ(br, expected);
    EXPECT_DOUBLE_EQ(bl, expected);
}

TEST_F(OmniDriveTest, PureTranslationX)
{
    double fr, fl, br, bl;
    omni_.CalcVel(1.0, 0.0, 0.0, fr, fl, br, bl);
    // 対称性チェック
    EXPECT_NEAR(fr, -fl, 1e-9);
    EXPECT_NEAR(br, -bl, 1e-9);
}

TEST_F(OmniDriveTest, WithOffsetRotation)
{
    double fr1, fl1, br1, bl1;
    double fr2, fl2, br2, bl2;
    omni_.CalcVel(1.0, 0.0, 0.0, fr1, fl1, br1, bl1, 0.0);
    omni_.CalcVel(1.0, 0.0, 0.0, fr2, fl2, br2, bl2, M_PI / 2.0);
    // オフセットがあると結果が変わる
    EXPECT_FALSE(std::abs(fr1 - fr2) < 1e-9 &&
                 std::abs(fl1 - fl2) < 1e-9 &&
                 std::abs(br1 - br2) < 1e-9 &&
                 std::abs(bl1 - bl2) < 1e-9);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
