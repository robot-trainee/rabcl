#include <gtest/gtest.h>
#include "rabcl/utils/utils.hpp"

namespace rabcl
{

TEST(UtilsTest, DegToRadReturnValue)
{
    EXPECT_DOUBLE_EQ(Utils::DegToRad(0.0), 0.0);
    EXPECT_DOUBLE_EQ(Utils::DegToRad(180.0), M_PI);
    EXPECT_DOUBLE_EQ(Utils::DegToRad(90.0), M_PI / 2.0);
    EXPECT_DOUBLE_EQ(Utils::DegToRad(-90.0), -M_PI / 2.0);
}

TEST(UtilsTest, DegToRadOutParam)
{
    double rad = 0.0;
    Utils::DegToRad(180.0, rad);
    EXPECT_DOUBLE_EQ(rad, M_PI);

    Utils::DegToRad(0.0, rad);
    EXPECT_DOUBLE_EQ(rad, 0.0);
}

TEST(UtilsTest, RadToDegReturnValue)
{
    EXPECT_DOUBLE_EQ(Utils::RadToDeg(0.0), 0.0);
    EXPECT_DOUBLE_EQ(Utils::RadToDeg(M_PI), 180.0);
    EXPECT_DOUBLE_EQ(Utils::RadToDeg(M_PI / 2.0), 90.0);
    EXPECT_DOUBLE_EQ(Utils::RadToDeg(-M_PI / 2.0), -90.0);
}

TEST(UtilsTest, RadToDegOutParam)
{
    double deg = 0.0;
    Utils::RadToDeg(M_PI, deg);
    EXPECT_DOUBLE_EQ(deg, 180.0);

    Utils::RadToDeg(0.0, deg);
    EXPECT_DOUBLE_EQ(deg, 0.0);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
