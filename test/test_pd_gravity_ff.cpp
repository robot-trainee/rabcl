#include <gtest/gtest.h>
#include <cmath>
#include "rabcl/controller/pd_gravity_ff.hpp"

namespace rabcl
{

class PdGravityFfTest : public ::testing::Test
{
protected:
  PdGravityFf ctrl_{100.0f, 10.0f, 500.0f, 2000.0f};
};

TEST_F(PdGravityFfTest, ZeroError)
{
  float out = ctrl_.Calc(0.0f, 0.0f, 0.0f);
  // kp*(0-0) - kd*0 + 500*sin(0) = 0
  EXPECT_FLOAT_EQ(out, 0.0f);
}

TEST_F(PdGravityFfTest, PositionError)
{
  float out = ctrl_.Calc(1.0f, 0.0f, 0.0f);
  // kp*(1-0) - kd*0 + 500*sin(0) = 100
  EXPECT_FLOAT_EQ(out, 100.0f);
}

TEST_F(PdGravityFfTest, VelocityDamping)
{
  float out = ctrl_.Calc(0.0f, 0.0f, 1.0f);
  // kp*(0-0) - kd*1 + 500*sin(0) = -10
  EXPECT_FLOAT_EQ(out, -10.0f);
}

TEST_F(PdGravityFfTest, GravityFf)
{
  float angle = 0.5f;
  float out = ctrl_.Calc(angle, angle, 0.0f);
  // kp*(0) - kd*0 + 500*sin(0.5)
  float expected = 500.0f * std::sin(0.5f);
  EXPECT_NEAR(out, expected, 0.01f);
}

TEST_F(PdGravityFfTest, ClampMax)
{
  PdGravityFf ctrl(10000.0f, 0.0f, 0.0f, 100.0f);
  float out = ctrl.Calc(1.0f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(out, 100.0f);
}

TEST_F(PdGravityFfTest, ClampMin)
{
  PdGravityFf ctrl(10000.0f, 0.0f, 0.0f, 100.0f);
  float out = ctrl.Calc(-1.0f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(out, -100.0f);
}

TEST_F(PdGravityFfTest, SetGains)
{
  ctrl_.SetGains(0.0f, 0.0f, 1000.0f);
  float angle = 0.3f;
  float out = ctrl_.Calc(angle, angle, 0.0f);
  float expected = 1000.0f * std::sin(0.3f);
  EXPECT_NEAR(out, expected, 0.01f);
}

TEST_F(PdGravityFfTest, Combined)
{
  float target = 0.5f;
  float actual = 0.3f;
  float vel = 0.1f;
  float out = ctrl_.Calc(target, actual, vel);
  // 100*(0.5-0.3) - 10*0.1 + 500*sin(0.3) = 20 - 1 + 147.8 = 166.8
  float expected = 100.0f * (target - actual) - 10.0f * vel + 500.0f * std::sin(actual);
  EXPECT_NEAR(out, expected, 0.01f);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
