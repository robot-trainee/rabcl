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

// --- CalcAngular tests ---

class CalcAngularTest : public ::testing::Test
{
protected:
  // kp=100, kd=10, gravity_ff=0, output_max=2000
  PdGravityFf ctrl_{100.0f, 10.0f, 0.0f, 2000.0f};

  static float Deg(float d) {return d * static_cast<float>(M_PI) / 180.0f;}
};

// --- basic ---

TEST_F(CalcAngularTest, ZeroError)
{
  EXPECT_FLOAT_EQ(ctrl_.CalcAngular(1.0f, 1.0f, 0.0f), 0.0f);
}

TEST_F(CalcAngularTest, PositiveSmallError)
{
  float out = ctrl_.CalcAngular(1.0f, 0.8f, 0.0f);
  EXPECT_GT(out, 0.0f);
  EXPECT_NEAR(out, 100.0f * 0.2f, 1e-3f);
}

TEST_F(CalcAngularTest, NegativeSmallError)
{
  float out = ctrl_.CalcAngular(0.8f, 1.0f, 0.0f);
  EXPECT_LT(out, 0.0f);
  EXPECT_NEAR(out, 100.0f * (-0.2f), 1e-3f);
}

// --- wraparound (most important) ---

TEST_F(CalcAngularTest, Wrap350To10)
{
  // 350° → 10°: shortest path = +20°, NOT -340°
  float out = ctrl_.CalcAngular(Deg(10.0f), Deg(350.0f), 0.0f);
  EXPECT_GT(out, 0.0f);
  EXPECT_NEAR(out, 100.0f * Deg(20.0f), 1e-2f);
}

TEST_F(CalcAngularTest, Wrap10To350)
{
  // 10° → 350°: shortest path = -20°, NOT +340°
  float out = ctrl_.CalcAngular(Deg(350.0f), Deg(10.0f), 0.0f);
  EXPECT_LT(out, 0.0f);
  EXPECT_NEAR(out, 100.0f * Deg(-20.0f), 1e-2f);
}

TEST_F(CalcAngularTest, Wrap179To181)
{
  // 179° → 181°: shortest path = +2°
  float out = ctrl_.CalcAngular(Deg(181.0f), Deg(179.0f), 0.0f);
  EXPECT_GT(out, 0.0f);
  EXPECT_NEAR(out, 100.0f * Deg(2.0f), 1e-2f);
}

TEST_F(CalcAngularTest, Wrap181To179)
{
  // 181° → 179°: shortest path = -2°
  float out = ctrl_.CalcAngular(Deg(179.0f), Deg(181.0f), 0.0f);
  EXPECT_LT(out, 0.0f);
  EXPECT_NEAR(out, 100.0f * Deg(-2.0f), 1e-2f);
}

// --- D term ---

TEST_F(CalcAngularTest, DampingPositiveVelocity)
{
  // err=0, positive velocity → negative output (braking)
  float out = ctrl_.CalcAngular(1.0f, 1.0f, 2.0f);
  EXPECT_FLOAT_EQ(out, -10.0f * 2.0f);
}

TEST_F(CalcAngularTest, DampingNegativeVelocity)
{
  // err=0, negative velocity → positive output (braking)
  float out = ctrl_.CalcAngular(1.0f, 1.0f, -2.0f);
  EXPECT_FLOAT_EQ(out, -10.0f * (-2.0f));
}

// --- output clamp ---

TEST_F(CalcAngularTest, ClampMax)
{
  PdGravityFf ctrl(10000.0f, 0.0f, 0.0f, 100.0f);
  float out = ctrl.CalcAngular(1.0f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(out, 100.0f);
}

TEST_F(CalcAngularTest, ClampMin)
{
  PdGravityFf ctrl(10000.0f, 0.0f, 0.0f, 100.0f);
  float out = ctrl.CalcAngular(0.0f, 1.0f, 0.0f);
  EXPECT_FLOAT_EQ(out, -100.0f);
}

// --- gravity_ff ---

TEST_F(CalcAngularTest, GravityFfAddsOffset)
{
  PdGravityFf ctrl(100.0f, 0.0f, 500.0f, 2000.0f);
  float actual = 0.5f;
  float out = ctrl.CalcAngular(actual, actual, 0.0f);
  // err=0, gravity_ff * sin(actual)
  EXPECT_NEAR(out, 500.0f * std::sin(actual), 0.01f);
}

// --- custom full_rotation ---

TEST_F(CalcAngularTest, CustomFullRotation)
{
  // full_rotation=36000 (0.01 deg units)
  PdGravityFf ctrl(1.0f, 0.0f, 0.0f, 100000.0f);
  // 35000 → 1000: shortest path = +2000, NOT -34000
  float out = ctrl.CalcAngular(1000.0f, 35000.0f, 0.0f, 36000.0f);
  EXPECT_GT(out, 0.0f);
  EXPECT_NEAR(out, 1.0f * 2000.0f, 1e-1f);
}

TEST_F(CalcAngularTest, CustomFullRotationReverse)
{
  PdGravityFf ctrl(1.0f, 0.0f, 0.0f, 100000.0f);
  // 1000 → 35000: shortest path = -2000
  float out = ctrl.CalcAngular(35000.0f, 1000.0f, 0.0f, 36000.0f);
  EXPECT_LT(out, 0.0f);
  EXPECT_NEAR(out, 1.0f * (-2000.0f), 1e-1f);
}

// --- sign consistency (core anti-runaway check) ---

TEST_F(CalcAngularTest, SignConsistencyMultipleAngles)
{
  // For various (actual, target) pairs, torque sign must reduce the error
  PdGravityFf ctrl(100.0f, 0.0f, 0.0f, 10000.0f);
  float angles[] = {Deg(0), Deg(45), Deg(90), Deg(135), Deg(180),
    Deg(225), Deg(270), Deg(315)};
  for (float actual : angles) {
    // target = actual + 30°: torque should be positive
    float target_cw = actual + Deg(30.0f);
    float out_cw = ctrl.CalcAngular(target_cw, actual, 0.0f);
    EXPECT_GT(out_cw, 0.0f) << "actual=" << actual << " target=" << target_cw;

    // target = actual - 30°: torque should be negative
    float target_ccw = actual - Deg(30.0f);
    float out_ccw = ctrl.CalcAngular(target_ccw, actual, 0.0f);
    EXPECT_LT(out_ccw, 0.0f) << "actual=" << actual << " target=" << target_ccw;
  }
}

TEST_F(CalcAngularTest, SignConsistencyNearZeroCrossing)
{
  PdGravityFf ctrl(100.0f, 0.0f, 0.0f, 10000.0f);
  // actual=355°, target=5°: should go +10° → positive torque
  EXPECT_GT(ctrl.CalcAngular(Deg(5.0f), Deg(355.0f), 0.0f), 0.0f);
  // actual=5°, target=355°: should go -10° → negative torque
  EXPECT_LT(ctrl.CalcAngular(Deg(355.0f), Deg(5.0f), 0.0f), 0.0f);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
