#include "rabcl/controller/pd_gravity_ff.hpp"

#include <cmath>

namespace rabcl
{
PdGravityFf::PdGravityFf(float kp, float kd, float gravity_ff, float output_max)
: kp_(kp), kd_(kd), gravity_ff_(gravity_ff), output_max_(output_max)
{
}

PdGravityFf::~PdGravityFf()
{
}

float PdGravityFf::Calc(float pos_target, float pos_actual, float vel_actual)
{
  float output = kp_ * (pos_target - pos_actual) - kd_ * vel_actual +
    gravity_ff_ * std::sin(pos_actual);
  if (output > output_max_) {
    output = output_max_;
  }
  if (output < -output_max_) {
    output = -output_max_;
  }
  return output;
}

void PdGravityFf::SetGains(float kp, float kd, float gravity_ff)
{
  kp_ = kp;
  kd_ = kd;
  gravity_ff_ = gravity_ff;
}
}  // namespace rabcl
