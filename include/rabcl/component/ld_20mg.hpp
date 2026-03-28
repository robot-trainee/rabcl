#ifndef RABCL__COMPONENT__LD_20MG_HPP_
#define RABCL__COMPONENT__LD_20MG_HPP_

#include <cmath>
#include <cstdint>

namespace rabcl
{
class LD_20MG
{
private:
  static constexpr double CONTROL_FREQUENCY = 0.01;    // s
  static constexpr double CONTROL_RESOLUTION = 2.0;    // us
  static constexpr double MIN_PULSE_WIDTH = 500.0;    // us
  static constexpr double MAX_PULSE_WIDTH = 2500.0;    // us
  static constexpr double MAX_POS = M_PI;    // rad

public:
  LD_20MG(
    double min_pos, double max_pos,
    double offset_pos, double reduction_ratio);
  ~LD_20MG();

  void Update(double cmd_vel);
  int16_t CalcMotorOutput();

  double GetCmdPos();

private:
  double min_pos_;    // rad
  double max_pos_;    // rad
  double offset_pos_;    // rad
  double reduction_ratio_;

  double cmd_pos_;    // rad
};
}  // namespace rabcl

#endif  // RABCL__COMPONENT__LD_20MG_HPP_
