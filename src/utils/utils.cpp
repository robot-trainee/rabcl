#include "rabcl/utils/utils.hpp"

namespace rabcl
{
void Utils::DegToRad(double deg, double & rad)
{
  rad = deg * M_PI / 180.0;
}

double Utils::DegToRad(double deg)
{
  double rad;
  DegToRad(deg, rad);
  return rad;
}

void Utils::RadToDeg(double rad, double & deg)
{
  deg = rad * 180.0 / M_PI;
}

double Utils::RadToDeg(double rad)
{
  double deg;
  RadToDeg(rad, deg);
  return deg;
}

double Utils::ShortestPathMultiTurn(double current, double target, double full_rotation)
{
  double half = full_rotation / 2.0;
  double target_mod = std::fmod(target, full_rotation);
  if (target_mod < 0.0) {
    target_mod += full_rotation;
  }
  double current_mod = std::fmod(current, full_rotation);
  if (current_mod < 0.0) {
    current_mod += full_rotation;
  }
  double diff = target_mod - current_mod;
  if (diff > half) {
    diff -= full_rotation;
  }
  if (diff < -half) {
    diff += full_rotation;
  }
  return current + diff;
}
}  // namespace rabcl
