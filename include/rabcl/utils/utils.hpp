#ifndef RABCL__UTILS__UTILS_HPP_
#define RABCL__UTILS__UTILS_HPP_

#include <cmath>
#include <cstdint>

namespace rabcl
{
class Utils
{
public:
  static void DegToRad(double deg, double & rad);
  static double DegToRad(double deg);

  static void RadToDeg(double rad, double & deg);
  static double RadToDeg(double rad);

  static double ShortestPathMultiTurn(
    double current, double target,
    double full_rotation = 2.0 * M_PI);
};
}  // namespace rabcl

#endif  // RABCL__UTILS__UTILS_HPP_
