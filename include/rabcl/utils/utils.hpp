#ifndef RABCL_UTILS_HPP
#define RABCL_UTILS_HPP

#include <cstdint>
#include <math.h>

namespace rabcl
{
class Utils
{
public:
    static void DegToRad(double deg, double& rad);
    static double DegToRad(double deg);

    static void RadToDeg(double rad, double& deg);
    static double RadToDeg(double rad);

};
} // namespace rabcl

#endif
