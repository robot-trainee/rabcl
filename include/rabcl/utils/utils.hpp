#ifndef RABCL_UTILS_HPP
#define RABCL_UTILS_HPP

#include <cstdint>
#include <math.h>

namespace rabcl
{
class Utils
{
public:
    void DegToRad(double deg, double& rad);
    double DegToRad(double deg);

    void RadToDeg(double rad, double& deg);
    double RadToDeg(double rad);

};
} // namespace rabcl

#endif
