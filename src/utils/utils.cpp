#include "rabcl/utils/utils.hpp"

namespace rabcl
{
void Utils::DegToRad(double deg, double& rad)
{
    rad = deg * M_PI / 180.0;
}

double Utils::DegToRad(double deg)
{
    double rad;
    DegToRad(deg, rad);
    return rad;
}

void Utils::RadToDeg(double rad, double& deg)
{
    deg = rad * 180.0 / M_PI;
}

double Utils::RadToDeg(double rad)
{
    double deg;
    RadToDeg(rad, deg);
    return deg;
}
} // namespace rabcl
