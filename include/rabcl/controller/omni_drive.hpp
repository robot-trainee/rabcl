#ifndef RABCL_OMNI_DRIVE_HPP
#define RABCL_OMNI_DRIVE_HPP

#include <cstdint>

namespace rabcl
{
class OmniDrive
{
public:
    OmniDrive(double wheel_d, double body_d);
    ~OmniDrive();
    void CalcVel(
        double cmd_vel_x, double cmd_vel_y, double cmd_vel_z,
        double& front_right, double& front_left, double& back_right, double& back_left);

private:
    double wheel_d_;
    double body_d_;

};
} // namespace rabcl

#endif
