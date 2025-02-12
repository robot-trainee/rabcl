#include "rabcl/controller/omni_drive.hpp"

#include <math.h>

namespace rabcl
{
    OmniDrive::OmniDrive(double wheel_d, double body_d)
{
    wheel_d_ = wheel_d;
    body_d_ = body_d;
}

OmniDrive::~OmniDrive()
{
    // NOP
}

void OmniDrive::CalcVel(
    double cmd_vel_x, double cmd_vel_y, double cmd_vel_z, double& front_right, double& front_left, double& back_right, double& back_left)
{
    double theta = std::atan2(cmd_vel_y, cmd_vel_x);
    double norm = std::sqrt(std::pow(cmd_vel_x, 2) + std::pow(cmd_vel_y, 2));

    // rad/s = (m/s) * (rad) / (wheel_d)
    front_right = norm * std::sin(theta + M_PI * 1.25) * (2.0 * M_PI) / (M_PI * wheel_d_);
    front_left = norm * std::sin(theta - M_PI * 1.25) * (2.0 * M_PI) / (M_PI * wheel_d_);
    back_right = norm * std::sin(theta - M_PI * 0.25) * (2.0 * M_PI) / (M_PI * wheel_d_);
    back_left = norm * std::sin(theta + M_PI * 0.25) * (2.0 * M_PI) / (M_PI * wheel_d_);

    // rad/s = (rad/s) * (body_d / wheel_d)
    front_right -= cmd_vel_z * body_d_ / wheel_d_;
    front_left -= cmd_vel_z * body_d_ / wheel_d_;
    back_right -= cmd_vel_z * body_d_ / wheel_d_;
    back_left -= cmd_vel_z * body_d_ / wheel_d_;
}
} // namespace rabcl
