#include "rabcl/component/ld_20mg.hpp"

#include <numeric>

namespace rabcl
{
LD_20MG::LD_20MG(
    double min_pos, double max_pos, double offset_pos, double reduction_ratio)
: min_pos_(min_pos), max_pos_(max_pos), offset_pos_(offset_pos), reduction_ratio_(reduction_ratio)
{
    if (min_pos_ < 0.0)
    {
        min_pos_ = 0.0;
    }

    if (max_pos_ > MAX_POS * reduction_ratio_)
    {
        max_pos_ = MAX_POS * reduction_ratio_;
    }

    cmd_pos_ = min_pos_;
}

LD_20MG::~LD_20MG()
{
    // NOP
}

void LD_20MG::Updata(double cmd_pos)
{
    cmd_pos_ = cmd_pos + offset_pos_;

    if (cmd_pos_ < min_pos_)
    {
        cmd_pos_ = min_pos_;
    }
    else if (cmd_pos_ > max_pos_)
    {
        cmd_pos_ = max_pos_;
    }
}

int16_t LD_20MG::CalcMotorOutput()
{
    double cmd_pos_in = cmd_pos_ / reduction_ratio_;
    int16_t output = (int16_t)((MIN_PULSE_WIDTH / CONTROL_RESOLUTION) +
        ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / CONTROL_RESOLUTION) * cmd_pos_in / MAX_POS);
    return output;
}

double LD_20MG::GetCmdPos()
{
    return cmd_pos_;
}
} // namespace rabcl
