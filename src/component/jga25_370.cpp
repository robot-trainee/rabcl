#include "rabcl/component/jga25_370.hpp"

#include <math.h>
#include <numeric>

namespace rabcl
{
JGA25_370::JGA25_370(
    uint16_t motor_output_max, double reduction_ratio, uint8_t control_mode)
:   motor_output_max_(motor_output_max), control_mode_(control_mode),
    encoder_count_sum_(0), pre_diff_(0.0), cmd_value_(0.0), pre_output_(0.0)
{
    count_to_pos_factor_ =
        2.0 * M_PI / (NUM_ENCODER_RESOLUTION * reduction_ratio);
    count_to_vel_factor_ =
        2.0 * M_PI / (NUM_ENCODER_RESOLUTION * ENCODER_SAMPLING_FREQUENCY * reduction_ratio);
}

JGA25_370::~JGA25_370()
{
    // NOP
}

void JGA25_370::SetCmdValue(double cmd_val)
{
    cmd_value_ = cmd_val;
}

void JGA25_370::SetEncoderCount(int16_t encoder_count)
{
    encoder_count_buf_.insert(encoder_count_buf_.begin(), encoder_count);
    encoder_count_buf_.resize(NUM_ENCODER_DATA_BUF);
    encoder_count_sum_ += encoder_count;
}

void JGA25_370::UpdataEncoder()
{
    int16_t encoder_count = std::accumulate(
        std::begin(encoder_count_buf_),
        std::end(encoder_count_buf_),
        std::remove_reference<decltype(*std::begin(encoder_count_buf_))>::type{});
    act_pos_ = -1.0 * (double)encoder_count_sum_ * count_to_pos_factor_;
    act_pos_ = std::fmod(act_pos_, 2.0 * M_PI);
    if (act_pos_ < 2.0 * M_PI) {
        act_pos_ += 2.0 * M_PI;
    }
    act_vel_ = -1.0 * (double)encoder_count * count_to_vel_factor_;
}

int16_t JGA25_370::CalcMotorOutput()
{
    if (0 == control_mode_) {
        return CalcMotorPosOutput();
    } else if (1 == control_mode_) {
        return CalcMotorVelOutput();
    }
    return 0;
}

double JGA25_370::GetCmdValue()
{
    return cmd_value_;
}

double JGA25_370::GetActVel()
{
    return act_vel_;
}

double JGA25_370::GetActPos()
{
    return act_pos_;
}

int16_t JGA25_370::CalcMotorPosOutput()
{
    // TODO: 制御器入れる、とりあえず適当なPD制御入れとく
    // TODO: -pi ~ piで値取らないとだめな気がする
    int16_t error;
    double diff = cmd_value_ - act_pos_;
    error = (int16_t)(
        MOTOR_CONTROL_POS_P * diff + MOTOR_CONTROL_POS_D * (diff - pre_diff_));
    pre_diff_ = diff;

    int16_t output = pre_output_ + error;

    if (std::abs(output) > motor_output_max_)
    {
        if (output >= 0)
        {
            output = (int16_t)motor_output_max_;
        }
        else
        {
            output = -1.0 * (int16_t)motor_output_max_;
        }
    }
    pre_output_ = output;

    return output;
}

int16_t JGA25_370::CalcMotorVelOutput()
{
    // TODO: 制御器入れる、とりあえず適当なPD制御入れとく
    int16_t error;
    double diff = cmd_value_ - act_vel_;
    error = (int16_t)(
        MOTOR_CONTROL_VEL_P * diff + MOTOR_CONTROL_VEL_D * (diff - pre_diff_));
    pre_diff_ = diff;

    int16_t output = pre_output_ + error;

    if (std::abs(output) > motor_output_max_)
    {
        if (output >= 0)
        {
            output = (int16_t)motor_output_max_;
        }
        else
        {
            output = -1.0 * (int16_t)motor_output_max_;
        }
    }
    pre_output_ = output;

    return output;
}
} // namespace rabcl
