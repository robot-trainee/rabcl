#ifndef RABCL_JGA25_370_HPP
#define RABCL_JGA25_370_HPP

#include <cstdint>
#include <vector>

namespace rabcl
{
class JGA25_370
{
private:
    static constexpr double ENCODER_SAMPLING_FREQUENCY = 0.2;
    static constexpr int NUM_ENCODER_DATA_BUF = 20; // motor:10ms, sampling_frequency:200ms
    static constexpr int NUM_ENCODER_RESOLUTION = 44;
    static constexpr double MOTOR_CONTROL_P = 3.0;
    static constexpr double MOTOR_CONTROL_D = 0.4;

public:
    JGA25_370(
        uint16_t motor_output_max, double reduction_ratio);
    ~JGA25_370();

    void SetCmdVel(double cmd_vel);
    void SetEncoderCount(int16_t encoder_count);
    void UpdataEncoder();
    int16_t CalcMotorOutput();

    double GetCmdVel();
    double GetActVel();

private:
    uint16_t motor_output_max_;
    double count_to_vel_factor_; // rad

    std::vector<int16_t> encoder_count_buf_;
    double pre_diff_; // rad
    double cmd_vel_; // rad
    double act_vel_; // rad
    double pre_output_;
};
} // namespace rabcl

#endif
