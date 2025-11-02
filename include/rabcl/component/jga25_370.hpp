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
    static constexpr double MOTOR_CONTROL_POS_P = 0.1;
    static constexpr double MOTOR_CONTROL_POS_D = 0.0;
    static constexpr double MOTOR_CONTROL_VEL_P = 1.6;
    static constexpr double MOTOR_CONTROL_VEL_D = 0.2;

public:
    JGA25_370(
        uint16_t motor_output_max, double reduction_ratio, uint8_t control_mode = 1);
    ~JGA25_370();

    void SetCmdValue(double cmd_value);
    void SetEncoderCount(int16_t encoder_count);
    void UpdataEncoder();
    int16_t CalcMotorOutput();

    double GetCmdValue();
    double GetActVel();
    double GetActPos();

private:
    int16_t CalcMotorPosOutput();
    int16_t CalcMotorVelOutput();

private:
    uint16_t motor_output_max_;
    double count_to_pos_factor_; // rad
    double count_to_vel_factor_; // rad/s
    uint8_t control_mode_; // 0: pos, 1: vel

    std::vector<int16_t> encoder_count_buf_;
    int32_t encoder_count_sum_;
    double pre_diff_; // rad or rad/s
    double cmd_value_; // rad or rad/s
    double act_pos_; // rad
    double act_vel_; // rad/s
    double pre_output_;
};
} // namespace rabcl

#endif
