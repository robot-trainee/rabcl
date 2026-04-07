#include "rabcl/component/bno055.hpp"

namespace rabcl
{
BNO055::BNO055() {}
BNO055::~BNO055() {}

void BNO055::UpdateAccel(const uint8_t raw[6], ImuInfo & imu)
{
  imu.acc_x_ = static_cast<int16_t>((raw[1] << 8) | raw[0]) * ACC_SCALE;
  imu.acc_y_ = static_cast<int16_t>((raw[3] << 8) | raw[2]) * ACC_SCALE;
  imu.acc_z_ = static_cast<int16_t>((raw[5] << 8) | raw[4]) * ACC_SCALE;
}

void BNO055::UpdateGyro(const uint8_t raw[6], ImuInfo & imu)
{
  imu.gyro_x_ = static_cast<int16_t>((raw[1] << 8) | raw[0]) * GYRO_SCALE;
  imu.gyro_y_ = static_cast<int16_t>((raw[3] << 8) | raw[2]) * GYRO_SCALE;
  imu.gyro_z_ = static_cast<int16_t>((raw[5] << 8) | raw[4]) * GYRO_SCALE;
}

void BNO055::UpdateEuler(const uint8_t raw[6], ImuInfo & imu)
{
  imu.euler_heading_ = static_cast<int16_t>((raw[1] << 8) | raw[0]) * EULER_SCALE;
  imu.euler_roll_ = static_cast<int16_t>((raw[3] << 8) | raw[2]) * EULER_SCALE;
  imu.euler_pitch_ = static_cast<int16_t>((raw[5] << 8) | raw[4]) * EULER_SCALE;
}
}  // namespace rabcl
