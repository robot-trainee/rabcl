#ifndef RABCL__COMPONENT__BNO055_HPP_
#define RABCL__COMPONENT__BNO055_HPP_

#include <cstdint>
#include <cmath>
#include "rabcl/utils/type.hpp"

namespace rabcl
{
class BNO055
{
public:
  static constexpr uint8_t I2C_ADDR = 0x28 << 1;
  static constexpr uint8_t CHIP_ID_ADDR = 0x00;
  static constexpr uint8_t OPR_MODE_ADDR = 0x3D;
  static constexpr uint8_t ACC_DATA_X_LSB = 0x08;
  static constexpr uint8_t GYR_DATA_X_LSB = 0x14;
  static constexpr uint8_t EULER_H_LSB = 0x1A;
  static constexpr uint8_t EXPECTED_CHIP_ID = 0xA0;
  static constexpr uint8_t MODE_CONFIG = 0x00;
  static constexpr uint8_t MODE_NDOF = 0x0C;

  BNO055();
  ~BNO055();

  void UpdateAccel(const uint8_t raw[6], ImuInfo & imu);
  void UpdateGyro(const uint8_t raw[6], ImuInfo & imu);
  void UpdateEuler(const uint8_t raw[6], ImuInfo & imu);

private:
  static constexpr float ACC_SCALE = 0.01f;
  static constexpr float GYRO_SCALE = (1.0f / 16.0f) * (M_PI / 180.0f);
  static constexpr float EULER_SCALE = (1.0f / 16.0f) * (M_PI / 180.0f);
};
}  // namespace rabcl

#endif  // RABCL__COMPONENT__BNO055_HPP_
