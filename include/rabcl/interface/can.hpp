#ifndef RABCL__INTERFACE__CAN_HPP_
#define RABCL__INTERFACE__CAN_HPP_

#include <cstdint>

#include "rabcl/utils/type.hpp"

namespace rabcl
{
class Can
{
public:
  // DM2325 decode parameters (must match DM AC-Tools settings)
  static constexpr float DM_PMAX = 78.5f;   // [rad] motor shaft
  static constexpr float DM_VMAX = 200.0f;  // [rad/s] motor shaft
  static constexpr float DM_TMAX = 3.0f;    // [Nm] output shaft
  static constexpr float DM_GR   = 25.0f;   // gear ratio

public:
  Can();
  ~Can();
  static bool UpdateData(uint32_t idx, const uint8_t can_data[8], Info & data);
  static void Prepare2FloatData(float in_1, float in_2, uint8_t can_data[8]);
  static void Prepare1Float4IntData(float in_1, const uint8_t in_2[4], uint8_t can_data[8]);
  static void PrepareLKMotorMotorOff(uint8_t can_data[8]);
  static void PrepareLKMotorMotorOn(uint8_t can_data[8]);
  static void PrepareLKMotorMotorStop(uint8_t can_data[8]);
  static void PrepareLKMotorPositionCmd(int32_t pos /* [0.01 deg] */, uint16_t max_speed /* [dps] */, uint8_t can_data[8]);
  static void PrepareLKMotorReadParam(uint8_t param_id, uint8_t can_data[8]);
  static void PrepareLKMotorWritePID(uint8_t param_id, uint16_t kp, uint16_t ki, uint16_t kd, uint8_t can_data[8]);
  static void PrepareRMDMotorReadPID(uint8_t can_data[8]);
  static void PrepareRMDMotorWritePIDToRAM(
    uint8_t curr_kp, uint8_t curr_ki,
    uint8_t speed_kp, uint8_t speed_ki,
    uint8_t pos_kp, uint8_t pos_ki,
    uint8_t can_data[8]);
  static void PrepareDMMotorEnable(uint8_t can_data[8]);
  static void PrepareDMMotorVelocityCmd(float velocity /* [rad/s, output shaft] */, uint8_t can_data[8]);

private:
  static void ParseLKMotorFeedback(const uint8_t can_data[8], MotorInfo & motor);
  static void ParseDMMotorFeedback(const uint8_t can_data[8], MotorInfo & motor);
};
}  // namespace rabcl

#endif  // RABCL__INTERFACE__CAN_HPP_
