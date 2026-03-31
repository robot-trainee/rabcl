#ifndef RABCL__INTERFACE__CAN_HPP_
#define RABCL__INTERFACE__CAN_HPP_

#include <cstdint>

#include "rabcl/utils/type.hpp"

namespace rabcl
{
class Can
{
public:
  Can();
  ~Can();
  static bool UpdateData(uint32_t idx, const uint8_t can_data[8], Info & data);
  static void Prepare2FloatData(float in_1, float in_2, uint8_t can_data[8]);
  static void Prepare1Float4IntData(float in_1, const uint8_t in_2[4], uint8_t can_data[8]);
  static void PrepareLKMotorMotorOffCmd(uint8_t can_data[8]);
  static void PrepareLKMotorMotorOnCmd(uint8_t can_data[8]);
  static void PrepareLKMotorMotorStopCmd(uint8_t can_data[8]);
  static void PrepareLKMotorPositionCmd(int32_t pos, uint16_t max_speed, uint8_t can_data[8]);
};
}  // namespace rabcl

#endif  // RABCL__INTERFACE__CAN_HPP_
