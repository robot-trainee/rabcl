#ifndef RABCL_CAN_HPP
#define RABCL_CAN_HPP

#include "rabcl/utils/type.hpp"

#include <cstdint>

namespace rabcl
{
class Can
{
public:
    Can();
    ~Can();
    static bool UpdateData(uint32_t idx, const uint8_t can_data[8], Info& data);
    static void Prepare2FloatData(float in_1, float in_2, uint8_t can_data[8]);
    static void Prepare1Float4IntData(float in_1, const uint8_t in_2[4], uint8_t can_data[8]);

};
} // namespace rabcl

#endif
