#include "rabcl/interface/can.hpp"

namespace rabcl
{
Can::Can()
{
    // NOP
}

Can::~Can()
{
    // NOP
}

void Can::Prepare2FloatData(float in_1, float in_2, uint8_t can_data[8])
{
    union {
        float f;
        int32_t ui;
    } buf;
    buf.f = in_1;
    can_data[0] = (uint8_t)((buf.ui & 0xFF000000) >> 24);
    can_data[1] = (uint8_t)((buf.ui & 0x00FF0000) >> 16);
    can_data[2] = (uint8_t)((buf.ui & 0x0000FF00) >> 8);
    can_data[3] = (uint8_t)((buf.ui & 0x000000FF) >> 0);
    buf.f = in_2;
    can_data[4] = (uint8_t)((buf.ui & 0xFF000000) >> 24);
    can_data[5] = (uint8_t)((buf.ui & 0x00FF0000) >> 16);
    can_data[6] = (uint8_t)((buf.ui & 0x0000FF00) >> 8);
    can_data[7] = (uint8_t)((buf.ui & 0x000000FF) >> 0);
}

void Can::Prepare1Float4IntData(float in_1, const uint8_t in_2[4], uint8_t can_data[8])
{
    union {
        float f;
        int32_t ui;
    } buf;
    buf.f = in_1;
    can_data[0] = (uint8_t)((buf.ui & 0xFF000000) >> 24);
    can_data[1] = (uint8_t)((buf.ui & 0x00FF0000) >> 16);
    can_data[2] = (uint8_t)((buf.ui & 0x0000FF00) >> 8);
    can_data[3] = (uint8_t)((buf.ui & 0x000000FF) >> 0);
    can_data[4] = in_2[0];
    can_data[5] = in_2[1];
    can_data[6] = in_2[2];
    can_data[7] = in_2[3];
}
} // namespace rabcl
