#pragma once

#include <Mitov.h>
#include <stdint.h> // uint32_t

namespace Swellington
{
    class InverseSquareRoot
    {
    private:
        inline static float Q_rsqrt(float number)
        {
            union
            {
                float f;
                uint32_t i;
            } conv = {.f = number};
            conv.i = 0x5f3759df - (conv.i >> 1);
            conv.f *= 1.5F - (number * 0.5F * conv.f * conv.f);
            return conv.f;
        }

    public:
        inline static float CalculateOutput(float AInValue)
        {
            return InverseSquareRoot::Q_rsqrt(AInValue);
        }
    };
}
