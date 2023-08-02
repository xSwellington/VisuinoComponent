#pragma once

#include <Mitov.h>

namespace Swellington
{

    template <typename T_OutputPin>
    class TextToJoaat : public T_OutputPin
    {
    private:
        static uint32_t _joaat(const Mitov::String &str)
        {
            uint32_t hash = 0;
            for (const char c : str)
            {
                hash += c;
                hash += (hash << 10);
                hash ^= (hash >> 6);
            }
            hash += (hash << 3);
            hash ^= (hash >> 11);
            hash += (hash << 15);
            return hash;
        }

    public:
        _V_PIN_(OutputPin)

    public:
        void InputPin_o_Receive(void *_Data)
        {
           char *AText = (char *)_Data;            
           uint32_t AValue = _joaat(Mitov::String(AText));
           OutputPin().SetPinValue( AValue );
        }
    };

}