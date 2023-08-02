#pragma once

#include <Mitov.h>
#include <Wire.h>
#include <MFRC522v2.h>
#include <MFRC522Constants.h>
#include <MFRC522Driver.h>

namespace Swellington
{

    class MFRC522DriverI2C_Visuino : public MFRC522Driver
    {

    public:
        bool init() override
        {
            return true;
        };

        void PCD_WriteRegister(const PCD_Register reg, const byte value) override
        {
            _wire.beginTransmission(_slaveAdr);
            _wire.write(reg);
            _wire.write(value);
            _wire.endTransmission();
        };

        void PCD_WriteRegister(const PCD_Register reg, const byte count, byte *const values) override
        {
            _wire.beginTransmission(_slaveAdr);
            _wire.write(reg);
            _wire.write(values, count);
            _wire.endTransmission();
        };

        byte PCD_ReadRegister(const PCD_Register reg) override
        {
            byte value;

            _wire.beginTransmission(_slaveAdr);
            _wire.write(reg);
            _wire.endTransmission();

            _wire.requestFrom(_slaveAdr, (uint8_t)1);
            // while(!_wire.available()); // Dangerous! Might block! Wait for byte to be available. TODO timeout detection.
            value = (uint8_t)_wire.read();
            _wire.endTransmission();

            return value;
        };

        void PCD_ReadRegister(const PCD_Register reg, const byte count, byte *const values, const byte rxAlign = 0) override
        {
            if (count == 0 || values == nullptr)
            {
                return;
            }

            byte index = 0;

            _wire.beginTransmission(_slaveAdr);
            _wire.write(reg); // Tell MFRC522 which register address we want to read.
            _wire.endTransmission();

            _wire.requestFrom(_slaveAdr, count);

            // Todo: is waiting for byte to be available required?
            // With thanks to arozcan (https://github.com/arozcan/MFRC522-I2C-Library), but slightly modified:
            while (_wire.available() && index < count)
            {
                if (index == 0 && rxAlign)
                { // Only update bit positions rxAlign..7 in values[0]
                    // Create bit mask for bit positions rxAlign..7
                    byte mask = 0;

                    for (byte i = rxAlign; i <= 7; i++)
                    {
                        mask |= (1 << i);
                    }
                    // Read value and tell that we want to read the same address again.
                    byte value = (byte)_wire.read(); // returns int but only with uint8 content

                    // Apply mask to both current value of values[0] and the new data in value.
                    values[0] = (values[index] & ~mask) | (value & mask);
                }
                else
                {                                       // Normal case
                    values[index] = (byte)_wire.read(); // returns int but only with uint8 content
                }
                index++;
            }
        };

        MFRC522DriverI2C_Visuino(
            TwoWire &wire,
            byte slaveAdr = 0x28) : MFRC522Driver(),
                                    _slaveAdr(slaveAdr),
                                    _wire(wire)
        {
        }

    protected:
        byte _slaveAdr;
        TwoWire &_wire;
    };

    template <
        typename T_I2C, T_I2C &C_I2C,
        typename T_Address,
        typename T_AntennaGain,
        typename T_CheckInterval,
        typename T_Enabled,
        typename T_StartedOutputPin,
        typename T_TagKeyOutputPin>
    class RFID_I2C : public T_Address,
                     public T_AntennaGain,
                     public T_CheckInterval,
                     public T_Enabled,
                     public T_StartedOutputPin,
                     public T_TagKeyOutputPin
    {
    private:
        using Uid = MFRC522Constants::Uid;
        bool started = false;
        unsigned long FLastTime = 0L;
        MFRC522 *mfrc522;

    private:
        void SetSystemStarted(bool state)
        {
            started = state;
            if (T_StartedOutputPin::GetPinIsConnected())
            {
                if (started)
                {
                    T_StartedOutputPin::SetPinValueHigh();
                }
                else
                {
                    T_StartedOutputPin::SetPinValueLow();
                }
            }
        }

        void SetAntennaGain()
        {
            uint8_t mask = 0;
            switch (AntennaGain().GetValue())
            {
            case 18:
            {
                mask = 0b000;
                break;
            }
            case 23:
            {
                mask = 0b001;
                break;
            }
            case 33:
            {
                mask = 0b100;
                break;
            }
            case 38:
            {
                mask = 0b101;
                break;
            }
            case 43:
            {
                mask = 0b110;
                break;
            }
            case 48:
            {
                mask = 0b111;
                break;
            }
            }
            mfrc522->PCD_SetAntennaGain(mask);
        }

    public:
        _V_PIN_(TagKeyOutputPin)
        _V_PIN_(StartedOutputPin)

    public:
        _V_PROP_(Address)
        _V_PROP_(Enabled)
        _V_PROP_(AntennaGain)
        _V_PROP_(CheckInterval)

    public:
        inline void SystemInit()
        {
            SetSystemStarted(false);
            MFRC522DriverI2C_Visuino driver{C_I2C, Address().GetValue()};
            mfrc522 = new MFRC522{driver};
            FLastTime = millis();
            mfrc522->PCD_Init();
            SetAntennaGain();
            while (millis() - FLastTime < 2000)
                ;
            SetSystemStarted(true);
        }

        inline void SystemLoopBegin()
        {
            if (millis() - FLastTime > CheckInterval().GetValue() && Enabled() && started)
            {
                if (mfrc522->PICC_IsNewCardPresent() && mfrc522->PICC_ReadCardSerial())
                {
                    Uid _uid = mfrc522->uid;
                    Mitov::String _serial;
                    _serial.reserve(_uid.size);
                    for (int i = 0; i < _uid.size; i++)
                    {
                        _serial += Mitov::String(_uid.uidByte[i], HEX);
                    }
                    T_TagKeyOutputPin::SetPinValue(_serial);
                }

                FLastTime = millis();
            }
        }
    };
} // Swellington