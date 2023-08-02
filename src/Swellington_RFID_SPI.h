#pragma once

#include <Mitov.h>
#include <Mitov_Arduino_SPI.h>
#include <MFRC522v2.h>
#include <MFRC522Constants.h>
#include <MFRC522Driver.h>
#include <MFRC522DriverPin.h>
#include <MFRC522DriverPinSimple.h>

namespace Swellington
{

    class MFRC522DriverSPI_Visuino : public MFRC522Driver
    {

    public:
        bool init() override
        {
            _spiClass.SystemInit();
            if (_chipSelectPin.init() == false)
                return false;
            _chipSelectPin.high();
            return true;
        };

        void PCD_WriteRegister(const PCD_Register reg, const byte value) override
        {
            _spiClass.beginTransaction(_spiSettings);
            _chipSelectPin.low();
            _spiClass.transfer(reg << 1);
            _spiClass.transfer(value);
            _chipSelectPin.high();
            _spiClass.endTransaction();
        };

        void PCD_WriteRegister(const PCD_Register reg, const byte count, byte *const values) override
        {
            _spiClass.beginTransaction(_spiSettings);
            _chipSelectPin.low();
            _spiClass.transfer(reg << 1);
            for (byte index = 0; index < count; index++)
            {
                _spiClass.transfer(values[index]);
            }
            _chipSelectPin.high();
            _spiClass.endTransaction();
        };

        byte PCD_ReadRegister(const PCD_Register reg) override
        {
            byte value;
            _spiClass.beginTransaction(_spiSettings);    // Set the settings to work with SPI bus
            _chipSelectPin.low();                        // Select slave
            _spiClass.transfer((byte)0x80 | (reg << 1)); // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
            value = _spiClass.transfer(0);               // Read the value back. Send 0 to stop reading.
            _chipSelectPin.high();                       // Release slave again
            _spiClass.endTransaction();                  // Stop using the SPI bus
            return value;
        };

        void PCD_ReadRegister(const PCD_Register reg, const byte count, byte *const values, const byte rxAlign = 0) override
        {
            if (count == 0)
                return;
            // Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
            byte address = (byte)0x80 | (reg << 1);   // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
            byte index = 0;                           // Index in values array.
            _spiClass.beginTransaction(_spiSettings); // Set the settings to work with SPI bus
            _chipSelectPin.low();                     // Select slave
            // count--;								// One read is performed outside of the loop // TODO is this correct?
            _spiClass.transfer(address); // Tell MFRC522 which address we want to read
            if (rxAlign)
            { // Only update bit positions rxAlign..7 in values[0]
                // Create bit mask for bit positions rxAlign..7
                byte mask = (byte)(0xFF << rxAlign) & 0xFF;
                // Read value and tell that we want to read the same address again.
                byte value = _spiClass.transfer(address);
                // Apply mask to both current value of values[0] and the new data in value.
                values[0] = (values[0] & ~mask) | (value & mask);
                index++;
            }
            // while (index < count) { // changed because count changed to const
            while (index < count - 1)
            {
                values[index] = _spiClass.transfer(address); // Read value and tell that we want to read the same address again.
                index++;
            }
            values[index] = _spiClass.transfer(0); // Read the final byte. Send 0 to stop reading.
            _chipSelectPin.high();                 // Release slave again
            _spiClass.endTransaction();            // Stop using the SPI bus
        };

        MFRC522DriverSPI_Visuino(
            MFRC522DriverPinSimple pin,
            Mitov::ArduinoSPI<SPI> &spiClass,
            const SPISettings spiSettings = SPISettings(4000000u /* 4MHz */, MSBFIRST, SPI_MODE0)) : MFRC522Driver(),
                                                                                                     _chipSelectPin(pin),
                                                                                                     _spiClass(spiClass),
                                                                                                     _spiSettings(spiSettings) {}

    protected:
        MFRC522DriverPin &_chipSelectPin;
        Mitov::ArduinoSPI<SPI> &_spiClass;
        const SPISettings _spiSettings;
    };

    template <
        typename T_SPI, T_SPI &C_SPI,
        typename T_AntennaGain,
        typename T_CheckInterval,
        int T_ChipSelectOutputPin,
        typename T_Enabled,
        typename T_StartedOutputPin,
        typename T_TagKeyOutputPin>
    class RFID_SPI : public T_AntennaGain,
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
        _V_PROP_(Enabled)
        _V_PROP_(AntennaGain)
        _V_PROP_(CheckInterval)

    public:
        inline void SystemInit()
        {
            SetSystemStarted(false);
            MFRC522DriverPinSimple ss_pin(T_ChipSelectOutputPin);
            MFRC522DriverSPI_Visuino driver{ss_pin, C_SPI};
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