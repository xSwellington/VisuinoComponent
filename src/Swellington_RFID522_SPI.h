
#ifndef _SWELLINGTON_RFID522_SPI_H
#define _SWELLINGTON_RFID522_SPI_H

#pragma once

#include <Mitov.h>

namespace Swellington
{
    namespace MIFARE
    {
        typedef struct
        {
            byte size;
            byte uidByte[10];
            byte sak;
        } Uid;
    }

    template <
        typename T_OutputPin>
    class TArduinoRFID522Key : public T_OutputPin
    {
    public:
        _V_PIN_(OutputPin)
    };

    template <
        typename T_SPI, T_SPI &C_SPI,
        typename T_ChipSelectOutputPin,
        typename T_Enabled,
        typename T_Key,
        uint8_t C_ResetOutputPin>
    class RFID522_SPI : public T_ChipSelectOutputPin,
                        public T_Enabled,
                        public T_Key
    {

    public:
        _V_PIN_(ChipSelectOutputPin)
        _V_PROP_(Key)
        _V_PROP_(Enabled)

    public:
        // Size of the MFRC522 FIFO
        static constexpr byte FIFO_SIZE = 64; // The FIFO is 64 bytes.
        // Default value for unused pin
        static constexpr uint8_t UNUSED_PIN = UINT8_MAX;

        // MFRC522 registers. Described in chapter 9 of the datasheet.
        // When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
        enum PCD_Register : byte
        {
            // Page 0: Command and status
            CommandReg = 0x02,
            ComIEnReg = 0x04,
            DivIEnReg = 0x06,
            ComIrqReg = 0x08,
            DivIrqReg = 0x0A,
            ErrorReg = 0x0C,
            Status1Reg = 0x0E,
            Status2Reg = 0x10,
            FIFODataReg = 0x12,
            FIFOLevelReg = 0x14,
            WaterLevelReg = 0x16,
            ControlReg = 0x18,
            BitFramingReg = 0x1A,
            CollReg = 0x1C,

            // Page 1: Command
            ModeReg = 0x22,
            TxModeReg = 0x24,
            RxModeReg = 0x26,
            TxControlReg = 0x28,
            TxASKReg = 0x2A,
            TxSelReg = 0x2C,
            RxSelReg = 0x2E,
            RxThresholdReg = 0x30,
            DemodReg = 0x32,
            MfTxReg = 0x38,
            MfRxReg = 0x3A,
            SerialSpeedReg = 0x3E,

            // Page 2: Configuration
            CRCResultRegH = 0x42,
            CRCResultRegL = 0x44,
            ModWidthReg = 0x48,
            RFCfgReg = 0x4C,
            GsNReg = 0x4E,
            CWGsPReg = 0x50,
            ModGsPReg = 0x52,
            TModeReg = 0x54,
            TPrescalerReg = 0x56,
            TReloadRegH = 0x58,
            TReloadRegL = 0x5A,
            TCounterValueRegH = 0x5C,
            TCounterValueRegL = 0x5E,

            // Page 3: Test Registers
            TestSel1Reg = 0x62,
            TestSel2Reg = 0x64,
            TestPinEnReg = 0x66,
            TestPinValueReg = 0x68,
            TestBusReg = 0x6A,
            AutoTestReg = 0x6C,
            VersionReg = 0x6E,
            AnalogTestReg = 0x70,
            TestDAC1Reg = 0x72,
            TestDAC2Reg = 0x74,
            TestADCReg = 0x76
        };

        // MFRC522 commands. Described in chapter 10 of the datasheet.
        enum PCD_Command : byte
        {
            PCD_Idle = 0x00,             // no action, cancels current command execution
            PCD_Mem = 0x01,              // stores 25 bytes into the internal buffer
            PCD_GenerateRandomID = 0x02, // generates a 10-byte random ID number
            PCD_CalcCRC = 0x03,          // activates the CRC coprocessor or performs a self-test
            PCD_Transmit = 0x04,         // transmits data from the FIFO buffer
            PCD_NoCmdChange = 0x07,      // no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
            PCD_Receive = 0x08,          // activates the receiver circuits
            PCD_Transceive = 0x0C,       // transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
            PCD_MFAuthent = 0x0E,        // performs the MIFARE standard authentication as a reader
            PCD_SoftReset = 0x0F         // resets the MFRC522
        };

        // MFRC522 RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the PCD).
        // Described in 9.3.3.6 / table 98 of the datasheet at http://www.nxp.com/documents/data_sheet/MFRC522.pdf
        enum PCD_RxGain : byte
        {
            RxGain_18dB = 0x00,   // 000b - 18 dB, minimum
            RxGain_23dB = 0x10,   // 001b - 23 dB
            RxGain_18dB_2 = 0x20, // 010b - 18 dB, it seems 010b is a duplicate for 000b
            RxGain_23dB_2 = 0x30, // 011b - 23 dB, it seems 011b is a duplicate for 001b
            RxGain_33dB = 0x40,   // 100b - 33 dB, average, and typical default
            RxGain_38dB = 0x50,   // 101b - 38 dB
            RxGain_43dB = 0x60,   // 110b - 43 dB
            RxGain_48dB = 0x70,   // 111b - 48 dB, maximum
            RxGain_min = 0x00,    // 000b - 18 dB, minimum, convenience for RxGain_18dB
            RxGain_avg = 0x40,    // 100b - 33 dB, average, convenience for RxGain_33dB
            RxGain_max = 0x70     // 111b - 48 dB, maximum, convenience for RxGain_48dB
        };

        // Commands sent to the PICC.
        enum PICC_Command : byte
        {
            // The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
            PICC_CMD_REQA = 0x26,          // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
            PICC_CMD_WUPA = 0x52,          // Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
            PICC_CMD_CT = 0x88,            // Cascade Tag. Not really a command, but used during anti collision.
            PICC_CMD_SEL_CL1 = 0x93,       // Anti collision/Select, Cascade Level 1
            PICC_CMD_SEL_CL2 = 0x95,       // Anti collision/Select, Cascade Level 2
            PICC_CMD_SEL_CL3 = 0x97,       // Anti collision/Select, Cascade Level 3
            PICC_CMD_HLTA = 0x50,          // HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
            PICC_CMD_RATS = 0xE0,          // Request command for Answer To Reset.
                                           // The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
                                           // Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
                                           // The read/write commands can also be used for MIFARE Ultralight.
            PICC_CMD_MF_AUTH_KEY_A = 0x60, // Perform authentication with Key A
            PICC_CMD_MF_AUTH_KEY_B = 0x61, // Perform authentication with Key B
            PICC_CMD_MF_READ = 0x30,       // Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
            PICC_CMD_MF_WRITE = 0xA0,      // Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
            PICC_CMD_MF_DECREMENT = 0xC0,  // Decrements the contents of a block and stores the result in the internal data register.
            PICC_CMD_MF_INCREMENT = 0xC1,  // Increments the contents of a block and stores the result in the internal data register.
            PICC_CMD_MF_RESTORE = 0xC2,    // Reads the contents of a block into the internal data register.
            PICC_CMD_MF_TRANSFER = 0xB0,   // Writes the contents of the internal data register to a block.
                                           // The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
                                           // The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
            PICC_CMD_UL_WRITE = 0xA2       // Writes one 4 byte page to the PICC.
        };

        // MIFARE constants that does not fit anywhere else
        enum MIFARE_Misc
        {
            MF_ACK = 0xA,   // The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
            MF_KEY_SIZE = 6 // A Mifare Crypto1 key is 6 bytes.
        };

        // PICC types we can detect. Remember to update PICC_GetTypeName() if you add more.
        // last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
        enum PICC_Type : byte
        {
            PICC_TYPE_UNKNOWN,
            PICC_TYPE_ISO_14443_4,        // PICC compliant with ISO/IEC 14443-4
            PICC_TYPE_ISO_18092,          // PICC compliant with ISO/IEC 18092 (NFC)
            PICC_TYPE_MIFARE_MINI,        // MIFARE Classic protocol, 320 bytes
            PICC_TYPE_MIFARE_1K,          // MIFARE Classic protocol, 1KB
            PICC_TYPE_MIFARE_4K,          // MIFARE Classic protocol, 4KB
            PICC_TYPE_MIFARE_UL,          // MIFARE Ultralight or Ultralight C
            PICC_TYPE_MIFARE_PLUS,        // MIFARE Plus
            PICC_TYPE_MIFARE_DESFIRE,     // MIFARE DESFire
            PICC_TYPE_TNP3XXX,            // Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
            PICC_TYPE_NOT_COMPLETE = 0xff // SAK indicates UID is not complete.
        };

        // Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
        // last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
        enum StatusCode : byte
        {
            STATUS_OK,                // Success
            STATUS_ERROR,             // Error in communication
            STATUS_COLLISION,         // Collission detected
            STATUS_TIMEOUT,           // Timeout in communication.
            STATUS_NO_ROOM,           // A buffer is not big enough.
            STATUS_INTERNAL_ERROR,    // Internal error in the code. Should not happen ;-)
            STATUS_INVALID,           // Invalid argument.
            STATUS_CRC_WRONG,         // The CRC_A does not match
            STATUS_MIFARE_NACK = 0xff // A MIFARE PICC responded with NAK.
        };

    private:
        MIFARE::Uid _uid;

    private:
        void PCD_SetRegisterBitMask(PCD_Register reg, uint8_t mask)
        {
            PCD_WriteRegister(reg, PCD_ReadRegister(reg) | mask);
        }

        void PCD_ClearRegisterBitMask(PCD_Register reg, uint8_t mask)
        {
            PCD_WriteRegister(reg, PCD_ReadRegister(reg) & (~mask));
        }

        void PCD_WriteRegister(PCD_Register reg, uint8_t count, uint8_t *values)
        {
            C_SPI.beginTransaction(SPISettings(4000000u, MSBFIRST, SPI_MODE0));
            T_ChipSelectOutputPin::SetPinValueLow();
            C_SPI.transfer(reg & 0x7E);
            for (int index = 0; index < count; index++)
            {
                C_SPI.transfer(values[index]);
            }
            T_ChipSelectOutputPin::SetPinValueHigh();
            C_SPI.endTransaction();
        }

        void PCD_WriteRegister(PCD_Register reg, uint8_t value)
        {
            C_SPI.beginTransaction(SPISettings(4000000u, MSBFIRST, SPI_MODE0));
            T_ChipSelectOutputPin::SetPinValueLow();
            C_SPI.transfer(reg);
            C_SPI.transfer(value);
            T_ChipSelectOutputPin::SetPinValueHigh();
            C_SPI.endTransaction();
        }

        void PCD_ReadRegister(PCD_Register reg, uint8_t count, uint8_t *values, uint8_t rxAlign = 0)
        {
            if (count == 0)
                return;
            uint8_t address = 0x80 | (reg & 0x7E);
            uint8_t index = 0;
            C_SPI.beginTransaction(SPISettings(4000000u, MSBFIRST, SPI_MODE0));
            T_ChipSelectOutputPin::SetPinValueLow();
            count--;
            SPI.transfer(address);
            while (index < count)
            {
                if (index == 0 && rxAlign)
                {
                    uint8_t mask = 0;
                    for (int i = rxAlign; i <= 7; i++)
                    {
                        mask |= (1 << i);
                    }
                    uint8_t value = SPI.transfer(address);
                    values[0] = (values[index] & ~mask) | (value & mask);
                }
                else
                {
                    values[index] = SPI.transfer(address);
                }
                index++;
            }
            values[index] = SPI.transfer(0);
            T_ChipSelectOutputPin::SetPinValueHigh();
            C_SPI.endTransaction();
        }

        uint8_t PCD_ReadRegister(PCD_Register reg)
        {
            uint8_t value;
            C_SPI.beginTransaction(SPISettings(4000000u, MSBFIRST, SPI_MODE0));
            T_ChipSelectOutputPin::SetPinValueLow();
            C_SPI.transfer(0x80 | reg);
            value = C_SPI.transfer(0);
            T_ChipSelectOutputPin::SetPinValueHigh();
            C_SPI.endTransaction();
            return value;
        }

        void PCD_AntennaOn()
        {
            uint8_t value = PCD_ReadRegister(TxControlReg);
            if ((value & 0x03) != 0x03)
            {
                PCD_WriteRegister(TxControlReg, value | 0x03);
            }
        }

        StatusCode PCD_CalculateCRC(byte *data,  ///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
                                    byte length, ///< In: The number of bytes to transfer.
                                    byte *result ///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
        )
        {
            PCD_WriteRegister(CommandReg, PCD_Idle);      // Stop any active command.
            PCD_WriteRegister(DivIrqReg, 0x04);           // Clear the CRCIRq interrupt request bit
            PCD_WriteRegister(FIFOLevelReg, 0x80);        // FlushBuffer = 1, FIFO initialization
            PCD_WriteRegister(FIFODataReg, length, data); // Write data to the FIFO
            PCD_WriteRegister(CommandReg, PCD_CalcCRC);   // Start the calculation

            // Wait for the CRC calculation to complete. Check for the register to
            // indicate that the CRC calculation is complete in a loop. If the
            // calculation is not indicated as complete in ~90ms, then time out
            // the operation.
            const uint32_t deadline = millis() + 89;

            do
            {
                // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
                byte n = PCD_ReadRegister(DivIrqReg);
                if (n & 0x04)
                {                                            // CRCIRq bit set - calculation done
                    PCD_WriteRegister(CommandReg, PCD_Idle); // Stop calculating CRC for new content in the FIFO.
                    // Transfer the result from the registers to the result buffer
                    result[0] = PCD_ReadRegister(CRCResultRegL);
                    result[1] = PCD_ReadRegister(CRCResultRegH);
                    return STATUS_OK;
                }
                yield();
            } while (static_cast<uint32_t>(millis()) < deadline);

            // 89ms passed and nothing happened. Communication with the MFRC522 might be down.
            return STATUS_TIMEOUT;
        } // End PCD_CalculateCRC()

        StatusCode PCD_TransceiveData(byte *sendData,            ///< Pointer to the data to transfer to the FIFO.
                                      byte sendLen,              ///< Number of bytes to transfer to the FIFO.
                                      byte *backData,            ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                      byte *backLen,             ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                      byte *validBits = nullptr, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
                                      byte rxAlign = 0,          ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                      bool checkCRC = false      ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
        )
        {
            byte waitIRq = 0x30; // RxIRq and IdleIRq
            return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
        }

        StatusCode PCD_CommunicateWithPICC(byte command,    ///< The command to execute. One of the PCD_Command enums.
                                           byte waitIRq,    ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                           byte *sendData,  ///< Pointer to the data to transfer to the FIFO.
                                           byte sendLen,    ///< Number of bytes to transfer to the FIFO.
                                           byte *backData,  ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                           byte *backLen,   ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                           byte *validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
                                           byte rxAlign,    ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                           bool checkCRC    ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
        )
        {
            // Prepare values for BitFramingReg
            byte txLastBits = validBits ? *validBits : 0;
            byte bitFraming = (rxAlign << 4) + txLastBits; // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            PCD_WriteRegister(CommandReg, PCD_Idle);           // Stop any active command.
            PCD_WriteRegister(ComIrqReg, 0x7F);                // Clear all seven interrupt request bits
            PCD_WriteRegister(FIFOLevelReg, 0x80);             // FlushBuffer = 1, FIFO initialization
            PCD_WriteRegister(FIFODataReg, sendLen, sendData); // Write sendData to the FIFO
            PCD_WriteRegister(BitFramingReg, bitFraming);      // Bit adjustments
            PCD_WriteRegister(CommandReg, command);            // Execute the command
            if (command == PCD_Transceive)
            {
                PCD_SetRegisterBitMask(BitFramingReg, 0x80); // StartSend=1, transmission of data starts
            }

            // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer
            // automatically starts when the PCD stops transmitting.
            //
            // Wait here for the command to complete. The bits specified in the
            // `waitIRq` parameter define what bits constitute a completed command.
            // When they are set in the ComIrqReg register, then the command is
            // considered complete. If the command is not indicated as complete in
            // ~36ms, then consider the command as timed out.
            const uint32_t deadline = millis() + 36;
            bool completed = false;

            do
            {
                byte n = PCD_ReadRegister(ComIrqReg); // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
                if (n & waitIRq)
                { // One of the interrupts that signal success has been set.
                    completed = true;
                    break;
                }
                if (n & 0x01)
                { // Timer interrupt - nothing received in 25ms
                    return STATUS_TIMEOUT;
                }
                yield();
            } while (static_cast<uint32_t>(millis()) < deadline);

            // 36ms and nothing happened. Communication with the MFRC522 might be down.
            if (!completed)
            {
                return STATUS_TIMEOUT;
            }

            // Stop now if any errors except collisions were detected.
            byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
            if (errorRegValue & 0x13)
            { // BufferOvfl ParityErr ProtocolErr
                return STATUS_ERROR;
            }

            byte _validBits = 0;

            // If the caller wants data back, get it from the MFRC522.
            if (backData && backLen)
            {
                byte n = PCD_ReadRegister(FIFOLevelReg); // Number of bytes in the FIFO
                if (n > *backLen)
                {
                    return STATUS_NO_ROOM;
                }
                *backLen = n;                                        // Number of bytes returned
                PCD_ReadRegister(FIFODataReg, n, backData, rxAlign); // Get received data from FIFO
                _validBits = PCD_ReadRegister(ControlReg) & 0x07;    // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
                if (validBits)
                {
                    *validBits = _validBits;
                }
            }

            // Tell about collisions
            if (errorRegValue & 0x08)
            { // CollErr
                return STATUS_COLLISION;
            }

            // Perform CRC_A validation if requested.
            if (backData && backLen && checkCRC)
            {
                // In this case a MIFARE Classic NAK is not OK.
                if (*backLen == 1 && _validBits == 4)
                {
                    return STATUS_MIFARE_NACK;
                }
                // We need at least the CRC_A value and all 8 bits of the last byte must be received.
                if (*backLen < 2 || _validBits != 0)
                {
                    return STATUS_CRC_WRONG;
                }
                // Verify CRC_A - do our own calculation and store the control in controlBuffer.
                byte controlBuffer[2];
                StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
                if (status != STATUS_OK)
                {
                    return status;
                }
                if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1]))
                {
                    return STATUS_CRC_WRONG;
                }
            }

            return STATUS_OK;
        }

        StatusCode PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize)
        {
            uint8_t validBits;
            StatusCode status;
            if (bufferATQA == nullptr || *bufferSize < 2)
                return STATUS_NO_ROOM;
            PCD_ClearRegisterBitMask(CollReg, 0x80);
            validBits = 7;
            status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
            if (status != STATUS_OK)
            {
                return status;
            }
            if (*bufferSize != 2 || validBits != 0)
            { // ATQA must be exactly 16 bits.
                return STATUS_ERROR;
            }
            return STATUS_OK;
        }

        StatusCode PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize)
        {
            return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
        }

        StatusCode PICC_Select(Swellington::MIFARE::Uid *uid, uint8_t validBits = 0)
        {
            bool uidComplete;
            bool selectDone;
            bool useCascadeTag;
            byte cascadeLevel = 1;
            StatusCode result;
            byte count;
            byte checkBit;
            byte index;
            byte uidIndex;                // The first index in uid->uidByte[] that is used in the current Cascade Level.
            int8_t currentLevelKnownBits; // The number of known UID bits in the current Cascade Level.
            byte buffer[9];               // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
            byte bufferUsed;              // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
            byte rxAlign;                 // Used in BitFramingReg. Defines the bit position for the first bit received.
            byte txLastBits;              // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
            byte *responseBuffer;
            byte responseLength;

            // Sanity checks
            if (validBits > 80)
            {
                return STATUS_INVALID;
            }

            // Prepare MFRC522
            PCD_ClearRegisterBitMask(CollReg, 0x80); // ValuesAfterColl=1 => Bits received after collision are cleared.

            // Repeat Cascade Level loop until we have a complete UID.
            uidComplete = false;
            while (!uidComplete)
            {
                // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
                switch (cascadeLevel)
                {
                case 1:
                    buffer[0] = PICC_CMD_SEL_CL1;
                    uidIndex = 0;
                    useCascadeTag = validBits && uid->size > 4; // When we know that the UID has more than 4 bytes
                    break;

                case 2:
                    buffer[0] = PICC_CMD_SEL_CL2;
                    uidIndex = 3;
                    useCascadeTag = validBits && uid->size > 7; // When we know that the UID has more than 7 bytes
                    break;

                case 3:
                    buffer[0] = PICC_CMD_SEL_CL3;
                    uidIndex = 6;
                    useCascadeTag = false; // Never used in CL3.
                    break;

                default:
                    return STATUS_INTERNAL_ERROR;
                    break;
                }

                // How many UID bits are known in this Cascade Level?
                currentLevelKnownBits = validBits - (8 * uidIndex);
                if (currentLevelKnownBits < 0)
                {
                    currentLevelKnownBits = 0;
                }
                // Copy the known bits from uid->uidByte[] to buffer[]
                index = 2; // destination index in buffer[]
                if (useCascadeTag)
                {
                    buffer[index++] = PICC_CMD_CT;
                }
                byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
                if (bytesToCopy)
                {
                    byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
                    if (bytesToCopy > maxBytes)
                    {
                        bytesToCopy = maxBytes;
                    }
                    for (count = 0; count < bytesToCopy; count++)
                    {
                        buffer[index++] = uid->uidByte[uidIndex + count];
                    }
                }
                // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
                if (useCascadeTag)
                {
                    currentLevelKnownBits += 8;
                }

                // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
                selectDone = false;
                while (!selectDone)
                {
                    // Find out how many bits and bytes to send and receive.
                    if (currentLevelKnownBits >= 32)
                    { // All UID bits in this Cascade Level are known. This is a SELECT.
                        // Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                        buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
                        // Calculate BCC - Block Check Character
                        buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                        // Calculate CRC_A
                        result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
                        if (result != STATUS_OK)
                        {
                            return result;
                        }
                        txLastBits = 0; // 0 => All 8 bits are valid.
                        bufferUsed = 9;
                        // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                        responseBuffer = &buffer[6];
                        responseLength = 3;
                    }
                    else
                    { // This is an ANTICOLLISION.
                        // Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                        txLastBits = currentLevelKnownBits % 8;
                        count = currentLevelKnownBits / 8;     // Number of whole bytes in the UID part.
                        index = 2 + count;                     // Number of whole bytes: SEL + NVB + UIDs
                        buffer[1] = (index << 4) + txLastBits; // NVB - Number of Valid Bits
                        bufferUsed = index + (txLastBits ? 1 : 0);
                        // Store response in the unused part of buffer
                        responseBuffer = &buffer[index];
                        responseLength = sizeof(buffer) - index;
                    }

                    // Set bit adjustments
                    rxAlign = txLastBits;                                          // Having a separate variable is overkill. But it makes the next line easier to read.
                    PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits); // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

                    // Transmit the buffer and receive the response.
                    result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
                    if (result == STATUS_COLLISION)
                    {                                                    // More than one PICC in the field => collision.
                        byte valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                        if (valueOfCollReg & 0x20)
                        {                            // CollPosNotValid
                            return STATUS_COLLISION; // Without a valid collision position we cannot continue
                        }
                        byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
                        if (collisionPos == 0)
                        {
                            collisionPos = 32;
                        }
                        if (collisionPos <= currentLevelKnownBits)
                        { // No progress - should not happen
                            return STATUS_INTERNAL_ERROR;
                        }
                        // Choose the PICC with the bit set.
                        currentLevelKnownBits = collisionPos;
                        count = currentLevelKnownBits % 8; // The bit to modify
                        checkBit = (currentLevelKnownBits - 1) % 8;
                        index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
                        buffer[index] |= (1 << checkBit);
                    }
                    else if (result != STATUS_OK)
                    {
                        return result;
                    }
                    else
                    { // STATUS_OK
                        if (currentLevelKnownBits >= 32)
                        {                      // This was a SELECT.
                            selectDone = true; // No more anticollision
                                               // We continue below outside the while.
                        }
                        else
                        { // This was an ANTICOLLISION.
                            // We now have all 32 bits of the UID in this Cascade Level
                            currentLevelKnownBits = 32;
                            // Run loop again to do the SELECT.
                        }
                    }
                } // End of while (!selectDone)

                // We do not check the CBB - it was constructed by us above.

                // Copy the found UID bytes from buffer[] to uid->uidByte[]
                index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
                bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
                for (count = 0; count < bytesToCopy; count++)
                {
                    uid->uidByte[uidIndex + count] = buffer[index++];
                }

                // Check response SAK (Select Acknowledge)
                if (responseLength != 3 || txLastBits != 0)
                { // SAK must be exactly 24 bits (1 byte + CRC_A).
                    return STATUS_ERROR;
                }
                // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
                result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
                if (result != STATUS_OK)
                {
                    return result;
                }
                if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
                {
                    return STATUS_CRC_WRONG;
                }
                if (responseBuffer[0] & 0x04)
                { // Cascade bit set - UID not complete yes
                    cascadeLevel++;
                }
                else
                {
                    uidComplete = true;
                    uid->sak = responseBuffer[0];
                }
            } // End of while (!uidComplete)

            // Set correct uid->size
            uid->size = 3 * cascadeLevel + 1;

            return STATUS_OK;
        }

        bool PICC_ReadCardSerial()
        {
            StatusCode status = PICC_Select(&_uid);
            return status == STATUS_OK;
        }

        bool PICC_IsNewCardPresent()
        {
            byte bufferATQA[2];
            byte bufferSize = sizeof(bufferATQA);

            // Reset baud rates
            PCD_WriteRegister(TxModeReg, 0x00);
            PCD_WriteRegister(RxModeReg, 0x00);
            // Reset ModWidthReg
            PCD_WriteRegister(ModWidthReg, 0x26);

            StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);

            return (result == STATUS_OK || result == STATUS_COLLISION);
        }

        StatusCode PICC_HaltA()
        {
            StatusCode result;
            uint8_t buffer[4];

            // Build command buffer
            buffer[0] = PICC_CMD_HLTA;
            buffer[1] = 0;
            // Calculate CRC_A
            result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
            if (result != STATUS_OK)
            {
                return result;
            }

            result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0);
            if (result == STATUS_TIMEOUT)
            {
                return STATUS_OK;
            }
            if (result == STATUS_OK)
            { // That is ironically NOT ok in this case ;-)
                return STATUS_ERROR;
            }
            return result;
        }

        void PCD_StopCrypto1()
        {
            PCD_ClearRegisterBitMask(Status2Reg, 0x08);
        }

        void Soft_Reset()
        {
            PCD_WriteRegister(CommandReg, PCD_SoftReset);
            uint8_t count = 0;
            do
            {
                delay(50);
            } while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
        }

    private:
        inline void UpdateKeyOutput() const
        {
            unsigned long k = 0;
            memcpy(&k, _uid.uidByte, _uid.size);
            Key().OutputPin().SetPinValue(k);
        }

    public:
        inline void ClockInputPin_o_Receive(void *_data) const
        {
            if (!Enabled() || (!Key().OutputPin().GetPinIsConnected()))
                return;

            if (!PICC_IsNewCardPresent())
                return;
            if (!PICC_ReadCardSerial())
                return;

            UpdateKeyOutput();
            PICC_HaltA();
        }

        inline void SystemInit() const
        {
            if (!Enabled())
                return;
            bool hardReset = false;
            ChipSelectOutputPin().SetPinValueHigh();
            if (C_ResetOutputPin != UNUSED_PIN)
            {
                pinMode(C_ResetOutputPin, INPUT);
                if (digitalRead(C_ResetOutputPin) == LOW)
                {
                    pinMode(C_ResetOutputPin, OUTPUT);
                    digitalWrite(C_ResetOutputPin, LOW);
                    delayMicroseconds(2); // 8.8.1 Reset timing requirements says about 100ns. Let us be generous: 2μsl
                    digitalWrite(C_ResetOutputPin, HIGH);
                    delay(50); // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
                    hardReset = true;
                }
            }

            if (!hardReset)
                Soft_Reset();

            PCD_WriteRegister(TxModeReg, 0x00);
            PCD_WriteRegister(RxModeReg, 0x00);
            PCD_WriteRegister(ModWidthReg, 0x26);
            PCD_WriteRegister(TModeReg, 0x80);
            PCD_WriteRegister(TPrescalerReg, 0xA9);
            PCD_WriteRegister(TReloadRegH, 0x03);
            PCD_WriteRegister(TReloadRegL, 0xE8);
            PCD_WriteRegister(TxASKReg, 0x40);
            PCD_WriteRegister(ModeReg, 0x3D);
            PCD_AntennaOn();
            delay(50);
        }

        inline void SystemLoopBegin() const
        {
            ClockInputPin_o_Receive(nullptr);
        }

        inline void UpdateEnabled() const
        {
            //not implemented yet
        }
    };

}

#endif