#include "SX1262HAL.h"

// ============================================================
//  STATIC MEMBER INITIALIZATION
// ============================================================
SX1262HAL* SX1262HAL::_instance = nullptr;

// Static ISR trampoline for DIO1 interrupt. Redirects to the registered
// instance callback without requiring std::function overhead on AVR.
void SX1262HAL::_isrTrampoline() {
    if (_instance && _instance->_rxCallback) {
        _instance->_rxCallback();
    }
}

// ============================================================
//  CONSTRUCTOR
// ============================================================
SX1262HAL::SX1262HAL(uint8_t nssPin, uint8_t busyPin, uint8_t rstPin, uint8_t dio1Pin)
    : _nssPin(nssPin), _busyPin(busyPin), _rstPin(rstPin), _dio1Pin(dio1Pin),
      _preambleLength(8), _rxCallback(nullptr) {
    _instance = this;
    // deliberately no hardware setup here — pins get configured in begin(),
    // same reasoning as OledHAL: constructors run early, before we control
    // exact timing; real init happens explicitly, later, in begin()
}

// ============================================================
//  LOW-LEVEL SPI PRIMITIVES
// ============================================================

// Waits for the BUSY pin to go low before allowing a new SPI transaction.
// Per the SX126x datasheet, the chip drives BUSY high while internally
// processing a command — starting a new transaction before it clears
// produces undefined behavior.
void SX1262HAL::_waitBusy() {
    uint32_t start = millis();
    while (digitalRead(_busyPin) == HIGH) {
        if (millis() - start > 1000) {
            return;  // timeout — don't hang forever; caller's next
                     // transaction may fail, which _writeCommand's
                     // status check will catch
        }
    }
}

// Sends an opcode + parameter bytes to the chip.
// SPI is full-duplex: the status byte comes back on the SAME transfer
// that clocks out the opcode (not a separate read) — every SPI.transfer()
// call sends one byte out and receives one byte back, simultaneously.
// Bits 3:1 of the status byte indicate command status (Table 13-76).
// Verified against SX1262Registers.h: RADIOLIB_SX126X_STATUS_CMD_FAILED
// (0b00001010) is pre-shifted into bit position 3:1, matching the mask
// below — this comparison is correct as written.
int16_t SX1262HAL::_writeCommand(uint8_t opcode, const uint8_t* params, uint8_t paramLen) {
    _waitBusy();
    digitalWrite(_nssPin, LOW);

    uint8_t statusByte = SPI.transfer(opcode);
    for (uint8_t i = 0; i < paramLen; i++) {
        SPI.transfer(params[i]);
    }

    digitalWrite(_nssPin, HIGH);

    // bits 3:1 of the status byte = command status (datasheet Table 13-76)
    if ((statusByte & 0b00001110) == RADIOLIB_SX126X_STATUS_CMD_FAILED) {
        return RADIOLIB_ERR_SPI_CMD_FAILED;
    }
    return RADIOLIB_ERR_NONE;
}

// Sends an opcode, then clocks out resultLen bytes to read a response.
// The byte immediately after the opcode is a status byte per the datasheet's
// read-command framing; we discard it here.
void SX1262HAL::_readCommand(uint8_t opcode, uint8_t* result, uint8_t resultLen) {
    _waitBusy();
    digitalWrite(_nssPin, LOW);

    SPI.transfer(opcode);
    SPI.transfer(0x00);  // status byte, discarded here

    for (uint8_t i = 0; i < resultLen; i++) {
        result[i] = SPI.transfer(0x00);
    }

    digitalWrite(_nssPin, HIGH);
}

// Datasheet §13.2.1 (Table 13-24) — WriteRegister. Opcode 0x0D.
// Writes data to the SX126x configuration registers starting at 16-bit address `addr`.
// Register access uses a 3-part SPI framing (opcode, address MSB, address LSB, data bytes...),
// differing from command writes which take a 1-byte opcode followed by parameters.
int16_t SX1262HAL::_writeRegister(uint16_t addr, const uint8_t* data, uint8_t len) {
    _waitBusy();
    digitalWrite(_nssPin, LOW);

    uint8_t statusByte = SPI.transfer(RADIOLIB_SX126X_CMD_WRITE_REGISTER);
    SPI.transfer((uint8_t)((addr >> 8) & 0xFF));
    SPI.transfer((uint8_t)(addr & 0xFF));
    for (uint8_t i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }

    digitalWrite(_nssPin, HIGH);

    if ((statusByte & 0b00001110) == RADIOLIB_SX126X_STATUS_CMD_FAILED) {
        return RADIOLIB_ERR_SPI_CMD_FAILED;
    }
    return RADIOLIB_ERR_NONE;
}

// Datasheet §13.2.2 (Table 13-25) — ReadRegister. Opcode 0x1D.
// Reads data from the SX126x configuration registers starting at 16-bit address `addr`.
// Per datasheet Table 13-25, the host sends opcode, address MSB, address LSB, followed by
// a mandatory NOP byte (during which the chip returns status) before the register data starts.
int16_t SX1262HAL::_readRegister(uint16_t addr, uint8_t* data, uint8_t len) {
    _waitBusy();
    digitalWrite(_nssPin, LOW);

    SPI.transfer(RADIOLIB_SX126X_CMD_READ_REGISTER);
    SPI.transfer((uint8_t)((addr >> 8) & 0xFF));
    SPI.transfer((uint8_t)(addr & 0xFF));
    SPI.transfer(0x00);  // NOP byte required before register data begins

    for (uint8_t i = 0; i < len; i++) {
        data[i] = SPI.transfer(0x00);
    }

    digitalWrite(_nssPin, HIGH);
    return RADIOLIB_ERR_NONE;
}

// Datasheet §13.2.3 (Table 13-26) — WriteBuffer. Opcode 0x0E.
// Writes data to the SX126x internal packet data buffer starting at `offset`.
int16_t SX1262HAL::_writeBuffer(uint8_t offset, const uint8_t* data, uint8_t len) {
    _waitBusy();
    digitalWrite(_nssPin, LOW);

    uint8_t statusByte = SPI.transfer(RADIOLIB_SX126X_CMD_WRITE_BUFFER);
    SPI.transfer(offset);
    for (uint8_t i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }

    digitalWrite(_nssPin, HIGH);

    if ((statusByte & 0b00001110) == RADIOLIB_SX126X_STATUS_CMD_FAILED) {
        return RADIOLIB_ERR_SPI_CMD_FAILED;
    }
    return RADIOLIB_ERR_NONE;
}

// Datasheet §13.2.4 (Table 13-27) — ReadBuffer. Opcode 0x1E.
// Reads received packet data from the internal packet buffer starting at `offset`.
// Per datasheet Table 13-27, after sending opcode and offset, the host clocks out a NOP byte
// (the chip echoes status here) before reading out the payload bytes.
int16_t SX1262HAL::_readBuffer(uint8_t offset, uint8_t* data, uint8_t len) {
    _waitBusy();
    digitalWrite(_nssPin, LOW);

    SPI.transfer(RADIOLIB_SX126X_CMD_READ_BUFFER);
    SPI.transfer(offset);
    SPI.transfer(0x00);  // NOP byte per datasheet Table 13-27

    for (uint8_t i = 0; i < len; i++) {
        data[i] = SPI.transfer(0x00);
    }

    digitalWrite(_nssPin, HIGH);
    return RADIOLIB_ERR_NONE;
}

// Datasheet §13.1.4 (Table 13-8) — SetTx. Opcode 0x83.
// Commands the chip into transmit mode. Parameter is a 24-bit timeout value in units
// of 15.625µs (Table 13-8). RADIOLIB_SX126X_TX_TIMEOUT_NONE (0x000000) disables timeout.
int16_t SX1262HAL::_setTx(uint32_t timeout) {
    uint8_t data[3] = {
        (uint8_t)((timeout >> 16) & 0xFF),
        (uint8_t)((timeout >> 8) & 0xFF),
        (uint8_t)(timeout & 0xFF)
    };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_TX, data, 3);
}

// Datasheet §13.1.5 (Table 13-9) — SetRx. Opcode 0x82.
// Commands the chip into receive mode. Parameter is a 24-bit timeout value in units
// of 15.625µs (Table 13-9). RADIOLIB_SX126X_RX_TIMEOUT_INF (0xFFFFFF) selects continuous RX.
int16_t SX1262HAL::_setRx(uint32_t timeout) {
    uint8_t data[3] = {
        (uint8_t)((timeout >> 16) & 0xFF),
        (uint8_t)((timeout >> 8) & 0xFF),
        (uint8_t)(timeout & 0xFF)
    };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_RX, data, 3);
}

// Datasheet §13.3.3 (Table 13-30) — GetIrqStatus. Opcode 0x12.
// Returns the 16-bit interrupt status word.
uint16_t SX1262HAL::_getIrqStatus() {
    uint8_t data[2] = { 0x00, 0x00 };
    _readCommand(RADIOLIB_SX126X_CMD_GET_IRQ_STATUS, data, 2);
    return ((uint16_t)data[0] << 8) | data[1];
}

// Datasheet §13.3.4 (Table 13-31) — ClearIrqStatus. Opcode 0x02.
// Clears interrupt flags indicated by bits set in `clearIrqParams`.
int16_t SX1262HAL::_clearIrqStatus(uint16_t clearIrqParams) {
    uint8_t data[2] = {
        (uint8_t)((clearIrqParams >> 8) & 0xFF),
        (uint8_t)(clearIrqParams & 0xFF)
    };
    return _writeCommand(RADIOLIB_SX126X_CMD_CLEAR_IRQ_STATUS, data, 2);
}

// Datasheet §13.5.2 (Table 13-78) — GetRxBufferStatus. Opcode 0x13.
// Retrieves length of received packet (byte 0) and start offset pointer in SRAM (byte 1).
int16_t SX1262HAL::_getRxBufferStatus(uint8_t* payloadLen, uint8_t* startOffset) {
    uint8_t data[2] = { 0x00, 0x00 };
    _readCommand(RADIOLIB_SX126X_CMD_GET_RX_BUFFER_STATUS, data, 2);
    if (payloadLen) {
        *payloadLen = data[0];
    }
    if (startOffset) {
        *startOffset = data[1];
    }
    return RADIOLIB_ERR_NONE;
}

// Datasheet §13.5.3 (Table 13-79) — GetPacketStatus. Opcode 0x14.
// Retrieves 3 bytes of status for the last received LoRa packet:
// byte 0: RssiPkt, byte 1: SnrPkt, byte 2: SignalRssiPkt.
int16_t SX1262HAL::_getPacketStatus(uint8_t* status) {
    _readCommand(RADIOLIB_SX126X_CMD_GET_PACKET_STATUS, status, 3);
    return RADIOLIB_ERR_NONE;
}

// ============================================================
//  begin() SUB-STEPS  (built in the order begin() will call them)
// ============================================================

// Hardware reset pulse, then retry SetStandby until success or timeout.
// The SX126x is known to refuse its first few commands right after reset.
int16_t SX1262HAL::_reset() {
    pinMode(_rstPin, OUTPUT);
    digitalWrite(_rstPin, LOW);
    delay(1);
    digitalWrite(_rstPin, HIGH);

    uint32_t start = millis();
    while (true) {
        int16_t state = _standby();
        if (state == RADIOLIB_ERR_NONE) {
            return RADIOLIB_ERR_NONE;
        }
        if (millis() - start >= 1000) {
            return state;
        }
        delay(10);
    }
}

// SetStandby(STDBY_RC). STDBY_XOSC intentionally unsupported — original
// SNIPS lora_hal.cpp never used it.
int16_t SX1262HAL::_standby() {
    uint8_t data[] = { RADIOLIB_SX126X_STANDBY_RC };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_STANDBY, data, 1);
}

// Datasheet §13.3.6 (Table 13-34) — SetDIO3AsTCXOCtrl. Opcode 0x97.
// 4 parameter bytes: 1 byte tcxoVoltage code followed by 3 bytes delay(23:0),
// where delay = delayUs / 15.625 (delay duration = delay(23:0) * 15.625µs).
// Critical ordering requirement: TCXO control MUST be established before the radio
// enters STDBY_XOSC/FS/TX/RX modes, as those clock states require a stable TCXO.
int16_t SX1262HAL::_setTCXO(float tcxoVoltage, uint32_t delayUs) {
    uint8_t voltCode;
    if (tcxoVoltage <= 1.65f) {
        voltCode = RADIOLIB_SX126X_DIO3_OUTPUT_1_6;
    } else if (tcxoVoltage <= 1.75f) {
        voltCode = RADIOLIB_SX126X_DIO3_OUTPUT_1_7;
    } else if (tcxoVoltage <= 1.9f) {
        voltCode = RADIOLIB_SX126X_DIO3_OUTPUT_1_8;
    } else if (tcxoVoltage <= 2.3f) {
        voltCode = RADIOLIB_SX126X_DIO3_OUTPUT_2_2;
    } else if (tcxoVoltage <= 2.55f) {
        voltCode = RADIOLIB_SX126X_DIO3_OUTPUT_2_4;
    } else if (tcxoVoltage <= 2.85f) {
        voltCode = RADIOLIB_SX126X_DIO3_OUTPUT_2_7;
    } else if (tcxoVoltage <= 3.15f) {
        voltCode = RADIOLIB_SX126X_DIO3_OUTPUT_3_0;
    } else {
        voltCode = RADIOLIB_SX126X_DIO3_OUTPUT_3_3;
    }

    uint32_t delayVal = (uint32_t)((float)delayUs / 15.625f);
    uint8_t data[4] = {
        voltCode,
        (uint8_t)((delayVal >> 16) & 0xFF),
        (uint8_t)((delayVal >> 8) & 0xFF),
        (uint8_t)(delayVal & 0xFF)
    };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, data, 4);
}

// Datasheet §13.4.2 (Table 13-37) — SetPacketType. Opcode 0x8A.
// Configures radio modem to LoRa mode (RADIOLIB_SX126X_PACKET_TYPE_LORA = 0x01).
// Per datasheet §13.4.2: "The command SetPacketType(...) must be the first of
// the radio configuration sequence" and "The switch from one packet type to
// another must be done in STDBY_RC mode."
int16_t SX1262HAL::_setPacketType() {
    uint8_t packetType = RADIOLIB_SX126X_PACKET_TYPE_LORA;
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_PACKET_TYPE, &packetType, 1);
}

// Datasheet §13.4.1 (Table 13-36) — SetRfFrequency. Opcode 0x86.
// 4 parameter bytes, big-endian RfFreq(31:0).
// Formula: RfFreq = freqHz * 2^25 / F_XTAL = freqMHz * 10^6 * 2^25 / (32 * 10^6) = freqMHz * 1048576.
// Evaluated with RADIOLIB_SX126X_DIV_EXPONENT (25) and RADIOLIB_SX126X_CRYSTAL_FREQ (32.0).
int16_t SX1262HAL::_setRfFrequency(float freqMHz) {
    uint32_t frf = (uint32_t)(freqMHz * ((float)(1UL << RADIOLIB_SX126X_DIV_EXPONENT) / RADIOLIB_SX126X_CRYSTAL_FREQ));
    uint8_t data[4] = {
        (uint8_t)((frf >> 24) & 0xFF),
        (uint8_t)((frf >> 16) & 0xFF),
        (uint8_t)((frf >> 8) & 0xFF),
        (uint8_t)(frf & 0xFF)
    };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_RF_FREQUENCY, data, 4);
}

// Datasheet §13.1.13 (Table 13-19) — CalibrateImage. Opcode 0x98.
// Calibrates image rejection for the requested frequency band using predefined
// calibration coefficient pairs (Table 9-2).
// Note: Custom range fallback is intentionally omitted since SNIPS nodes operate
// in the 863-870 MHz ISM band; if operated outside predefined bands, that path would
// need to be re-added.
int16_t SX1262HAL::_calibrateImage(float freqMHz) {
    uint8_t data[2] = { 0, 0 };
    int16_t freqBand = (int16_t)freqMHz;
    if ((freqBand >= 902) && (freqBand <= 928)) {
        data[0] = RADIOLIB_SX126X_CAL_IMG_902_MHZ_1;
        data[1] = RADIOLIB_SX126X_CAL_IMG_902_MHZ_2;
    } else if ((freqBand >= 863) && (freqBand <= 870)) {
        data[0] = RADIOLIB_SX126X_CAL_IMG_863_MHZ_1;
        data[1] = RADIOLIB_SX126X_CAL_IMG_863_MHZ_2;
    } else if ((freqBand >= 779) && (freqBand <= 787)) {
        data[0] = RADIOLIB_SX126X_CAL_IMG_779_MHZ_1;
        data[1] = RADIOLIB_SX126X_CAL_IMG_779_MHZ_2;
    } else if ((freqBand >= 470) && (freqBand <= 510)) {
        data[0] = RADIOLIB_SX126X_CAL_IMG_470_MHZ_1;
        data[1] = RADIOLIB_SX126X_CAL_IMG_470_MHZ_2;
    } else if ((freqBand >= 430) && (freqBand <= 440)) {
        data[0] = RADIOLIB_SX126X_CAL_IMG_430_MHZ_1;
        data[1] = RADIOLIB_SX126X_CAL_IMG_430_MHZ_2;
    }

    if (data[0] != 0) {
        return _writeCommand(RADIOLIB_SX126X_CMD_CALIBRATE_IMAGE, data, 2);
    }
    return RADIOLIB_ERR_NONE;
}

// Datasheet §13.1.11 (Table 13-16) — SetRegulatorMode. Opcode 0x96.
// Always hardcoded to DC-DC (RADIOLIB_SX126X_REGULATOR_DC_DC = 0x01) for superior
// power efficiency on hardware with DC-DC inductors fitted, such as the Seeed Wio-SX1262.
int16_t SX1262HAL::_setRegulatorMode() {
    uint8_t mode = RADIOLIB_SX126X_REGULATOR_DC_DC;
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_REGULATOR_MODE, &mode, 1);
}

// Datasheet §13.1.14 (Table 13-20) — SetPaConfig. Opcode 0x95.
// Parameter bytes fixed for SX1262 high-power operation up to +22 dBm:
// paDutyCycle = 0x04, hpMax = RADIOLIB_SX126X_PA_CONFIG_HP_MAX (0x07),
// deviceSel = RADIOLIB_SX126X_PA_CONFIG_SX1262_8 (0x00), paLut = RADIOLIB_SX126X_PA_CONFIG_PA_LUT (0x01).
int16_t SX1262HAL::_setPaConfig() {
    uint8_t data[4] = {
        0x04,
        RADIOLIB_SX126X_PA_CONFIG_HP_MAX,
        RADIOLIB_SX126X_PA_CONFIG_SX1262_8,
        RADIOLIB_SX126X_PA_CONFIG_PA_LUT
    };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_PA_CONFIG, data, 4);
}

// Datasheet §5.1 (Table 5-2, page 34) — OCP Configuration, register 0x08E7.
// Sets Over-Current Protection current limit in mA.
// Formula: rawLimit = (uint8_t)(milliAmps / 2.5f), 2.5mA per LSB step (confirmed:
// max OCP = 63 * 2.5mA = 157.5mA per datasheet note under Table 5-2).
//
// IMPORTANT ORDERING RULE (datasheet Table 5-2, explicit note):
// "the default value is re-configured automatically each time the function
// SetPaConfig(...) is called" — for SX1262 that default is 140mA (0x38).
// This means _setPaConfig() MUST be called BEFORE _setCurrentLimit(), or
// SetPaConfig() silently overwrites whatever limit was just set. begin() below
// has been reordered accordingly (this was the bug in the original file).
int16_t SX1262HAL::_setCurrentLimit(float milliAmps) {
    if (milliAmps < 0.0f) {
        milliAmps = 0.0f;
    } else if (milliAmps > 140.0f) {
        milliAmps = 140.0f;
    }
    uint8_t rawLimit = (uint8_t)(milliAmps / 2.5f);
    return _writeRegister(RADIOLIB_SX126X_REG_OCP_CONFIGURATION, &rawLimit, 1);
}

// Datasheet §13.4.4 (Table 13-40) — SetTxParams. Opcode 0x8E.
// 2 parameter bytes: power (signed, clamped to -9..+22 dBm) and ramp time.
int16_t SX1262HAL::_setTxParams(int8_t power, uint8_t rampTime) {
    if (power < -9) {
        power = -9;
    } else if (power > 22) {
        power = 22;
    }
    uint8_t data[2] = { (uint8_t)power, rampTime };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_TX_PARAMS, data, 2);
}

// Datasheet §13.4.5 (Table 13-42) — SetModulationParams. Opcode 0x8B.
// Sends 8 parameter bytes for LoRa (per §13.4.5.2):
// ModParam1: Spreading Factor (0x05 to 0x0C)
// ModParam2: Bandwidth code
// ModParam3: Coding Rate code
// ModParam4: LowDataRateOptimize (0x00: off, 0x01: on)
// ModParam5-8: 0x00 (unused for LoRa)
int16_t SX1262HAL::_setModulationParams(uint8_t sf, float bw, uint8_t cr) {
    if (sf < 5 || sf > 12) {
        return RADIOLIB_ERR_SPI_CMD_FAILED;
    }

    uint8_t bwCode;
    if (bw <= 9.1f) {
        bwCode = RADIOLIB_SX126X_LORA_BW_7_8;
    } else if (bw <= 13.0f) {
        bwCode = RADIOLIB_SX126X_LORA_BW_10_4;
    } else if (bw <= 18.2f) {
        bwCode = RADIOLIB_SX126X_LORA_BW_15_6;
    } else if (bw <= 26.0f) {
        bwCode = RADIOLIB_SX126X_LORA_BW_20_8;
    } else if (bw <= 36.5f) {
        bwCode = RADIOLIB_SX126X_LORA_BW_31_25;
    } else if (bw <= 52.1f) {
        bwCode = RADIOLIB_SX126X_LORA_BW_41_7;
    } else if (bw <= 93.8f) {
        bwCode = RADIOLIB_SX126X_LORA_BW_62_5;
    } else if (bw <= 187.5f) {
        bwCode = RADIOLIB_SX126X_LORA_BW_125_0;
    } else if (bw <= 375.0f) {
        bwCode = RADIOLIB_SX126X_LORA_BW_250_0;
    } else {
        bwCode = RADIOLIB_SX126X_LORA_BW_500_0;
    }

    // Datasheet Table 13-49 also defines 0x05-0x07 as "long interleaver" CR
    // variants (SX1262 only); this HAL exposes only the base 4/5-4/8 rates
    // (0x01-0x04) since that's what SNIPS's cr parameter (1-4 or 5-8 alias) needs.
    uint8_t crCode;
    if (cr == 5 || cr == 1) {
        crCode = RADIOLIB_SX126X_LORA_CR_4_5;
    } else if (cr == 6 || cr == 2) {
        crCode = RADIOLIB_SX126X_LORA_CR_4_6;
    } else if (cr == 7 || cr == 3) {
        crCode = RADIOLIB_SX126X_LORA_CR_4_7;
    } else if (cr == 8 || cr == 4) {
        crCode = RADIOLIB_SX126X_LORA_CR_4_8;
    } else {
        crCode = RADIOLIB_SX126X_LORA_CR_4_5;
    }

    // NOT CHANGED — flagging, not fixing: datasheet §13.4.5 (under Table 13-50)
    // says LDRO is "usually set when the LoRa symbol time is equal or above
    // 16.38ms (typically SF11/BW125 and SF12/BW125+BW250)". That's a
    // recommendation, not a hard protocol requirement, so hardcoding it OFF
    // won't break interop — it just gives up some link margin at high
    // SF/low BW. Left exactly as in the original file since you didn't ask
    // for this behavior to change; revisit if SNIPS ever runs SF11/SF12 at
    // BW125 or narrower.
    uint8_t ldro = RADIOLIB_SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF;

    uint8_t data[8] = {
        sf,
        bwCode,
        crCode,
        ldro,
        0x00, 0x00, 0x00, 0x00
    };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_MODULATION_PARAMS, data, 8);
}

// Datasheet §13.4.6 (Table 13-51) — SetPacketParams. Opcode 0x8C.
// 9 parameter bytes for LoRa (per §13.4.6.2, Tables 13-66 through 13-70):
// Param1-2: Preamble length (16-bit big endian, number of LoRa symbols)
// Param3: Header type (RADIOLIB_SX126X_LORA_HEADER_EXPLICIT = 0x00)
// Param4: Payload length
// Param5: CRC configuration — confirmed against Table 13-69:
//         0x00 = CRC OFF, 0x01 = CRC ON. RADIOLIB_SX126X_LORA_CRC_ON is defined
//         as 0x01 in the header, so this is correct as written.
// Param6: InvertIQ configuration (RADIOLIB_SX126X_LORA_IQ_STANDARD = 0x00)
// Param7-9: 0x00 (reserved for LoRa)
int16_t SX1262HAL::_setPacketParams(uint16_t preambleLen, uint8_t payloadLen) {
    uint8_t data[9] = {
        (uint8_t)((preambleLen >> 8) & 0xFF),
        (uint8_t)(preambleLen & 0xFF),
        RADIOLIB_SX126X_LORA_HEADER_EXPLICIT,
        payloadLen,
        RADIOLIB_SX126X_LORA_CRC_ON,
        RADIOLIB_SX126X_LORA_IQ_STANDARD,
        0x00, 0x00, 0x00
    };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_PACKET_PARAMS, data, 9);
}

// Register map (page 70) — LoRa Sync Word.
// Sets LoRa sync word by writing to registers 0x0740 and 0x0741.
// Formats `syncWord` into upper nibbles with 0x04 control bits in lower nibbles,
// converting standard private sync word 0x12 to 0x1424 in hardware registers
// (0x3444 for public — RADIOLIB_SX126X_SYNC_WORD_PUBLIC/PRIVATE constants
// document this exact quirk).
int16_t SX1262HAL::_setSyncWord(uint8_t syncWord) {
    uint8_t data[2] = {
        (uint8_t)((syncWord & 0xF0) | 0x04),
        (uint8_t)(((syncWord & 0x0F) << 4) | 0x04)
    };
    return _writeRegister(RADIOLIB_SX126X_REG_LORA_SYNC_WORD_MSB, data, 2);
}

// Datasheet §13.4.8 (Table 13-74) — SetBufferBaseAddress. Opcode 0x8F.
// Configures base addresses in the internal 256-byte SRAM for TX and RX buffers.
int16_t SX1262HAL::_setBufferBaseAddress(uint8_t txBase, uint8_t rxBase) {
    uint8_t data[2] = { txBase, rxBase };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_BUFFER_BASE_ADDRESS, data, 2);
}

// Datasheet §13.3.1 (Table 13-28) — SetDioIrqParams. Opcode 0x08.
// Configures interrupt generation and routes IRQs to DIO lines (8 parameter bytes).
// IrqMask (2 bytes), DIO1Mask (2 bytes), DIO2Mask (0x0000), DIO3Mask (0x0000).
int16_t SX1262HAL::_setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask) {
    uint8_t data[8] = {
        (uint8_t)((irqMask >> 8) & 0xFF),
        (uint8_t)(irqMask & 0xFF),
        (uint8_t)((dio1Mask >> 8) & 0xFF),
        (uint8_t)(dio1Mask & 0xFF),
        0x00, 0x00,
        0x00, 0x00
    };
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_DIO_IRQ_PARAMS, data, 8);
}

// FIX: added for Wio-SX1262 antenna switch
int16_t SX1262HAL::_setDio2AsRfSwitchCtrl(bool enable) {
    uint8_t data = enable ? 0x01 : 0x00;
    return _writeCommand(RADIOLIB_SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1);
}


// ============================================================
//  PUBLIC API IMPLEMENTATION
// ============================================================

// Full initialization sequence assembling hardware pin setup, reset, TCXO,
// RF frequency, image calibration, PA, modulation, packet params, sync word,
// buffer base addresses, and DIO1 interrupt routing.
//
// FIX: _setPaConfig() now runs BEFORE _setCurrentLimit(). Per datasheet
// Table 5-2 (page 34), SetPaConfig() silently resets the OCP register to a
// device-dependent default (140mA for SX1262) every time it's called. In the
// original order, the 60mA limit was set first and then immediately clobbered
// by _setPaConfig() — the chip was left at 140mA regardless of the requested
// value. Swapping the order makes _setCurrentLimit()'s value actually stick.
int16_t SX1262HAL::begin(float freq, float bw, uint8_t sf, uint8_t cr,
                         uint8_t syncWord, int8_t power, uint16_t preambleLength) {
    pinMode(_nssPin, OUTPUT);
    digitalWrite(_nssPin, HIGH);
    pinMode(_busyPin, INPUT);
    pinMode(_dio1Pin, INPUT);

    SPI.begin();

    int16_t state = _reset();
    if (state != RADIOLIB_ERR_NONE) return state;

    // TCXO setup: 1.8V, 5000µs (5ms) delay. Must be called before SetPacketType/STDBY_XOSC
    state = _setTCXO(1.8f, 5000);
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _setDio2AsRfSwitchCtrl(true);
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _standby();
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _setPacketType();
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _setRfFrequency(freq);
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _calibrateImage(freq);
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _setRegulatorMode();
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _setPaConfig();
    if (state != RADIOLIB_ERR_NONE) return state;

    // Must come AFTER _setPaConfig() — see FIX note above and the comment on
    // _setCurrentLimit() itself.
    state = _setCurrentLimit(60.0f);
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _setTxParams(power, RADIOLIB_SX126X_PA_RAMP_200U);
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _setModulationParams(sf, bw, cr);
    if (state != RADIOLIB_ERR_NONE) return state;

    _preambleLength = preambleLength;
    state = _setPacketParams(preambleLength, SX1262HAL_MAX_PACKET_LEN);
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _setSyncWord(syncWord);
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _setBufferBaseAddress(0x00, 0x00);
    if (state != RADIOLIB_ERR_NONE) return state;

    uint16_t irqMask = RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT | RADIOLIB_SX126X_IRQ_CRC_ERR;
    state = _setDioIrqParams(irqMask, irqMask);
    if (state != RADIOLIB_ERR_NONE) return state;

    return RADIOLIB_ERR_NONE;
}

// Blocking transmit method. Writes data to buffer, commands SetTx, polls
// GetIrqStatus for TX_DONE or timeout, clears IRQs, and returns status.
int16_t SX1262HAL::transmit(uint8_t* data, size_t len) {
    if (len > SX1262HAL_MAX_PACKET_LEN) {
        len = SX1262HAL_MAX_PACKET_LEN;
    }

    int16_t state = _setPacketParams(_preambleLength, (uint8_t)len);
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _writeBuffer(0x00, data, (uint8_t)len);
    if (state != RADIOLIB_ERR_NONE) return state;

    state = _setTx(RADIOLIB_SX126X_TX_TIMEOUT_NONE);
    if (state != RADIOLIB_ERR_NONE) return state;

    uint32_t start = millis();
    bool txDone = false;
    while (millis() - start < 2000) {
        uint16_t irq = _getIrqStatus();
        if (irq & RADIOLIB_SX126X_IRQ_TX_DONE) {
            txDone = true;
            break;
        }
        if (irq & RADIOLIB_SX126X_IRQ_TIMEOUT) {
            break;
        }
        delay(1);
    }

    _clearIrqStatus(RADIOLIB_SX126X_IRQ_ALL);
    if (!txDone) {
        return RADIOLIB_ERR_SPI_CMD_FAILED;
    }
    return RADIOLIB_ERR_NONE;
}

// Re-arms continuous RX mode with DIO1 interrupt mapping for packet reception.
int16_t SX1262HAL::startReceive() {
    uint16_t irqMask = RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT | RADIOLIB_SX126X_IRQ_CRC_ERR;
    int16_t state = _setDioIrqParams(irqMask, irqMask);
    if (state != RADIOLIB_ERR_NONE) return state;

    return _setRx(RADIOLIB_SX126X_RX_TIMEOUT_INF);
}

// Reads received payload from the SX1262 RX buffer, clears all IRQ flags,
// and restarts continuous reception mode. Returns actual number of bytes read.
size_t SX1262HAL::readData(uint8_t* data, size_t maxLen) {
    uint8_t payloadLen = 0;
    uint8_t startOffset = 0;
    _getRxBufferStatus(&payloadLen, &startOffset);

    if (payloadLen > maxLen) {
        payloadLen = (uint8_t)maxLen;
    }

    _readBuffer(startOffset, data, payloadLen);
    _clearIrqStatus(RADIOLIB_SX126X_IRQ_ALL);
    startReceive();

    return (size_t)payloadLen;
}

// Stores the ISR callback and attaches the static trampoline to DIO1 rising edge.
void SX1262HAL::setPacketReceivedAction(void (*callback)(void)) {
    _rxCallback = callback;
    attachInterrupt(digitalPinToInterrupt(_dio1Pin), _isrTrampoline, RISING);
}

// Datasheet §13.5.3 (Table 13-79/13-80) — GetPacketStatus.
// Derives RSSI of last received packet in dBm from byte 0 (RssiPkt):
// RSSI [dBm] = -RssiPkt / 2. Confirmed correct against Table 13-80.
int8_t SX1262HAL::getRSSI() {
    uint8_t status[3] = { 0, 0, 0 };
    _getPacketStatus(status);
    uint8_t rssiPkt = status[0];
    return (int8_t)(-(int16_t)rssiPkt / 2);
}

// Datasheet §13.5.3 (Table 13-79/13-80) — GetPacketStatus.
// Derives SNR of last received packet in dB from byte 1 (SnrPkt):
// SnrPkt is a signed byte in two's complement, actual SNR = SnrPkt / 4.
// Confirmed correct against Table 13-80 — no longer a TODO.
float SX1262HAL::getSNR() {
    uint8_t status[3] = { 0, 0, 0 };
    _getPacketStatus(status);
    int8_t snrPkt = (int8_t)status[1];
    return (float)snrPkt / 4.0f;
}
