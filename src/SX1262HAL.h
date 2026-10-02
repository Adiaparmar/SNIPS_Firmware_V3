#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "SX1262Registers.h"

class SX1262HAL
{
private:
    uint8_t _nssPin, _busyPin, _rstPin, _dio1Pin;
    uint16_t _preambleLength;
    void (*_rxCallback)(void);
    static SX1262HAL* _instance;
    static void _isrTrampoline();

    // ---- low-level SPI transaction primitives ----
    int16_t _writeCommand(uint8_t opcode, const uint8_t* params, uint8_t paramLen);
    void    _readCommand(uint8_t opcode, uint8_t* result, uint8_t resultLen);
    void    _waitBusy();

    // ---- begin() sub-steps, built in call order ----
    int16_t _reset();
    int16_t _standby();
    int16_t _setTCXO(float tcxoVoltage, uint32_t delayUs);
    int16_t _setDio2AsRfSwitchCtrl(bool enable);  // FIX: added for Wio-SX1262 antenna switch
    int16_t _setPacketType();
    int16_t _setRfFrequency(float freqMHz);
    int16_t _calibrateImage(float freqMHz);
    int16_t _setRegulatorMode();
    int16_t _setCurrentLimit(float milliAmps);
    int16_t _setPaConfig();
    int16_t _setTxParams(int8_t power, uint8_t rampTime);
    int16_t _setModulationParams(uint8_t sf, float bw, uint8_t cr);
    int16_t _setPacketParams(uint16_t preambleLen, uint8_t payloadLen);
    int16_t _setSyncWord(uint8_t syncWord);
    int16_t _setBufferBaseAddress(uint8_t txBase, uint8_t rxBase);
    int16_t _setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask);
    int16_t _writeRegister(uint16_t addr, const uint8_t* data, uint8_t len);
    int16_t _readRegister(uint16_t addr, uint8_t* data, uint8_t len);

    // ---- internal helpers for transmit, receive, and status ----
    int16_t  _writeBuffer(uint8_t offset, const uint8_t* data, uint8_t len);
    int16_t  _readBuffer(uint8_t offset, uint8_t* data, uint8_t len);
    int16_t  _setTx(uint32_t timeout);
    int16_t  _setRx(uint32_t timeout);
    uint16_t _getIrqStatus();
    int16_t  _clearIrqStatus(uint16_t clearIrqParams);
    int16_t  _getRxBufferStatus(uint8_t* payloadLen, uint8_t* startOffset);
    int16_t  _getPacketStatus(uint8_t* status);

public:
    SX1262HAL(uint8_t nssPin, uint8_t busyPin, uint8_t rstPin, uint8_t dio1Pin);

    int16_t begin(float freq, float bw, uint8_t sf, uint8_t cr,
                  uint8_t syncWord, int8_t power, uint16_t preambleLength);

    int16_t transmit(uint8_t* data, size_t len);
    int16_t startReceive();
    size_t  readData(uint8_t* data, size_t maxLen);
    void    setPacketReceivedAction(void (*callback)(void));

    int8_t  getRSSI();
    float   getSNR();
};
