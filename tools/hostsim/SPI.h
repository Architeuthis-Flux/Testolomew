#pragma once
#include "Arduino.h"
#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
class SPISettings { public: SPISettings(uint32_t = 0, BitOrder = MSBFIRST, int = 0) {} };
class SPIClass {
public:
    void begin() {} void end() {} void beginTransaction(SPISettings) {} void endTransaction() {}
    uint8_t transfer(uint8_t) { return 0; } uint16_t transfer16(uint16_t) { return 0; }
    void transfer(void*, size_t) {}
};
extern SPIClass SPI;
