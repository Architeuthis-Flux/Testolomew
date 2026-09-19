#pragma once
#include "Arduino.h"
class TwoWire {
public:
    void begin() {} void end() {} void setClock(uint32_t) {}
    bool setSCL(int) { return true; } bool setSDA(int) { return true; }
    void beginTransmission(uint8_t) {} size_t write(uint8_t) { return 1; }
    uint8_t endTransmission(bool = true) { return 2; }
    uint8_t peripheral() const { return 3; }
    size_t requestFrom(uint8_t, size_t) { return 0; } int read() { return -1; }
};
extern TwoWire Wire;
