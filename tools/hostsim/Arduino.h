// Host shim of just enough Arduino for a render test. Not part of the project.
#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define ARDUINO 10808
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
enum PinStatus { LOW = 0, HIGH = 1 };
enum BitOrder { LSBFIRST = 0, MSBFIRST = 1 };
#define SPI_MODE3 3
typedef int pin_size_t;
typedef bool boolean;
enum { PA8 = 1, PA13, PA14, PA15, PC6, PC7, PC8, PC10, PD6, PD7, PB12, PB13, PB15, PC2, PC3, PC9, PC11, PC12, PD0, PD1, PD2, PD3, PD4, PD5, PD8, PD9, PD13, PD10, PE15, PD15, PF2, PE14, PD12, PA3, PA4, PD14, PA5, PA6, PB3, PB4 };
extern uint32_t simMillis;
inline uint32_t millis() { return simMillis; }
inline uint32_t micros() { return simMillis * 1000u; }
inline void delay(uint32_t) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
extern int simPinLevel[64];
inline int digitalRead(int p) { return (p >= 0 && p < 64) ? simPinLevel[p] : 1; }
inline int analogRead(int) { return 2048; }
inline void analogReadResolution(int) {}
inline int analogReadResolutionBits() { return 12; }
class __FlashStringHelper;
#define F(s) (s)
class String {
public:
    String(const char* s = "") : p(s) {}
    unsigned length() const { return strlen(p); }
    const char* c_str() const { return p; }
    char operator[](unsigned i) const { return p[i]; }
private:
    const char* p;
};
#include "Print.h"
class Stream : public Print {
public:
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    virtual void flush() {}
    void setTimeout(unsigned long) {}
    long parseInt() { return 0; }
    size_t write(uint8_t c) override { putchar(c); return 1; }
};
extern Stream Serial;
inline float radians(float d) { return d * 0.017453292f; }
