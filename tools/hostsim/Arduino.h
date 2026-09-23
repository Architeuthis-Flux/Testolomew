// Host shim of just enough Arduino for the simulator and the bench programs.
// Not part of the firmware. Time is simMicros, which the program advances.
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
enum { PA8 = 1, PA13, PA14, PA15, PC6, PC7, PC8, PC10, PD6, PD7, PB12, PB13, PB15, PC2, PC3, PC9, PC11, PC12, PD0, PD1, PD2, PD3, PD4, PD5, PD8, PD9, PD13, PD10, PE15, PD15, PF2, PE14, PD12, PA3, PA4, PD14, PA5, PA6, PB3, PB4, PF12, PF13 };
extern uint64_t simMicros; // the clock: whoever runs the loop advances it
inline uint32_t millis() { return (uint32_t)(simMicros / 1000u); }
inline uint32_t micros() { return (uint32_t)simMicros; }
inline void delay(uint32_t) {}
inline void delayMicroseconds(uint32_t) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
extern int simPinLevel[64];
inline int digitalRead(int p) { return (p >= 0 && p < 64) ? simPinLevel[p] : 1; }
extern int simAnalog[64];
inline int analogRead(int p) { return (p >= 0 && p < 64) ? simAnalog[p] : 2048; }
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
    using Print::write;
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    virtual void flush() {}
    void setTimeout(unsigned long) {}
    long parseInt() { return 0; }
    size_t write(uint8_t c) override { putchar(c); return 1; }
};
// The serial port: what the program types goes in with type(); what the
// firmware prints goes to onWrite (stdout by default).
class HardwareSerialStub : public Stream {
public:
    using Print::write;
    void begin(unsigned long) {}
    void end() {}
    int available() override { return count; }
    int read() override { if (!count) return -1; char c = q[head]; head = (head + 1) % QSIZE; count--; return c; }
    int peek() override { return count ? q[head] : -1; }
    size_t write(uint8_t c) override { if (onWrite) onWrite(c); else putchar(c); return 1; }
    void type(const char* s) { while (*s) { if (count < QSIZE) { q[(head + count) % QSIZE] = *s; count++; } s++; } }
    void (*onWrite)(uint8_t) = nullptr;
private:
    enum { QSIZE = 8192 };
    char q[QSIZE];
    int head = 0, count = 0;
};
extern HardwareSerialStub Serial;
inline float radians(float d) { return d * 0.017453292f; }
