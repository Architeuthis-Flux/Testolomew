// SPDX-License-Identifier: MIT
#ifndef TMAG5273_H
#define TMAG5273_H
// ---------------------------------------------------------------------------
// TI TMAG5273 3-axis Hall-effect sensor, I2C. Register-level driver: no state
// beyond the little struct the caller owns, any TwoWire bus, any number of
// sensors.
//
// The one thing to know about this part: ITS I2C ADDRESS IS NOT STORED. The
// I2C_ADDRESS register is loaded from the factory value at every power-up, so
// a changed address lasts exactly as long as VCC does (datasheet 7.2.2: "Repeat
// the above steps if there is a power outage or power-up reset condition").
// Several sensors of the same variant on one bus therefore need their supplies
// switched one at a time at EVERY boot - see MagArray.
//
// Factory addresses by orderable: A = 0x35, B = 0x22, C = 0x78, D = 0x44.
// The digit after the letter is the range: x1 = +/-40 / 80 mT,
// x2 = +/-133 / 266 mT (read back from DEVICE_ID).
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>

#define TMAG5273_POWER_UP_US 270 // VCC valid to ready for I2C (tstart_power_up)

// CONV_AVG field: samples averaged per result. More = quieter and slower.
// Result rate with all three axes on: 1x 10 kHz, 2x 5.7 kHz, 4x 3.1 kHz,
// 8x 1.6 kHz, 16x 800 Hz, 32x 400 Hz.
#define TMAG5273_AVG_1X 0
#define TMAG5273_AVG_2X 1
#define TMAG5273_AVG_4X 2
#define TMAG5273_AVG_8X 3
#define TMAG5273_AVG_16X 4
#define TMAG5273_AVG_32X 5

struct TMAG5273 {
    uint8_t address;  // 7-bit, as currently assigned
    uint8_t version;  // DEVICE_ID VER: 1 = 40/80 mT part, 2 = 133/266 mT part
    float rangeMt;    // full scale in use (the same on all three axes)
    bool temperature; // the temperature channel is on (and in every read)
};

struct TMAG5273Reading {
    float x, y, z;      // mT, sensor frame
    float temperatureC; // die temperature
    uint8_t status;     // CONV_STATUS as read (bit 0 = a finished conversion)
};

// The four factory addresses, for finding a freshly powered sensor.
extern const uint8_t tmag5273FactoryAddresses[ 4 ];

// Does a TMAG5273 answer at this address? (ACK + TI's manufacturer ID.)
bool tmag5273Present( TwoWire* bus, uint8_t address );

// Move the sensor at oldAddress to newAddress (until its next power cycle).
bool tmag5273SetAddress( TwoWire* bus, uint8_t oldAddress, uint8_t newAddress );

// Continuous conversion of X, Y and Z, low-noise mode, NdFeB temperature
// compensation (the part scales its readings by 0.12 %/degC of its own
// temperature, to follow the magnet's), the INT pin masked (it may be
// unconnected). highRange picks the wider of the part's two ranges.
// temperature also converts and returns the die temperature, which costs a
// quarter of the conversion rate at 32x averaging (3.2 ms per result instead
// of 2.4) and two more bytes per read. Fills in *dev.
//
// The part is left in its "1-byte read" mode: a bare read returns the enabled
// channels and the conversion status with no register address written first,
// which is a quarter less bus traffic per sample. In that mode ordinary
// register READS no longer work (every read returns sensor data), so this
// function switches it back to standard reads first - it can be called again
// on a sensor that is already running.
bool tmag5273Begin( TwoWire* bus, TMAG5273* dev, uint8_t address, uint8_t averaging, bool highRange, bool temperature );

// Does anything acknowledge at this address? Works in either read mode, which
// tmag5273Present() (it reads the manufacturer ID) does not.
bool tmag5273Acknowledges( TwoWire* bus, uint8_t address );

// The latest result: field in mT, and die temperature if that channel is on
// (else 0). One 7- or 9-byte read. false on a bus error.
bool tmag5273Read( TwoWire* bus, const TMAG5273* dev, TMAG5273Reading* reading );

// The last bus transaction that failed, for diagnostics: which register, and
// Wire's endTransmission() code (2 = address not acknowledged, 3 = data not
// acknowledged, 4 = bus fault / timeout, 5 = short read).
extern uint8_t tmag5273LastFailedRegister;
extern uint8_t tmag5273LastFailedCode;

// Single register access, for poking at things from the console.
bool tmag5273WriteRegister( TwoWire* bus, uint8_t address, uint8_t reg, uint8_t value );
bool tmag5273ReadRegister( TwoWire* bus, uint8_t address, uint8_t reg, uint8_t* value );

#endif // TMAG5273_H
