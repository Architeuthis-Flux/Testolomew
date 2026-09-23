// SPDX-License-Identifier: MIT
#ifndef MMC56X3_H
#define MMC56X3_H
// ---------------------------------------------------------------------------
// Memsic MMC56x3 3-axis AMR magnetometer, I2C: the MMC5603NJ and the
// MMC5633NJL (the same register map; the 5633 adds I3C, unused here).
// Register-level driver like TMAG5273.h: no state beyond the struct the
// caller owns, any TwoWire bus.
//
// What is different from the Hall sensor: +/-30 G (3 mT) full scale, 20-bit
// output at 0.0625 mG a count, 1.5-4 mG rms noise a sample (30-50x quieter
// than a TMAG5273 frame), and a FIXED I2C address, 0x30 - one part per bus,
// or a factory-programmed variant. AMR bridges flip in a field above the
// "disturbing field" (32 G); the on-chip SET/RESET puts them back, and with
// Auto_SR_en the part does that around every measurement, so a magnet that
// passes over one costs the clipped readings and nothing after.
//
// Run continuously: ODR in Hz (1-255; 1000 with hpower), bandwidth = the
// decimation filter (6.6 / 3.5 / 2.0 / 1.2 ms a measurement, the noise
// rising as it shortens). The data registers hold the latest measurement;
// a read is the nine bytes from 0x00. Nothing in them counts conversions:
// the same nine bytes twice is the same measurement read twice.
//
// The control registers are WRITE-ONLY (no read-back); only the product
// ID, the status and the data can be read. Sign convention: datasheet
// page 18 - +Z out of the package top, +X from the pin-2 column toward the
// pin-1 column, +Y from row A toward row B (a right-handed frame with z up,
// unlike the TMAG5273's z-into-the-package).
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>

#include "MMC56x3Codes.h"

#define MMC56X3_ADDRESS 0x30      // the factory address (all parts, unless ordered otherwise)
#define MMC56X3_PRODUCT_ID 0x10   // register 0x39 on the MMC5603NJ (Adafruit's driver also accepts 0x00)
#define MMC56X3_RANGE_MT 3.0f     // +/-30 G on each axis
#define MMC56X3_DATA_REGISTER 0x00
#define MMC56X3_DATA_BYTES 9
#define MMC56X3_RESET_MS 20 // power-on / software reset to ready

// Internal Control 1 BW[1:0]: measurement time, and the noise that goes with it.
#define MMC56X3_BW_6_6MS 0 // 1.5 mG rms, up to 75 Hz
#define MMC56X3_BW_3_5MS 1 // 2.0 mG, 150 Hz
#define MMC56X3_BW_2_0MS 2 // 3.0 mG, 255 Hz
#define MMC56X3_BW_1_2MS 3 // 4.0 mG, 255 Hz (1000 Hz with hpower)

struct MMC56x3 {
    uint8_t address;
    uint8_t productId; // register 0x39 as read at begin (0x10 expected)
    uint8_t bandwidth; // MMC56X3_BW_*
    uint16_t odrHz;    // as configured
};

struct MMC56x3Reading {
    float x, y, z; // mT, sensor frame
};

// Does something answer at this address, and what product ID does it give?
// True on an ACK and a readable ID (whatever its value: *productId says).
bool mmc56x3Present( TwoWire* bus, uint8_t address, uint8_t* productId );

// Software reset, a SET/RESET pulse pair, then continuous measurement at
// odrHz (1-255, or 1000: hpower) with the given bandwidth and automatic
// SET/RESET. Fills in *dev. Takes ~25 ms (the reset).
bool mmc56x3Begin( TwoWire* bus, MMC56x3* dev, uint8_t address, uint8_t bandwidth, uint16_t odrHz, bool autoSetReset = true );

// The raw nine data bytes, and Status1 (0x18: bit 6 = a measurement is
// done and unread, bit 4 = the OTP was read at power-up), for a look at
// the part from the console (MagArray's :mmc).
bool mmc56x3ReadRaw( TwoWire* bus, uint8_t address, uint8_t* raw );
bool mmc56x3ReadStatus( TwoWire* bus, uint8_t address, uint8_t* status );

// A SET then a RESET pulse by hand (the bridges re-magnetised). It writes
// Internal Control 0 whole, which drops Auto_SR_en and Cmm_freq_en on a
// running part: for use before the mode is set (mmc56x3Begin does), or
// followed by mmc56x3Begin.
bool mmc56x3SetReset( TwoWire* bus, uint8_t address );

// The latest measurement: one 9-byte register read from 0x00 (Wire, or the
// I2C block directly on the device), and what the bytes mean.
size_t mmc56x3ReadBytes( void );
bool mmc56x3Read( TwoWire* bus, const MMC56x3* dev, MMC56x3Reading* reading );
void mmc56x3Decode( const uint8_t* raw, MMC56x3Reading* reading );

// Single register access, for poking at things from the console.
bool mmc56x3WriteRegister( TwoWire* bus, uint8_t address, uint8_t reg, uint8_t value );
bool mmc56x3ReadRegister( TwoWire* bus, uint8_t address, uint8_t reg, uint8_t* value );

// The last bus transaction that failed: the register and Wire's code (2 =
// address not acknowledged, 3 = data not acknowledged, 4 = bus fault, 5 =
// short read).
extern uint8_t mmc56x3LastFailedRegister;
extern uint8_t mmc56x3LastFailedCode;

#endif // MMC56X3_H
