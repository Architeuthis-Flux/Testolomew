// SPDX-License-Identifier: MIT
#include "MMC56x3.h"

#include "MagI2c.h"

#if __has_include( <ch32h4_i2c.h> )
#define MMC_DIRECT_READ 1
#else
#define MMC_DIRECT_READ 0 // the host simulation reads through Wire
#endif

// Register map (MMC5603NJ datasheet rev B, page 7).
#define REG_XOUT0 0x00
#define REG_TOUT 0x09
#define REG_STATUS1 0x18
#define REG_ODR 0x1A
#define REG_CONTROL0 0x1B
#define REG_CONTROL1 0x1C
#define REG_CONTROL2 0x1D
#define REG_PRODUCT_ID 0x39

// Internal Control 0 (write-only).
#define CTRL0_TAKE_MEAS_M 0x01
#define CTRL0_TAKE_MEAS_T 0x02
#define CTRL0_DO_SET 0x08
#define CTRL0_DO_RESET 0x10
#define CTRL0_AUTO_SR_EN 0x20
#define CTRL0_CMM_FREQ_EN 0x80
// Internal Control 1 (write-only): BW[1:0] in the low bits.
#define CTRL1_SW_RESET 0x80
// Internal Control 2 (write-only).
#define CTRL2_CMM_EN 0x10
#define CTRL2_HPOWER 0x80

uint8_t mmc56x3LastFailedRegister = 0;
uint8_t mmc56x3LastFailedCode = 0;

bool mmc56x3WriteRegister( TwoWire* bus, uint8_t address, uint8_t reg, uint8_t value ) {
    bus->beginTransmission( address );
    bus->write( reg );
    bus->write( value );
    uint8_t code = bus->endTransmission( );
    if ( code != 0 ) {
        mmc56x3LastFailedRegister = reg;
        mmc56x3LastFailedCode = code;
    }
    return code == 0;
}

static bool readRegisters( TwoWire* bus, uint8_t address, uint8_t reg, uint8_t* out, size_t count ) {
    bus->beginTransmission( address );
    bus->write( reg );
    uint8_t code = bus->endTransmission( false );
    if ( code != 0 ) {
        mmc56x3LastFailedRegister = reg;
        mmc56x3LastFailedCode = code;
        return false;
    }
    if ( bus->requestFrom( address, count ) != count ) {
        mmc56x3LastFailedRegister = reg;
        mmc56x3LastFailedCode = 5;
        return false;
    }
    for ( size_t i = 0; i < count; i++ ) {
        out[ i ] = (uint8_t)bus->read( );
    }
    return true;
}

bool mmc56x3ReadRegister( TwoWire* bus, uint8_t address, uint8_t reg, uint8_t* value ) {
    return readRegisters( bus, address, reg, value, 1 );
}

bool mmc56x3Present( TwoWire* bus, uint8_t address, uint8_t* productId ) {
    uint8_t id = 0;
    if ( !readRegisters( bus, address, REG_PRODUCT_ID, &id, 1 ) ) {
        return false;
    }
    if ( productId != nullptr ) {
        *productId = id;
    }
    return true;
}

bool mmc56x3SetReset( TwoWire* bus, uint8_t address ) {
    // Each pulse is 375 ns of coil current; the bit clears itself. A
    // millisecond between them is Adafruit's driver's margin.
    if ( !mmc56x3WriteRegister( bus, address, REG_CONTROL0, CTRL0_DO_SET ) ) {
        return false;
    }
    delay( 1 );
    if ( !mmc56x3WriteRegister( bus, address, REG_CONTROL0, CTRL0_DO_RESET ) ) {
        return false;
    }
    delay( 1 );
    return true;
}

bool mmc56x3ReadRaw( TwoWire* bus, uint8_t address, uint8_t* raw ) {
    return readRegisters( bus, address, REG_XOUT0, raw, MMC56X3_DATA_BYTES );
}

bool mmc56x3ReadStatus( TwoWire* bus, uint8_t address, uint8_t* status ) {
    return readRegisters( bus, address, REG_STATUS1, status, 1 );
}

bool mmc56x3Begin( TwoWire* bus, MMC56x3* dev, uint8_t address, uint8_t bandwidth, uint16_t odrHz, bool autoSetReset ) {
    // A software reset: every register to its power-up value and the OTP
    // re-read, 20 ms. It leaves a part that was running continuous mode
    // stopped, which is the state the sequence below expects.
    if ( !mmc56x3WriteRegister( bus, address, REG_CONTROL1, CTRL1_SW_RESET ) ) {
        return false;
    }
    delay( MMC56X3_RESET_MS );
    uint8_t id = 0;
    if ( !readRegisters( bus, address, REG_PRODUCT_ID, &id, 1 ) ) {
        return false;
    }
    dev->address = address;
    dev->productId = id;
    dev->bandwidth = bandwidth & 3;
    dev->odrHz = odrHz;

    bool ok = mmc56x3SetReset( bus, address );
    // The order the datasheet gives for continuous mode: the bandwidth, a
    // non-zero ODR, Cmm_freq_en (the part works out its measurement period
    // from the ODR; self-clearing) with Auto_SR_en (a SET/RESET around
    // every measurement, which the noise figures assume and which heals a
    // bridge a magnet flipped), then Cmm_en - with hpower for 1000 Hz.
    ok &= mmc56x3WriteRegister( bus, address, REG_CONTROL1, dev->bandwidth );
    bool highPower = odrHz > 255;
    ok &= mmc56x3WriteRegister( bus, address, REG_ODR, highPower ? 255 : (uint8_t)( odrHz == 0 ? 1 : odrHz ) );
    ok &= mmc56x3WriteRegister( bus, address, REG_CONTROL0, CTRL0_CMM_FREQ_EN | ( autoSetReset ? CTRL0_AUTO_SR_EN : 0 ) );
    ok &= mmc56x3WriteRegister( bus, address, REG_CONTROL2, CTRL2_CMM_EN | ( highPower ? CTRL2_HPOWER : 0 ) );
    return ok;
}

size_t mmc56x3ReadBytes( void ) {
    return MMC56X3_DATA_BYTES;
}

void mmc56x3Decode( const uint8_t* raw, MMC56x3Reading* reading ) {
    mmc56x3DecodeBytes( raw, &reading->x, &reading->y, &reading->z );
}

bool mmc56x3Read( TwoWire* bus, const MMC56x3* dev, MMC56x3Reading* reading ) {
    uint8_t raw[ MMC56X3_DATA_BYTES ];
#if MMC_DIRECT_READ
    if ( !magI2cBurstRead( bus->peripheral( ), dev->address, MMC56X3_DATA_REGISTER, raw, MMC56X3_DATA_BYTES ) ) {
        mmc56x3LastFailedRegister = REG_XOUT0;
        mmc56x3LastFailedCode = 5;
        return false;
    }
#else
    if ( !readRegisters( bus, dev->address, REG_XOUT0, raw, MMC56X3_DATA_BYTES ) ) {
        return false;
    }
#endif
    mmc56x3Decode( raw, reading );
    return true;
}
