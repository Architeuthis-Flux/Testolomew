// SPDX-License-Identifier: MIT
#include "TMAG5273.h"

#if __has_include( <ch32h4_i2c.h> )
#include <ch32h4_i2c.h> // the I2C block's registers: the burst read below runs on them
extern "C" {
#include <ch32h4_itcm.h>  // __itcm_func
#include <ch32h4_xcore.h> // ch32h4_core_num(): which core is reading
}
#define TMAG_DIRECT_READ 1
#else
#define TMAG_DIRECT_READ 0 // the host simulation reads through Wire
#endif

// Register map (datasheet table 8-1).
#define REG_DEVICE_CONFIG_1 0x00
#define REG_DEVICE_CONFIG_2 0x01
#define REG_SENSOR_CONFIG_1 0x02
#define REG_SENSOR_CONFIG_2 0x03
#define REG_T_CONFIG 0x07
#define REG_INT_CONFIG_1 0x08
#define REG_I2C_ADDRESS 0x0C
#define REG_DEVICE_ID 0x0D
#define REG_MANUFACTURER_ID_LSB 0x0E
#define REG_MANUFACTURER_ID_MSB 0x0F
#define REG_T_MSB_RESULT 0x10
#define REG_CONV_STATUS 0x18

#define I2C_RD_STANDARD 0x00         // DEVICE_CONFIG_1: reads name a register
#define I2C_RD_ONE_BYTE_16BIT 0x01   // DEVICE_CONFIG_1: a bare read returns the channels + status
#define MAG_TEMPCO_NDFEB ( 1u << 5 ) // DEVICE_CONFIG_1: 0.12 %/degC
#define LOW_NOISE_MODE ( 1u << 4 )   // DEVICE_CONFIG_2: LP_LN
#define MODE_CONTINUOUS 0x02         // DEVICE_CONFIG_2: OPERATING_MODE
#define MAG_CH_EN_XYZ ( 7u << 4 )    // SENSOR_CONFIG_1
#define RANGE_XY_HIGH ( 1u << 1 )    // SENSOR_CONFIG_2
#define RANGE_Z_HIGH ( 1u << 0 )
#define T_CH_EN 0x01                // T_CONFIG: temperature channel on
#define MASK_INTB 0x01              // INT_CONFIG_1: ignore the INT pin
#define I2C_ADDRESS_UPDATE_EN 0x01  // I2C_ADDRESS bit 0
#define CONV_STATUS_POR ( 1u << 4 ) // write 1 to clear

// Temperature result: code 17508 at 25 degC, 58 codes per degC (datasheet
// electrical characteristics: TADC_T0, TSENS_T0, TADC_RES).
#define TADC_T0 17508.0f
#define TSENS_T0 25.0f
#define TADC_RES 58.0f

const uint8_t tmag5273FactoryAddresses[ 4 ] = { 0x35, 0x22, 0x78, 0x44 };

uint8_t tmag5273LastFailedRegister = 0;
uint8_t tmag5273LastFailedCode = 0;

bool tmag5273WriteRegister( TwoWire* bus, uint8_t address, uint8_t reg, uint8_t value ) {
    bus->beginTransmission( address );
    bus->write( reg );
    bus->write( value );
    uint8_t code = bus->endTransmission( );
    if ( code != 0 ) {
        tmag5273LastFailedRegister = reg;
        tmag5273LastFailedCode = code;
    }
    return code == 0;
}

static bool readRegisters( TwoWire* bus, uint8_t address, uint8_t reg, uint8_t* out, size_t count ) {
    bus->beginTransmission( address );
    bus->write( reg );
    uint8_t code = bus->endTransmission( false );
    if ( code != 0 ) {
        tmag5273LastFailedRegister = reg;
        tmag5273LastFailedCode = code;
        return false;
    }
    if ( bus->requestFrom( address, count ) != count ) {
        tmag5273LastFailedRegister = reg;
        tmag5273LastFailedCode = 5;
        return false;
    }
    for ( size_t i = 0; i < count; i++ ) {
        out[ i ] = (uint8_t)bus->read( );
    }
    return true;
}

bool tmag5273ReadRegister( TwoWire* bus, uint8_t address, uint8_t reg, uint8_t* value ) {
    return readRegisters( bus, address, reg, value, 1 );
}

bool tmag5273Present( TwoWire* bus, uint8_t address ) {
    uint8_t id[ 2 ];
    if ( !readRegisters( bus, address, REG_MANUFACTURER_ID_LSB, id, 2 ) ) {
        return false;
    }
    return id[ 0 ] == 0x49 && id[ 1 ] == 0x54; // "TI"
}

bool tmag5273SetAddress( TwoWire* bus, uint8_t oldAddress, uint8_t newAddress ) {
    uint8_t value = (uint8_t)( newAddress << 1 ) | I2C_ADDRESS_UPDATE_EN;
    if ( !tmag5273WriteRegister( bus, oldAddress, REG_I2C_ADDRESS, value ) ) {
        return false;
    }
    return tmag5273Present( bus, newAddress );
}

bool tmag5273Acknowledges( TwoWire* bus, uint8_t address ) {
    bus->beginTransmission( address );
    return bus->endTransmission( ) == 0;
}

bool tmag5273Begin( TwoWire* bus, TMAG5273* dev, uint8_t address, uint8_t averaging, bool highRange, bool temperature ) {
    // Standard reads first: if the part is already in 1-byte read mode the
    // DEVICE_ID read below would come back as sensor data.
    // No magnet temperature compensation: it scales every reading by 0.12 %/degC
    // of the DIE's temperature, to follow a magnet that sits at the same
    // temperature as the sensor. The probe's magnet is in a hand, the die is
    // on the board, and the fit solves the magnet's strength every frame
    // anyway, so the setting only adds a temperature-tracking gain error
    // (datasheet 5.9, 7.1.2; see docs/magnetometer-fusion-prior-art.md 2.5).
    uint8_t config1 = (uint8_t)( averaging << 2 );
    if ( !tmag5273WriteRegister( bus, address, REG_DEVICE_CONFIG_1, config1 | I2C_RD_STANDARD ) ) {
        return false;
    }
    uint8_t id = 0;
    if ( !tmag5273ReadRegister( bus, address, REG_DEVICE_ID, &id ) ) {
        return false;
    }
    dev->address = address;
    dev->version = id & 0x03;
    float low = ( dev->version == 2 ) ? 133.0f : 40.0f;
    dev->rangeMt = highRange ? 2.0f * low : low;
    dev->temperature = temperature;

    bool ok = true;
    ok &= tmag5273WriteRegister( bus, address, REG_SENSOR_CONFIG_1, MAG_CH_EN_XYZ );
    ok &= tmag5273WriteRegister( bus, address, REG_SENSOR_CONFIG_2, highRange ? ( RANGE_XY_HIGH | RANGE_Z_HIGH ) : 0 );
    ok &= tmag5273WriteRegister( bus, address, REG_T_CONFIG, temperature ? T_CH_EN : 0 );
    ok &= tmag5273WriteRegister( bus, address, REG_INT_CONFIG_1, MASK_INTB );
    ok &= tmag5273WriteRegister( bus, address, REG_CONV_STATUS, CONV_STATUS_POR );
    ok &= tmag5273WriteRegister( bus, address, REG_DEVICE_CONFIG_2, LOW_NOISE_MODE | MODE_CONTINUOUS );
    // Last: from here on a bare read returns [T] X Y Z status.
    ok &= tmag5273WriteRegister( bus, address, REG_DEVICE_CONFIG_1, config1 | I2C_RD_ONE_BYTE_16BIT );
    return ok;
}

bool tmag5273SetLowNoise( TwoWire* bus, uint8_t address, bool lowNoise ) {
    return tmag5273WriteRegister( bus, address, REG_DEVICE_CONFIG_2, ( lowNoise ? LOW_NOISE_MODE : 0 ) | MODE_CONTINUOUS );
}

#if !TMAG_DIRECT_READ
bool tmag5273BurstRead( int, uint8_t, uint8_t*, size_t ) {
    return false; // no I2C block on the host
}
void tmag5273BusReset( int, uint32_t ) {}
#else
// The I2C block reset and set up again as a master at `hz`, the way Wire's
// begin() does it (minus the pins, which stay as they are): what gets a
// block out of a wedge a glitch put it in. Registers only, so the sampler
// can do it from the other core.
void tmag5273BusReset( int peripheral, uint32_t hz ) {
    uint8_t id = (uint8_t)peripheral;
    I2C_TypeDef* dev = ch32h4_i2c_regs( id );
    if ( dev == nullptr ) {
        return;
    }
    ch32h4_i2c_reset( id );
    ch32h4_i2c_clock_enable( id );
    I2C_InitTypeDef init = { };
    init.I2C_Mode = I2C_Mode_I2C;
    init.I2C_DutyCycle = I2C_DutyCycle_2;
    init.I2C_OwnAddress1 = 0x00;
    init.I2C_Ack = I2C_Ack_Enable;
    init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    init.I2C_ClockSpeed = hz;
    I2C_Init( dev, &init );
    I2C_Cmd( dev, ENABLE );
}

// The burst read on the I2C block itself rather than through Wire, for the
// waits: Wire waits a fixed 10 ms for every flag, and a glitch on the lines
// (2026-09-18: the LED strip's data wire coupling into SDA, seen by the
// block as a STOP) drops the block out of master mode in the middle of a
// read, after which each remaining flag is waited for in vain - 10 ms gone
// from every service behind this one, ten times a second with the strip
// busy. Here a wait ends the moment the block is no longer master or
// reports a bus error, and in TMAG_READ_TIMEOUT_US anyway (a 9-byte read
// at 400 kHz is 225 us). The STOP request such an aborted read leaves
// pending would stop the next START ever happening; it is cleared first.
#define TMAG_READ_TIMEOUT_US 1000
#define TMAG_I2C_ERRORS ( I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_AF )

// The clock for the timeout. micros() is right on the V5F only: its
// sub-millisecond part is that core's own SysTick, which the V3F does not
// have (there it reads as noise, and a timeout of noise fails one read in
// eight - seen 2026-09-18). On the V3F, where the sampler runs, millis()
// (kept by the V5F in shared memory) is used instead, with a coarser limit.
#define TMAG_READ_TIMEOUT_MS 3

struct ReadClock {
    bool coarse;
    uint32_t start;
};

__itcm_func static ReadClock clockStart( ) {
    ReadClock c;
    c.coarse = ch32h4_core_num( ) == 0;
    c.start = c.coarse ? millis( ) : micros( );
    return c;
}

__itcm_func static bool clockExpired( const ReadClock* c ) {
    return c->coarse ? ( millis( ) - c->start > TMAG_READ_TIMEOUT_MS ) : ( micros( ) - c->start > TMAG_READ_TIMEOUT_US );
}

// The read's hot path is __itcm_func: the V3F, which runs it most, has no
// instruction cache and fetches flash at a crawl (a pass over eight sensors
// took 6 ms against 1.6 ms of bus time: the block stretched the clock every
// byte waiting for the core to notice). ITCM it reaches over the bus in a
// few cycles. The SDK's calls (in flash) are replaced by the register
// writes they make.
__itcm_func static bool waitFlag( I2C_TypeDef* dev, uint16_t flag, const ReadClock* clock, bool mustBeMaster ) {
    // The clock is looked at every 32nd turn only: it is a safety net, and
    // on the V3F reading it means the other core's memory, with waits, which
    // at every turn of a 20 us wait halved the sampler's pass rate.
    for ( uint32_t turn = 0;; turn++ ) {
        uint16_t star1 = dev->STAR1;
        if ( star1 & flag ) {
            return true;
        }
        if ( star1 & TMAG_I2C_ERRORS ) {
            return false;
        }
        if ( mustBeMaster && !( dev->STAR2 & I2C_STAR2_MSL ) ) {
            return false; // a STOP the block did not send: the lines glitched
        }
        if ( ( turn & 31u ) == 31u && clockExpired( clock ) ) {
            return false;
        }
    }
}

// false = failed; the block is left idle and ready for the next transaction.
// On the I2C block's registers alone, so it runs the same from either core
// (the sampler on the V3F calls it, MagSampler.cpp).
__itcm_func static bool burstReadOn( I2C_TypeDef* dev, uint8_t address, uint8_t* out, size_t count ) {
    if ( ( dev->CTLR1 & I2C_CTLR1_STOP ) && !( dev->STAR2 & I2C_STAR2_MSL ) ) {
        dev->CTLR1 &= (uint16_t)~I2C_CTLR1_STOP; // left over from an aborted transaction
    }
    dev->STAR1 &= (uint16_t)~TMAG_I2C_ERRORS;
    ReadClock t0 = clockStart( );
    dev->CTLR1 |= I2C_CTLR1_ACK;
    dev->CTLR1 |= I2C_CTLR1_START;
    if ( !waitFlag( dev, I2C_STAR1_SB, &t0, false ) ) {
        dev->CTLR1 |= I2C_CTLR1_STOP;
        return false;
    }
    dev->DATAR = (uint16_t)( ( address << 1 ) | 1u ); // the address, read direction
    if ( !waitFlag( dev, I2C_STAR1_ADDR, &t0, true ) ) {
        dev->STAR1 &= (uint16_t)~I2C_STAR1_AF; // a NACK: nobody there
        dev->CTLR1 |= I2C_CTLR1_STOP;
        return false;
    }
    (void)dev->STAR2; // reading STAR1 then STAR2 clears ADDR
    for ( size_t i = 0; i < count; i++ ) {
        if ( i + 1 == count ) {
            // NACK the last byte, and the STOP after it, before it arrives.
            dev->CTLR1 &= (uint16_t)~I2C_CTLR1_ACK;
            dev->CTLR1 |= I2C_CTLR1_STOP;
        }
        if ( !waitFlag( dev, I2C_STAR1_RXNE, &t0, i + 1 < count ) ) {
            dev->CTLR1 |= I2C_CTLR1_STOP;
            return false;
        }
        out[ i ] = (uint8_t)dev->DATAR;
    }
    return true;
}

bool tmag5273BurstRead( int peripheral, uint8_t address, uint8_t* out, size_t count ) {
    I2C_TypeDef* dev = ch32h4_i2c_regs( (uint8_t)peripheral );
    if ( dev == nullptr ) {
        return false;
    }
    return burstReadOn( dev, address, out, count );
}
#endif

size_t tmag5273ReadBytes( const TMAG5273* dev ) {
    return dev->temperature ? 9 : 7;
}

void tmag5273Decode( const TMAG5273* dev, const uint8_t* raw, TMAG5273Reading* reading ) {
    // 1-byte read mode: the part sends the enabled channels (T first if it
    // is on, each MSB first) and then CONV_STATUS.
    const uint8_t* xyz = raw;
    reading->temperatureC = 0.0f;
    if ( dev->temperature ) {
        reading->temperatureC = TSENS_T0 + ( (float)( ( raw[ 0 ] << 8 ) | raw[ 1 ] ) - TADC_T0 ) / TADC_RES;
        xyz = raw + 2;
    }
    float scale = dev->rangeMt / 32768.0f;
    reading->x = (int16_t)( ( xyz[ 0 ] << 8 ) | xyz[ 1 ] ) * scale;
    reading->y = (int16_t)( ( xyz[ 2 ] << 8 ) | xyz[ 3 ] ) * scale;
    reading->z = (int16_t)( ( xyz[ 4 ] << 8 ) | xyz[ 5 ] ) * scale;
    reading->status = xyz[ 6 ];
}

bool tmag5273Read( TwoWire* bus, const TMAG5273* dev, TMAG5273Reading* reading ) {
    uint8_t raw[ 9 ];
    size_t count = tmag5273ReadBytes( dev );
#if TMAG_DIRECT_READ
    if ( !tmag5273BurstRead( bus->peripheral( ), dev->address, raw, count ) ) {
        tmag5273LastFailedRegister = REG_T_MSB_RESULT;
        tmag5273LastFailedCode = 5;
        return false;
    }
#else
    if ( bus->requestFrom( dev->address, count ) != count ) {
        tmag5273LastFailedRegister = REG_T_MSB_RESULT;
        tmag5273LastFailedCode = 5;
        return false;
    }
    for ( size_t i = 0; i < count; i++ ) {
        raw[ i ] = (uint8_t)bus->read( );
    }
#endif
    tmag5273Decode( dev, raw, reading );
    return true;
}
