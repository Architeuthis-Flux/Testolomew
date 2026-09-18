// SPDX-License-Identifier: MIT
#include "TMAG5273.h"

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
    uint8_t config1 = MAG_TEMPCO_NDFEB | (uint8_t)( averaging << 2 );
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

bool tmag5273Read( TwoWire* bus, const TMAG5273* dev, TMAG5273Reading* reading ) {
    // 1-byte read mode: no register address, the part just sends the enabled
    // channels (T first if it is on, each MSB first) and then CONV_STATUS.
    uint8_t raw[ 9 ];
    size_t count = dev->temperature ? 9 : 7;
    if ( bus->requestFrom( dev->address, count ) != count ) {
        tmag5273LastFailedRegister = REG_T_MSB_RESULT;
        tmag5273LastFailedCode = 5;
        return false;
    }
    for ( size_t i = 0; i < count; i++ ) {
        raw[ i ] = (uint8_t)bus->read( );
    }
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
    return true;
}
