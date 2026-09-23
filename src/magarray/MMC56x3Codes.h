// SPDX-License-Identifier: MIT
#ifndef MMC56X3CODES_H
#define MMC56X3CODES_H
// ---------------------------------------------------------------------------
// The MMC56x3's output codes to a field, pure arithmetic (no Arduino), so the
// host tests check it. Nine data bytes from register 0x00: Xout0 Xout1 Yout0
// Yout1 Zout0 Zout1 (the high 16 bits of each axis, MSB first) then Xout2
// Yout2 Zout2 (the low 4 bits in each byte's high nibble). 20-bit unsigned,
// null field at 2^19, 16384 counts per gauss (datasheet: 0.0625 mG per LSB).
// ---------------------------------------------------------------------------
#include <stdint.h>

#define MMC56X3_NULL_CODE ( 1L << 19 )
#define MMC56X3_COUNTS_PER_GAUSS 16384.0f
#define MMC56X3_MT_PER_COUNT ( 0.1f / MMC56X3_COUNTS_PER_GAUSS ) // 10 gauss to the millitesla

static inline int32_t mmc56x3Code20( uint8_t high, uint8_t middle, uint8_t low ) {
    uint32_t code = ( (uint32_t)high << 12 ) | ( (uint32_t)middle << 4 ) | ( (uint32_t)low >> 4 );
    return (int32_t)code - MMC56X3_NULL_CODE;
}

static inline float mmc56x3CodeToMt( int32_t code ) {
    return (float)code * MMC56X3_MT_PER_COUNT;
}

// The three axes of one 9-byte read, mT, sensor frame.
static inline void mmc56x3DecodeBytes( const uint8_t* raw, float* x, float* y, float* z ) {
    *x = mmc56x3CodeToMt( mmc56x3Code20( raw[ 0 ], raw[ 1 ], raw[ 6 ] ) );
    *y = mmc56x3CodeToMt( mmc56x3Code20( raw[ 2 ], raw[ 3 ], raw[ 7 ] ) );
    *z = mmc56x3CodeToMt( mmc56x3Code20( raw[ 4 ], raw[ 5 ], raw[ 8 ] ) );
}

#endif // MMC56X3CODES_H
