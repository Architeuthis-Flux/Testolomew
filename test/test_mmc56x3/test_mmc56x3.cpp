// SPDX-License-Identifier: MIT
// Host-side test of the MMC56x3's output decoding (MMC56x3Codes.h): the
// nine data bytes of one read to a field in mT. Run with `pio test -e native`.
#include <unity.h>

#include "MMC56x3Codes.h"

void setUp( void ) {}
void tearDown( void ) {}

void test_null_field_is_zero( void ) {
    // 2^19 = 0x80000: Xout0 0x80, Xout1 0x00, Xout2 0x0 in the high nibble.
    uint8_t raw[ 9 ] = { 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00 };
    float x, y, z;
    mmc56x3DecodeBytes( raw, &x, &y, &z );
    TEST_ASSERT_FLOAT_WITHIN( 1e-9f, 0.0f, x );
    TEST_ASSERT_FLOAT_WITHIN( 1e-9f, 0.0f, y );
    TEST_ASSERT_FLOAT_WITHIN( 1e-9f, 0.0f, z );
}

void test_one_gauss_each_way( void ) {
    // +1 G = 16384 counts above null: 0x80000 + 0x4000 = 0x84000 -> bytes 0x84, 0x00, low nibble 0.
    // -1 G = 0x7C000 -> 0x7C, 0x00, 0.
    uint8_t raw[ 9 ] = { 0x84, 0x00, 0x7C, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00 };
    float x, y, z;
    mmc56x3DecodeBytes( raw, &x, &y, &z );
    TEST_ASSERT_FLOAT_WITHIN( 1e-6f, 0.1f, x );  // 1 G = 0.1 mT
    TEST_ASSERT_FLOAT_WITHIN( 1e-6f, -0.1f, y );
    TEST_ASSERT_FLOAT_WITHIN( 1e-9f, 0.0f, z );
}

void test_low_nibble_counts( void ) {
    // The low four bits ride in the high nibble of the third byte: 0x80000 + 0x5 -> 5 counts.
    uint8_t raw[ 9 ] = { 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x50, 0x00, 0xF0 };
    float x, y, z;
    mmc56x3DecodeBytes( raw, &x, &y, &z );
    TEST_ASSERT_FLOAT_WITHIN( 1e-9f, 5.0f * MMC56X3_MT_PER_COUNT, x );
    TEST_ASSERT_FLOAT_WITHIN( 1e-9f, 0.0f, y );
    TEST_ASSERT_FLOAT_WITHIN( 1e-9f, 15.0f * MMC56X3_MT_PER_COUNT, z );
    // ...and the low nibble of that byte (reserved / FIFO on the 5616) is ignored.
    raw[ 6 ] = 0x5F;
    mmc56x3DecodeBytes( raw, &x, &y, &z );
    TEST_ASSERT_FLOAT_WITHIN( 1e-9f, 5.0f * MMC56X3_MT_PER_COUNT, x );
}

void test_full_scale_is_about_three_millitesla( void ) {
    // All ones (0xFFFFF) is +3.2 mT less a count; all zeros is -3.2 mT: the +/-30 G range with a little over.
    uint8_t top[ 9 ] = { 0xFF, 0xFF, 0x00, 0x00, 0x80, 0x00, 0xF0, 0x00, 0x00 };
    float x, y, z;
    mmc56x3DecodeBytes( top, &x, &y, &z );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 3.2f, x );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, -3.2f, y );
    TEST_ASSERT_TRUE( x > 3.0f * 0.95f ); // a clipped reading is past the saturation limit
}

int main( void ) {
    UNITY_BEGIN( );
    RUN_TEST( test_null_field_is_zero );
    RUN_TEST( test_one_gauss_each_way );
    RUN_TEST( test_low_nibble_counts );
    RUN_TEST( test_full_scale_is_about_three_millitesla );
    return UNITY_END( );
}
