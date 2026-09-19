// SPDX-License-Identifier: MIT
// Host-side test of the closed-form far-field estimate: the field a known
// magnet makes at the bench array, and whether magFarEstimate() puts it
// roughly where it is - exactly for a magnet far out (the gradient is then a
// true derivative), and within its own error bar closer in, with noise.
// Run with `pio test -e native`.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unity.h>

#include "MagFar.h"
#include "MagFit.h"

#define SENSOR_COUNT 8

// The bench array, as measured (src/magarray/MagArrayConfig.h).
static const Vec3 sensors[ SENSOR_COUNT ] = {
    { 0.10f, 44.29f, 0 }, { 16.75f, 44.04f, 0 }, { 38.78f, 43.95f, 0 }, { 54.18f, 44.72f, 0 },
    { 0.00f, 0.00f, 0 },  { 15.60f, 0.19f, 0 },  { 37.37f, 0.42f, 0 },  { 53.40f, 0.00f, 0 },
};

void setUp( void ) {}
void tearDown( void ) {}

static float gaussian( float sigma ) {
    float u1 = ( rand( ) + 1.0f ) / ( (float)RAND_MAX + 2.0f );
    float u2 = ( rand( ) + 1.0f ) / ( (float)RAND_MAX + 2.0f );
    return sigma * sqrtf( -2.0f * logf( u1 ) ) * cosf( 2.0f * (float)M_PI * u2 );
}

static void makeFields( Vec3 magnet, Vec3 moment, float noiseMt, Vec3* fields ) {
    for ( int i = 0; i < SENSOR_COUNT; i++ ) {
        fields[ i ] = magFitDipoleField( sensors[ i ], magnet, moment );
        fields[ i ].x += gaussian( noiseMt );
        fields[ i ].y += gaussian( noiseMt );
        fields[ i ].z += gaussian( noiseMt );
    }
}

static float distance( Vec3 a, Vec3 b ) {
    return sqrtf( ( a.x - b.x ) * ( a.x - b.x ) + ( a.y - b.y ) * ( a.y - b.y ) + ( a.z - b.z ) * ( a.z - b.z ) );
}

// Far out, with perfect readings, the formula is exact up to the finite
// difference across the array - here a magnet 150 mm up is found within 4 mm.
// This is also what pins the sign convention: point - r, not point + r.
void test_far_magnet_found_closely_and_the_sign_is_right( void ) {
    Vec3 fields[ SENSOR_COUNT ];
    Vec3 magnet = { 27.0f, 22.0f, 150.0f };
    Vec3 moment = { 1500.0f, -2000.0f, -4000.0f };
    makeFields( magnet, moment, 0.0f, fields );
    MagFarEstimate e;
    TEST_ASSERT_TRUE( magFarEstimate( sensors, fields, nullptr, SENSOR_COUNT, 0.0f, &e ) );
    TEST_ASSERT_FLOAT_WITHIN( 4.0f, magnet.x, e.position.x );
    TEST_ASSERT_FLOAT_WITHIN( 4.0f, magnet.y, e.position.y );
    TEST_ASSERT_FLOAT_WITHIN( 4.0f, magnet.z, e.position.z );
    TEST_ASSERT_TRUE( e.position.z > 100.0f ); // not mirrored under the board
}

// Over the usable range the answer stays inside its own error bar: a probe's
// magnet (4200 mT*mm^3, along a tilted shaft), all over and around the array,
// 40 to 100 mm out, with the bench noise on top.
void test_error_bar_holds_over_the_far_range( void ) {
    srand( 7 );
    int tried = 0, inside = 0, valid = 0;
    float worstRatio = 0.0f;
    for ( float z = 40.0f; z <= 100.0f; z += 15.0f ) {
        for ( float x = -20.0f; x <= 75.0f; x += 19.0f ) {
            for ( float y = -20.0f; y <= 65.0f; y += 17.0f ) {
                for ( int tilt = 0; tilt < 3; tilt++ ) {
                    float t = tilt * 25.0f * (float)M_PI / 180.0f;
                    Vec3 moment = { 4200.0f * sinf( t ), 0.0f, -4200.0f * cosf( t ) };
                    Vec3 magnet = { x, y, z };
                    Vec3 fields[ SENSOR_COUNT ];
                    makeFields( magnet, moment, 0.005f, fields );
                    MagFarEstimate e;
                    magFarEstimate( sensors, fields, nullptr, SENSOR_COUNT, 4200.0f, &e );
                    tried++;
                    if ( !e.valid )
                        continue;
                    valid++;
                    float miss = distance( e.position, magnet );
                    if ( miss < 2.0f * e.sigmaMm )
                        inside++;
                    if ( miss / e.sigmaMm > worstRatio )
                        worstRatio = miss / e.sigmaMm;
                }
            }
        }
    }
    printf( "far-field: %d of %d poses gave an estimate, %d of those within 2 sigma, worst miss %.1f sigma\n", valid, tried, inside, worstRatio );
    TEST_ASSERT_TRUE( valid > tried / 2 );
    TEST_ASSERT_TRUE( inside >= valid * 9 / 10 );
}

// The strength-based range agrees with the distance to within the 26 % that
// the magnet's unknown orientation allows (m/r^3 to 2m/r^3).
void test_range_from_strength( void ) {
    Vec3 fields[ SENSOR_COUNT ];
    Vec3 magnet = { 27.0f, 22.0f, 70.0f };
    Vec3 moment = { 0.0f, 0.0f, -4200.0f };
    makeFields( magnet, moment, 0.0f, fields );
    MagFarEstimate e;
    magFarEstimate( sensors, fields, nullptr, SENSOR_COUNT, 4200.0f, &e );
    TEST_ASSERT_TRUE( e.rangeMm > 0.0f );
    TEST_ASSERT_FLOAT_WITHIN( 0.3f * 70.0f, 70.0f, e.rangeMm );
}

// Close in it says so: a magnet 10 mm over a sensor is not an answer this
// method may give.
void test_too_close_is_refused( void ) {
    Vec3 fields[ SENSOR_COUNT ];
    makeFields( { 16.0f, 44.0f, 10.0f }, { 0, 0, -4200.0f }, 0.0f, fields );
    MagFarEstimate e;
    TEST_ASSERT_FALSE( magFarEstimate( sensors, fields, nullptr, SENSOR_COUNT, 0.0f, &e ) );
}

// The same estimate with a dead sensor left out.
void test_dead_sensor_left_out( void ) {
    Vec3 fields[ SENSOR_COUNT ];
    Vec3 magnet = { 27.0f, 22.0f, 120.0f };
    makeFields( magnet, { 0, 0, -4200.0f }, 0.0f, fields );
    fields[ 3 ] = { 99.0f, 99.0f, 99.0f }; // garbage that must be ignored
    bool use[ SENSOR_COUNT ] = { true, true, true, false, true, true, true, true };
    MagFarEstimate e;
    TEST_ASSERT_TRUE( magFarEstimate( sensors, fields, use, SENSOR_COUNT, 0.0f, &e ) );
    TEST_ASSERT_FLOAT_WITHIN( 6.0f, magnet.z, e.position.z );
}

int main( int argc, char** argv ) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN( );
    RUN_TEST( test_far_magnet_found_closely_and_the_sign_is_right );
    RUN_TEST( test_error_bar_holds_over_the_far_range );
    RUN_TEST( test_range_from_strength );
    RUN_TEST( test_too_close_is_refused );
    RUN_TEST( test_dead_sensor_left_out );
    return UNITY_END( );
}
