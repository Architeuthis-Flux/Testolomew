// SPDX-License-Identifier: MIT
// Host-side test of the dipole fit: make the field a known magnet would give
// at a 4x2 array, and see that magFitSolve() finds the magnet again - exactly
// with perfect readings, and to within what the noise allows with real ones.
// Run with `pio test -e native`.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unity.h>

#include "MagFit.h"

#define SENSOR_COUNT 8
#define PITCH_MM 20.0f
#define N52_6X3_MOMENT 9800.0f // mT*mm^3

static Vec3 sensors[ SENSOR_COUNT ];

void setUp( void ) {
    for ( int i = 0; i < SENSOR_COUNT; i++ ) {
        sensors[ i ].x = ( i % 4 ) * PITCH_MM;
        sensors[ i ].y = ( i / 4 ) * PITCH_MM;
        sensors[ i ].z = 0.0f;
    }
}

void tearDown( void ) {}

static float noise( float amplitude ) {
    return amplitude * ( 2.0f * rand( ) / (float)RAND_MAX - 1.0f );
}

// Moment of the given strength tilted `tiltDeg` from straight down (-z, the
// way a probe held upright points), leaning toward `headingDeg`.
static Vec3 tiltedMoment( float strength, float tiltDeg, float headingDeg ) {
    float t = tiltDeg * (float)M_PI / 180.0f;
    float h = headingDeg * (float)M_PI / 180.0f;
    Vec3 m = { strength * sinf( t ) * cosf( h ), strength * sinf( t ) * sinf( h ), -strength * cosf( t ) };
    return m;
}

static void makeFields( Vec3 magnet, Vec3 moment, float noiseMt, Vec3* fields ) {
    for ( int i = 0; i < SENSOR_COUNT; i++ ) {
        fields[ i ] = magFitDipoleField( sensors[ i ], magnet, moment );
        fields[ i ].x += noise( noiseMt );
        fields[ i ].y += noise( noiseMt );
        fields[ i ].z += noise( noiseMt );
    }
}

static float distance( Vec3 a, Vec3 b ) {
    return sqrtf( ( a.x - b.x ) * ( a.x - b.x ) + ( a.y - b.y ) * ( a.y - b.y ) + ( a.z - b.z ) * ( a.z - b.z ) );
}

void test_forward_model_on_axis( void ) {
    // On the axis of a dipole the field is 2m/r^3, along the moment.
    Vec3 b = magFitDipoleField( { 0, 0, 0 }, { 0, 0, 10 }, { 0, 0, -N52_6X3_MOMENT } );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, -2.0f * N52_6X3_MOMENT / 1000.0f, b.z );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.0f, b.x );
    // Beside it (the equator) the field is m/r^3, against the moment.
    b = magFitDipoleField( { 10, 0, 0 }, { 0, 0, 0 }, { 0, 0, N52_6X3_MOMENT } );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, -N52_6X3_MOMENT / 1000.0f, b.z );
}

// Gaussian noise at the TMAG5273's datasheet level for 16x averaging in
// low-noise mode: 31 uT RMS on X and Y, 13 uT on Z (22 / 9 uT at 32x).
static float gaussian( float sigma ) {
    float u1 = ( rand( ) + 1.0f ) / ( (float)RAND_MAX + 2.0f );
    float u2 = ( rand( ) + 1.0f ) / ( (float)RAND_MAX + 2.0f );
    return sigma * sqrtf( -2.0f * logf( u1 ) ) * cosf( 2.0f * (float)M_PI * u2 );
}

static void addSensorNoise( Vec3* fields ) {
    for ( int i = 0; i < SENSOR_COUNT; i++ ) {
        fields[ i ].x += gaussian( 0.031f );
        fields[ i ].y += gaussian( 0.031f );
        fields[ i ].z += gaussian( 0.013f );
    }
}

void test_cold_start_is_exact_without_noise( void ) {
    // The solver itself: from a cold start, with perfect readings, it must land
    // on the magnet everywhere over and around the array, at any height/tilt.
    int cases = 0;
    float worst = 0.0f;
    Vec3 fields[ SENSOR_COUNT ];
    for ( float z = 3.0f; z <= 45.0f; z += 7.0f ) {
        for ( float y = -10.0f; y <= 30.0f; y += 5.0f ) {
            for ( float x = -10.0f; x <= 70.0f; x += 5.0f ) {
                for ( float tilt = 0.0f; tilt <= 60.0f; tilt += 20.0f ) {
                    Vec3 magnet = { x, y, z };
                    makeFields( magnet, tiltedMoment( N52_6X3_MOMENT, tilt, 35.0f ), 0.0f, fields );

                    MagFitResult result;
                    result.valid = false;
                    bool ok = magFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.25f, &result );
                    float error = distance( result.position, magnet );
                    if ( !ok || error > 0.05f ) {
                        printf( "  miss: magnet (%.1f %.1f %.1f) tilt %.0f -> (%.2f %.2f %.2f) valid %d\n",
                                x, y, z, tilt, result.position.x, result.position.y, result.position.z, ok );
                    }
                    TEST_ASSERT_TRUE( ok );
                    TEST_ASSERT_LESS_THAN_FLOAT( 0.05f, error );
                    if ( error > worst ) {
                        worst = error;
                    }
                    cases++;
                }
            }
        }
    }
    printf( "  %d cold starts, worst position error %.4f mm\n", cases, worst );
}

void test_accuracy_with_sensor_noise( void ) {
    // What one frame is worth with real sensor noise, magnet over the array's
    // footprint. Noise is the floor, not the whole error budget (sensor
    // placement, gain error and the dipole approximation come on top), but it
    // sets how high above the board a small magnet can still be tracked.
    srand( 1 );
    Vec3 fields[ SENSOR_COUNT ];
    printf( "  6x3 mm N52, 20 mm pitch, single frame:\n   height  rms error  worst\n" );
    for ( float z = 5.0f; z <= 30.0f; z += 5.0f ) {
        float sumSq = 0.0f;
        float worst = 0.0f;
        int cases = 0;
        for ( int repeat = 0; repeat < 5; repeat++ ) {
            for ( float y = 0.0f; y <= 20.0f; y += 5.0f ) {
                for ( float x = 0.0f; x <= 60.0f; x += 5.0f ) {
                    Vec3 magnet = { x, y, z };
                    makeFields( magnet, tiltedMoment( N52_6X3_MOMENT, 20.0f, 35.0f ), 0.0f, fields );
                    addSensorNoise( fields );

                    MagFitResult result;
                    result.valid = false;
                    magFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.25f, &result );
                    float error = distance( result.position, magnet );
                    sumSq += error * error;
                    if ( error > worst ) {
                        worst = error;
                    }
                    cases++;
                }
            }
        }
        float rms = sqrtf( sumSq / cases );
        printf( "   %3.0f mm   %6.3f mm  %6.3f mm\n", z, rms, worst );
        if ( z <= 15.0f ) {
            TEST_ASSERT_LESS_THAN_FLOAT( 0.5f, rms );
            TEST_ASSERT_LESS_THAN_FLOAT( 1.5f, worst );
        }
    }
}

void test_moment_recovered( void ) {
    srand( 2 );
    Vec3 fields[ SENSOR_COUNT ];
    Vec3 magnet = { 27.0f, 8.0f, 12.0f };
    Vec3 moment = tiltedMoment( N52_6X3_MOMENT, 25.0f, 120.0f );
    makeFields( magnet, moment, 0.02f, fields );

    MagFitResult result;
    result.valid = false;
    TEST_ASSERT_TRUE( magFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.25f, &result ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.03f * N52_6X3_MOMENT, N52_6X3_MOMENT, result.strength );
    TEST_ASSERT_FLOAT_WITHIN( 0.03f * N52_6X3_MOMENT, moment.x, result.moment.x );
    TEST_ASSERT_FLOAT_WITHIN( 0.03f * N52_6X3_MOMENT, moment.y, result.moment.y );
    TEST_ASSERT_FLOAT_WITHIN( 0.03f * N52_6X3_MOMENT, moment.z, result.moment.z );
}

void test_warm_start_tracks_a_moving_magnet( void ) {
    srand( 3 );
    Vec3 fields[ SENSOR_COUNT ];
    MagFitResult result;
    result.valid = false;
    int totalIterations = 0;
    int steps = 0;
    for ( float t = 0.0f; t < 1.0f; t += 0.01f ) {
        Vec3 magnet = { 60.0f * t, 10.0f + 8.0f * sinf( 6.28f * t ), 10.0f + 5.0f * cosf( 6.28f * t ) };
        makeFields( magnet, tiltedMoment( N52_6X3_MOMENT, 15.0f, 360.0f * t ), 0.05f, fields );
        TEST_ASSERT_TRUE( magFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.25f, &result ) );
        TEST_ASSERT_LESS_THAN_FLOAT( 1.0f, distance( result.position, magnet ) );
        totalIterations += result.iterations;
        steps++;
    }
    printf( "  tracking: %.1f iterations per fix\n", (float)totalIterations / steps );
}

void test_dead_sensors_are_left_out( void ) {
    srand( 4 );
    Vec3 fields[ SENSOR_COUNT ];
    bool use[ SENSOR_COUNT ] = { true, true, false, true, true, false, true, true };
    Vec3 magnet = { 22.0f, 12.0f, 10.0f };
    makeFields( magnet, tiltedMoment( N52_6X3_MOMENT, 10.0f, 0.0f ), 0.05f, fields );
    fields[ 2 ] = { 99.0f, -99.0f, 99.0f }; // garbage from a dead sensor
    fields[ 5 ] = { 0.0f, 0.0f, 0.0f };

    MagFitResult result;
    result.valid = false;
    TEST_ASSERT_TRUE( magFitSolve( sensors, fields, use, SENSOR_COUNT, 0.25f, &result ) );
    TEST_ASSERT_LESS_THAN_FLOAT( 1.0f, distance( result.position, magnet ) );
}

void test_known_strength_steadies_the_height( void ) {
    // With the strength free, "a bit higher and a bit stronger" fits nearly as
    // well as the truth, and the height wanders. Telling the fit the strength
    // takes that freedom away: the height scatter should drop by a third or
    // more, and x/y must not get worse.
    srand( 6 );
    Vec3 fields[ SENSOR_COUNT ];
    float freeZ = 0, knownZ = 0, freeXy = 0, knownXy = 0;
    int cases = 0;
    for ( int repeat = 0; repeat < 20; repeat++ ) {
        for ( float y = 0.0f; y <= 20.0f; y += 10.0f ) {
            for ( float x = 0.0f; x <= 60.0f; x += 10.0f ) {
                Vec3 magnet = { x, y, 25.0f };
                makeFields( magnet, tiltedMoment( N52_6X3_MOMENT, 15.0f, 35.0f ), 0.0f, fields );
                addSensorNoise( fields );

                MagFitResult a, b;
                a.valid = b.valid = false;
                magFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.25f, &a );
                magFitSolveKnownStrength( sensors, fields, nullptr, SENSOR_COUNT, 0.25f, N52_6X3_MOMENT, &b );
                freeZ += ( a.position.z - 25.0f ) * ( a.position.z - 25.0f );
                knownZ += ( b.position.z - 25.0f ) * ( b.position.z - 25.0f );
                freeXy += ( a.position.x - x ) * ( a.position.x - x ) + ( a.position.y - y ) * ( a.position.y - y );
                knownXy += ( b.position.x - x ) * ( b.position.x - x ) + ( b.position.y - y ) * ( b.position.y - y );
                TEST_ASSERT_FLOAT_WITHIN( 0.03f * N52_6X3_MOMENT, N52_6X3_MOMENT, b.strength );
                cases++;
            }
        }
    }
    printf( "  at 25 mm: height rms %.3f mm free, %.3f mm with the strength known; xy %.3f -> %.3f mm\n",
            sqrtf( freeZ / cases ), sqrtf( knownZ / cases ), sqrtf( freeXy / cases ), sqrtf( knownXy / cases ) );
    TEST_ASSERT_LESS_THAN_FLOAT( 0.67f * sqrtf( freeZ / cases ), sqrtf( knownZ / cases ) );
    TEST_ASSERT_LESS_THAN_FLOAT( 1.05f * sqrtf( freeXy / cases ), sqrtf( knownXy / cases ) );
}

void test_error_bar_is_honest( void ) {
    // The fit reports its own 1-sigma position error. With nothing wrong but
    // sensor noise that number has to match the error actually made: the rms
    // ratio near 1, and the truth inside one sigma about 68 % of the time.
    srand( 7 );
    Vec3 fields[ SENSOR_COUNT ];
    for ( int known = 0; known < 2; known++ ) {
        double actual = 0, predicted = 0;
        int cases = 0, inside = 0;
        for ( int repeat = 0; repeat < 30; repeat++ ) {
            for ( float y = 0.0f; y <= 20.0f; y += 10.0f ) {
                for ( float x = 0.0f; x <= 60.0f; x += 10.0f ) {
                    Vec3 magnet = { x, y, 20.0f };
                    makeFields( magnet, tiltedMoment( N52_6X3_MOMENT, 20.0f, 35.0f ), 0.0f, fields );
                    addSensorNoise( fields );
                    MagFitResult r;
                    r.valid = false;
                    if ( known ) {
                        magFitSolveKnownStrength( sensors, fields, nullptr, SENSOR_COUNT, 0.25f, N52_6X3_MOMENT, &r );
                    } else {
                        magFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.25f, &r );
                    }
                    float ex = r.position.x - x, ey = r.position.y - y, ez = r.position.z - 20.0f;
                    actual += ex * ex + ey * ey + ez * ez;
                    predicted += r.sigma.x * r.sigma.x + r.sigma.y * r.sigma.y + r.sigma.z * r.sigma.z;
                    inside += fabsf( ex ) < r.sigma.x ? 1 : 0;
                    cases++;
                }
            }
        }
        float ratio = sqrtf( (float)( actual / predicted ) );
        float coverage = (float)inside / cases;
        printf( "  %s: actual / predicted error %.2f, truth inside 1 sigma %.0f %% of the time\n", known ? "known strength" : "free fit", ratio, coverage * 100.0f );
        TEST_ASSERT_FLOAT_WITHIN( 0.25f, 1.0f, ratio );
        TEST_ASSERT_FLOAT_WITHIN( 0.12f, 0.68f, coverage );
    }
}

void test_noise_alone_is_not_a_magnet( void ) {
    srand( 5 );
    Vec3 fields[ SENSOR_COUNT ];
    int falsePositives = 0;
    for ( int trial = 0; trial < 50; trial++ ) {
        for ( int i = 0; i < SENSOR_COUNT; i++ ) {
            fields[ i ] = { noise( 0.05f ), noise( 0.05f ), noise( 0.05f ) };
        }
        MagFitResult result;
        result.valid = false;
        if ( magFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.25f, &result ) ) {
            falsePositives++;
        }
    }
    TEST_ASSERT_EQUAL_INT( 0, falsePositives );
}

// A weak magnet low over one corner: two sensors read it plainly, the rest are
// down near the noise. The fit uses all of them anyway ("almost nothing here"
// is information), so it must still land within half a breadboard row. This is
// why MagLocator attempts a fix on faint readings instead of waiting for three
// strong ones.
void test_weak_magnet_seen_plainly_by_two_sensors( void ) {
    srand( 9 );
    const float weak = 150.0f; // 1/65 of the N52 disc: a 1.5 mm cube
    int cases = 0, inRow = 0;
    for ( float x = 4.0f; x <= 16.0f; x += 4.0f ) {
        for ( int trial = 0; trial < 10; trial++ ) {
            Vec3 magnet = { x, 4.0f, 10.0f };
            Vec3 moment = { 0.26f * weak, 0.15f * weak, -0.95f * weak };
            Vec3 fields[ SENSOR_COUNT ];
            int plainly = 0;
            for ( int i = 0; i < SENSOR_COUNT; i++ ) {
                fields[ i ] = magFitDipoleField( sensors[ i ], magnet, moment );
                float b = sqrtf( fields[ i ].x * fields[ i ].x + fields[ i ].y * fields[ i ].y + fields[ i ].z * fields[ i ].z );
                plainly += b > 0.04f ? 1 : 0;
                fields[ i ].x += noise( 0.004f ); // smoothed noise, as MagLocator feeds the fit
                fields[ i ].y += noise( 0.004f );
                fields[ i ].z += noise( 0.004f );
            }
            TEST_ASSERT_TRUE( plainly <= 2 );
            MagFitResult r;
            r.valid = false;
            TEST_ASSERT_TRUE( magFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.40f, &r ) );
            inRow += fabsf( r.position.x - x ) < 1.27f ? 1 : 0;
            cases++;
        }
    }
    printf( "  weak magnet, at most two sensors above 0.04 mT: %d of %d fixes within half a row\n", inRow, cases );
    TEST_ASSERT_TRUE( inRow >= cases * 9 / 10 );
}

int main( void ) {
    UNITY_BEGIN( );
    RUN_TEST( test_forward_model_on_axis );
    RUN_TEST( test_cold_start_is_exact_without_noise );
    RUN_TEST( test_accuracy_with_sensor_noise );
    RUN_TEST( test_moment_recovered );
    RUN_TEST( test_warm_start_tracks_a_moving_magnet );
    RUN_TEST( test_dead_sensors_are_left_out );
    RUN_TEST( test_known_strength_steadies_the_height );
    RUN_TEST( test_error_bar_is_honest );
    RUN_TEST( test_noise_alone_is_not_a_magnet );
    RUN_TEST( test_weak_magnet_seen_plainly_by_two_sensors );
    return UNITY_END( );
}
