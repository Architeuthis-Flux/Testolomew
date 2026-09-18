// SPDX-License-Identifier: MIT
// Host tests for the two-magnet probe fit: pio test -e native
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unity.h>

#include "MagProbeFit.h"

#define SENSOR_COUNT 8
#define SPACING_MM 25.0f
#define TIP_STRENGTH 600.0f   // a 2 x 2 mm disc
#define BACK_STRENGTH 4500.0f // a 5 x 2.5 mm one

// The bench array as calibrated (MagArrayConfig.h).
static const Vec3 sensors[ SENSOR_COUNT ] = { { 0.10f, 44.29f, 0 }, { 16.75f, 44.04f, 0 }, { 38.78f, 43.95f, 0 }, { 54.18f, 44.72f, 0 }, { 0, 0, 0 }, { 15.60f, 0.19f, 0 }, { 37.37f, 0.42f, 0 }, { 53.40f, 0, 0 } };

void setUp( void ) {}
void tearDown( void ) {}

static float noise( float sigma ) { // roughly Gaussian: the sum of four uniforms
    float sum = 0.0f;
    for ( int k = 0; k < 4; k++ ) {
        sum += ( rand( ) % 2001 - 1000 ) / 1000.0f;
    }
    return sigma * sum * 0.866f;
}

static Vec3 axisAt( float tiltDeg, float azimuthDeg ) {
    float t = tiltDeg * (float)M_PI / 180.0f, a = azimuthDeg * (float)M_PI / 180.0f;
    Vec3 axis = { sinf( t ) * cosf( a ), sinf( t ) * sinf( a ), cosf( t ) };
    return axis;
}

static MagProbeShape shapeOf( float tipStrength, float backStrength, bool tipAcross = false, bool backAcross = false, float backTurnDeg = 0.0f ) {
    MagProbeShape shape = { SPACING_MM, tipStrength, backStrength, tipAcross ? 90.0f : 0.0f, backAcross ? 90.0f : 0.0f, backTurnDeg };
    return shape;
}

static void readings( Vec3 tip, Vec3 axis, float tipStrength, float backStrength, float sigma, Vec3* fields, bool tipAcross = false, bool backAcross = false, float rollDeg = 0.0f, float backTurnDeg = 0.0f ) {
    MagProbeShape truth = shapeOf( tipStrength, backStrength, tipAcross, backAcross, backTurnDeg );
    for ( int i = 0; i < SENSOR_COUNT; i++ ) {
        fields[ i ] = magProbeField( sensors[ i ], tip, axis, rollDeg, &truth );
        fields[ i ].x += noise( sigma );
        fields[ i ].y += noise( sigma );
        fields[ i ].z += noise( sigma );
    }
}

static void test_forward_model_is_two_dipoles( void ) {
    Vec3 tip = { 20, 15, 12 }, axis = axisAt( 20, 40 );
    Vec3 back = { tip.x + SPACING_MM * axis.x, tip.y + SPACING_MM * axis.y, tip.z + SPACING_MM * axis.z };
    Vec3 a = magFitDipoleField( sensors[ 2 ], tip, { TIP_STRENGTH * axis.x, TIP_STRENGTH * axis.y, TIP_STRENGTH * axis.z } );
    Vec3 b = magFitDipoleField( sensors[ 2 ], back, { BACK_STRENGTH * axis.x, BACK_STRENGTH * axis.y, BACK_STRENGTH * axis.z } );
    MagProbeShape shape = shapeOf( TIP_STRENGTH, BACK_STRENGTH );
    Vec3 both = magProbeField( sensors[ 2 ], tip, axis, 0.0f, &shape );
    TEST_ASSERT_FLOAT_WITHIN( 1e-6f, a.x + b.x, both.x );
    TEST_ASSERT_FLOAT_WITHIN( 1e-6f, a.z + b.z, both.z );
}

// No noise, no starting guess, strengths not given: the pose and both strengths come back.
static void test_cold_start_recovers_pose_and_strengths( void ) {
    int cases = 0, found = 0;
    for ( float tilt = 0.0f; tilt <= 40.0f; tilt += 20.0f ) {
        for ( float y = 10.0f; y <= 34.0f; y += 12.0f ) {
            for ( float x = 0.0f; x <= 54.0f; x += 9.0f ) {
                Vec3 tip = { x, y, 14.0f }, axis = axisAt( tilt, 3.0f * x + y ), fields[ SENSOR_COUNT ];
                readings( tip, axis, TIP_STRENGTH, BACK_STRENGTH, 0.0f, fields );
                MagProbeShape shape = shapeOf( 0.0f, 0.0f );
                MagProbeFitResult r = { };
                bool ok = magProbeFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.05f, &shape, false, &r );
                float miss = sqrtf( ( r.tipMagnet.x - x ) * ( r.tipMagnet.x - x ) + ( r.tipMagnet.y - y ) * ( r.tipMagnet.y - y ) + ( r.tipMagnet.z - 14.0f ) * ( r.tipMagnet.z - 14.0f ) );
                cases++;
                if ( ok && miss < 0.1f && fabsf( r.tipStrength - TIP_STRENGTH ) < 10.0f && fabsf( r.backStrength - BACK_STRENGTH ) < 25.0f ) {
                    found++;
                }
            }
        }
    }
    printf( "  cold starts that found the probe exactly: %d of %d\n", found, cases );
    TEST_ASSERT_EQUAL_INT( cases, found );
}

// The point of the second magnet. One strong magnet 25 mm up the shaft puts the
// tip at (25 mm) x (the error in the probe's angle); with the small magnet at
// the tip as well, and both strengths known, the tip error must at least halve.
static void test_two_magnets_beat_one_for_the_tip( void ) {
    srand( 21 );
    double oneSq = 0, twoSq = 0;
    int cases = 0;
    for ( float y = 10.0f; y <= 34.0f; y += 8.0f ) {
        for ( float x = -8.0f; x <= 62.0f; x += 7.0f ) {
            Vec3 tip = { x, y, 14.0f }, axis = axisAt( 15.0f, 5.0f * x ), both[ SENSOR_COUNT ], strongOnly[ SENSOR_COUNT ];
            srand( cases + 100 );
            readings( tip, axis, TIP_STRENGTH, BACK_STRENGTH, 0.004f, both );
            srand( cases + 100 );
            readings( tip, axis, 0.0f, BACK_STRENGTH, 0.004f, strongOnly );

            MagFitResult one = { };
            magFitSolveKnownStrength( sensors, strongOnly, nullptr, SENSOR_COUNT, 0.40f, BACK_STRENGTH, &one );
            float inv = ( one.moment.z < 0.0f ? -1.0f : 1.0f ) / one.strength; // the probe points up, whichever way the magnet faces
            float oneTipX = one.position.x - SPACING_MM * one.moment.x * inv, oneTipY = one.position.y - SPACING_MM * one.moment.y * inv;

            MagProbeShape shape = shapeOf( TIP_STRENGTH, BACK_STRENGTH );
            MagProbeFitResult two = { };
            TEST_ASSERT_TRUE( magProbeFitSolve( sensors, both, nullptr, SENSOR_COUNT, 0.40f, &shape, false, &two ) );

            oneSq += ( oneTipX - x ) * ( oneTipX - x ) + ( oneTipY - y ) * ( oneTipY - y );
            twoSq += ( two.tipMagnet.x - x ) * ( two.tipMagnet.x - x ) + ( two.tipMagnet.y - y ) * ( two.tipMagnet.y - y );
            cases++;
        }
    }
    float oneRms = sqrtf( (float)( oneSq / cases ) ), twoRms = sqrtf( (float)( twoSq / cases ) );
    printf( "  tip error across the board: one strong magnet %.2f mm, two magnets %.2f mm\n", oneRms, twoRms );
    TEST_ASSERT_TRUE( twoRms < 0.5f * oneRms );
    TEST_ASSERT_TRUE( twoRms < 0.7f );
}

// The strengths told to the fit are never exact (learned, or the magnet warmed up). 30 % out
// on the small one and 5 % on the big one must not cost more than a quarter of a row.
static void test_wrong_strengths_are_forgiven( void ) {
    double sq = 0;
    int cases = 0;
    for ( float y = 10.0f; y <= 34.0f; y += 8.0f ) {
        for ( float x = -8.0f; x <= 62.0f; x += 7.0f ) {
            Vec3 tip = { x, y, 14.0f }, axis = axisAt( 15.0f, 5.0f * x ), fields[ SENSOR_COUNT ];
            srand( cases + 300 );
            readings( tip, axis, TIP_STRENGTH, BACK_STRENGTH, 0.004f, fields );
            MagProbeShape shape = shapeOf( 1.3f * TIP_STRENGTH, 0.95f * BACK_STRENGTH );
            MagProbeFitResult r = { };
            TEST_ASSERT_TRUE( magProbeFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.40f, &shape, false, &r ) );
            sq += ( r.tipMagnet.x - x ) * ( r.tipMagnet.x - x );
            cases++;
        }
    }
    float rms = sqrtf( (float)( sq / cases ) );
    printf( "  tip x error with strengths 30 %% and 5 %% out: %.2f mm\n", rms );
    TEST_ASSERT_TRUE( rms < 0.9f );
}

// The error bar on the tip has to be as honest as the single magnet's.
static void test_tip_error_bar_is_honest( void ) {
    double actual = 0, predicted = 0;
    int inside = 0, cases = 0;
    for ( int rep = 0; rep < 6; rep++ ) {
        for ( float y = 10.0f; y <= 34.0f; y += 8.0f ) {
            for ( float x = 0.0f; x <= 54.0f; x += 9.0f ) {
                Vec3 tip = { x, y, 14.0f }, axis = axisAt( 10.0f, 40.0f * rep ), fields[ SENSOR_COUNT ];
                srand( cases + 500 );
                readings( tip, axis, TIP_STRENGTH, BACK_STRENGTH, 0.008f, fields );
                MagProbeShape shape = shapeOf( TIP_STRENGTH, BACK_STRENGTH );
                MagProbeFitResult r = { };
                magProbeFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.40f, &shape, false, &r );
                float ex = r.tipMagnet.x - x;
                actual += ex * ex;
                predicted += r.sigma.x * r.sigma.x;
                inside += fabsf( ex ) < r.sigma.x ? 1 : 0;
                cases++;
            }
        }
    }
    float ratio = sqrtf( (float)( actual / predicted ) ), coverage = (float)inside / cases;
    printf( "  actual / predicted tip error %.2f, truth inside 1 sigma %.0f %% of the time\n", ratio, coverage * 100.0f );
    TEST_ASSERT_FLOAT_WITHIN( 0.3f, 1.0f, ratio );
    TEST_ASSERT_FLOAT_WITHIN( 0.15f, 0.68f, coverage );
}

// Tracking: each frame starts from the last, as the locator does.
static void test_warm_start_follows_a_moving_probe( void ) {
    srand( 5 );
    MagProbeShape shape = shapeOf( TIP_STRENGTH, BACK_STRENGTH );
    MagProbeFitResult r = { };
    float worst = 0.0f;
    int iterations = 0, frames = 0;
    for ( float t = 0.0f; t < 1.0f; t += 0.01f ) {
        Vec3 tip = { 5.0f + 45.0f * t, 22.0f + 10.0f * sinf( 9.0f * t ), 14.0f + 6.0f * t }, axis = axisAt( 25.0f * sinf( 6.0f * t ) + 25.0f, 300.0f * t ), fields[ SENSOR_COUNT ];
        readings( tip, axis, TIP_STRENGTH, BACK_STRENGTH, 0.004f, fields );
        TEST_ASSERT_TRUE( magProbeFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.40f, &shape, false, &r ) );
        float miss = sqrtf( ( r.tipMagnet.x - tip.x ) * ( r.tipMagnet.x - tip.x ) + ( r.tipMagnet.y - tip.y ) * ( r.tipMagnet.y - tip.y ) );
        worst = miss > worst ? miss : worst;
        if ( frames > 0 ) {
            iterations += r.iterations;
        }
        frames++;
    }
    printf( "  worst tip miss while tracking %.2f mm, %.1f iterations per frame\n", worst, (float)iterations / ( frames - 1 ) );
    TEST_ASSERT_TRUE( worst < 1.5f );
    TEST_ASSERT_TRUE( (float)iterations / ( frames - 1 ) < 6.0f );
}

// Magnets stuck to the SIDE of the shaft (a disc taped to a pencil) point
// across it, and the probe's roll becomes part of the pose. Each mounting -
// tip along or across, back along or across - must be found from a cold start
// with the strengths free, and the wrong mounting must fit visibly worse, so
// that the locator can learn which it is.
static void test_across_magnets_are_found_and_told_apart( void ) {
    for ( int mounting = 1; mounting < 4; mounting++ ) {
        bool tipAcross = mounting & 1, backAcross = mounting & 2;
        int cases = 0, found = 0, wrongMountingWorse = 0;
        for ( float tilt = 0.0f; tilt <= 30.0f; tilt += 15.0f ) {
            for ( float roll = 0.0f; roll < 360.0f; roll += 90.0f ) {
                for ( float x = 9.0f; x <= 45.0f; x += 18.0f ) {
                    Vec3 tip = { x, 22.0f, 14.0f }, axis = axisAt( tilt, 7.0f * x + roll ), fields[ SENSOR_COUNT ];
                    readings( tip, axis, TIP_STRENGTH, BACK_STRENGTH, 0.0f, fields, tipAcross, backAcross, roll + 20.0f );
                    MagProbeShape shape = shapeOf( 0.0f, 0.0f, tipAcross, backAcross );
                    MagProbeFitResult r = { };
                    bool ok = magProbeFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.05f, &shape, false, &r );
                    float miss = sqrtf( ( r.tipMagnet.x - tip.x ) * ( r.tipMagnet.x - tip.x ) + ( r.tipMagnet.y - tip.y ) * ( r.tipMagnet.y - tip.y ) + ( r.tipMagnet.z - tip.z ) * ( r.tipMagnet.z - tip.z ) );
                    cases++;
                    if ( ok && miss < 0.2f && fabsf( fabsf( r.tipStrength ) - TIP_STRENGTH ) < 15.0f && fabsf( fabsf( r.backStrength ) - BACK_STRENGTH ) < 40.0f ) {
                        found++;
                    }
                    // the along-along model, which is what the first version assumed
                    MagProbeShape wrong = shapeOf( 0.0f, 0.0f, false, false );
                    MagProbeFitResult w = { };
                    magProbeFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 1e6f, &wrong, false, &w );
                    if ( w.residual > 3.0f * r.residual + 0.002f ) {
                        wrongMountingWorse++;
                    }
                }
            }
        }
        printf( "  tip %s, back %s: found exactly %d of %d; the along-along model fitted clearly worse %d of %d\n", tipAcross ? "across" : "along ", backAcross ? "across" : "along ", found, cases, wrongMountingWorse, cases );
        TEST_ASSERT_TRUE( found >= cases - 2 );
        TEST_ASSERT_TRUE( wrongMountingWorse >= cases - 2 );
    }
}

// ...and with noise and known strengths, an across probe locates its tip about as well as an along one.
static void test_across_probe_tip_accuracy( void ) {
    double sq = 0;
    int cases = 0;
    for ( float y = 10.0f; y <= 34.0f; y += 8.0f ) {
        for ( float x = -8.0f; x <= 62.0f; x += 7.0f ) {
            Vec3 tip = { x, y, 14.0f }, axis = axisAt( 15.0f, 5.0f * x ), fields[ SENSOR_COUNT ];
            srand( cases + 700 );
            readings( tip, axis, TIP_STRENGTH, BACK_STRENGTH, 0.004f, fields, true, true, 3.0f * x );
            MagProbeShape shape = shapeOf( TIP_STRENGTH, BACK_STRENGTH, true, true );
            MagProbeFitResult r = { };
            TEST_ASSERT_TRUE( magProbeFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.40f, &shape, false, &r ) );
            sq += ( r.tipMagnet.x - x ) * ( r.tipMagnet.x - x );
            cases++;
        }
    }
    float rms = sqrtf( (float)( sq / cases ) );
    printf( "  both magnets across the shaft, strengths known: tip x error %.2f mm\n", rms );
    TEST_ASSERT_TRUE( rms < 0.8f );
}

// The probe as actually built on the bench: two discs stuck to the side of
// the shaft on different sides (both across, the back one turned 100 degrees
// round), 26 mm apart. With learnShape every frame's fit also searches the
// spacing, both angles and the turn; the middle of many frames must give the
// shape back, from a starting guess that is wrong in every one of them.
static void test_shape_is_learned( void ) {
    const float turn = 100.0f, spacing = 26.0f;
    float learnedSpacing[ 60 ], learnedTip[ 60 ], learnedBack[ 60 ], learnedTurn[ 60 ];
    int n = 0;
    MagProbeFitResult r = { };
    for ( int k = 0; k < 60; k++ ) {
        float t = k / 60.0f;
        Vec3 tip = { 10.0f + 35.0f * t, 22.0f + 8.0f * sinf( 9.0f * t ), 14.0f + 8.0f * fabsf( sinf( 5.0f * t ) ) };
        Vec3 axis = axisAt( 20.0f * fabsf( sinf( 7.0f * t ) ), 200.0f * t ), fields[ SENSOR_COUNT ];
        MagProbeShape truth = { spacing, 3400.0f, 7000.0f, 90.0f, 90.0f, turn };
        srand( k + 900 );
        for ( int i = 0; i < SENSOR_COUNT; i++ ) {
            fields[ i ] = magProbeField( sensors[ i ], tip, axis, 300.0f * t, &truth );
            fields[ i ].x += noise( 0.004f );
            fields[ i ].y += noise( 0.004f );
            fields[ i ].z += noise( 0.004f );
        }
        MagProbeShape guess = { 35.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; // what was typed: 35 mm, and nothing known
        if ( magProbeFitSolve( sensors, fields, nullptr, SENSOR_COUNT, 0.40f, &guess, true, &r ) ) {
            learnedSpacing[ n ] = r.shape.spacingMm;
            learnedTip[ n ] = r.shape.tipAngleDeg;
            learnedBack[ n ] = r.shape.backAngleDeg;
            learnedTurn[ n ] = r.shape.backTurnDeg;
            n++;
        }
    }
    TEST_ASSERT_TRUE( n >= 40 );
    // medians
    for ( int pass = 0; pass < 4; pass++ ) {
        float* list = pass == 0 ? learnedSpacing : ( pass == 1 ? learnedTip : ( pass == 2 ? learnedBack : learnedTurn ) );
        for ( int i = 1; i < n; i++ ) {
            for ( int j = i; j > 0 && list[ j - 1 ] > list[ j ]; j-- ) {
                float tmp = list[ j ];
                list[ j ] = list[ j - 1 ];
                list[ j - 1 ] = tmp;
            }
        }
    }
    printf( "  learned from %d frames: spacing %.1f (true %.0f), tip angle %.0f, back angle %.0f (true 90, 90), turn %.0f (true %.0f)\n", n, learnedSpacing[ n / 2 ], spacing,
            learnedTip[ n / 2 ], learnedBack[ n / 2 ], learnedTurn[ n / 2 ], turn );
    TEST_ASSERT_FLOAT_WITHIN( 2.0f, spacing, learnedSpacing[ n / 2 ] );
    TEST_ASSERT_FLOAT_WITHIN( 10.0f, 90.0f, learnedTip[ n / 2 ] );
    TEST_ASSERT_FLOAT_WITHIN( 10.0f, 90.0f, learnedBack[ n / 2 ] );
    TEST_ASSERT_FLOAT_WITHIN( 12.0f, turn, learnedTurn[ n / 2 ] );
}

int main( int, char** ) {
    UNITY_BEGIN( );
    RUN_TEST( test_forward_model_is_two_dipoles );
    RUN_TEST( test_cold_start_recovers_pose_and_strengths );
    RUN_TEST( test_two_magnets_beat_one_for_the_tip );
    RUN_TEST( test_wrong_strengths_are_forgiven );
    RUN_TEST( test_tip_error_bar_is_honest );
    RUN_TEST( test_warm_start_follows_a_moving_probe );
    RUN_TEST( test_across_magnets_are_found_and_told_apart );
    RUN_TEST( test_across_probe_tip_accuracy );
    RUN_TEST( test_shape_is_learned );
    return UNITY_END( );
}
