// SPDX-License-Identifier: MIT
// Host test of the tip model (src/magfit/TipModel.h): a probe whose magnet
// sits up the shaft and whose fix is biased along the lean, rested in a
// hole and leaned several ways, gives back its model; the scalar tip is the
// model with A = tip I; too-alike leans or no upright sample are refused.
// pio test -e native
#include <math.h>
#include <stdlib.h>
#include <unity.h>

#include "TipModel.h"

void setUp( void ) {}
void tearDown( void ) {}

static float gaussian( float sigma ) {
    float u1 = ( rand( ) + 1.0f ) / ( (float)RAND_MAX + 2.0f );
    float u2 = ( rand( ) + 1.0f ) / ( (float)RAND_MAX + 2.0f );
    return sigma * sqrtf( -2.0f * logf( u1 ) ) * cosf( 2.0f * (float)M_PI * u2 );
}

static Vec3 shaftAt( float leanDeg, float dirDeg ) {
    float l = leanDeg * (float)M_PI / 180.0f, d = dirDeg * (float)M_PI / 180.0f;
    Vec3 s = { sinf( l ) * cosf( d ), sinf( l ) * sinf( d ), cosf( l ) };
    return s;
}

// A sample of a probe with the true model `m` resting at `point`, the fix noisy.
static TipSample sampleOf( const TipModel* m, Vec3 point, Vec3 shaft, int hole, float noiseMm ) {
    TipSample s;
    s.shaft = shaft;
    s.hole = hole;
    s.magnet = { point.x + m->a[ 0 ] * shaft.x + m->a[ 1 ] * shaft.y + gaussian( noiseMm ), point.y + m->a[ 2 ] * shaft.x + m->a[ 3 ] * shaft.y + gaussian( noiseMm ),
                 point.z + m->dz * shaft.z + gaussian( noiseMm ) };
    return s;
}

void test_the_lean_calibration_recovers_the_model( void ) {
    srand( 3 );
    TipModel truth = { true, { 3.0f, 0.4f, -0.3f, 3.6f }, 3.2f }; // the magnet 3 mm up, the fix biased more across than along
    Vec3 hole0 = { 26.4f, 30.0f, 17.5f }, hole1 = { 52.0f, 20.0f, 17.4f };
    TipSample samples[ 12 ];
    int n = 0;
    const float dirs[ 5 ] = { 0.0f, 72.0f, 144.0f, 216.0f, 288.0f };
    for ( int hole = 0; hole < 2; hole++ ) {
        Vec3 p = hole == 0 ? hole0 : hole1;
        samples[ n++ ] = sampleOf( &truth, p, shaftAt( 1.0f, 0.0f ), hole, 0.05f );
        for ( int k = 0; k < 5; k++ )
            samples[ n++ ] = sampleOf( &truth, p, shaftAt( 40.0f, dirs[ k ] ), hole, 0.05f );
    }
    TipModel m;
    TipFitReport report;
    TEST_ASSERT_TRUE( tipModelFit( samples, n, &m, &report ) );
    TEST_ASSERT_NULL( report.refused );
    for ( int k = 0; k < 4; k++ )
        TEST_ASSERT_FLOAT_WITHIN( 0.15f, truth.a[ k ], m.a[ k ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.15f, truth.dz, m.dz );
    TEST_ASSERT_FLOAT_WITHIN( 0.1f, 3.3f, report.tipMm );
    TEST_ASSERT_TRUE( report.rmsMm < 0.12f );
    TEST_ASSERT_EQUAL( 2, report.holes );
    // With the model, a lean lands the point where it rests; the scalar tip alone does not.
    Vec3 leaning = shaftAt( 40.0f, 90.0f );
    TipSample s = sampleOf( &truth, hole0, leaning, 0, 0.0f );
    Vec3 p = tipModelPoint( &m, 0.0f, s.magnet, s.shaft );
    TEST_ASSERT_FLOAT_WITHIN( 0.15f, hole0.x, p.x );
    TEST_ASSERT_FLOAT_WITHIN( 0.15f, hole0.y, p.y );
    TipModel none;
    tipModelClear( &none );
    Vec3 q = tipModelPoint( &none, 0.0f, s.magnet, s.shaft ); // tip 0: the magnet itself, 2 mm off across
    TEST_ASSERT_TRUE( fabsf( q.y - hole0.y ) > 1.5f );
    Vec3 r = tipModelPoint( &none, 3.3f, s.magnet, s.shaft ); // the scalar: closer, the anisotropy left
    TEST_ASSERT_TRUE( fabsf( r.y - hole0.y ) < 1.0f && fabsf( r.y - hole0.y ) > 0.1f );
    // Six samples in one hole are enough - dz least surely: its lever is
    // 1 - cos(40 deg) = 0.23 of the upright-to-lean difference, so 0.05 mm
    // of noise a sample is 0.25 mm of dz (the bench's 200-fix means are
    // quieter). A leans the whole sin(40 deg).
    TEST_ASSERT_TRUE( tipModelFit( samples, 6, &m, &report ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.5f, truth.dz, m.dz );
    for ( int k = 0; k < 4; k++ )
        TEST_ASSERT_FLOAT_WITHIN( 0.2f, truth.a[ k ], m.a[ k ] );
}

void test_the_scalar_tip_is_the_model_with_a_equal_to_tip_i( void ) {
    TipModel m = { true, { 4.0f, 0.0f, 0.0f, 4.0f }, 4.0f };
    TipModel none;
    tipModelClear( &none );
    Vec3 magnet = { 10.0f, 20.0f, 25.0f }, shaft = shaftAt( 35.0f, 120.0f );
    Vec3 a = tipModelPoint( &m, 0.0f, magnet, shaft ), b = tipModelPoint( &none, 4.0f, magnet, shaft );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, b.x, a.x );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, b.y, a.y );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, b.z, a.z );
}

// Samples that cannot pin the lean bias still pin the plain tip - the
// console's Q with three leans one way, as it always did - and the model
// comes back isotropic; too few, or all alike, are refused.
void test_the_plain_tip_when_the_leans_cannot_tell_more( void ) {
    TipModel truth = { true, { 3.0f, 0.0f, 0.0f, 3.0f }, 3.0f };
    Vec3 hole = { 26.4f, 30.0f, 17.5f };
    TipSample samples[ 8 ];
    TipModel m;
    TipFitReport report;
    // Three leans one way (Q's way): the plain tip.
    samples[ 0 ] = sampleOf( &truth, hole, shaftAt( 5.0f, 0.0f ), 0, 0.0f );
    samples[ 1 ] = sampleOf( &truth, hole, shaftAt( 35.0f, 0.0f ), 0, 0.0f );
    samples[ 2 ] = sampleOf( &truth, hole, shaftAt( 55.0f, 0.0f ), 0, 0.0f );
    TEST_ASSERT_TRUE( tipModelFit( samples, 3, &m, &report ) );
    TEST_ASSERT_TRUE( report.scalar );
    TEST_ASSERT_NOT_NULL( report.note );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 3.0f, report.tipMm );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 3.0f, m.dz );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.0f, m.a[ 1 ] );
    // Six leaning the same way (and one upright): A is not pinned across - the plain tip.
    samples[ 0 ] = sampleOf( &truth, hole, shaftAt( 1.0f, 0.0f ), 0, 0.0f );
    for ( int k = 1; k < 6; k++ )
        samples[ k ] = sampleOf( &truth, hole, shaftAt( 30.0f + 3.0f * k, 0.0f ), 0, 0.0f );
    TEST_ASSERT_TRUE( tipModelFit( samples, 6, &m, &report ) );
    TEST_ASSERT_TRUE( report.scalar );
    // Leaning three ways but never upright: dz is not pinned - the plain tip.
    const float dirs[ 3 ] = { 0.0f, 120.0f, 240.0f };
    for ( int k = 0; k < 6; k++ )
        samples[ k ] = sampleOf( &truth, hole, shaftAt( 40.0f, dirs[ k % 3 ] ), 0, 0.0f );
    TEST_ASSERT_TRUE( tipModelFit( samples, 6, &m, &report ) );
    TEST_ASSERT_TRUE( report.scalar );
    TEST_ASSERT_FLOAT_WITHIN( 0.05f, 3.0f, report.tipMm );
    // Too few, or all alike: refused.
    TEST_ASSERT_FALSE( tipModelFit( samples, 2, &m, &report ) );
    TEST_ASSERT_NOT_NULL( report.refused );
    for ( int k = 0; k < 4; k++ )
        samples[ k ] = sampleOf( &truth, hole, shaftAt( 40.0f, 0.0f ), 0, 0.0f );
    TEST_ASSERT_FALSE( tipModelFit( samples, 4, &m, &report ) );
    TEST_ASSERT_NOT_NULL( report.refused );
}

int main( int argc, char** argv ) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN( );
    RUN_TEST( test_the_lean_calibration_recovers_the_model );
    RUN_TEST( test_the_scalar_tip_is_the_model_with_a_equal_to_tip_i );
    RUN_TEST( test_the_plain_tip_when_the_leans_cannot_tell_more );
    return UNITY_END( );
}
