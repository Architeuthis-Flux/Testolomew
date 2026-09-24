// The zero audit's solve: a known zero and gain come back out; no spread, no solve.
#include <unity.h>
#include <math.h>
#include "ZeroAudit.h"

// The readings of sensor i: its zero z, plus each axis's own gain on the
// prediction; the prediction's size varies 0.5..2x over the samples (spread),
// or not at all. With `flatZ` the z prediction is the same every sample: no
// spread on that axis, so its gain cannot be told from its zero.
static void feed( ZeroAudit& a, int i, Vec3 z, Vec3 g, int n, bool spread, bool flatZ = false ) {
    for ( int k = 0; k < n; k++ ) {
        float s = spread ? 0.5f + 1.5f * ( k % 7 ) / 6.0f : 1.0f;
        Vec3 pred = { 0.20f * s, -0.10f * s, flatZ ? 0.30f : 0.30f * s };
        Vec3 read = { z.x + g.x * pred.x, z.y + g.y * pred.y, z.z + g.z * pred.z };
        a.addSample( i, pred, read, 1000 + 10 * k );
    }
}
static void feed( ZeroAudit& a, int i, Vec3 z, float g, int n, bool spread ) {
    feed( a, i, z, { g, g, g }, n, spread );
}

// Each axis's gain is its own (the TMAG5273's sensitivity spread is per axis:
// X-Y mismatch 0.5 %, Z up to 15 % with drift), solved with the zero.
void test_zero_and_gain_come_back( ) {
    ZeroAudit a;
    feed( a, 3, { 0.004f, -0.002f, 0.010f }, { 1.05f, 0.97f, 1.12f }, 50, true );
    TEST_ASSERT_TRUE( a.solve( 3, 40, 0.3f ) );
    const ZeroAuditSensor& s = a.sensor( 3 );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.004f, s.zero.x );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, -0.002f, s.zero.y );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.010f, s.zero.z );
    TEST_ASSERT_FLOAT_WITHIN( 1e-3f, 1.05f, s.gain.x );
    TEST_ASSERT_FLOAT_WITHIN( 1e-3f, 0.97f, s.gain.y );
    TEST_ASSERT_FLOAT_WITHIN( 1e-3f, 1.12f, s.gain.z );
    TEST_ASSERT_TRUE( s.rmsAfter < 5e-4f ); // float sums centred on the first sample: a residual floor of ~0.0002 mT on 0.3 mT readings, fifty times under a TMAG5273's noise
    TEST_ASSERT_TRUE( s.rmsBefore > 0.005f );
}

// An axis whose prediction never varies cannot tell its gain from its zero:
// its gain stays 1 and the offset takes the whole difference.
void test_an_axis_without_spread_keeps_its_gain( ) {
    ZeroAudit a;
    feed( a, 4, { 0.004f, -0.002f, 0.010f }, { 1.05f, 0.97f, 1.12f }, 50, true, true );
    TEST_ASSERT_TRUE( a.solve( 4, 40, 0.2f ) ); // the shared spread (all axes) is 0.27 with the largest axis flat: under the locator's 0.3, which is that bar doing its job
    const ZeroAuditSensor& s = a.sensor( 4 );
    TEST_ASSERT_FLOAT_WITHIN( 1e-3f, 1.05f, s.gain.x );
    TEST_ASSERT_FLOAT_WITHIN( 1e-3f, 1.0f, s.gain.z );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.010f + 0.12f * 0.30f, s.zero.z ); // the gain's share on the flat prediction lands in the zero
}

void test_no_spread_no_solve( ) {
    ZeroAudit a;
    feed( a, 8, { 0.05f, 0, 0 }, 1.0f, 50, false );
    TEST_ASSERT_FALSE( a.solve( 8, 40, 0.3f ) );
    TEST_ASSERT_TRUE( a.sensor( 8 ).spread < 0.01f );
}

void test_too_few_samples_no_solve( ) {
    ZeroAudit a;
    feed( a, 1, { 0.01f, 0, 0 }, 1.0f, 10, true );
    TEST_ASSERT_FALSE( a.solve( 1, 40, 0.3f ) );
}

void test_reset_keeps_the_applied_count( ) {
    ZeroAudit a;
    feed( a, 2, { 0.01f, 0, 0 }, 1.0f, 50, true );
    a.sensor( 2 ).applied = 3;
    a.reset( 2 );
    TEST_ASSERT_EQUAL( 0, a.sensor( 2 ).n );
    TEST_ASSERT_EQUAL( 3, a.sensor( 2 ).applied );
}

int main( ) {
    UNITY_BEGIN( );
    RUN_TEST( test_zero_and_gain_come_back );
    RUN_TEST( test_an_axis_without_spread_keeps_its_gain );
    RUN_TEST( test_no_spread_no_solve );
    RUN_TEST( test_too_few_samples_no_solve );
    RUN_TEST( test_reset_keeps_the_applied_count );
    return UNITY_END( );
}
