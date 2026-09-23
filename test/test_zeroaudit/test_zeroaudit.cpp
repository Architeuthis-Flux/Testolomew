// The zero audit's solve: a known zero and gain come back out; no spread, no solve.
#include <unity.h>
#include <math.h>
#include "ZeroAudit.h"

static void feed( ZeroAudit& a, int i, Vec3 z, float g, int n, bool spread ) {
    for ( int k = 0; k < n; k++ ) {
        float s = spread ? 0.5f + 1.5f * ( k % 7 ) / 6.0f : 1.0f; // the prediction's size varies 0.5..2x, or not at all
        Vec3 pred = { 0.20f * s, -0.10f * s, 0.30f * s };
        Vec3 read = { z.x + g * pred.x, z.y + g * pred.y, z.z + g * pred.z };
        a.addSample( i, pred, read, 1000 + 10 * k );
    }
}

void test_zero_and_gain_come_back( ) {
    ZeroAudit a;
    feed( a, 3, { 0.004f, -0.002f, 0.010f }, 1.05f, 50, true );
    TEST_ASSERT_TRUE( a.solve( 3, 40, 0.3f ) );
    const ZeroAuditSensor& s = a.sensor( 3 );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.004f, s.zero.x );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, -0.002f, s.zero.y );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.010f, s.zero.z );
    TEST_ASSERT_FLOAT_WITHIN( 1e-3f, 1.05f, s.gain );
    TEST_ASSERT_TRUE( s.rmsAfter < 1e-4f );
    TEST_ASSERT_TRUE( s.rmsBefore > 0.005f );
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
    RUN_TEST( test_no_spread_no_solve );
    RUN_TEST( test_too_few_samples_no_solve );
    RUN_TEST( test_reset_keeps_the_applied_count );
    return UNITY_END( );
}
