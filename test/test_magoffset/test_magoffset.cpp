// The per-sensor offset filter: a step in the reading is absorbed on the
// filter's own time, a held filter does not move, and its doubt grows while
// it is held.
#include <unity.h>
#include <math.h>
#include "MagOffsetFilter.h"

static const float NOISE = 0.012f;  // a TMAG5273's noise a frame, mT
static const float WALK = 0.00025f; // its zero's random walk, mT per sqrt(s) (ASSUMPTION: the datasheet's 3-10 uT/degC over a bench's slow drift)
static const float DT = 0.01f;      // 100 Hz frames

static void feed( MagOffsetFilter* f, Vec3 reading, int frames ) {
    for ( int k = 0; k < frames; k++ ) {
        magOffsetPredict( f, DT );
        magOffsetUpdate( f, reading, { NOISE, NOISE, NOISE } );
    }
}

void test_a_step_is_absorbed_on_the_filters_own_time( ) {
    MagOffsetFilter f;
    magOffsetInit( &f, { 0, 0, 0 }, 0.0005f, WALK ); // at its stationary doubt
    feed( &f, { 0.1f, 0, 0 }, 500 );                // the reading steps 0.1 mT and stays
    float at5s = f.offset.x;
    feed( &f, { 0.1f, 0, 0 }, 2500 );
    TEST_ASSERT_TRUE_MESSAGE( at5s > 0.05f && at5s < 0.08f, "about one time constant (5 s) in at 5 s" );
    TEST_ASSERT_FLOAT_WITHIN( 0.005f, 0.1f, f.offset.x ); // within 5 % at 30 s
    TEST_ASSERT_FLOAT_WITHIN( 1e-6f, 0.0f, f.offset.y );
    TEST_ASSERT_FLOAT_WITHIN( 1e-6f, 0.0f, f.offset.z );
}

void test_a_held_filter_does_not_move( ) {
    MagOffsetFilter f;
    magOffsetInit( &f, { 0.01f, -0.02f, 0.03f }, 0.0005f, WALK );
    for ( int k = 0; k < 1000; k++ ) magOffsetPredict( &f, DT ); // held: no update
    TEST_ASSERT_EQUAL_FLOAT( 0.01f, f.offset.x );
    TEST_ASSERT_EQUAL_FLOAT( -0.02f, f.offset.y );
    TEST_ASSERT_EQUAL_FLOAT( 0.03f, f.offset.z );
}

void test_doubt_grows_while_held( ) {
    MagOffsetFilter f;
    magOffsetInit( &f, { 0, 0, 0 }, 0.0005f, WALK );
    for ( int k = 0; k < 1000; k++ ) magOffsetPredict( &f, DT ); // 10 s
    float expected = 0.0005f * 0.0005f + WALK * WALK * 10.0f;   // sigma^2 + walk^2 * t
    TEST_ASSERT_FLOAT_WITHIN( 1e-9f, expected, f.variance.x );
    TEST_ASSERT_FLOAT_WITHIN( 1e-9f, expected, f.variance.z );
}

int main( ) {
    UNITY_BEGIN( );
    RUN_TEST( test_a_step_is_absorbed_on_the_filters_own_time );
    RUN_TEST( test_a_held_filter_does_not_move );
    RUN_TEST( test_doubt_grows_while_held );
    return UNITY_END( );
}
