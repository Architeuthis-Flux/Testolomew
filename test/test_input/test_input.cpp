// SPDX-License-Identifier: MIT
// Host-side test of the button classifier (src/ui/ButtonTracker.h: a tap
// is a click, a hold is not, a repeat is not, the press is immediate and
// bounce is nothing) and of the stick as a four-way (src/ui/StickDpad.h:
// hysteresis, one axis at a time, the release overshoot, the click's tilt).
// Run with `pio test -e native`.
#include <stdio.h>
#include <unity.h>

#include "ButtonTracker.h"
#include "StickDpad.h"

static ButtonTracker b;
static int counts[ IN_EVENT_KIND_COUNT ];
static uint32_t now;

void setUp( void ) {
    now = 1000;
    for ( int k = 0; k < IN_EVENT_KIND_COUNT; k++ )
        counts[ k ] = 0;
}
void tearDown( void ) {}

// Feed a level for `ms` in 5 ms samples, counting the events.
static void feed( bool down, uint32_t ms ) {
    for ( uint32_t t = 0; t < ms; t += 5 ) {
        InputEventKind out[ 2 ];
        int n = buttonFeed( &b, down, now, out );
        for ( int k = 0; k < n; k++ )
            counts[ out[ k ] ]++;
        now += 5;
    }
}

static void press( uint32_t ms ) {
    feed( true, ms );
    feed( false, 100 );
}

void test_a_tap_is_a_click( void ) {
    buttonInit( &b, 20, false );
    press( 100 );
    TEST_ASSERT_EQUAL( 1, counts[ IN_PRESS ] );
    TEST_ASSERT_EQUAL( 1, counts[ IN_CLICK ] );
    TEST_ASSERT_EQUAL( 1, counts[ IN_RELEASE ] );
    TEST_ASSERT_EQUAL( 0, counts[ IN_HOLD ] );
    TEST_ASSERT_EQUAL( 0, counts[ IN_REPEAT ] );
}

void test_a_hold_is_not_a_click( void ) {
    buttonInit( &b, 20, false );
    press( 700 );
    TEST_ASSERT_EQUAL( 1, counts[ IN_PRESS ] );
    TEST_ASSERT_EQUAL( 1, counts[ IN_HOLD ] );
    TEST_ASSERT_EQUAL( 0, counts[ IN_LONG_HOLD ] );
    TEST_ASSERT_EQUAL( 0, counts[ IN_CLICK ] );
    TEST_ASSERT_EQUAL( 1, counts[ IN_RELEASE ] );
}

void test_a_long_hold_fires_once_after_the_hold( void ) {
    buttonInit( &b, 20, false );
    press( 2000 );
    TEST_ASSERT_EQUAL( 1, counts[ IN_HOLD ] );
    TEST_ASSERT_EQUAL( 1, counts[ IN_LONG_HOLD ] );
    TEST_ASSERT_EQUAL( 0, counts[ IN_CLICK ] );
    TEST_ASSERT_EQUAL( 1, counts[ IN_RELEASE ] );
}

// A direction held: the first repeat after 400 ms, then every 80, then
// every 40 after twenty of them.
void test_repeat_cadence( void ) {
    buttonInit( &b, 0, true );
    feed( true, 395 );
    TEST_ASSERT_EQUAL( 0, counts[ IN_REPEAT ] );
    feed( true, 10 );
    TEST_ASSERT_EQUAL( 1, counts[ IN_REPEAT ] );
    feed( true, 800 ); // 10 more at 80 ms
    TEST_ASSERT_EQUAL( 11, counts[ IN_REPEAT ] );
    feed( true, 720 ); // 9 more at 80 ms: 20 in all, then the fast rate
    TEST_ASSERT_EQUAL( 20, counts[ IN_REPEAT ] );
    feed( true, 400 ); // 10 more at 40 ms
    TEST_ASSERT_EQUAL( 30, counts[ IN_REPEAT ] );
    feed( false, 100 );
    TEST_ASSERT_EQUAL( 0, counts[ IN_CLICK ] ); // a repeat is not a click
    TEST_ASSERT_EQUAL( 1, counts[ IN_RELEASE ] );
    TEST_ASSERT_EQUAL( 0, counts[ IN_HOLD ] ); // a repeating control never holds
}

// The press is taken on its first sample (no latency); bounce on the way
// down is then ignored for the bounce time; bounce on the way up does not
// release, and the release is taken once the contact has been open that long.
void test_press_is_immediate_and_bounce_is_nothing( void ) {
    buttonInit( &b, 20, false );
    InputEventKind out[ 2 ];
    TEST_ASSERT_EQUAL( 1, buttonFeed( &b, true, now, out ) ); // the first closed sample
    TEST_ASSERT_EQUAL( IN_PRESS, out[ 0 ] );
    counts[ out[ 0 ] ]++;
    now += 5;
    feed( false, 5 ); // the down bounce: open for a sample
    feed( true, 5 );
    feed( false, 5 );
    feed( true, 100 );
    TEST_ASSERT_EQUAL( 1, counts[ IN_PRESS ] );
    TEST_ASSERT_EQUAL( 0, counts[ IN_RELEASE ] );
    feed( false, 10 ); // the up bounce: not open long enough
    feed( true, 10 );
    TEST_ASSERT_EQUAL( 0, counts[ IN_RELEASE ] );
    feed( false, 25 ); // open 20 ms: released
    TEST_ASSERT_EQUAL( 1, counts[ IN_RELEASE ] );
    TEST_ASSERT_EQUAL( 1, counts[ IN_CLICK ] );
    // A re-close within the bounce time of that release is its bounce, not
    // a press.
    feed( true, 10 );
    feed( false, 100 );
    TEST_ASSERT_EQUAL( 1, counts[ IN_PRESS ] );
    // ...and one after the bounce time is a press, at once.
    feed( true, 5 );
    TEST_ASSERT_EQUAL( 2, counts[ IN_PRESS ] );
}

// A sample that comes late (the loop was held) still sees the edge.
void test_a_late_sample_still_counts( void ) {
    buttonInit( &b, 20, false );
    InputEventKind out[ 2 ];
    now += 90; // nothing sampled for 90 ms
    TEST_ASSERT_EQUAL( 1, buttonFeed( &b, true, now, out ) );
    TEST_ASSERT_EQUAL( IN_PRESS, out[ 0 ] );
    now += 60;
    TEST_ASSERT_EQUAL( 0, buttonFeed( &b, false, now, out ) );
    now += 30;
    TEST_ASSERT_EQUAL( 2, buttonFeed( &b, false, now, out ) );
    TEST_ASSERT_EQUAL( IN_CLICK, out[ 0 ] );
    TEST_ASSERT_EQUAL( IN_RELEASE, out[ 1 ] );
}

// Every press length: exactly one PRESS and one RELEASE, and a CLICK iff
// it came up before the hold (or, for a repeating control, before the
// first repeat). The press is immediate and the release settles 20 ms, so
// the debounced press outlives the raw one by that; right at the boundary
// the sample phase decides, so that length is skipped.
void test_press_length_sweep( void ) {
    for ( int repeating = 0; repeating < 2; repeating++ ) {
        for ( uint32_t ms = 30; ms <= 1800; ms += 10 ) {
            uint32_t boundary = ( repeating ? BUTTON_REPEAT_DELAY_MS : BUTTON_HOLD_MS ) - 20;
            if ( ms == boundary )
                continue;
            setUp( );
            buttonInit( &b, 20, repeating != 0 );
            press( ms );
            bool click = ms < boundary;
            char why[ 64 ];
            snprintf( why, sizeof( why ), "repeating %d, %lu ms", repeating, (unsigned long)ms );
            TEST_ASSERT_EQUAL_MESSAGE( 1, counts[ IN_PRESS ], why );
            TEST_ASSERT_EQUAL_MESSAGE( 1, counts[ IN_RELEASE ], why );
            TEST_ASSERT_EQUAL_MESSAGE( click ? 1 : 0, counts[ IN_CLICK ], why );
            TEST_ASSERT_TRUE_MESSAGE( counts[ IN_HOLD ] <= 1 && counts[ IN_LONG_HOLD ] <= 1, why );
            if ( !repeating )
                TEST_ASSERT_EQUAL_MESSAGE( click ? 0 : 1, counts[ IN_HOLD ], why );
        }
    }
}

// ---- the stick as a four-way ------------------------------------------------------

static StickDpad d;

// Feed a stick position for `ms` in 5 ms samples.
static void stick( float x, float y, bool pressed, uint32_t ms ) {
    for ( uint32_t t = 0; t < ms; t += 5 ) {
        stickDpadFeed( &d, x, y, pressed, now );
        now += 5;
    }
}

static int onlyDown( ) {
    int which = -1, n = 0;
    for ( int k = 0; k < 4; k++ ) {
        if ( d.down[ k ] ) {
            which = k;
            n++;
        }
    }
    return n == 1 ? which : ( n == 0 ? -1 : -2 );
}

void test_stick_direction_after_the_hold_with_hysteresis( void ) {
    stickDpadInit( &d, 0.25f, 0.15f, 30, 80 );
    stick( 0.0f, 0.0f, false, 50 );
    stick( 0.3f, 0.0f, false, 20 );
    TEST_ASSERT_EQUAL( -1, onlyDown( ) ); // not yet: the hold
    stick( 0.3f, 0.0f, false, 20 );
    TEST_ASSERT_EQUAL( STICK_RIGHT, onlyDown( ) );
    stick( 0.2f, 0.0f, false, 50 ); // back under 'on' but over 'off': still on
    TEST_ASSERT_EQUAL( STICK_RIGHT, onlyDown( ) );
    stick( 0.1f, 0.0f, false, 10 ); // inside 'off': over
    TEST_ASSERT_EQUAL( -1, onlyDown( ) );
}

// A diagonal push is the dominant axis only, and the other axis cannot
// start while it is on.
void test_stick_one_axis_at_a_time( void ) {
    stickDpadInit( &d, 0.25f, 0.15f, 30, 80 );
    stick( 0.0f, 0.0f, false, 50 );
    stick( 0.5f, 0.6f, false, 50 );
    TEST_ASSERT_EQUAL( STICK_UP, onlyDown( ) );
    stick( 0.9f, 0.6f, false, 100 ); // x now larger: y keeps it until y ends
    TEST_ASSERT_EQUAL( STICK_UP, onlyDown( ) );
    stick( 0.9f, 0.0f, false, 50 ); // y over; x is still armed (it never fired) and past 'on'
    TEST_ASSERT_EQUAL( STICK_RIGHT, onlyDown( ) );
}

// Let go from full tilt: the stick springs through the centre and rings.
// That is not the opposite direction; a push after a rest is.
void test_stick_release_overshoot_is_ignored( void ) {
    stickDpadInit( &d, 0.25f, 0.15f, 30, 80 );
    stick( 0.0f, 0.0f, false, 50 );
    stick( 1.0f, 0.0f, false, 200 );
    TEST_ASSERT_EQUAL( STICK_RIGHT, onlyDown( ) );
    stick( 0.1f, 0.0f, false, 5 );   // through the centre...
    stick( -0.5f, 0.0f, false, 40 ); // ...overshoot for 40 ms
    stick( 0.3f, 0.0f, false, 20 );  // ...and a ring back
    stick( -0.2f, 0.0f, false, 20 );
    TEST_ASSERT_EQUAL( -1, onlyDown( ) );
    stick( 0.0f, 0.0f, false, 100 ); // at rest: armed again
    stick( -0.5f, 0.0f, false, 50 ); // a real push left
    TEST_ASSERT_EQUAL( STICK_LEFT, onlyDown( ) );
    // The other axis was never disarmed by x's release.
    stick( 0.0f, 0.0f, false, 50 );
    stick( 0.0f, -0.5f, false, 50 );
    TEST_ASSERT_EQUAL( STICK_DOWN, onlyDown( ) );
}

// A click of the stick's own button tilts the stick: the tilt is not a
// direction, before the button goes down or while it is down.
void test_stick_click_tilt_is_not_a_direction( void ) {
    stickDpadInit( &d, 0.25f, 0.15f, 30, 80 );
    stick( 0.0f, 0.0f, false, 50 );
    stick( 0.3f, 0.1f, false, 15 ); // the tilt as the thumb presses...
    stick( 0.3f, 0.1f, true, 100 ); // ...then the button, within the hold
    TEST_ASSERT_EQUAL( -1, onlyDown( ) );
    stick( 0.6f, 0.0f, true, 100 ); // pushed hard while the button is down: still nothing
    TEST_ASSERT_EQUAL( -1, onlyDown( ) );
    stick( 0.6f, 0.0f, false, 50 ); // the button up, the push still there: now it counts
    TEST_ASSERT_EQUAL( STICK_RIGHT, onlyDown( ) );
}

int main( int argc, char** argv ) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN( );
    RUN_TEST( test_a_tap_is_a_click );
    RUN_TEST( test_a_hold_is_not_a_click );
    RUN_TEST( test_a_long_hold_fires_once_after_the_hold );
    RUN_TEST( test_repeat_cadence );
    RUN_TEST( test_press_is_immediate_and_bounce_is_nothing );
    RUN_TEST( test_a_late_sample_still_counts );
    RUN_TEST( test_press_length_sweep );
    RUN_TEST( test_stick_direction_after_the_hold_with_hysteresis );
    RUN_TEST( test_stick_one_axis_at_a_time );
    RUN_TEST( test_stick_release_overshoot_is_ignored );
    RUN_TEST( test_stick_click_tilt_is_not_a_direction );
    return UNITY_END( );
}
