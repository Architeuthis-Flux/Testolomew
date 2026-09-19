// SPDX-License-Identifier: MIT
// Host-side test of the button classifier (src/ui/ButtonTracker.h): a tap
// is a click, a hold is not, a repeat is not, bounces are nothing.
// Run with `pio test -e native`.
#include <stdio.h>
#include <unity.h>

#include "ButtonTracker.h"

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

void test_a_bounce_is_nothing( void ) {
    buttonInit( &b, 20, false );
    feed( true, 10 );
    feed( false, 10 );
    feed( true, 10 );
    feed( false, 100 );
    TEST_ASSERT_EQUAL( 0, counts[ IN_PRESS ] );
    TEST_ASSERT_EQUAL( 0, counts[ IN_CLICK ] );
    // ...and a bounce on the way up of a real press does not release it.
    feed( true, 100 );
    TEST_ASSERT_EQUAL( 1, counts[ IN_PRESS ] );
    feed( false, 10 );
    feed( true, 10 );
    TEST_ASSERT_EQUAL( 0, counts[ IN_RELEASE ] );
    feed( false, 100 );
    TEST_ASSERT_EQUAL( 1, counts[ IN_RELEASE ] );
    TEST_ASSERT_EQUAL( 1, counts[ IN_CLICK ] );
}

// A sample that comes late (the loop was held) still sees the edge.
void test_a_late_sample_still_counts( void ) {
    buttonInit( &b, 20, false );
    InputEventKind out[ 2 ];
    TEST_ASSERT_EQUAL( 0, buttonFeed( &b, true, now, out ) );
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
// first repeat). The debounce delays both edges alike, so the debounced
// press is as long as the raw one; right at the boundary the sample phase
// decides, so that length is skipped.
void test_press_length_sweep( void ) {
    for ( int repeating = 0; repeating < 2; repeating++ ) {
        for ( uint32_t ms = 30; ms <= 1800; ms += 10 ) {
            uint32_t boundary = repeating ? BUTTON_REPEAT_DELAY_MS : BUTTON_HOLD_MS;
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

int main( int argc, char** argv ) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN( );
    RUN_TEST( test_a_tap_is_a_click );
    RUN_TEST( test_a_hold_is_not_a_click );
    RUN_TEST( test_a_long_hold_fires_once_after_the_hold );
    RUN_TEST( test_repeat_cadence );
    RUN_TEST( test_a_bounce_is_nothing );
    RUN_TEST( test_a_late_sample_still_counts );
    RUN_TEST( test_press_length_sweep );
    return UNITY_END( );
}
