// SPDX-License-Identifier: MIT
// Host-side test of the paint brush (src/play/Paint.h): newest wins.
// Run with `pio test -e native`.
#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "Paint.h"
#include "ProbeLeds.h"

static LedLayout layout;
static ProbeLedPaint paint;
static PaintStroke stroke;

void setUp( void ) {
    ledLayoutV6( &layout );
    paintClear( &paint, &stroke );
}
void tearDown( void ) {}

// The LED at a row and hole on the rows 1-30 half.
static int ledAt( int row, int hole ) {
    for ( int i = 0; i < layout.count; i++ ) {
        if ( layout.kind[ i ] == PROBELED_HOLE && layout.row[ i ] == row && layout.hole[ i ] == hole )
            return i;
    }
    return -1;
}

static PaintBrush red( int size ) {
    PaintBrush b = { size, false, 0.5f, 255, 0, 0 };
    return b;
}

static PaintBrush blue( int size ) {
    PaintBrush b = { size, false, 0.5f, 0, 0, 255 };
    return b;
}

void test_weight_is_monotonic_and_continuous( void ) {
    for ( int size = 0; size <= PAINT_BRUSH_MAX; size++ ) {
        float last = paintWeight( 0.0f, size );
        TEST_ASSERT_FLOAT_WITHIN( 1e-6f, 1.0f, last );
        for ( float d = 0.01f; d <= paintReach( size ); d += 0.01f ) {
            float w = paintWeight( d, size );
            TEST_ASSERT_TRUE( w <= last + 1e-6f );       // never rises
            TEST_ASSERT_TRUE( last - w < 0.02f );        // no step
            TEST_ASSERT_TRUE( w >= PAINT_EDGE_WEIGHT - 1e-6f );
            last = w;
        }
        TEST_ASSERT_EQUAL_FLOAT( 0.0f, paintWeight( paintReach( size ) + 0.01f, size ) );
    }
    TEST_ASSERT_FLOAT_WITHIN( 1e-6f, 0.6f, paintWeight( 1.0f, 2 ) );
}

// A centre passing over an LED an earlier dab's edge touched takes it to
// full - and recolours it.
void test_centre_recolours_an_older_edge( void ) {
    int a = ledAt( 10, 3 ), b = ledAt( 11, 3 );
    paintStrokeBegin( &stroke );
    PaintBrush r = red( 1 );
    paintDab( &paint, &stroke, &layout, a, &r, -1 ); // b is on a's edge: 0.6 x 0.5
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.3f, paint.level[ b ] );
    PaintBrush bl = blue( 1 );
    paintDab( &paint, &stroke, &layout, b, &bl, -1 ); // now the centre is on b
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.5f, paint.level[ b ] );
    TEST_ASSERT_EQUAL( 255, paint.b[ b ] );
    TEST_ASSERT_EQUAL( 0, paint.r[ b ] );
}

// ...but an edge passing over a centre later in the same stroke leaves it.
void test_edge_never_dims_a_centre_within_a_stroke( void ) {
    int a = ledAt( 10, 3 ), b = ledAt( 11, 3 );
    paintStrokeBegin( &stroke );
    PaintBrush r = red( 1 );
    paintDab( &paint, &stroke, &layout, a, &r, -1 );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.5f, paint.level[ a ] );
    PaintBrush bl = blue( 1 );
    paintDab( &paint, &stroke, &layout, b, &bl, -1 ); // a is on b's edge now
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.5f, paint.level[ a ] );
    TEST_ASSERT_EQUAL( 255, paint.r[ a ] ); // still red
}

// A new stroke starts afresh: its edge paints over the old stroke's centre,
// colour and level.
void test_a_new_stroke_overwrites_an_old_centre( void ) {
    int a = ledAt( 10, 3 ), b = ledAt( 11, 3 );
    paintStrokeBegin( &stroke );
    PaintBrush r = red( 1 );
    paintDab( &paint, &stroke, &layout, a, &r, -1 );
    paintStrokeEnd( &stroke );
    paintStrokeBegin( &stroke );
    PaintBrush bl = blue( 1 );
    paintDab( &paint, &stroke, &layout, b, &bl, -1 );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.3f, paint.level[ a ] ); // the edge's 0.6 x 0.5
    TEST_ASSERT_EQUAL( 255, paint.b[ a ] );
    TEST_ASSERT_EQUAL( 0, paint.r[ a ] );
}

void test_erase_clears_and_the_target_is_skipped( void ) {
    int a = ledAt( 10, 3 ), b = ledAt( 11, 3 );
    paintStrokeBegin( &stroke );
    PaintBrush r = red( 1 );
    paintDab( &paint, &stroke, &layout, a, &r, -1 );
    TEST_ASSERT_TRUE( paint.level[ a ] > 0.0f && paint.level[ b ] > 0.0f );
    PaintBrush e = red( 1 );
    e.erase = true;
    paintDab( &paint, &stroke, &layout, a, &e, b ); // b is the target: left alone
    TEST_ASSERT_EQUAL_FLOAT( 0.0f, paint.level[ a ] );
    TEST_ASSERT_TRUE( paint.level[ b ] > 0.0f );
    TEST_ASSERT_EQUAL_FLOAT( 0.0f, stroke.base[ a ] );
    paintDab( &paint, &stroke, &layout, a, &r, -1 ); // erased: a paints again at full
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.5f, paint.level[ a ] );
}

void test_size_zero_paints_one_led( void ) {
    int a = ledAt( 10, 3 );
    paintStrokeBegin( &stroke );
    PaintBrush r = red( 0 );
    paintDab( &paint, &stroke, &layout, a, &r, -1 );
    int painted = 0;
    for ( int i = 0; i < layout.count; i++ )
        painted += paint.level[ i ] > 0.0f;
    TEST_ASSERT_EQUAL( 1, painted );
}

// The colour wheel's marker against the rim (2026-09-28): a push that is not
// inward starts a cycle round the rim at the stick's tilt - straight out
// anticlockwise, sideways the way it points - that runs until the stick is
// let go; a fresh push inward brings the marker in; inside the wheel a move
// is the plain move.
void test_picker_cycles_round_the_rim_until_the_stick_is_let_go( void ) {
    float hue = 0.0f, sat = 1.0f; // at the rim, on the right
    int cycling = 0;
    for ( int i = 0; i < 100; i++ ) // straight out (to the right), 4 radii a second at 100 Hz: a second
        paintPickerMove( &hue, &sat, &cycling, 0.04f, 0.0f );
    TEST_ASSERT_EQUAL( 1, cycling );
    TEST_ASSERT_FLOAT_WITHIN( 2.0f, 229.2f, hue ); // 4 radians anticlockwise, still going
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 1.0f, sat );
    for ( int i = 0; i < 100; i++ ) // the stick still held, now pointing anywhere: it keeps turning
        paintPickerMove( &hue, &sat, &cycling, -0.02f, -0.03f );
    TEST_ASSERT_EQUAL( 1, cycling );
    TEST_ASSERT_FLOAT_WITHIN( 2.0f, fmodf( 229.2f + 206.6f, 360.0f ), hue );
    paintPickerRelease( &cycling ); // let go: a fresh push, this one inward, brings it in
    TEST_ASSERT_EQUAL( 0, cycling );
    float rad = hue * ( 3.14159265f / 180.0f );
    paintPickerMove( &hue, &sat, &cycling, -0.5f * cosf( rad ), -0.5f * sinf( rad ) );
    TEST_ASSERT_EQUAL( 0, cycling );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.5f, sat );
    // A sideways push at the rim sets the way: from the right, pushed down, clockwise.
    hue = 0.0f;
    sat = 1.0f;
    cycling = 0;
    for ( int i = 0; i < 50; i++ )
        paintPickerMove( &hue, &sat, &cycling, 0.0f, -0.04f );
    TEST_ASSERT_EQUAL( -1, cycling );
    TEST_ASSERT_FLOAT_WITHIN( 2.0f, 360.0f - 114.6f, hue );
    // Inside: the plain move.
    hue = 0.0f;
    sat = 0.5f;
    cycling = 0;
    paintPickerMove( &hue, &sat, &cycling, 0.0f, 0.2f );
    TEST_ASSERT_EQUAL( 0, cycling );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, sqrtf( 0.29f ), sat );
    TEST_ASSERT_FLOAT_WITHIN( 0.5f, 21.8f, hue );
}

int main( int argc, char** argv ) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN( );
    RUN_TEST( test_weight_is_monotonic_and_continuous );
    RUN_TEST( test_centre_recolours_an_older_edge );
    RUN_TEST( test_edge_never_dims_a_centre_within_a_stroke );
    RUN_TEST( test_a_new_stroke_overwrites_an_old_centre );
    RUN_TEST( test_erase_clears_and_the_target_is_skipped );
    RUN_TEST( test_size_zero_paints_one_led );
    RUN_TEST( test_picker_cycles_round_the_rim_until_the_stick_is_let_go );
    return UNITY_END( );
}
