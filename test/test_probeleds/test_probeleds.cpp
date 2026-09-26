// SPDX-License-Identifier: MIT
// Host-side test of the LED cursor: the layouts put LEDs where the holes are
// and map to JumperlOS's pixel numbers, a sharp fix lights one hole, a vague
// one a wide dim patch of about the same total light, a far probe a dim
// purple glow, the pointed-mode tail marks the point, and LEDs fade rather
// than snap. Run with `pio test -e native`.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "ProbeLeds.h"

static LedLayout v6, v5;
static ProbeLedStyle style;
static ProbeLedFrame frame;

void setUp( void ) {
    ledLayoutV6( &v6 );
    ledLayoutV5( &v5 );
    probeLedDefaultStyle( &style );
    probeLedClear( &frame, v6.count );
}
void tearDown( void ) {}

static int find( const LedLayout* l, int row, int hole ) {
    for ( int i = 0; i < l->count; i++ ) {
        if ( l->kind[ i ] == PROBELED_HOLE && l->row[ i ] == row && l->hole[ i ] == hole )
            return i;
    }
    return -1;
}

static ProbeLedInput at( float along, float acrossMm, float sigmaRows, float sigmaAcrossMm ) {
    ProbeLedInput in = { };
    in.state = PROBELED_TRACKING;
    in.along = along;
    in.acrossMm = acrossMm;
    in.sigmaRows = sigmaRows;
    in.sigmaAcrossMm = sigmaAcrossMm;
    in.confidence = 1.0f;
    return in;
}

static float total( const ProbeLedFrame* f ) {
    float sum = 0.0f;
    for ( int i = 0; i < f->count; i++ )
        sum += f->target[ i ];
    return sum;
}

static int brightest( const ProbeLedFrame* f ) {
    int best = 0;
    for ( int i = 1; i < f->count; i++ )
        if ( f->target[ i ] > f->target[ best ] )
            best = i;
    return best;
}

void test_v6_layout( void ) {
    TEST_ASSERT_EQUAL( 16 * 30, v6.count );
    int i = find( &v6, 1, 1 );
    TEST_ASSERT_TRUE( i >= 0 );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 1.0f, v6.along[ i ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 1.27f, v6.acrossMm[ i ] );
    i = find( &v6, 31, 1 ); // faces row 1 across the centre, one pitch away
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 1.0f, v6.along[ i ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, -1.27f, v6.acrossMm[ i ] );
    i = find( &v6, 60, 6 );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 30.0f, v6.along[ i ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, -( 1.27f + 5 * 2.54f ), v6.acrossMm[ i ] );
    int rails = 0;
    for ( int k = 0; k < v6.count; k++ )
        rails += v6.kind[ k ] == PROBELED_RAIL;
    TEST_ASSERT_EQUAL( 4 * 30, rails );
}

void test_v5_layout_and_pixels( void ) {
    TEST_ASSERT_EQUAL( 400, v5.count );
    int i = find( &v5, 1, 1 );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 3.81f, v5.acrossMm[ i ] );
    // Column 0 is the outer hole on the top half; the bottom half runs the other way.
    TEST_ASSERT_EQUAL( 4, ledLayoutV5Pixel( &v5, i ) );
    TEST_ASSERT_EQUAL( 0, ledLayoutV5Pixel( &v5, find( &v5, 1, 5 ) ) );
    TEST_ASSERT_EQUAL( 30 * 5 + 0, ledLayoutV5Pixel( &v5, find( &v5, 31, 1 ) ) );
    TEST_ASSERT_EQUAL( 59 * 5 + 4, ledLayoutV5Pixel( &v5, find( &v5, 60, 5 ) ) );
    // Every hole pixel 0..299 is used exactly once.
    int seen[ 300 ] = { 0 };
    for ( int k = 0; k < v5.count; k++ ) {
        int p = ledLayoutV5Pixel( &v5, k );
        if ( v5.kind[ k ] == PROBELED_HOLE ) {
            TEST_ASSERT_TRUE( p >= 0 && p < 300 );
            seen[ p ]++;
        } else {
            TEST_ASSERT_TRUE( p >= 300 && p < 400 );
        }
    }
    for ( int p = 0; p < 300; p++ )
        TEST_ASSERT_EQUAL( 1, seen[ p ] );
    // Rail LEDs sit half a pitch along from the rows and skip a position
    // every five (the r8 board); the first rail pair is on the rows 1-30
    // side, outer rail first, and the bottom pair inner rail first.
    int r0 = 300; // first rail LED in layout order
    TEST_ASSERT_TRUE( v5.acrossMm[ r0 ] > 0.0f );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 1.5f, v5.along[ r0 ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 5.5f, v5.along[ r0 + 4 ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 7.5f, v5.along[ r0 + 5 ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, PROBELED_V5_RAIL_OUTER_MM, v5.acrossMm[ r0 ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, PROBELED_V5_RAIL_INNER_MM, v5.acrossMm[ r0 + 25 ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, -PROBELED_V5_RAIL_INNER_MM, v5.acrossMm[ r0 + 50 ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, -PROBELED_V5_RAIL_OUTER_MM, v5.acrossMm[ r0 + 75 ] );
    TEST_ASSERT_EQUAL( 0, ledLayoutV5RailPixel( &v5, r0 ) );
    TEST_ASSERT_EQUAL( 99, ledLayoutV5RailPixel( &v5, r0 + 99 ) );
    TEST_ASSERT_EQUAL( -1, ledLayoutV5RailPixel( &v5, 0 ) );
}

void test_sharp_fix_lights_one_hole( void ) {
    ProbeLedInput in = at( 14.0f, 1.27f + 2 * 2.54f, 0.05f, 0.2f ); // row 14, hole 3, dead centre
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    int b = brightest( &frame );
    TEST_ASSERT_EQUAL( 14, v6.row[ b ] );
    TEST_ASSERT_EQUAL( 3, v6.hole[ b ] );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, style.peak, frame.target[ b ] );
    // The next row over is nearly dark, the next hole faint.
    TEST_ASSERT_TRUE( frame.target[ find( &v6, 15, 3 ) ] < 0.03f );
    TEST_ASSERT_TRUE( frame.target[ find( &v6, 14, 4 ) ] < 0.03f );
    TEST_ASSERT_TRUE( frame.target[ find( &v6, 44, 3 ) ] < 0.01f ); // the other half stays dark
    // With a whole second of dt the smoothed level is there too, and white.
    uint8_t r, g, bl;
    probeLedRgb( &frame, b, &r, &g, &bl );
    TEST_ASSERT_TRUE( r > 240 && g > 240 && bl > 240 );
}

void test_vague_fix_is_wide_and_dim_with_the_same_light( void ) {
    style.fullPeak = false; // the constant-light mode
    ProbeLedInput sharp = at( 14.0f, 6.35f, 0.05f, 0.2f );
    probeLedRender( &v6, &sharp, &style, 1.0f, &frame );
    float sharpTotal = total( &frame );
    ProbeLedInput vague = at( 14.0f, 6.35f, 2.0f, 5.0f );
    probeLedRender( &v6, &vague, &style, 1.0f, &frame );
    float vagueTotal = total( &frame );
    int lit = 0;
    for ( int i = 0; i < frame.count; i++ )
        lit += frame.target[ i ] > 0.02f;
    int b = brightest( &frame );
    printf( "  sharp: total %.2f; vague: total %.2f over %d LEDs, peak %.2f\n", sharpTotal, vagueTotal, lit, frame.target[ b ] );
    TEST_ASSERT_TRUE( frame.target[ b ] < 0.2f * style.peak );
    TEST_ASSERT_TRUE( lit > 5 ); // a patch (the floor that once kept a wide bell at 0.08 everywhere fades with the width)
    // Dimmer as it widens (the floor fades out with the width, PROBELED_MAX_SIGMA_*): no more light than the sharp fix, and some.
    TEST_ASSERT_TRUE( vagueTotal < 1.5f * sharpTotal && vagueTotal > 0.1f * sharpTotal );
}

void test_vague_fix_at_full_peak_fades_with_width( void ) {
    style.fullPeak = true; // a bell up to twice the narrowest keeps an LED at the peak; wider, the peak fades with the width
    ProbeLedInput narrow = at( 14.0f, 6.35f, 0.5f, 1.5f );
    probeLedRender( &v6, &narrow, &style, 1.0f, &frame );
    int b = brightest( &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, style.peak * ( 0.5f + 0.5f * narrow.confidence ), frame.target[ b ] );
    ProbeLedInput vague = at( 14.0f, 6.35f, 2.0f, 5.0f );
    probeLedRender( &v6, &vague, &style, 1.0f, &frame );
    b = brightest( &frame );
    int lit = 0;
    for ( int i = 0; i < frame.count; i++ )
        lit += frame.target[ i ] > 0.02f;
    TEST_ASSERT_TRUE( frame.target[ b ] > 0.2f * style.peak && frame.target[ b ] < 0.7f * style.peak );
    TEST_ASSERT_TRUE( lit > 20 );
    ProbeLedInput huge = at( 14.0f, 6.35f, 8.0f, 20.0f ); // wider than the widest allowed: held at it, the floor gone
    probeLedRender( &v6, &huge, &style, 1.0f, &frame );
    b = brightest( &frame );
    lit = 0;
    for ( int i = 0; i < frame.count; i++ )
        lit += frame.target[ i ] > 0.02f;
    TEST_ASSERT_TRUE( frame.target[ b ] < 0.1f * style.peak );
    TEST_ASSERT_TRUE( lit < 60 ); // a patch, not the board
}

void test_looks( void ) {
    // Bloom lights more LEDs than the bare cursor; sparkle turns some of
    // them white; pulse makes the peak breathe; a touch ring appears when
    // the point lands after being lifted.
    ProbeLedInput in = at( 14.0f, 6.35f, 0.3f, 0.9f );
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    int plain = 0;
    for ( int i = 0; i < frame.count; i++ )
        plain += frame.target[ i ] > 0.02f;
    style.bloom = 1.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    int bloomed = 0;
    for ( int i = 0; i < frame.count; i++ )
        bloomed += frame.target[ i ] > 0.02f;
    printf( "  lit LEDs: plain %d, with bloom %d\n", plain, bloomed );
    TEST_ASSERT_TRUE( bloomed > plain + 10 );
    style.bloom = 0.0f;

    // A wide glow (many LEDs to twinkle), the cursor blue (lifted) so a flash
    // stands out: some LED other than the cursor's own is at the peak, near
    // white but each with a little colour of its own.
    style.sparkle = 1.0f;
    ProbeLedInput wide = at( 14.0f, 6.35f, 1.0f, 2.5f );
    wide.heightMm = 30.0f;
    int twinkles = 0, pureWhite = 0, tinted = 0;
    int centre = find( &v6, 14, 3 ); // the cursor's own LED, at the peak too
    for ( int n = 0; n < 50; n++ ) {
        probeLedRender( &v6, &wide, &style, 0.02f, &frame );
        for ( int i = 0; i < frame.count; i++ ) {
            if ( i == centre || frame.target[ i ] < 0.99f * style.peak )
                continue;
            int lo = frame.r[ i ] < frame.g[ i ] ? ( frame.r[ i ] < frame.b[ i ] ? frame.r[ i ] : frame.b[ i ] ) : ( frame.g[ i ] < frame.b[ i ] ? frame.g[ i ] : frame.b[ i ] );
            int hi = frame.r[ i ] > frame.g[ i ] ? ( frame.r[ i ] > frame.b[ i ] ? frame.r[ i ] : frame.b[ i ] ) : ( frame.g[ i ] > frame.b[ i ] ? frame.g[ i ] : frame.b[ i ] );
            twinkles++;
            TEST_ASSERT_TRUE( lo >= 180 && hi == 255 ); // near white...
            if ( hi - lo == 0 )
                pureWhite++;
            else
                tinted++;
        }
    }
    printf( "  twinkles in 50 frames: %d (%d tinted, %d pure white)\n", twinkles, tinted, pureWhite );
    TEST_ASSERT_TRUE( twinkles > 5 );
    TEST_ASSERT_TRUE( tinted > pureWhite ); // ...but not the same white
    // On the board, the same glow twinkles far less.
    wide.heightMm = 0.0f;
    int low = 0;
    for ( int n = 0; n < 50; n++ ) {
        probeLedRender( &v6, &wide, &style, 0.02f, &frame );
        for ( int i = 0; i < frame.count; i++ )
            low += i != centre && frame.target[ i ] >= 0.99f * style.peak;
    }
    printf( "  ...and %d on the board\n", low );
    TEST_ASSERT_TRUE( low * 4 < twinkles );
    style.sparkle = 0.0f;

    style.pulse = 1.0f;
    float lowest = 1.0f, highest = 0.0f;
    for ( int n = 0; n < 100; n++ ) {
        probeLedRender( &v6, &in, &style, 0.02f, &frame );
        float p = frame.target[ brightest( &frame ) ];
        if ( p < lowest )
            lowest = p;
        if ( p > highest )
            highest = p;
    }
    TEST_ASSERT_TRUE( highest > 1.8f * lowest );
    style.pulse = 0.0f;

    style.touchRing = true;
    in.heightMm = 10.0f; // lifted...
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    in.heightMm = 0.5f; // ...and down
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    for ( int n = 0; n < 2; n++ )
        probeLedRender( &v6, &in, &style, 0.02f, &frame ); // 0.06 s: the ring is ~3 rows out (PROBELED_RING_ROWS_PER_S)
    int ringLit = 0, inside = 0;
    for ( int i = 0; i < frame.count; i++ ) {
        ringLit += frame.target[ i ] > 0.05f && ( v6.row[ i ] == 11 || v6.row[ i ] == 17 );
        inside += frame.level[ i ] > 0.3f * style.peak && ( v6.row[ i ] == 12 || v6.row[ i ] == 16 ) && v6.hole[ i ] == 3; // where the ring passed a frame ago
    }
    TEST_ASSERT_TRUE( ringLit > 0 );
    TEST_ASSERT_TRUE( inside == 0 ); // the ring's LEDs go dark at its own pace: a ring, not a filled circle
    for ( int n = 0; n < 8; n++ )
        probeLedRender( &v6, &in, &style, 0.02f, &frame ); // the ring is over and gone
    style.touchRing = false;

    // Every scheme: white with the point on the board, its own colour lifted.
    in.tiltDeg = 30.0f; // (the aim scheme's colour is the lean's: straight up it is white)
    for ( int scheme = 0; scheme < PROBELED_SCHEME_COUNT; scheme++ ) {
        style.scheme = scheme;
        in.heightMm = 0.5f;
        probeLedRender( &v6, &in, &style, 0.02f, &frame );
        int b = brightest( &frame );
        TEST_ASSERT_TRUE( frame.r[ b ] >= 250 && frame.g[ b ] >= 250 && frame.b[ b ] >= 250 );
        in.heightMm = 20.0f;
        probeLedRender( &v6, &in, &style, 0.02f, &frame );
        b = brightest( &frame );
        int lo = frame.r[ b ] < frame.g[ b ] ? ( frame.r[ b ] < frame.b[ b ] ? frame.r[ b ] : frame.b[ b ] ) : ( frame.g[ b ] < frame.b[ b ] ? frame.g[ b ] : frame.b[ b ] );
        TEST_ASSERT_TRUE( lo < 200 ); // some colour to it
        TEST_ASSERT_TRUE( frame.r[ b ] + frame.g[ b ] + frame.b[ b ] > 100 );
    }
    in.heightMm = 0.5f;
    style.scheme = PROBELED_SCHEME_CLASSIC;
}

// The colours (2026-09-25): every scheme uses the whole wheel in its own
// way, and the mapping is the style's - how many turns of the wheel over
// the scale, where it starts, the height the scale runs to, and the height
// by which the colour is all in (white on the board below it).
void test_hue_scale_turns_and_start( void ) {
    TEST_ASSERT_EQUAL( 5, PROBELED_SCHEME_COUNT ); // classic, height, sure, aim, rainbow: the single-colour ones are gone
    TEST_ASSERT_EQUAL_STRING( "aim", probeLedSchemeNames[ PROBELED_SCHEME_AIM ] );
    style.scheme = PROBELED_SCHEME_HEIGHT;
    style.liftFullMm = 20.0f;
    style.colourByMm = 6.0f;
    ProbeLedInput in = at( 14.0f, 6.35f, 0.3f, 0.9f );
    in.heightMm = 10.0f; // halfway up the scale
    style.hueTurns = 1.0f;
    style.hueStartDeg = 0.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    int b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.r[ b ] < 10 && frame.g[ b ] == 255 && frame.b[ b ] == 255 ); // 180 degrees: cyan
    style.hueStartDeg = 120.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.r[ b ] == 255 && frame.g[ b ] < 10 && frame.b[ b ] == 255 ); // 300: magenta
    style.hueStartDeg = 0.0f;
    style.hueTurns = 2.0f; // two rainbows over the scale: halfway is a whole turn round, red again
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.r[ b ] == 255 && frame.g[ b ] < 10 && frame.b[ b ] < 10 );
    style.hueTurns = 0.25f; // a quarter of the wheel as a gradient: halfway is 45 degrees, orange
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.r[ b ] == 255 && frame.g[ b ] > 150 && frame.g[ b ] < 220 && frame.b[ b ] < 10 );
    // Above the scale the colour holds at the wheel's end (the review,
    // 2026-09-25: unclamped, a probe 20-45 mm up cycled the wheel and read as
    // any height on the scale).
    style.hueTurns = 0.667f;
    in.heightMm = 20.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    uint8_t topR = frame.r[ b ], topG = frame.g[ b ], topB = frame.b[ b ];
    in.heightMm = 45.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    TEST_ASSERT_TRUE( abs( (int)frame.r[ b ] - (int)topR ) < 3 && abs( (int)frame.g[ b ] - (int)topG ) < 3 && abs( (int)frame.b[ b ] - (int)topB ) < 3 );
    in.heightMm = 10.0f;
    style.hueTurns = 0.25f;
    // The colour is all in from colourByMm up; on the board it is white.
    style.colourByMm = 12.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame ); // 10 mm: most of the way from 1.5 to 12
    b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.b[ b ] > 40 && frame.b[ b ] < 120 ); // some white still in it
    in.heightMm = 0.5f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.r[ b ] >= 250 && frame.g[ b ] >= 250 && frame.b[ b ] >= 250 );
    // Sure: the same wheel over how sure the row is.
    style.scheme = PROBELED_SCHEME_SURE;
    style.colourByMm = 6.0f;
    style.hueTurns = 1.0f;
    in.heightMm = 20.0f;
    in.confidence = 0.5f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.r[ b ] < 10 && frame.g[ b ] == 255 && frame.b[ b ] == 255 );
    // Rainbow: the turns run along the board - two rows a whole turn apart read the same.
    style.scheme = PROBELED_SCHEME_RAINBOW;
    style.hueTurns = 1.0f; // one turn over the 30 rows
    ProbeLedInput wide = at( 15.0f, 6.35f, 3.0f, 7.0f );
    wide.heightMm = 20.0f;
    style.fullPeak = false;
    probeLedRender( &v6, &wide, &style, 0.02f, &frame );
    int a = find( &v6, 10, 3 ), c = find( &v6, 20, 3 );
    TEST_ASSERT_TRUE( frame.target[ a ] > 0.0f && frame.target[ c ] > 0.0f );
    TEST_ASSERT_TRUE( abs( (int)frame.r[ a ] - (int)frame.r[ c ] ) > 60 || abs( (int)frame.g[ a ] - (int)frame.g[ c ] ) > 60 ); // ten rows apart: a third of a turn
}

// The aim scheme: the hue is the direction the probe leans, white when it
// stands straight, all colour from PROBELED_AIM_FULL_DEG of lean.
void test_aim_scheme_colours_by_the_lean( void ) {
    style.scheme = PROBELED_SCHEME_AIM;
    style.hueTurns = 1.0f;
    style.hueStartDeg = 0.0f;
    ProbeLedInput in = at( 14.0f, 6.35f, 0.3f, 0.9f );
    in.heightMm = 20.0f;
    in.tiltDeg = 0.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    int b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.r[ b ] >= 250 && frame.g[ b ] >= 250 && frame.b[ b ] >= 250 ); // straight up: white
    in.tiltDeg = 45.0f;
    in.aimDeg = 0.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.r[ b ] == 255 && frame.g[ b ] < 60 && frame.b[ b ] < 60 ); // leaning along +x: red
    in.aimDeg = 120.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.g[ b ] == 255 && frame.r[ b ] < 60 && frame.b[ b ] < 60 ); // a third round: green
    // The compass is one turn of the wheel whatever the turns lever says: a
    // lean a hair either side of +x is the same colour (the review,
    // 2026-09-25: at 0.667 turns the hue jumped red/blue at the seam).
    style.hueTurns = 0.667f;
    in.aimDeg = 359.5f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    uint8_t r0 = frame.r[ b ], g0 = frame.g[ b ], b0 = frame.b[ b ];
    in.aimDeg = 0.5f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    b = brightest( &frame );
    TEST_ASSERT_TRUE( abs( (int)frame.r[ b ] - (int)r0 ) < 8 && abs( (int)frame.g[ b ] - (int)g0 ) < 8 && abs( (int)frame.b[ b ] - (int)b0 ) < 8 );
}

// What dims the cursor, and by how much, is the style's: unsure (the old
// rule, a coin-toss row at half), height, tilt, speed, or nothing.
void test_brightness_follows_the_chosen_data( void ) {
    ProbeLedInput in = at( 14.0f, 6.35f, 0.3f, 0.9f );
    in.confidence = 0.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame ); // the default: by unsure, half
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, 0.5f * style.peak, frame.target[ brightest( &frame ) ] );
    style.brightAmount = 1.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_TRUE( frame.target[ brightest( &frame ) ] < 0.02f ); // a toss-up: dark
    in.confidence = 1.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, style.peak, frame.target[ brightest( &frame ) ] );
    style.brightBy = PROBELED_DATA_NONE;
    in.confidence = 0.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, style.peak, frame.target[ brightest( &frame ) ] ); // nothing dims it
    style.brightBy = PROBELED_DATA_TILT;
    style.brightAmount = 0.5f;
    in.tiltDeg = 60.0f; // the far end of the tilt scale
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, 0.5f * style.peak, frame.target[ brightest( &frame ) ] );
    style.brightBy = PROBELED_DATA_HEIGHT;
    style.brightAmount = 1.0f;
    in.heightMm = style.liftFullMm * 0.5f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, 0.5f * style.peak, frame.target[ brightest( &frame ) ] );
    style.brightBy = PROBELED_DATA_SPEED;
    in.heightMm = 0.0f;
    in.speedMmS = 400.0f; // past the scale's end: all of the amount
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_TRUE( frame.target[ brightest( &frame ) ] < 0.02f );
}

// ...and what the sparkle's density follows: by height (the default: few
// on the board, all of it lifted), or nothing (the lever alone, the same
// on the board).
void test_sparkle_density_follows_the_chosen_data( void ) {
    style.sparkle = 1.0f;
    style.sparkleBy = PROBELED_DATA_NONE;
    ProbeLedInput wide = at( 14.0f, 6.35f, 1.0f, 2.5f );
    int centre = find( &v6, 14, 3 );
    int lifted = 0, onBoard = 0;
    wide.heightMm = 30.0f;
    for ( int n = 0; n < 50; n++ ) {
        probeLedRender( &v6, &wide, &style, 0.02f, &frame );
        for ( int i = 0; i < frame.count; i++ )
            lifted += i != centre && frame.target[ i ] >= 0.99f * style.peak;
    }
    wide.heightMm = 0.0f;
    for ( int n = 0; n < 50; n++ ) {
        probeLedRender( &v6, &wide, &style, 0.02f, &frame );
        for ( int i = 0; i < frame.count; i++ )
            onBoard += i != centre && frame.target[ i ] >= 0.99f * style.peak;
    }
    printf( "  sparkle by nothing: %d twinkles lifted, %d on the board\n", lifted, onBoard );
    TEST_ASSERT_TRUE( lifted > 5 && onBoard * 2 > lifted ); // about as many
}

void test_far_probe_glows_purple( void ) {
    // Dim whatever the peak rule: a hundred LEDs wide at the peak would be
    // amps out of a breadboard's rail.
    style.fullPeak = true;
    ProbeLedInput in = at( 20.0f, 0.0f, 6.0f, 12.0f );
    in.state = PROBELED_ROUGH;
    in.confidence = 0.1f;
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    int b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.target[ b ] >= 0.029f && frame.target[ b ] < 0.15f ); // PROBELED_ROUGH_LEAST: a faint patch, held at the widest width
    style.fullPeak = false;
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    b = brightest( &frame );
    TEST_ASSERT_TRUE( frame.target[ b ] >= 0.029f && frame.target[ b ] < 0.15f );
    TEST_ASSERT_EQUAL( style.roughR, frame.r[ b ] );
    TEST_ASSERT_EQUAL( style.roughB, frame.b[ b ] );
    ProbeLedInput none = { };
    probeLedRender( &v6, &none, &style, 1.0f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.0f, total( &frame ) );
}

void test_pointed_tail_marks_the_point( void ) {
    ProbeLedInput in = at( 10.0f, 6.35f, 0.1f, 0.3f );
    in.heightMm = 12.0f;
    in.haveUnder = true;
    in.underAlong = 14.0f; // the point is four rows away from where it aims
    in.underAcrossMm = 6.35f;
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    int cursor = find( &v6, 10, 3 ), under = find( &v6, 14, 3 ), between = find( &v6, 12, 3 );
    TEST_ASSERT_TRUE( frame.target[ cursor ] > 0.5f );
    TEST_ASSERT_TRUE( frame.target[ under ] > 0.2f && frame.target[ under ] < frame.target[ cursor ] );
    TEST_ASSERT_TRUE( frame.target[ between ] > 0.05f );
    TEST_ASSERT_EQUAL( style.tailR, frame.r[ under ] );
    // Lifted 12 mm the cursor is mostly the "lift" colour.
    TEST_ASSERT_TRUE( frame.b[ cursor ] > 200 && frame.r[ cursor ] < 120 );
}

void test_leds_fade_rather_than_snap( void ) {
    ProbeLedInput a = at( 5.0f, 6.35f, 0.05f, 0.2f );
    probeLedRender( &v6, &a, &style, 1.0f, &frame );
    int wasAt = find( &v6, 5, 3 );
    TEST_ASSERT_TRUE( frame.level[ wasAt ] > 0.9f );
    ProbeLedInput b = at( 20.0f, 6.35f, 0.05f, 0.2f );
    probeLedRender( &v6, &b, &style, 0.02f, &frame ); // one 50 Hz frame later
    int nowAt = find( &v6, 20, 3 );
    TEST_ASSERT_TRUE( frame.level[ nowAt ] > 0.5f );                                 // lit within one attack constant
    TEST_ASSERT_TRUE( frame.level[ wasAt ] > 0.7f && frame.level[ wasAt ] < 0.95f ); // still glowing
    for ( int k = 0; k < 25; k++ )
        probeLedRender( &v6, &b, &style, 0.02f, &frame ); // half a second on
    TEST_ASSERT_TRUE( frame.level[ wasAt ] < 0.05f );
    TEST_ASSERT_TRUE( frame.level[ nowAt ] > 0.98f );
}

int main( int argc, char** argv ) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN( );
    RUN_TEST( test_v6_layout );
    RUN_TEST( test_v5_layout_and_pixels );
    RUN_TEST( test_sharp_fix_lights_one_hole );
    RUN_TEST( test_vague_fix_is_wide_and_dim_with_the_same_light );
    RUN_TEST( test_vague_fix_at_full_peak_fades_with_width );
    RUN_TEST( test_looks );
    RUN_TEST( test_hue_scale_turns_and_start );
    RUN_TEST( test_aim_scheme_colours_by_the_lean );
    RUN_TEST( test_brightness_follows_the_chosen_data );
    RUN_TEST( test_sparkle_density_follows_the_chosen_data );
    RUN_TEST( test_far_probe_glows_purple );
    RUN_TEST( test_pointed_tail_marks_the_point );
    RUN_TEST( test_leds_fade_rather_than_snap );
    return UNITY_END( );
}
