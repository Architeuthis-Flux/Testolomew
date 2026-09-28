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
    // The look these tests were written against (the defaults are the
    // bench's settings since 2026-09-27: the height scheme from 240 deg,
    // a wide spot by height, no dimming, a long tail, the ring on).
    style.scheme = PROBELED_SCHEME_CLASSIC;
    style.hueTurns = 0.667f;
    style.hueStartDeg = 0.0f;
    style.liftFullMm = 15.0f;
    style.colourByMm = 6.0f;
    style.brightBy = PROBELED_DATA_SURE;
    style.brightAmount = -0.5f; // (signed since 2026-09-28: a toss-up row at half)
    style.sparkleBy = PROBELED_DATA_HEIGHT;
    style.spot = 1.0f;
    style.spotByHeight = 0.0f;
    style.decayS = 0.12f;
    style.touchRing = false;
    style.tailLength = 0.5f;
    style.tailBright = 0.5f;
    style.tailHueDeg = 25.0f;
    style.errorWidth = 1.0f;
    style.falloff = 1.0f;
    style.whiteOnBoard = true;
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
    TEST_ASSERT_TRUE( lit > 5 ); // a patch, at the floor (0.08 whatever the width since 2026-09-27; it faded out with the width before)
    TEST_ASSERT_TRUE( vagueTotal > 0.1f * sharpTotal );
}

void test_vague_fix_at_full_peak_keeps_the_peak( void ) {
    style.fullPeak = true; // the brightest LED is the peak whatever the width (2026-09-27; up to twice the narrowest until then, fading with the width beyond)
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
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, style.peak, frame.target[ b ] );
    TEST_ASSERT_TRUE( lit > 20 );
    ProbeLedInput huge = at( 14.0f, 6.35f, 8.0f, 20.0f ); // wider than the widest allowed: held at it, still at the peak - a broad patch the chain's budget dims as a whole
    probeLedRender( &v6, &huge, &style, 1.0f, &frame );
    b = brightest( &frame );
    lit = 0;
    for ( int i = 0; i < frame.count; i++ )
        lit += frame.target[ i ] > 0.02f;
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, style.peak, frame.target[ b ] );
    TEST_ASSERT_TRUE( lit > 60 );
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
        float shown;
        uint8_t sr, sg, sb;
        probeLedShown( &frame, i, &shown, &sr, &sg, &sb ); // the ring is a layer over the LEDs (2026-09-28)
        ringLit += shown > 0.05f && ( v6.row[ i ] == 11 || v6.row[ i ] == 17 );
        inside += shown > 0.3f * style.peak && ( v6.row[ i ] == 12 || v6.row[ i ] == 16 ) && v6.hole[ i ] == 3; // where the ring passed a frame ago
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
// "white on board" off (2026-09-28): a wheel scheme's colour at every
// height, the point on the board included - no white plateau.
void test_white_on_board_can_be_turned_off( void ) {
    style.scheme = PROBELED_SCHEME_HEIGHT;
    style.hueStartDeg = 0.0f; // red at the bottom of the wheel
    ProbeLedInput down = at( 14.0f, 6.35f, 0.05f, 0.2f );
    down.heightMm = 0.0f;
    probeLedRender( &v6, &down, &style, 1.0f, &frame );
    int i = find( &v6, 14, 3 );
    TEST_ASSERT_TRUE( frame.r[ i ] > 240 && frame.g[ i ] > 240 && frame.b[ i ] > 240 ); // white on the board
    style.whiteOnBoard = false;
    probeLedRender( &v6, &down, &style, 1.0f, &frame );
    TEST_ASSERT_TRUE( frame.r[ i ] > 240 && frame.g[ i ] < 40 && frame.b[ i ] < 40 ); // the wheel's red, on the board
}

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
    style.brightAmount = -1.0f;
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
    style.brightAmount = -0.5f;
    in.tiltDeg = 60.0f; // the far end of the tilt scale
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, 0.5f * style.peak, frame.target[ brightest( &frame ) ] );
    style.brightBy = PROBELED_DATA_HEIGHT;
    style.brightAmount = -1.0f;
    in.heightMm = style.liftFullMm * 0.5f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, 0.5f * style.peak, frame.target[ brightest( &frame ) ] );
    style.brightBy = PROBELED_DATA_SPEED;
    in.heightMm = 0.0f;
    in.speedMmS = 400.0f; // past the scale's end: all of the amount
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_TRUE( frame.target[ brightest( &frame ) ] < 0.02f );
    // Signed (2026-09-28): a positive amount brightens toward the far end -
    // +1 by speed doubles a half peak when fast - and never past the full peak.
    style.brightBy = PROBELED_DATA_SPEED;
    style.brightAmount = 1.0f;
    style.peak = 0.5f;
    in.speedMmS = 0.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, 0.5f, frame.target[ brightest( &frame ) ] );
    in.speedMmS = PROBELED_SPEED_FULL_MM_S;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, 1.0f, frame.target[ brightest( &frame ) ] );
    style.peak = 1.0f;
    probeLedRender( &v6, &in, &style, 0.02f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, 1.0f, frame.target[ brightest( &frame ) ] ); // full is full
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

// A far probe is drawn like any other (2026-09-27; a dim purple glow of its
// own until then): the same colour by height, the same peak, as wide as its
// bar - held at the widest allowed - so a far fix at the peak is a broad
// patch the chain's current budget dims as a whole. Nothing: dark.
void test_far_probe_is_drawn_like_any_other( void ) {
    style.fullPeak = true;
    ProbeLedInput in = at( 20.0f, 0.0f, 6.0f, 12.0f );
    in.heightMm = 40.0f;
    in.confidence = 0.1f;
    in.state = PROBELED_ROUGH;
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    ProbeLedFrame rough = frame;
    probeLedClear( &frame, v6.count );
    in.state = PROBELED_TRACKING;
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    int b = brightest( &frame );
    TEST_ASSERT_EQUAL( b, brightest( &rough ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, frame.target[ b ], rough.target[ b ] );
    TEST_ASSERT_EQUAL( frame.r[ b ], rough.r[ b ] );
    TEST_ASSERT_EQUAL( frame.b[ b ], rough.b[ b ] );
    TEST_ASSERT_TRUE( rough.target[ b ] > 0.5f * style.peak ); // bright, not a faint "about here"
    TEST_ASSERT_TRUE( rough.b[ b ] > 200 && rough.r[ b ] < 120 );  // classic, lifted: blue
    ProbeLedInput none = { };
    probeLedRender( &v6, &none, &style, 1.0f, &frame );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.0f, total( &frame ) );
}

// In pointed mode a tail runs from the cursor BACK toward the point for
// tailLength of the way (Kevin, 2026-09-26: the pattern from the point to
// where it aims was too long - now it hugs the pointing end), brightest at
// the cursor in the cursor's own colour, fading to the tail hue at its far
// end; with tailLength 1 it reaches the point itself.
void test_pointed_tail_points_back_at_the_tip( void ) {
    ProbeLedInput in = at( 10.0f, 6.35f, 0.1f, 0.3f );
    in.heightMm = 12.0f;
    in.haveUnder = true;
    in.underAlong = 18.0f; // the point is eight rows away from where it aims
    in.underAcrossMm = 6.35f;
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.5f, style.tailLength );
    int cursor = find( &v6, 10, 3 ), near = find( &v6, 11, 3 ), mid = find( &v6, 13, 3 ), end = find( &v6, 14, 3 ), past = find( &v6, 15, 3 ), under = find( &v6, 18, 3 );
    // Off by default (2026-09-27): the spot alone, nothing along the lean.
    TEST_ASSERT_FALSE( style.tail );
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    TEST_ASSERT_TRUE( frame.target[ cursor ] > 0.5f );
    TEST_ASSERT_TRUE( frame.target[ near ] < 0.05f && frame.target[ mid ] < 0.02f );
    style.tail = true;
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    TEST_ASSERT_TRUE( frame.target[ cursor ] > 0.5f );
    TEST_ASSERT_TRUE( frame.target[ near ] > 0.1f && frame.target[ near ] < frame.target[ cursor ] ); // the tail's bright end, next to the cursor
    TEST_ASSERT_TRUE( frame.target[ mid ] > 0.03f && frame.target[ mid ] < frame.target[ near ] );   // fading toward its far end...
    TEST_ASSERT_TRUE( frame.target[ end ] > 0.03f );                                                  // ...halfway to the point
    TEST_ASSERT_TRUE( frame.target[ past ] < 0.02f && frame.target[ under ] < 0.02f );                // beyond it, and the point itself: dark
    // Lifted 12 mm the cursor is mostly the "lift" colour (classic: blue),
    // the tail's near end takes it, and its far end is warmer (the tail hue, amber).
    TEST_ASSERT_TRUE( frame.b[ cursor ] > 200 && frame.r[ cursor ] < 120 );
    TEST_ASSERT_TRUE( frame.b[ near ] > 150 );
    TEST_ASSERT_TRUE( frame.r[ mid ] > frame.r[ near ] && frame.r[ mid ] > 180 );
    // The whole way: the point is marked.
    style.tailLength = 1.0f;
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    TEST_ASSERT_TRUE( frame.target[ under ] > 0.05f );
    // Brighter: the tail's near end follows the lever.
    style.tailLength = 0.5f;
    style.tailBright = 1.0f;
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    TEST_ASSERT_TRUE( frame.target[ near ] > 0.5f );
    // A wide, dim cursor (a coasting track's bar) keeps a dimmer tail: the
    // tail follows what the cursor actually shows, not the peak lever
    // (the review, 2026-09-26: the tail's near end outshone the cursor a
    // row over, and the brightest LED sat toward the tip).
    style.tailBright = 0.5f;
    ProbeLedInput wide = in;
    wide.sigmaRows = 2.5f;
    wide.sigmaAcrossMm = 6.0f;
    probeLedRender( &v6, &wide, &style, 1.0f, &frame );
    TEST_ASSERT_TRUE( frame.target[ near ] < frame.target[ cursor ] );
    TEST_ASSERT_EQUAL( cursor, brightest( &frame ) );
    // A long tail is a line, not beads: with the point twenty rows away and
    // the tail the whole way, every row of it is lit (the review: twelve
    // steps over twenty rows left dark rows between the bells).
    style.tailLength = 1.0f;
    in.underAlong = 30.0f;
    probeLedRender( &v6, &in, &style, 1.0f, &frame );
    for ( int row = 11; row <= 29; row++ ) {
        TEST_ASSERT_TRUE_MESSAGE( frame.target[ find( &v6, row, 3 ) ] > 0.03f, "a dark row in a long tail" );
    }
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

// A hand at writing speed moves a row and a half a frame (200 mm/s at 50
// Hz): the bell alone left the LED between two frames' cursors at 6 %, a
// dotted line ("gaps in the LEDs", Kevin, 2026-09-26). The cursor is swept
// from where it was to where it is, so a move is a stroke; a jump after a
// pause is a jump, and a cursor that has just appeared starts where it is.
void test_a_moving_cursor_sweeps_the_rows_between( void ) {
    ProbeLedInput a = at( 10.0f, 6.35f, 0.05f, 0.2f );
    probeLedRender( &v6, &a, &style, 1.0f, &frame );
    ProbeLedInput b = at( 11.6f, 6.35f, 0.05f, 0.2f );
    probeLedRender( &v6, &b, &style, 0.02f, &frame );
    ProbeLedInput c = at( 13.2f, 6.35f, 0.05f, 0.2f );
    probeLedRender( &v6, &c, &style, 0.02f, &frame );
    int passed = find( &v6, 12, 3 ), head = find( &v6, 13, 3 ), before = find( &v6, 11, 3 );
    TEST_ASSERT_TRUE( frame.target[ head ] > 0.5f );
    TEST_ASSERT_TRUE( frame.target[ passed ] > 0.3f );                  // the row the cursor passed over this frame is lit...
    TEST_ASSERT_TRUE( frame.target[ passed ] < frame.target[ head ] );  // ...a little less than the head: a comet, not a bar
    TEST_ASSERT_TRUE( frame.target[ before ] < frame.target[ passed ] ); // the last frame's stretch is not drawn again (it fades)
    // A jump after a pause is not a stroke...
    probeLedClear( &frame, v6.count );
    probeLedRender( &v6, &a, &style, 1.0f, &frame );
    ProbeLedInput far = at( 25.0f, 6.35f, 0.05f, 0.2f );
    probeLedRender( &v6, &far, &style, 0.5f, &frame );
    TEST_ASSERT_TRUE( frame.target[ find( &v6, 17, 3 ) ] < 0.02f );
    // ...nor is where a cursor first appears.
    probeLedClear( &frame, v6.count );
    ProbeLedInput none = { };
    probeLedRender( &v6, &none, &style, 1.0f, &frame );
    probeLedRender( &v6, &far, &style, 0.02f, &frame );
    TEST_ASSERT_TRUE( frame.target[ find( &v6, 17, 3 ) ] < 0.02f );
}

// The spot's size: "spot" scales the narrowest bell, and "spot by height"
// widens it as the point lifts - a flashlight's cone, the peak kept (Kevin,
// 2026-09-26: "we should be able to adjust spot size by height"). At 0 the
// height does nothing, as before.
void test_spot_widens_with_height( void ) {
    style.fullPeak = true;
    ProbeLedInput low = at( 14.0f, 6.35f, 0.05f, 0.2f );
    low.heightMm = 0.0f;
    probeLedRender( &v6, &low, &style, 1.0f, &frame );
    int litLow = 0;
    for ( int i = 0; i < frame.count; i++ )
        litLow += frame.target[ i ] > 0.1f;
    ProbeLedInput high = low;
    high.heightMm = PROBELED_SPOT_HEIGHT_MM; // the lever counts the lift in these (2026-09-28: over the colours page's height scale until then)
    probeLedRender( &v6, &high, &style, 1.0f, &frame );
    int litHigh = 0;
    for ( int i = 0; i < frame.count; i++ )
        litHigh += frame.target[ i ] > 0.1f;
    TEST_ASSERT_EQUAL( litLow, litHigh ); // the lever at 0: the height changes nothing
    style.spotByHeight = 2.0f;           // three times as wide 10 mm up
    probeLedRender( &v6, &high, &style, 1.0f, &frame );
    litHigh = 0;
    for ( int i = 0; i < frame.count; i++ )
        litHigh += frame.target[ i ] > 0.1f;
    TEST_ASSERT_TRUE( litHigh > 3 * litLow );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, style.peak, frame.target[ brightest( &frame ) ] ); // still at the peak: a wider spot, not a dimmer one
    probeLedRender( &v6, &low, &style, 1.0f, &frame ); // on the board: the narrow spot again
    int litBack = 0;
    for ( int i = 0; i < frame.count; i++ )
        litBack += frame.target[ i ] > 0.1f;
    TEST_ASSERT_EQUAL( litLow, litBack );
    style.spotByHeight = 0.0f;
    style.spot = 2.0f; // the base size itself
    probeLedRender( &v6, &low, &style, 1.0f, &frame );
    int litBig = 0;
    for ( int i = 0; i < frame.count; i++ )
        litBig += frame.target[ i ] > 0.1f;
    TEST_ASSERT_TRUE( litBig > litLow );
    // Both levers at their ends, lifted: a spot grown past the widest bell
    // allowed is held at it, never brighter than the peak.
    style.spot = 4.0f;
    style.spotByHeight = 4.0f;
    probeLedRender( &v6, &high, &style, 1.0f, &frame );
    TEST_ASSERT_TRUE( frame.target[ brightest( &frame ) ] <= style.peak + 0.001f );
    // The fix's error bar widens the spot by "error width" (2026-09-28): a
    // bar a row wide is a patch at 1, a pin at 0 whatever the bar says, and
    // wider at 2.
    style.spot = 1.0f;
    style.spotByHeight = 0.0f;
    ProbeLedInput wide = at( 14.0f, 6.35f, 1.0f, 2.5f );
    probeLedRender( &v6, &wide, &style, 1.0f, &frame );
    int litWide = 0;
    for ( int i = 0; i < frame.count; i++ )
        litWide += frame.target[ i ] > 0.1f;
    style.errorWidth = 0.0f;
    probeLedRender( &v6, &wide, &style, 1.0f, &frame );
    int litPin = 0;
    for ( int i = 0; i < frame.count; i++ )
        litPin += frame.target[ i ] > 0.1f;
    style.errorWidth = 2.0f;
    probeLedRender( &v6, &wide, &style, 1.0f, &frame );
    int litWider = 0;
    for ( int i = 0; i < frame.count; i++ )
        litWider += frame.target[ i ] > 0.1f;
    TEST_ASSERT_TRUE( litWide > 8 );
    TEST_ASSERT_TRUE( litPin <= 3 );
    TEST_ASSERT_TRUE( litWider > litWide );
    style.errorWidth = 1.0f;
}

// At least one pixel (2026-09-28): a pin of a spot between two holes lights
// the nearer one at the bell's peak, never nothing; and "falloff" shapes the
// bell - higher a flatter top and a sharper edge, lower a peak with a skirt.
void test_one_pixel_at_least_and_the_falloff( void ) {
    style.fullPeak = true;
    style.spot = 0.1f;
    ProbeLedInput between = at( 14.4f, 6.35f, 0.05f, 0.2f ); // 0.4 rows from row 14's hole: at sigma 0.033 rows the bell gives it nothing
    probeLedRender( &v6, &between, &style, 1.0f, &frame );
    int nearest = find( &v6, 14, 3 );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, style.peak, frame.target[ nearest ] );
    TEST_ASSERT_TRUE( frame.target[ find( &v6, 15, 3 ) ] < 0.02f );
    // The falloff, on a bell two rows wide: the LED a row out (half a
    // sigma) brighter with a flat top, the one three rows out (1.5 sigma)
    // dimmer with a sharp edge; the centre the peak either way. (At exactly
    // one sigma every falloff gives the same, q^p with q = 1.)
    style.spot = 1.0f;
    ProbeLedInput wide = at( 14.0f, 6.35f, 2.0f, 5.0f );
    probeLedRender( &v6, &wide, &style, 1.0f, &frame );
    float halfSigma = frame.target[ find( &v6, 15, 3 ) ], edge = frame.target[ find( &v6, 17, 3 ) ];
    style.falloff = 3.0f;
    probeLedRender( &v6, &wide, &style, 1.0f, &frame );
    TEST_ASSERT_TRUE( frame.target[ find( &v6, 15, 3 ) ] > halfSigma );
    TEST_ASSERT_TRUE( frame.target[ find( &v6, 17, 3 ) ] < edge );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, style.peak, frame.target[ find( &v6, 14, 3 ) ] );
    style.falloff = 0.5f;
    probeLedRender( &v6, &wide, &style, 1.0f, &frame );
    TEST_ASSERT_TRUE( frame.target[ find( &v6, 15, 3 ) ] < halfSigma );
    TEST_ASSERT_TRUE( frame.target[ find( &v6, 20, 3 ) ] > 0.02f ); // the skirt reaches three sigma
}

// The ring passes over what the cursor lit and leaves it as it was (Kevin,
// 2026-09-28: "when the touch ring runs, it clears the leds under it"): the
// comet behind a cursor that moved on decays the same with the ring as
// without, since the ring is a layer over the LEDs, not a change to them.
void test_the_ring_leaves_what_was_under_it( void ) {
    float withRing[ 2 ], without[ 2 ];
    for ( int run = 0; run < 2; run++ ) {
        probeLedClear( &frame, v6.count );
        style.touchRing = run == 0;
        ProbeLedInput lifted = at( 17.0f, 6.35f, 0.05f, 0.2f ); // lit a second at row 17, lifted
        lifted.heightMm = 10.0f;
        probeLedRender( &v6, &lifted, &style, 1.0f, &frame );
        ProbeLedInput down = at( 20.0f, 6.35f, 0.05f, 0.2f ); // lands three rows on: a ring spreads from row 20, over row 17 at 0.08 s
        down.heightMm = 0.0f;
        int comet = find( &v6, 17, 3 );
        for ( int k = 0; k < 5; k++ )
            probeLedRender( &v6, &down, &style, 0.02f, &frame ); // 0.1 s: the ring has passed row 17
        ( run == 0 ? withRing : without )[ 0 ] = frame.level[ comet ];
        for ( int k = 0; k < 4; k++ )
            probeLedRender( &v6, &down, &style, 0.02f, &frame ); // 0.18 s: the ring is over
        ( run == 0 ? withRing : without )[ 1 ] = frame.level[ comet ];
    }
    TEST_ASSERT_TRUE( without[ 0 ] > 0.3f ); // the comet is still glowing at 0.1 s...
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, without[ 0 ], withRing[ 0 ] ); // ...the same under the ring
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, without[ 1 ], withRing[ 1 ] ); // ...and after it
    style.touchRing = false;
}

// The touch ring rings when the point lands; sliding on to another hole
// while down rings again only with "ring repeat" (2026-09-27: it always did).
void test_touch_ring_repeats_only_when_asked( void ) {
    style.touchRing = true;
    ProbeLedInput lifted = at( 14.0f, 6.35f, 0.05f, 0.2f );
    lifted.heightMm = 10.0f;
    probeLedRender( &v6, &lifted, &style, 0.02f, &frame );
    ProbeLedInput down = lifted;
    down.heightMm = 0.0f;
    probeLedRender( &v6, &down, &style, 0.02f, &frame );
    TEST_ASSERT_TRUE( frame.ringAgeS >= 0.0f ); // landed: a ring
    for ( int k = 0; k < 12; k++ )
        probeLedRender( &v6, &down, &style, 0.02f, &frame ); // the ring over
    TEST_ASSERT_TRUE( frame.ringAgeS < 0.0f );
    ProbeLedInput slid = down;
    slid.along = 16.0f;
    probeLedRender( &v6, &slid, &style, 0.02f, &frame );
    TEST_ASSERT_TRUE( frame.ringAgeS < 0.0f ); // slid two rows on, still down: no ring
    style.ringRepeat = true;
    ProbeLedInput slid2 = down;
    slid2.along = 18.0f;
    probeLedRender( &v6, &slid2, &style, 0.02f, &frame );
    TEST_ASSERT_TRUE( frame.ringAgeS >= 0.0f ); // asked to: it rings again
}

int main( int argc, char** argv ) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN( );
    RUN_TEST( test_v6_layout );
    RUN_TEST( test_v5_layout_and_pixels );
    RUN_TEST( test_sharp_fix_lights_one_hole );
    RUN_TEST( test_vague_fix_is_wide_and_dim_with_the_same_light );
    RUN_TEST( test_vague_fix_at_full_peak_keeps_the_peak );
    RUN_TEST( test_looks );
    RUN_TEST( test_white_on_board_can_be_turned_off );
    RUN_TEST( test_hue_scale_turns_and_start );
    RUN_TEST( test_aim_scheme_colours_by_the_lean );
    RUN_TEST( test_brightness_follows_the_chosen_data );
    RUN_TEST( test_sparkle_density_follows_the_chosen_data );
    RUN_TEST( test_far_probe_is_drawn_like_any_other );
    RUN_TEST( test_pointed_tail_points_back_at_the_tip );
    RUN_TEST( test_leds_fade_rather_than_snap );
    RUN_TEST( test_a_moving_cursor_sweeps_the_rows_between );
    RUN_TEST( test_spot_widens_with_height );
    RUN_TEST( test_one_pixel_at_least_and_the_falloff );
    RUN_TEST( test_touch_ring_repeats_only_when_asked );
    RUN_TEST( test_the_ring_leaves_what_was_under_it );
    return UNITY_END( );
}
