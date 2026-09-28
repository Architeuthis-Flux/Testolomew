// SPDX-License-Identifier: MIT
#include "ProbeLeds.h"

#include <math.h>

#define PITCH_MM 2.54f
#define RAIL_GROUP 5 // V5 rail LEDs per group

// ---- layouts ---------------------------------------------------------------

static void addLed( LedLayout* l, float along, float acrossMm, uint8_t kind, int row, int hole ) {
    if ( l->count >= PROBELED_MAX ) {
        return;
    }
    int i = l->count++;
    l->along[ i ] = along;
    l->acrossMm[ i ] = acrossMm;
    l->kind[ i ] = kind;
    l->row[ i ] = (int16_t)row;
    l->hole[ i ] = (int8_t)hole;
}

// Both halves' holes, then the rails: side +1 is the rows 1-30 half.
static void addHoles( LedLayout* l, int holes, float innerMm ) {
    for ( int side = 0; side < 2; side++ ) {
        float sign = side == 0 ? 1.0f : -1.0f;
        for ( int r = 1; r <= PROBELED_ROWS; r++ ) {
            for ( int h = 1; h <= holes; h++ ) {
                addLed( l, (float)r, sign * ( innerMm + ( h - 1 ) * PITCH_MM ), PROBELED_HOLE, side == 0 ? r : r + PROBELED_ROWS, h );
            }
        }
    }
}

void ledLayoutV6( LedLayout* layout ) {
    layout->count = 0;
    addHoles( layout, PROBELED_V6_HOLES, PROBELED_V6_INNER_MM );
    for ( int side = 0; side < 2; side++ ) {
        float sign = side == 0 ? 1.0f : -1.0f;
        for ( int rail = 0; rail < 2; rail++ ) {
            for ( int r = 1; r <= PROBELED_ROWS; r++ ) {
                addLed( layout, (float)r, sign * ( PROBELED_V6_RAIL_MM + rail * PITCH_MM ), PROBELED_RAIL, 0, 0 );
            }
        }
    }
}

// V5 rails: 25 LEDs in five groups of five with one empty position between
// groups, so group g LED i sits at along = 1 + 6 g + i.
void ledLayoutV5( LedLayout* layout ) {
    layout->count = 0;
    addHoles( layout, PROBELED_V5_HOLES, PROBELED_V5_INNER_MM );
    // railsToPixelMap order, top pair (the rows 1-30 side) outer then inner,
    // bottom pair inner then outer - the board's reference designators in
    // chain order (ProbeLeds.h); rail LEDs sit half a pitch along from the
    // rows, in groups of five with one position skipped between groups.
    for ( int rail = 0; rail < 4; rail++ ) {
        float sign = rail < 2 ? 1.0f : -1.0f;
        bool outer = rail == 0 || rail == 3;
        for ( int n = 0; n < 25; n++ ) {
            float along = PROBELED_V5_RAIL_ALONG0 + ( n / RAIL_GROUP ) * ( RAIL_GROUP + 1 ) + ( n % RAIL_GROUP );
            addLed( layout, along, sign * ( outer ? PROBELED_V5_RAIL_OUTER_MM : PROBELED_V5_RAIL_INNER_MM ), PROBELED_RAIL, 0, 0 );
        }
    }
}

int ledLayoutV5RailPixel( const LedLayout* layout, int i ) {
    if ( i < 0 || i >= layout->count || layout->kind[ i ] != PROBELED_RAIL ) {
        return -1;
    }
    int railIndex = i - 2 * PROBELED_ROWS * PROBELED_V5_HOLES;
    return railIndex >= 0 && railIndex < 100 ? railIndex : -1;
}

int ledLayoutV5Pixel( const LedLayout* layout, int i ) {
    if ( i < 0 || i >= layout->count ) {
        return -1;
    }
    if ( layout->kind[ i ] == PROBELED_HOLE ) {
        int row = layout->row[ i ];
        int column = PROBELED_V5_COLUMN0_IS_INNER ? layout->hole[ i ] - 1 : PROBELED_V5_HOLES - layout->hole[ i ];
        if ( row > PROBELED_ROWS ) {
            column = PROBELED_V5_HOLES - 1 - column; // JumperlOS reverses the bottom half
        }
        return ( row - 1 ) * PROBELED_V5_HOLES + column;
    }
    // Rails come after the 300 holes in layout order: rail r LED n = 300 + 25 r + n.
    int railIndex = i - 2 * PROBELED_ROWS * PROBELED_V5_HOLES;
    return railIndex >= 0 && railIndex < 100 ? 300 + railIndex : -1;
}

// ---- rendering -------------------------------------------------------------

const char* const probeLedSchemeNames[ PROBELED_SCHEME_COUNT ] = { "classic", "height", "sure", "aim", "rainbow" };
const char* const probeLedDataNames[ PROBELED_DATA_COUNT ] = { "none", "unsure", "height", "tilt", "speed" };

void probeLedHue( float hueDeg, float brightness, uint8_t* r, uint8_t* g, uint8_t* b ) {
    while ( hueDeg < 0.0f )
        hueDeg += 360.0f;
    while ( hueDeg >= 360.0f )
        hueDeg -= 360.0f;
    float h = hueDeg / 60.0f;
    int sector = (int)h;
    float f = h - sector;
    float q = 1.0f - f;
    float rr = 0, gg = 0, bb = 0;
    switch ( sector ) {
    case 0:
        rr = 1;
        gg = f;
        break;
    case 1:
        rr = q;
        gg = 1;
        break;
    case 2:
        gg = 1;
        bb = f;
        break;
    case 3:
        gg = q;
        bb = 1;
        break;
    case 4:
        rr = f;
        bb = 1;
        break;
    default:
        rr = 1;
        bb = q;
        break;
    }
    *r = (uint8_t)( 255.0f * brightness * rr + 0.5f );
    *g = (uint8_t)( 255.0f * brightness * gg + 0.5f );
    *b = (uint8_t)( 255.0f * brightness * bb + 0.5f );
}

static uint32_t xorshift( uint32_t* state ) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

void probeLedDefaultStyle( ProbeLedStyle* s ) {
    s->touchR = 255;
    s->touchG = 255;
    s->touchB = 255; // white on the board
    s->liftR = 40;
    s->liftG = 120;
    s->liftB = 255; // blue in the air
    s->tail = false;       // no tail unless asked
    // The defaults are the bench's settings as they stood on 2026-09-27 (Kevin:
    // "grab the settings off the board right now and set those all as
    // defaults"); what they were before is noted where it differs. The tests
    // pin the look they were written against (test_probeleds setUp).
    s->tailHueDeg = 30.0f; // (25)
    s->tailLength = 1.0f;  // the whole way back to the point (0.5)
    s->tailBright = 1.0f;  // (0.5)
    s->peak = 1.0f;
    s->minSigmaRows = 0.33f;
    s->minSigmaAcrossMm = 0.9f;
    s->spot = 0.9f;         // (1.0)
    s->spotByHeight = 3.5f; // (0)
    s->errorWidth = 1.0f;   // the bar as it is
    s->falloff = 1.0f;      // a Gaussian
    s->liftFullMm = 60.0f; // (15)
    s->attackS = 0.02f;
    s->decayS = 0.35f;                  // (0.12)
    s->scheme = PROBELED_SCHEME_HEIGHT; // (classic)
    s->hueTurns = 1.05f;                // (0.667: two thirds of the wheel, red to blue)
    s->hueStartDeg = 240.0f;            // (0)
    s->colourByMm = 2.5f;               // (6)
    s->whiteOnBoard = true;
    s->brightBy = PROBELED_DATA_NONE;   // (sure)
    s->brightAmount = 1.0f;             // signed since 2026-09-28: + brightens toward the data's far end, - dims (0.5 dimmed a coin-toss row by half until then; by "none" it is inert)
    s->sparkleBy = PROBELED_DATA_NONE;  // (height)
    s->fullPeak = true;
    s->bloom = 0.0f;
    s->sparkle = 0.0f;
    s->pulse = 0.0f;
    s->touchRing = true; // (off)
    s->ringRepeat = false;
}

void probeLedClear( ProbeLedFrame* frame, int count ) {
    frame->count = count;
    for ( int i = 0; i < count; i++ ) {
        frame->level[ i ] = 0.0f;
        frame->target[ i ] = 0.0f;
        frame->r[ i ] = frame->g[ i ] = frame->b[ i ] = 0;
    }
    frame->timeS = 0.0f;
    frame->rng = 0x9E3779B9u;
    frame->lastHeightMm = 0.0f;
    frame->wasLifted = false;
    frame->ringAgeS = -1.0f;
    frame->ringHole = -1;
    for ( int i = 0; i < PROBELED_MAX; i++ )
        frame->ring[ i ] = 0.0f;
    frame->ringR = frame->ringG = frame->ringB = 0;
    frame->ringAlong = frame->ringAcrossMm = 0.0f;
    frame->haveLast = false;
    frame->lastAlong = frame->lastAcrossMm = frame->lastTimeS = 0.0f;
}

static float clamp01( float v ) {
    return v < 0.0f ? 0.0f : ( v > 1.0f ? 1.0f : v );
}

static void blendColour( uint8_t* c, uint8_t a, uint8_t b, float t ) {
    *c = (uint8_t)( a + ( b - a ) * t + 0.5f );
}

// A bell of the given widths centred on (along, across), with total light
// held constant: the peak is scaled by (minimum area / area), so a bell twice
// as wide is a quarter as bright - but never under `floor` (the peak itself
// with fullPeak). Held at the widest allowed (PROBELED_MAX_SIGMA_*). Returns
// the amplitude it drew with (what the bell's centre shows: the tail
// follows it).
static float bellPower = 1.0f; // the style's falloff, set per frame: exp( -q^power / 2 ), q the squared distance in sigmas

static float splat( const LedLayout* layout, ProbeLedFrame* frame, float along, float acrossMm, float sigmaRows, float sigmaAcrossMm, float peak, float floor, float minSigmaRows,
                    float minSigmaAcrossMm, float maxSigmaRows, float maxSigmaAcrossMm, uint8_t r, uint8_t g, uint8_t b ) {
    if ( sigmaRows < minSigmaRows )
        sigmaRows = minSigmaRows;
    if ( sigmaAcrossMm < minSigmaAcrossMm )
        sigmaAcrossMm = minSigmaAcrossMm;
    if ( sigmaRows > maxSigmaRows )
        sigmaRows = maxSigmaRows;
    if ( sigmaAcrossMm > maxSigmaAcrossMm )
        sigmaAcrossMm = maxSigmaAcrossMm;
    // A minimum grown past the widest (the spot levers at their ends) is the
    // widest: the ratio below never takes a bell over the peak.
    if ( minSigmaRows > maxSigmaRows )
        minSigmaRows = maxSigmaRows;
    if ( minSigmaAcrossMm > maxSigmaAcrossMm )
        minSigmaAcrossMm = maxSigmaAcrossMm;
    float amplitude = peak * ( minSigmaRows * minSigmaAcrossMm ) / ( sigmaRows * sigmaAcrossMm );
    if ( amplitude < floor )
        amplitude = floor;
    // Reciprocals once, and the along test alone first: most LEDs fail it,
    // and this runs 400 x (1 + the tail's steps) times a frame.
    float invRows = 1.0f / sigmaRows, invAcross = 1.0f / sigmaAcrossMm;
    for ( int i = 0; i < layout->count; i++ ) {
        float da = ( layout->along[ i ] - along ) * invRows;
        float q = da * da;
        if ( q > 16.0f )
            continue; // beyond 4 sigma along the rows
        float dc = ( layout->acrossMm[ i ] - acrossMm ) * invAcross;
        q += dc * dc;
        if ( q > 16.0f )
            continue; // beyond 4 sigma
        float v = amplitude * expf( -0.5f * ( bellPower == 1.0f ? q : powf( q, bellPower ) ) );
        if ( v <= frame->target[ i ] )
            continue;
        // The brighter of the two marks colours the LED.
        frame->target[ i ] = v;
        frame->r[ i ] = r;
        frame->g[ i ] = g;
        frame->b[ i ] = b;
    }
    return amplitude;
}

// How much of the scheme's colour the cursor shows at this height: none
// (white) with the point on the board, all of it from the style's
// colourByMm up. The classic scheme has its own, longer ramp.
static float colourByHeight( const ProbeLedStyle* style, float heightMm ) {
    if ( !style->whiteOnBoard )
        return 1.0f; // the scheme's colour at every height
    float by = style->colourByMm > PROBELED_WHITE_BELOW_MM + 0.1f ? style->colourByMm : PROBELED_WHITE_BELOW_MM + 0.1f;
    return clamp01( ( heightMm - PROBELED_WHITE_BELOW_MM ) / ( by - PROBELED_WHITE_BELOW_MM ) );
}

// One of the data a look can follow, 0 at its near end, 1 at its far end.
static float dataOf( const ProbeLedStyle* style, const ProbeLedInput* in, int which ) {
    switch ( which ) {
    case PROBELED_DATA_SURE:
        return 1.0f - clamp01( in->confidence );
    case PROBELED_DATA_HEIGHT:
        return clamp01( in->heightMm / style->liftFullMm );
    case PROBELED_DATA_TILT:
        return clamp01( in->tiltDeg / PROBELED_TILT_FULL_DEG );
    case PROBELED_DATA_SPEED:
        return clamp01( in->speedMmS / PROBELED_SPEED_FULL_MM_S );
    default:
        return 0.0f;
    }
}

static void whiteTo( uint8_t* r, uint8_t* g, uint8_t* b, float amount ) {
    blendColour( r, 255, *r, amount );
    blendColour( g, 255, *g, amount );
    blendColour( b, 255, *b, amount );
}

// The cursor's colour under the scheme, for this frame's state: white on
// the board in every scheme, the scheme's colour as the point lifts.
static void schemeColour( const ProbeLedStyle* style, const ProbeLedInput* in, float timeS, uint8_t* r, uint8_t* g, uint8_t* b ) {
    float wheel = style->hueTurns * 360.0f; // the wheel over the scheme's scale (probeLedHue wraps: over a turn is several rainbows)
    switch ( style->scheme ) {
    case PROBELED_SCHEME_HEIGHT:
        probeLedHue( style->hueStartDeg + wheel * clamp01( in->heightMm / style->liftFullMm ), 1.0f, r, g, b ); // above the scale the wheel's end holds (unclamped, a probe 20-45 mm up read as any height: the review, 2026-09-25)
        whiteTo( r, g, b, colourByHeight( style, in->heightMm ) );
        break;
    case PROBELED_SCHEME_SURE:
        probeLedHue( style->hueStartDeg + wheel * clamp01( in->confidence ), 1.0f, r, g, b ); // the start of the wheel unsure, its end sure
        whiteTo( r, g, b, colourByHeight( style, in->heightMm ) );
        break;
    case PROBELED_SCHEME_AIM:
        // Which way the probe leans, once round the wheel (the turns lever
        // does not apply: at anything but a whole turn the compass had a
        // seam at +x, red one frame and blue the next as the lean's y
        // jittered about zero); white standing straight.
        probeLedHue( style->hueStartDeg + in->aimDeg, 1.0f, r, g, b );
        whiteTo( r, g, b, colourByHeight( style, in->heightMm ) * clamp01( in->tiltDeg / PROBELED_AIM_FULL_DEG ) );
        break;
    case PROBELED_SCHEME_RAINBOW:
        probeLedHue( style->hueStartDeg + timeS * 90.0f, 1.0f, r, g, b ); // the per-LED spread is applied after the splats
        whiteTo( r, g, b, colourByHeight( style, in->heightMm ) );
        break;
    default: {
        // Classic: white on the board (and the same white plateau as the
        // others, PROBELED_WHITE_BELOW_MM), then its own long ramp to blue.
        float ramp = clamp01( ( in->heightMm - PROBELED_WHITE_BELOW_MM ) / ( style->liftFullMm - PROBELED_WHITE_BELOW_MM ) );
        blendColour( r, style->touchR, style->liftR, ramp );
        blendColour( g, style->touchG, style->liftG, ramp );
        blendColour( b, style->touchB, style->liftB, ramp );
        break;
    }
    }
}

void probeLedCursorColour( const ProbeLedStyle* style, const ProbeLedInput* in, float timeS, uint8_t* r, uint8_t* g, uint8_t* b ) {
    schemeColour( style, in, timeS, r, g, b );
}

void probeLedPaintClear( ProbeLedPaint* paint ) {
    for ( int i = 0; i < PROBELED_MAX; i++ ) {
        paint->level[ i ] = 0.0f;
        paint->r[ i ] = paint->g[ i ] = paint->b[ i ] = 0;
    }
}

int ledLayoutNearest( const LedLayout* layout, float along, float acrossMm, float withinRows ) {
    int best = -1;
    float bestQ = withinRows * withinRows;
    for ( int i = 0; i < layout->count; i++ ) {
        float da = layout->along[ i ] - along;
        float dc = ( layout->acrossMm[ i ] - acrossMm ) / PITCH_MM;
        float q = da * da + dc * dc;
        if ( q < bestQ ) {
            bestQ = q;
            best = i;
        }
    }
    return best;
}

// The brush preview in place of the cursor (see ProbeLedBrush).
static void brushRing( const LedLayout* layout, const ProbeLedInput* in, const ProbeLedBrush* brush, ProbeLedFrame* frame ) {
    if ( in->state != PROBELED_TRACKING && in->state != PROBELED_COASTING )
        return;
    float along = in->haveUnder ? in->underAlong : in->along;
    float acrossMm = in->haveUnder ? in->underAcrossMm : in->acrossMm;
    int centre = ledLayoutNearest( layout, along, acrossMm, 1.5f );
    if ( centre < 0 )
        return;
    // The ring is the LEDs just outside what the brush would paint (the
    // brush reaches radius + 0.1 rows, as paintAt has it): for the one-LED
    // brush its four neighbours, a + with the centre dark; for a wider one
    // the outline a row beyond its edge.
    float reach = (float)brush->radiusRows + 0.1f;
    for ( int k = 0; k < layout->count; k++ ) {
        float da = layout->along[ k ] - layout->along[ centre ];
        float dc = ( layout->acrossMm[ k ] - layout->acrossMm[ centre ] ) / PITCH_MM;
        float d = sqrtf( da * da + dc * dc );
        if ( d <= reach || d > reach + 1.0f )
            continue;
        frame->target[ k ] = brush->level;
        frame->r[ k ] = brush->r;
        frame->g[ k ] = brush->g;
        frame->b[ k ] = brush->b;
    }
}

void probeLedRender( const LedLayout* layout, const ProbeLedInput* in, const ProbeLedStyle* style, float dtS, ProbeLedFrame* frame, const ProbeLedPaint* paint, const ProbeLedBrush* brush ) {
    frame->count = layout->count;
    frame->timeS += dtS;
    for ( int i = 0; i < layout->count; i++ ) {
        frame->target[ i ] = 0.0f;
    }
    bool asBrush = brush != nullptr && brush->active;
    bool sweep = frame->haveLast && frame->timeS - frame->lastTimeS <= PROBELED_SWEEP_S;
    frame->haveLast = false;

    // A touch ring starts when the point lands after having been lifted -
    // and, still down, when it moves on to another hole: a slide along the
    // board taps every hole it passes (2026-09-19).
    if ( in->state == PROBELED_TRACKING ) {
        float along = in->haveUnder ? in->underAlong : in->along;
        float acrossMm = in->haveUnder ? in->underAcrossMm : in->acrossMm;
        bool down = in->heightMm < PROBELED_TOUCH_MM;
        int hole = down ? ledLayoutNearest( layout, along, acrossMm, 1.0f ) : -1;
        if ( in->heightMm > PROBELED_LIFTED_MM ) {
            frame->wasLifted = true;
        }
        bool landed = down && frame->wasLifted;
        bool movedOn = false;
        if ( style->ringRepeat && down && !frame->wasLifted && hole >= 0 && hole != frame->ringHole ) {
            float da = along - frame->ringAlong, dc = ( acrossMm - frame->ringAcrossMm ) / PITCH_MM;
            bool far = frame->ringHole < 0 || da * da + dc * dc > PROBELED_RING_STEP_ROWS * PROBELED_RING_STEP_ROWS;
            bool rearmed = frame->ringAgeS < 0.0f || frame->ringAgeS > PROBELED_RING_REARM_S;
            movedOn = far && rearmed;
        }
        if ( landed || movedOn ) {
            frame->wasLifted = false;
            frame->ringHole = hole;
            frame->ringAlong = along;
            frame->ringAcrossMm = acrossMm;
            if ( style->touchRing ) {
                frame->ringAgeS = 0.0f;
            }
        }
        if ( !down ) {
            frame->ringHole = -1;
        }
    } else {
        frame->wasLifted = false;
        frame->ringHole = -1;
    }
    frame->lastHeightMm = in->heightMm;

    if ( asBrush ) {
        brushRing( layout, in, brush, frame );
    } else if ( in->state != PROBELED_NONE ) {
        uint8_t r, g, b;
        float peak = style->peak;
        if ( style->pulse > 0.0f ) {
            // Breathing: between the peak and (1 - pulse / 2) of it.
            float breath = 0.5f - 0.5f * cosf( 2.0f * (float)M_PI * frame->timeS / PROBELED_PULSE_PERIOD_S );
            peak *= 1.0f - 0.5f * style->pulse * breath;
        }
        // One mapping for every state, a far fix included (2026-09-27):
        // the scheme's colour by the height, dimmed by the chosen data
        // (unsure by default: a coin-toss row at half), and a coasting
        // track a little dimmer.
        schemeColour( style, in, frame->timeS, &r, &g, &b );
        // ...by the chosen data, SIGNED (2026-09-28): +1 doubles the peak at the
        // data's far end (a fast hand, a lifted probe), -1 takes it to nothing;
        // never past the full peak, which the LEDs cannot show.
        float amount = style->brightAmount < -1.0f ? -1.0f : ( style->brightAmount > 1.0f ? 1.0f : style->brightAmount );
        peak *= 1.0f + amount * dataOf( style, in, style->brightBy );
        if ( peak > 1.0f )
            peak = 1.0f;
        if ( in->state == PROBELED_COASTING )
            peak *= 0.7f;
        // The cursor keeps its total light (a wide bell is a dim one) down to a
        // floor an LED can still show; with fullPeak the floor IS the peak,
        // whatever the width. A far probe's bell at the peak is a hundred
        // LEDs: the chain's current budget (ProbeLedService::sendFrame) is
        // what keeps that off a breadboard's 5 V rail, by dimming the frame
        // as a whole - not a dimmer glow of its own (that went 2026-09-27).
        float floor = style->fullPeak ? peak : 0.08f;
        // The spot's size: "spot" scales the whole width - the narrowest bell
        // and the fix's error bar alike, so a small spot is small whatever
        // the bar says - and "spot by height" widens it by that much per
        // PROBELED_SPOT_HEIGHT_MM of the point's lift (a cone; the floor
        // above keeps the peak, so it is a wider spot and not a dimmer one).
        float lift = in->heightMm > 0.0f ? in->heightMm / PROBELED_SPOT_HEIGHT_MM : 0.0f;
        if ( lift > PROBELED_SPOT_HEIGHT_MAX )
            lift = PROBELED_SPOT_HEIGHT_MAX;
        float grow = ( style->spot > 0.0f ? style->spot : 1.0f ) * ( 1.0f + style->spotByHeight * lift );
        float minRows = style->minSigmaRows * grow, minAcross = style->minSigmaAcrossMm * grow;
        // ...and the fix's error bar, by "error width", is the other floor.
        float barRows = in->sigmaRows * style->errorWidth, barAcross = in->sigmaAcrossMm * style->errorWidth;
        float sigRows = barRows > minRows ? barRows : minRows, sigAcross = barAcross > minAcross ? barAcross : minAcross;
        bellPower = style->falloff > 0.1f ? style->falloff : 0.1f;
        float shown = splat( layout, frame, in->along, in->acrossMm, sigRows, sigAcross, peak, floor, minRows, minAcross, PROBELED_MAX_SIGMA_ROWS, PROBELED_MAX_SIGMA_ACROSS_MM, r, g, b );
        // At least one pixel: the LED nearest the cursor at the bell's peak,
        // so a pin between two holes lights the nearer one, never nothing.
        int nearest = ledLayoutNearest( layout, in->along, in->acrossMm, PROBELED_ONE_PIXEL_ROWS );
        if ( nearest >= 0 && frame->target[ nearest ] < shown ) {
            frame->target[ nearest ] = shown;
            frame->r[ nearest ] = r;
            frame->g[ nearest ] = g;
            frame->b[ nearest ] = b;
        }
        if ( style->bloom > 0.0f ) {
            // The halo: three times as wide, a fraction as bright, the same
            // colour; the max rule in splat() keeps it under the cursor.
            float wide = 3.0f;
            splat( layout, frame, in->along, in->acrossMm, sigRows * wide, sigAcross * wide, 0.3f * style->bloom * peak, 0.0f, minRows * wide, minAcross * wide,
                   PROBELED_MAX_SIGMA_ROWS * wide, PROBELED_MAX_SIGMA_ACROSS_MM * wide, r, g, b );
        }

        // The sweep: from where the cursor was last frame to where it is, at
        // half-row steps (the tail's pattern), the cursor's own width and
        // colour, full at the head and PROBELED_SWEEP_TAIL of it at the old
        // end - a stroke, not beads (PROBELED_SWEEP_S).
        if ( sweep ) {
            float dAlong = frame->lastAlong - in->along, dAcross = frame->lastAcrossMm - in->acrossMm;
            float length = sqrtf( dAlong * dAlong + dAcross * dAcross / ( PITCH_MM * PITCH_MM ) ); // in rows
            int steps = (int)( length * 2.0f ) + 1;
            if ( steps > 40 )
                steps = 40;
            for ( int k = 1; k < steps; k++ ) {
                float f = (float)k / ( steps - 1 ); // 0 at the head, 1 at the old end
                float fade = 1.0f - ( 1.0f - PROBELED_SWEEP_TAIL ) * f;
                splat( layout, frame, in->along + f * dAlong, in->acrossMm + f * dAcross, sigRows, sigAcross, peak * fade, floor * fade, minRows, minAcross,
                       PROBELED_MAX_SIGMA_ROWS, PROBELED_MAX_SIGMA_ACROSS_MM, r, g, b );
            }
        }
        frame->haveLast = true;
        frame->lastAlong = in->along;
        frame->lastAcrossMm = in->acrossMm;
        frame->lastTimeS = frame->timeS;

        // Where the point itself is, when that is somewhere else: a tail from
        // the cursor back toward it for tailLength of the way, brightest at
        // the cursor (tailBright of what the cursor's own bell shows - a
        // dim or a wide cursor gets a dim tail, never outshone) in the
        // cursor's colour, fading to the tail hue at its far end. A step
        // every half row, so a long tail is a line (twelve steps over twenty
        // rows were beads; the review, 2026-09-26).
        if ( style->tail && in->haveUnder && style->tailLength > 0.0f ) {
            float dAlong = ( in->underAlong - in->along ) * style->tailLength, dAcross = ( in->underAcrossMm - in->acrossMm ) * style->tailLength;
            float length = sqrtf( dAlong * dAlong + dAcross * dAcross / ( PITCH_MM * PITCH_MM ) ); // in rows
            int steps = (int)( length * 2.0f ) + 1;
            if ( steps > 40 )
                steps = 40; // twenty rows: the board's length at half-row steps
            uint8_t tr, tg, tb;
            probeLedHue( style->tailHueDeg, 1.0f, &tr, &tg, &tb );
            for ( int k = 1; k < steps; k++ ) {
                float f = (float)k / ( steps - 1 ); // 0 at the cursor, 1 at the far end
                float fade = style->tailBright * ( 1.0f - 0.7f * f ) * shown / ( peak > 0.0f ? peak : 1.0f );
                uint8_t sr = r, sg = g, sb = b;
                blendColour( &sr, r, tr, f );
                blendColour( &sg, g, tg, f );
                blendColour( &sb, b, tb, f );
                splat( layout, frame, in->along + f * dAlong, in->acrossMm + f * dAcross, style->minSigmaRows, style->minSigmaAcrossMm, peak * fade, 0.0f, style->minSigmaRows,
                       style->minSigmaAcrossMm, PROBELED_MAX_SIGMA_ROWS, PROBELED_MAX_SIGMA_ACROSS_MM, sr, sg, sb );
            }
        }
    }

    // The touch ring: a thin ring a row wide spreading from where the point
    // landed, fading as it goes, in the cursor's colour - in a layer of its
    // own OVER the LEDs (probeLedShown composites it), decaying at its own
    // pace so it stays a ring, and leaving what it passes over as it was.
    float ringKeep = expf( -dtS / PROBELED_RING_DECAY_S );
    float ringUp = style->attackS > 0.0f ? 1.0f - expf( -dtS / style->attackS ) : 1.0f; // it rises as an LED does
    for ( int i = 0; i < layout->count; i++ ) {
        frame->ring[ i ] *= ringKeep;
        if ( frame->ring[ i ] < 0.002f )
            frame->ring[ i ] = 0.0f;
    }
    if ( frame->ringAgeS >= 0.0f && !asBrush ) {
        frame->ringAgeS += dtS;
        if ( frame->ringAgeS > PROBELED_RING_S ) {
            frame->ringAgeS = -1.0f;
        } else {
            float radius = frame->ringAgeS * PROBELED_RING_ROWS_PER_S;
            float amp = style->peak * 0.6f * ( 1.0f - frame->ringAgeS / PROBELED_RING_S );
            schemeColour( style, in, frame->timeS, &frame->ringR, &frame->ringG, &frame->ringB );
            for ( int i = 0; i < layout->count; i++ ) {
                float da = layout->along[ i ] - frame->ringAlong;
                float dc = ( layout->acrossMm[ i ] - frame->ringAcrossMm ) / PITCH_MM;
                float d = sqrtf( da * da + dc * dc ) - radius;
                if ( d < -PROBELED_RING_WIDTH_ROWS || d > PROBELED_RING_WIDTH_ROWS )
                    continue;
                float v = amp * ( 1.0f - fabsf( d ) / PROBELED_RING_WIDTH_ROWS ); // full on the ring, fading to nothing a row and a half out
                if ( v > frame->ring[ i ] )
                    frame->ring[ i ] += ringUp * ( v - frame->ring[ i ] );
            }
        }
    }

    // Rainbow: the hue runs along the board (hueTurns of the wheel over its
    // 30 rows) and round with time; every lit LED gets its own - white on
    // the board, like the other schemes. (Before the sparkles, which go over it.)
    if ( style->scheme == PROBELED_SCHEME_RAINBOW && in->state != PROBELED_NONE && !asBrush ) {
        float amount = colourByHeight( style, in->heightMm );
        float perRow = style->hueTurns * 360.0f / PROBELED_ROWS;
        for ( int i = 0; i < layout->count; i++ ) {
            if ( frame->target[ i ] <= 0.0f )
                continue;
            probeLedHue( style->hueStartDeg + frame->timeS * 90.0f + layout->along[ i ] * perRow + layout->acrossMm[ i ] * 4.0f, 1.0f, &frame->r[ i ], &frame->g[ i ], &frame->b[ i ] );
            whiteTo( &frame->r[ i ], &frame->g[ i ], &frame->b[ i ], amount );
        }
    }

    // Sparkle: any LED in the glow may flash this frame, the odds rising
    // with the lever and with the chosen data (the height by default: the
    // point on the board is a plain white dot with a few, lifted it is a
    // soft glow full of them) - white with a little of a random hue in it,
    // so no two twinkle quite alike; the decay below turns each flash into
    // a twinkle.
    if ( style->sparkle > 0.0f && in->state != PROBELED_NONE && !asBrush ) {
        float byData = style->sparkleBy == PROBELED_DATA_NONE ? 1.0f : PROBELED_SPARKLE_FLOOR + ( 1.0f - PROBELED_SPARKLE_FLOOR ) * dataOf( style, in, style->sparkleBy );
        uint32_t odds = (uint32_t)( style->sparkle * byData * 0.03f * 4294967296.0f );
        for ( int i = 0; i < layout->count; i++ ) {
            if ( frame->target[ i ] < 0.02f )
                continue;
            if ( xorshift( &frame->rng ) < odds ) {
                frame->target[ i ] = style->peak;
                float hue = ( xorshift( &frame->rng ) >> 8 ) * ( 360.0f / 16777216.0f );
                probeLedHue( hue, 1.0f, &frame->r[ i ], &frame->g[ i ], &frame->b[ i ] );
                whiteTo( &frame->r[ i ], &frame->g[ i ], &frame->b[ i ], PROBELED_SPARKLE_TINT );
            }
        }
    }

    // The paint, under everything: where nothing brighter is drawn, it shows.
    if ( paint != nullptr ) {
        for ( int i = 0; i < layout->count; i++ ) {
            if ( paint->level[ i ] > frame->target[ i ] ) {
                frame->target[ i ] = paint->level[ i ];
                frame->r[ i ] = paint->r[ i ];
                frame->g[ i ] = paint->g[ i ];
                frame->b[ i ] = paint->b[ i ];
            }
        }
    }

    // Follow: fast up, slower down (the brush preview: at once, it is a
    // ruler, not a glow).
    float up = style->attackS > 0.0f && !asBrush ? 1.0f - expf( -dtS / style->attackS ) : 1.0f;
    float down = style->decayS > 0.0f && !asBrush ? 1.0f - expf( -dtS / style->decayS ) : 1.0f;
    for ( int i = 0; i < layout->count; i++ ) {
        float t = frame->target[ i ], l = frame->level[ i ];
        l += ( t >= l ? up : down ) * ( t - l );
        if ( l < 0.002f )
            l = 0.0f;
        frame->level[ i ] = l;
    }
}

void probeLedShown( const ProbeLedFrame* frame, int i, float* level, uint8_t* r, uint8_t* g, uint8_t* b ) {
    if ( frame->ring[ i ] > frame->level[ i ] ) {
        *level = frame->ring[ i ];
        *r = frame->ringR;
        *g = frame->ringG;
        *b = frame->ringB;
    } else {
        *level = frame->level[ i ];
        *r = frame->r[ i ];
        *g = frame->g[ i ];
        *b = frame->b[ i ];
    }
}

// Gamma 2.2 for the LEDs' eye-linearity, blended with a little of the
// straight level so the dim end survives 8 bits: pure gamma turns a 0.12
// glow into 0.009 x 255 = 2, which a WS2812 can barely show; with the blend
// it is 7. (The LCD preview uses the level as it is.)
void probeLedRgb( const ProbeLedFrame* frame, int i, uint8_t* r, uint8_t* g, uint8_t* b ) {
    // The curve as a table (a powf per LED per frame adds up), 1/256 steps
    // with the level rounded to the nearest.
    static float curve[ 257 ];
    static bool made = false;
    if ( !made ) {
        for ( int n = 0; n <= 256; n++ ) {
            float v = n / 256.0f;
            curve[ n ] = 0.85f * powf( v, 2.2f ) + 0.15f * v;
        }
        made = true;
    }
    float level;
    uint8_t cr, cg, cb;
    probeLedShown( frame, i, &level, &cr, &cg, &cb );
    float v = clamp01( level );
    float k = curve[ (int)( v * 256.0f + 0.5f ) ];
    *r = (uint8_t)( cr * k + 0.5f );
    *g = (uint8_t)( cg * k + 0.5f );
    *b = (uint8_t)( cb * k + 0.5f );
}
