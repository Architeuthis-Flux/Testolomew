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
    s->roughR = 120;
    s->roughG = 40;
    s->roughB = 160; // purple haze from afar
    s->tailR = 255;
    s->tailG = 120;
    s->tailB = 30; // amber: where the point is
    s->peak = 1.0f;
    s->minSigmaRows = 0.33f;
    s->minSigmaAcrossMm = 0.9f;
    s->liftFullMm = 15.0f;
    s->attackS = 0.02f;
    s->decayS = 0.12f;
    s->scheme = PROBELED_SCHEME_CLASSIC;
    s->hueTurns = 0.667f; // two thirds of the wheel: red to blue (the height scheme as it was)
    s->hueStartDeg = 0.0f;
    s->colourByMm = 6.0f;
    s->brightBy = PROBELED_DATA_SURE;
    s->brightAmount = 0.5f; // a coin-toss row at half
    s->sparkleBy = PROBELED_DATA_HEIGHT;
    s->fullPeak = true;
    s->bloom = 0.0f;
    s->sparkle = 0.0f;
    s->pulse = 0.0f;
    s->touchRing = false;
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
        frame->ringLit[ i ] = 0;
    frame->ringAlong = frame->ringAcrossMm = 0.0f;
}

static float clamp01( float v ) {
    return v < 0.0f ? 0.0f : ( v > 1.0f ? 1.0f : v );
}

static void blendColour( uint8_t* c, uint8_t a, uint8_t b, float t ) {
    *c = (uint8_t)( a + ( b - a ) * t + 0.5f );
}

// A bell of the given widths centred on (along, across), with total light
// held constant: the peak is scaled by (minimum area / area), so a bell twice
// as wide is a quarter as bright.
static void splat( const LedLayout* layout, ProbeLedFrame* frame, float along, float acrossMm, float sigmaRows, float sigmaAcrossMm, float peak, float floor, float least,
                   float minSigmaRows, float minSigmaAcrossMm, float maxSigmaRows, float maxSigmaAcrossMm, uint8_t r, uint8_t g, uint8_t b ) {
    if ( sigmaRows < minSigmaRows )
        sigmaRows = minSigmaRows;
    if ( sigmaAcrossMm < minSigmaAcrossMm )
        sigmaAcrossMm = minSigmaAcrossMm;
    // How far past twice the narrowest the bell has widened, 0..1 at the
    // widest it is allowed: the floor fades out over it, and the bell is
    // held at the widest (see PROBELED_MAX_SIGMA_*).
    float wideRows = ( sigmaRows - 2.0f * minSigmaRows ) / ( maxSigmaRows - 2.0f * minSigmaRows );
    float wideAcross = ( sigmaAcrossMm - 2.0f * minSigmaAcrossMm ) / ( maxSigmaAcrossMm - 2.0f * minSigmaAcrossMm );
    float wide = clamp01( wideRows > wideAcross ? wideRows : wideAcross );
    if ( sigmaRows > maxSigmaRows )
        sigmaRows = maxSigmaRows;
    if ( sigmaAcrossMm > maxSigmaAcrossMm )
        sigmaAcrossMm = maxSigmaAcrossMm;
    float amplitude = peak * ( minSigmaRows * minSigmaAcrossMm ) / ( sigmaRows * sigmaAcrossMm );
    floor *= 1.0f - wide; // a wide bell still shows as something the eye can find, until it is as wide as it gets
    if ( floor < least )
        floor = least; // ...but never less than this (the far probe's faint "about here")
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
        float v = amplitude * expf( -0.5f * q );
        if ( v <= frame->target[ i ] )
            continue;
        // The brighter of the two marks colours the LED.
        frame->target[ i ] = v;
        frame->r[ i ] = r;
        frame->g[ i ] = g;
        frame->b[ i ] = b;
    }
}

// How much of the scheme's colour the cursor shows at this height: none
// (white) with the point on the board, all of it from the style's
// colourByMm up. The classic scheme has its own, longer ramp.
static float colourByHeight( const ProbeLedStyle* style, float heightMm ) {
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
        probeLedHue( style->hueStartDeg + wheel * in->heightMm / style->liftFullMm, 1.0f, r, g, b );
        whiteTo( r, g, b, colourByHeight( style, in->heightMm ) );
        break;
    case PROBELED_SCHEME_SURE:
        probeLedHue( style->hueStartDeg + wheel * clamp01( in->confidence ), 1.0f, r, g, b ); // the start of the wheel unsure, its end sure
        whiteTo( r, g, b, colourByHeight( style, in->heightMm ) );
        break;
    case PROBELED_SCHEME_AIM:
        // Which way the probe leans, round the wheel; white standing straight.
        probeLedHue( style->hueStartDeg + style->hueTurns * in->aimDeg, 1.0f, r, g, b );
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
        if ( down && !frame->wasLifted && hole >= 0 && hole != frame->ringHole ) {
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
        if ( in->state == PROBELED_ROUGH ) {
            r = style->roughR;
            g = style->roughG;
            b = style->roughB;
            peak *= 0.6f;
        } else {
            schemeColour( style, in, frame->timeS, &r, &g, &b );
            // Dimmed by the chosen data (unsure by default: a coin-toss row at half).
            peak *= 1.0f - clamp01( style->brightAmount ) * dataOf( style, in, style->brightBy );
            if ( in->state == PROBELED_COASTING )
                peak *= 0.7f;
        }
        // The cursor keeps its total light (a wide bell is a dim one) down to a
        // floor an LED can still show; a far probe's glow is a flashlight, and
        // gets a higher one. With fullPeak the floor IS the peak: the widest
        // bell still has one LED at full - but never for the far probe's
        // glow, which is a hundred LEDs wide: at the peak that is amps out of
        // a breadboard's 5 V rail (a V5 browned out on 2026-09-18), and it
        // is meant to be a faint "about here", not a floodlight.
        float floor = style->fullPeak && in->state != PROBELED_ROUGH ? peak : ( in->state == PROBELED_ROUGH ? 0.12f : 0.08f );
        splat( layout, frame, in->along, in->acrossMm, in->sigmaRows, in->sigmaAcrossMm, peak, floor, in->state == PROBELED_ROUGH ? PROBELED_ROUGH_LEAST : 0.0f, style->minSigmaRows,
               style->minSigmaAcrossMm, PROBELED_MAX_SIGMA_ROWS, PROBELED_MAX_SIGMA_ACROSS_MM, r, g, b );
        if ( style->bloom > 0.0f ) {
            // The halo: three times as wide, a fraction as bright, the same
            // colour; the max rule in splat() keeps it under the cursor.
            float wide = 3.0f;
            splat( layout, frame, in->along, in->acrossMm, in->sigmaRows * wide, in->sigmaAcrossMm * wide, 0.3f * style->bloom * peak, 0.0f, 0.0f,
                   style->minSigmaRows * wide, style->minSigmaAcrossMm * wide, PROBELED_MAX_SIGMA_ROWS * wide, PROBELED_MAX_SIGMA_ACROSS_MM * wide, r, g, b );
        }

        // Where the point itself is, when that is somewhere else: a fainter
        // mark there and a thin tail to the cursor.
        if ( in->haveUnder && in->state != PROBELED_ROUGH ) {
            float dAlong = in->along - in->underAlong, dAcross = in->acrossMm - in->underAcrossMm;
            float length = sqrtf( dAlong * dAlong + dAcross * dAcross / ( PITCH_MM * PITCH_MM ) ); // in rows
            // The tail follows the cursor's own brightness (a dim cursor gets
            // a dim tail, never outshone) and stops a step short of it.
            int steps = (int)( length * 2.0f );
            if ( steps > 12 )
                steps = 12;
            for ( int k = 0; k < steps; k++ ) {
                float f = (float)k / steps;
                float fade = 0.35f * ( 1.0f - 0.7f * f ); // brightest under the point, fading toward the cursor
                splat( layout, frame, in->underAlong + f * dAlong, in->underAcrossMm + f * dAcross, style->minSigmaRows, style->minSigmaAcrossMm,
                       peak * fade, 0.0f, 0.0f, style->minSigmaRows, style->minSigmaAcrossMm, PROBELED_MAX_SIGMA_ROWS, PROBELED_MAX_SIGMA_ACROSS_MM, style->tailR, style->tailG,
                       style->tailB );
            }
        }
    }

    // The touch ring: a thin ring a row wide spreading from where the point
    // landed, fading as it goes, in the cursor's colour.
    if ( frame->ringAgeS >= 0.0f && !asBrush ) {
        frame->ringAgeS += dtS;
        if ( frame->ringAgeS > PROBELED_RING_S ) {
            frame->ringAgeS = -1.0f;
        } else {
            float radius = frame->ringAgeS * PROBELED_RING_ROWS_PER_S;
            float amp = style->peak * 0.6f * ( 1.0f - frame->ringAgeS / PROBELED_RING_S );
            uint8_t r, g, b;
            schemeColour( style, in, frame->timeS, &r, &g, &b );
            for ( int i = 0; i < layout->count; i++ ) {
                float da = layout->along[ i ] - frame->ringAlong;
                float dc = ( layout->acrossMm[ i ] - frame->ringAcrossMm ) / PITCH_MM;
                float d = sqrtf( da * da + dc * dc ) - radius;
                if ( d < -PROBELED_RING_WIDTH_ROWS || d > PROBELED_RING_WIDTH_ROWS )
                    continue;
                float v = amp * ( 1.0f - fabsf( d ) / PROBELED_RING_WIDTH_ROWS ); // full on the ring, fading to nothing a row and a half out
                if ( v > frame->target[ i ] ) {
                    frame->target[ i ] = v;
                    frame->r[ i ] = r;
                    frame->g[ i ] = g;
                    frame->b[ i ] = b;
                    frame->ringLit[ i ] = 2; // lit by the ring this frame
                }
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
        float byData = in->state == PROBELED_ROUGH || style->sparkleBy == PROBELED_DATA_NONE ? 1.0f : PROBELED_SPARKLE_FLOOR + ( 1.0f - PROBELED_SPARKLE_FLOOR ) * dataOf( style, in, style->sparkleBy );
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
    float ringDown = 1.0f - expf( -dtS / PROBELED_RING_DECAY_S ); // an LED the ring passed: dark again at once, so the ring stays a ring
    for ( int i = 0; i < layout->count; i++ ) {
        float t = frame->target[ i ], l = frame->level[ i ];
        if ( t >= l ) {
            l += up * ( t - l );
            frame->ringLit[ i ] = frame->ringLit[ i ] == 2 ? 1 : 0; // the ring's this frame stays the ring's; anything else's is not
        } else {
            if ( frame->ringLit[ i ] == 2 )
                frame->ringLit[ i ] = 1;
            l += ( frame->ringLit[ i ] ? ringDown : down ) * ( t - l );
        }
        if ( l < 0.002f ) {
            l = 0.0f;
            frame->ringLit[ i ] = 0;
        }
        frame->level[ i ] = l;
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
    float v = clamp01( frame->level[ i ] );
    float k = curve[ (int)( v * 256.0f + 0.5f ) ];
    *r = (uint8_t)( frame->r[ i ] * k + 0.5f );
    *g = (uint8_t)( frame->g[ i ] * k + 0.5f );
    *b = (uint8_t)( frame->b[ i ] * k + 0.5f );
}
