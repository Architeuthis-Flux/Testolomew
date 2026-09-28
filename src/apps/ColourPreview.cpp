// SPDX-License-Identifier: MIT
#include "ColourPreview.h"

#include <Adafruit_GFX.h>
#include <string.h>

#include "Apps.h"
#include "FastDraw.h"
#include "UiLayout.h"
#include "config.h"
#if MODULE_PROBE_LEDS
#include "ProbeLedService.h"
#endif

#define PREVIEW_LABEL_W 42 // size-1 text, up to seven characters, before each band

int colourPreviewRows( const char* page ) {
    return strcmp( page, "colors" ) == 0 ? 3 : 0; // (the page is "colors" since 2026-09-28)
}

#if MODULE_PROBE_LEDS
// One band: `count` columns across, each coloured by the caller's function of t (0..1).
typedef void ( *BandColour )( float t, uint8_t* r, uint8_t* g, uint8_t* b );

static void band( GFXcanvas16* canvas, int x, int y, int w, int h, BandColour colour ) {
    for ( int px = 0; px < w; px++ ) {
        uint8_t r, g, b;
        colour( w > 1 ? (float)px / ( w - 1 ) : 0.0f, &r, &g, &b );
        fastFillRect( canvas, x + px, y, 1, h, RGB565( r, g, b ) );
    }
}

// The wheel as the levers map it: hue start, then turns of it across the
// band (aim: one turn, the compass; classic: its own white-to-blue ramp).
static void wheel( float t, uint8_t* r, uint8_t* g, uint8_t* b ) {
    const ProbeLedStyle& s = probeLeds.style;
    if ( s.scheme == PROBELED_SCHEME_CLASSIC ) {
        *r = (uint8_t)( s.touchR + ( s.liftR - s.touchR ) * t );
        *g = (uint8_t)( s.touchG + ( s.liftG - s.touchG ) * t );
        *b = (uint8_t)( s.touchB + ( s.liftB - s.touchB ) * t );
        return;
    }
    float turns = s.scheme == PROBELED_SCHEME_AIM ? 1.0f : s.hueTurns;
    probeLedHue( s.hueStartDeg + turns * 360.0f * t, 1.0f, r, g, b );
}

// The cursor's colour across the scheme's data, as the renderer paints it:
// the height from the board to the height scale (white on the board, the
// colour from "colour from" up), the sureness from a toss-up to certain, the
// lean round the compass, the rows along the board.
static void cursorAcross( float t, uint8_t* r, uint8_t* g, uint8_t* b ) {
    const ProbeLedStyle& s = probeLeds.style;
    ProbeLedInput in = { };
    in.state = PROBELED_TRACKING;
    in.confidence = 1.0f;
    in.heightMm = s.liftFullMm; // lifted: all the colour (the height scheme runs the height itself)
    in.tiltDeg = PROBELED_AIM_FULL_DEG;
    switch ( s.scheme ) {
    case PROBELED_SCHEME_SURE:
        in.confidence = t;
        break;
    case PROBELED_SCHEME_AIM:
        in.aimDeg = 360.0f * t;
        break;
    case PROBELED_SCHEME_RAINBOW:
        // The per-LED hue along the board (ProbeLeds.cpp), at time 0.
        probeLedHue( s.hueStartDeg + ( 1.0f + t * ( PROBELED_ROWS - 1 ) ) * s.hueTurns * 360.0f / PROBELED_ROWS, 1.0f, r, g, b );
        return;
    default:
        in.heightMm = t * s.liftFullMm;
        break;
    }
    probeLedCursorColour( &s, &in, 0.0f, r, g, b );
}

// The dimming by the chosen data: the peak at the near end, (1 - amount) of it at the far end.
static void dimming( float t, uint8_t* r, uint8_t* g, uint8_t* b ) {
    const ProbeLedStyle& s = probeLeds.style;
    float k = s.brightBy == PROBELED_DATA_NONE ? 1.0f : 1.0f + s.brightAmount * t; // signed (2026-09-28): + brighter toward the far end, - dimmer
    if ( k > 1.0f )
        k = 1.0f;
    if ( k < 0.0f )
        k = 0.0f;
    *r = *g = *b = (uint8_t)( 255.0f * s.peak * k );
}

static void swatch( GFXcanvas16* canvas, int x, int y, int w, int h, float hueDeg ) {
    uint8_t r, g, b;
    probeLedHue( hueDeg, 1.0f, &r, &g, &b );
    fastFillRect( canvas, x, y, w, h, RGB565( r, g, b ) );
}

void colourPreviewDraw( GFXcanvas16* canvas, const char* page, int x, int y, int w, int h ) {
    if ( colourPreviewRows( page ) == 0 )
        return;
    const ProbeLedStyle& s = probeLeds.style;
    int bandH = ( h - 4 ) / 3, bandX = x + PREVIEW_LABEL_W, bandW = w - PREVIEW_LABEL_W;
    static const char* const across[ PROBELED_SCHEME_COUNT ] = { "height", "height", "sure", "lean", "rows" };
    // 1. the wheel as mapped
    fastText( canvas, x, y + ( bandH - 8 ) / 2, 1, UI_COLOR_DIM, "wheel" );
    band( canvas, bandX, y, bandW, bandH, wheel );
    // 2. the cursor across the scheme's data
    int y2 = y + bandH + 2;
    fastText( canvas, x, y2 + ( bandH - 8 ) / 2, 1, UI_COLOR_DIM, across[ s.scheme >= 0 && s.scheme < PROBELED_SCHEME_COUNT ? s.scheme : 0 ] );
    band( canvas, bandX, y2, bandW, bandH, cursorAcross );
    // 3. the dimming, the tail, the poles
    int y3 = y2 + bandH + 2;
    fastText( canvas, x, y3 + ( bandH - 8 ) / 2, 1, UI_COLOR_DIM, "dim" );
    int dimW = bandW / 2 - 4;
    band( canvas, bandX, y3, dimW, bandH, dimming );
    int sx = bandX + dimW + 6, sw = ( bandW - dimW - 6 - 8 ) / 3;
    swatch( canvas, sx, y3, sw, bandH, s.tailHueDeg );
    fastText( canvas, sx + 2, y3 + ( bandH - 8 ) / 2, 1, UI_COLOR_PANEL, "tail" );
    if ( viewStyle.poles ) {
        swatch( canvas, sx + sw + 4, y3, sw, bandH, viewStyle.northHueDeg );
        fastText( canvas, sx + sw + 6, y3 + ( bandH - 8 ) / 2, 1, UI_COLOR_PANEL, "N" );
        swatch( canvas, sx + 2 * sw + 8, y3, sw, bandH, viewStyle.southHueDeg );
        fastText( canvas, sx + 2 * sw + 10, y3 + ( bandH - 8 ) / 2, 1, UI_COLOR_PANEL, "S" );
    } else {
        fastRect( canvas, sx + sw + 4, y3, 2 * sw + 4, bandH, UI_COLOR_DIM );
        fastText( canvas, sx + sw + 8, y3 + ( bandH - 8 ) / 2, 1, UI_COLOR_DIM, "poles off" );
    }
}
#else
void colourPreviewDraw( GFXcanvas16*, const char*, int, int, int, int ) {}
#endif
