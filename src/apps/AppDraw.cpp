// SPDX-License-Identifier: MIT
// The Draw app: the paint app's screen (src/play/Play.h). Going to it
// starts painting, leaving it stops - the drawing stays. The point's place
// over a top-down map of the board: every hole a dot, the painted ones
// squares in their colour at their level (as the LEDs have them), the
// brush's ring round the point, the point itself a cross; the colour wheel
// bottom right with the paint's settings beside it.
//
// Controls: the joystick moves the wheel's marker (hue round, saturation
// out; against the rim the stick's tilt cycles the hue until it is let go);
// the joystick's press clicks draw/erase; the nav stick's up/down set
// the paint's brightness, left/right the brush; the nav press held clears
// the drawing.
#include <Adafruit_GFX.h>
#include <math.h>

#include "Apps.h"
#include "BoardPins.h"
#include "Console.h"
#include "Display.h"
#include "FastDraw.h"
#include "MagArray.h"
#include "MagLocator.h"
#include "UiLayout.h"
#include "config.h"
#include "Play.h"
#include "ProbeLedService.h"

#define COLOR_GRID RGB565( 30, 40, 60 )
#define COLOR_SHADOW RGB565( 240, 200, 40 )
#define COLOR_TEXT RGB565( 220, 220, 220 )
#define COLOR_TEXT_DIM RGB565( 120, 120, 120 )
#define COLOR_SURE RGB565( 80, 255, 110 )

static GFXcanvas16* canvas = nullptr;
static void drawColourWheel( );

static void textAt( GFXcanvas16* c, int x, int y, int size, uint16_t color, const char* text ) {
    fastText( c, x, y, size, color, text );
}

// The drawing over the board from above: every hole (and rail LED) a dot, the painted
// ones squares in their colour at their level (as the LEDs have them), the
// brush's ring round the point as hollow squares in the paint colour, the
// point itself a cross. The map is the LED screen's: 7 px a row.
// The board from above: every hole a dot, the painted ones squares in
// their colour at their level, the brush's ring round the point (if asked),
// the point itself a cross (green when touching).
void drawBoardMap( GFXcanvas16* into, bool brushRing ) {
    canvas = into;
#if MODULE_PLAY
    const LedLayout& layout = probeLeds.layout;
    const ProbeLedFrame& frame = probeLeds.frame;
    const ProbeLedBrush& brush = probeLeds.brush;
    for ( int i = 0; i < layout.count; i++ ) {
        // Every LED, the rails' too (2026-09-28, Kevin: "we need to show the
        // top and bottom rails on the screen in draw mode"): a dot each,
        // where the layout has it - the V5's four rails beyond the holes.
        int x, y;
        PlayService::tracePlace( layout.along[ i ], layout.acrossMm[ i ], &x, &y );
        float painted = play.paint.level[ i ];
        if ( painted > 0.0f ) {
            // As lit: the paint's colour at its level (the LEDs' own gamma
            // is theirs; the panel shows the level as it is).
            float k = painted > 1.0f ? 1.0f : painted;
            fastFillRect( canvas, x - 2, y - 2, 5, 5, RGB565( (uint8_t)( play.paint.r[ i ] * k ), (uint8_t)( play.paint.g[ i ] * k ), (uint8_t)( play.paint.b[ i ] * k ) ) );
        } else {
            fastFillRect( canvas, x, y, 1, 1, COLOR_GRID );
        }
        // The brush ring (what the LED renderer is showing as the cursor).
        if ( brushRing && brush.active && frame.target[ i ] > 0.0f && frame.target[ i ] != painted ) {
            fastRect( canvas, x - 3, y - 3, 7, 7, RGB565( brush.r, brush.g, brush.b ) );
        }
    }
    const ProbeLedInput& in = probeLeds.input;
    if ( in.state != PROBELED_NONE ) {
        int x, y;
        PlayService::tracePlace( in.haveUnder ? in.underAlong : in.along, in.haveUnder ? in.underAcrossMm : in.acrossMm, &x, &y );
        uint16_t c = in.heightMm < play.touchMm ? COLOR_SURE : COLOR_SHADOW;
        fastFillRect( canvas, x - 4, y, 9, 1, c );
        fastFillRect( canvas, x, y - 4, 1, 9, c );
    }
#else
    (void)brushRing;
    textAt( canvas, 4, 4, UI_TEXT, COLOR_TEXT_DIM, "MODULE_PLAY is off" );
#endif
}

static void drawTraceScreen( ) {
#if MODULE_PLAY
    drawBoardMap( canvas, true );
    const ProbeLedInput& in = probeLeds.input;
    char line[ 40 ];
    snprintf( line, sizeof( line ), "%s: %s", playModeNames[ play.mode ], in.state == PROBELED_NONE ? "no probe" : ( in.heightMm < play.touchMm ? "touching" : "lifted" ) ); // 20 columns
    textAt( canvas, 2, 2, UI_TEXT, COLOR_TEXT_DIM, line );
    if ( play.mode == PLAY_PAINT ) {
        drawColourWheel( );
    }
    if ( play.mode == PLAY_TARGET ) {
        snprintf( line, sizeof( line ), "%lu hit %.2fs %.1fmm", (unsigned long)play.hits, play.meanMs * 1e-3f, play.meanMissMm );
        textAt( canvas, 2, LCD_HEIGHT - UI_LINE_H - 2, UI_TEXT, COLOR_TEXT_DIM, line );
    }
#else
    textAt( canvas, 4, 4, UI_TEXT, COLOR_TEXT_DIM, "MODULE_PLAY is off" );
#endif
}

// The paint's colour wheel, bottom right of the draw screen: hue round it,
// saturation out from the white centre, the marker where the paint colour
// is (the joystick moves it), a swatch of the colour at its brightness, and
// the brush and draw/erase state. The wheel's pixels are worked out once.
#define WHEEL_R 26
#define WHEEL_CX ( LCD_WIDTH - WHEEL_R - 4 )
#define WHEEL_CY ( LCD_HEIGHT - WHEEL_R - 4 )
#define WHEEL_PANEL_Y ( LCD_HEIGHT - 5 * UI_LINE_H ) // the settings, five lines to the wheel's left (the map is above, PLAY_TRACE_MID_Y)
static uint16_t wheelPixels[ ( 2 * WHEEL_R + 1 ) * ( 2 * WHEEL_R + 1 ) ];
static bool wheelReady = false;

static void drawColourWheel( ) {
#if MODULE_PLAY
    if ( !wheelReady ) {
        for ( int dy = -WHEEL_R; dy <= WHEEL_R; dy++ ) {
            for ( int dx = -WHEEL_R; dx <= WHEEL_R; dx++ ) {
                float r = sqrtf( (float)( dx * dx + dy * dy ) ) / WHEEL_R;
                uint16_t c = 0;
                if ( r <= 1.0f ) {
                    float hue = atan2f( (float)-dy, (float)dx ) * ( 180.0f / 3.14159265f ); // y up
                    uint8_t cr, cg, cb;
                    playHsvToRgb( hue, r, &cr, &cg, &cb );
                    c = RGB565( cr, cg, cb );
                    if ( c == 0 )
                        c = 1; // 0 is the "outside" mark
                }
                wheelPixels[ ( dy + WHEEL_R ) * ( 2 * WHEEL_R + 1 ) + dx + WHEEL_R ] = c;
            }
        }
        wheelReady = true;
    }
    if ( canvas == nullptr )
        return; // only the table was wanted (drawBegin)
    uint16_t* buffer = canvas->getBuffer( );
    for ( int dy = -WHEEL_R; dy <= WHEEL_R; dy++ ) {
        int y = WHEEL_CY + dy;
        if ( y < 0 || y >= LCD_HEIGHT )
            continue;
        for ( int dx = -WHEEL_R; dx <= WHEEL_R; dx++ ) {
            uint16_t c = wheelPixels[ ( dy + WHEEL_R ) * ( 2 * WHEEL_R + 1 ) + dx + WHEEL_R ];
            int x = WHEEL_CX + dx;
            if ( c != 0 && x >= 0 && x < LCD_WIDTH )
                buffer[ y * LCD_WIDTH + x ] = c;
        }
    }
    // The marker: a ring where the colour is.
    float rad = play.paintHue * ( 3.14159265f / 180.0f );
    int mx = WHEEL_CX + (int)( play.paintSat * WHEEL_R * cosf( rad ) + 0.5f );
    int my = WHEEL_CY - (int)( play.paintSat * WHEEL_R * sinf( rad ) + 0.5f );
    fastRect( canvas, mx - 3, my - 3, 7, 7, RGB565( 0, 0, 0 ) );
    fastRect( canvas, mx - 2, my - 2, 5, 5, RGB565( 255, 255, 255 ) );
    // Every setting of the play page, to the wheel's left: the colour, the
    // brush's brightness and width, draw or erase (with the swatch: the
    // colour as it will be painted), and how to clear. What the controls do
    // is in the README; the joystick, its click and the nav stick are it.
    uint8_t r, g, b;
    play.paintColour( &r, &g, &b );
    float k = play.erase ? 0.0f : play.paintBright;
    char line[ 24 ];
    int y = WHEEL_PANEL_Y;
    snprintf( line, sizeof( line ), "hue %3d sat %.2f", (int)( play.paintHue + 0.5f ) % 360, play.paintSat );
    textAt( canvas, 2, y, UI_TEXT, COLOR_TEXT_DIM, line );
    y += UI_LINE_H;
    snprintf( line, sizeof( line ), "bright %3d%%", (int)( play.paintBright * 100.0f + 0.5f ) );
    textAt( canvas, 2, y, UI_TEXT, COLOR_TEXT_DIM, line );
    y += UI_LINE_H;
    snprintf( line, sizeof( line ), "brush %d", (int)( play.brushSize + 0.5f ) );
    textAt( canvas, 2, y, UI_TEXT, COLOR_TEXT_DIM, line );
    int sx = 2 + 7 * UI_CHAR_W + 8, sy = y + 2; // the swatch after "brush 1", then draw / ERASE
    fastFillRect( canvas, sx, sy, 12, 12, RGB565( (uint8_t)( r * k ), (uint8_t)( g * k ), (uint8_t)( b * k ) ) );
    fastRect( canvas, sx - 1, sy - 1, 14, 14, play.erase ? COLOR_SHADOW : COLOR_TEXT_DIM );
    textAt( canvas, sx + 18, y, UI_TEXT, play.erase ? COLOR_SHADOW : COLOR_TEXT, play.erase ? "ERASE" : "draw" );
    y += UI_LINE_H;
    snprintf( line, sizeof( line ), "touch %.1f mm", play.touchMm );
    textAt( canvas, 2, y, UI_TEXT, COLOR_TEXT_DIM, line );
    y += UI_LINE_H;
    textAt( canvas, 2, y, UI_TEXT, COLOR_TEXT_DIM, "hold nav: clear" );
#endif
}


// ---- the app -----------------------------------------------------------------------

// Once, at boot: the wheel's pixels (53 x 53 HSV conversions, 29 ms - not
// something to do in the first frame).
void drawBegin( ) {
    drawColourWheel( );
}

void drawEnter( ) {
    if ( play.mode == PLAY_OFF )
        play.mode = PLAY_PAINT;
}

void drawExit( ) {
    if ( play.mode == PLAY_PAINT )
        play.mode = PLAY_OFF;
}

void drawDraw( GFXcanvas16* into ) {
    canvas = into;
    drawTraceScreen( );
}

// The joystick moves the colour wheel's marker.
void drawTick( float dtS, float joyX, float joyY ) {
    if ( play.mode == PLAY_PAINT && ( joyX != 0.0f || joyY != 0.0f ) ) {
        play.movePicker( joyX * PLAY_PICK_PER_S * dtS, joyY * PLAY_PICK_PER_S * dtS );
    } else {
        play.releasePicker( );
    }
}

bool drawEvent( const InputEvent* e ) {
    if ( play.mode != PLAY_PAINT )
        return false;
    bool press = e->kind == IN_PRESS || e->kind == IN_REPEAT;
    switch ( e->control ) {
    case IN_JOY_PRESS:
        if ( e->kind == IN_CLICK )
            play.erase = !play.erase;
        return true;
    case IN_NAV_PRESS:
        if ( e->kind == IN_HOLD )
            play.clearPaint( ); // the centre held: the drawing gone
        return true;
    case IN_NAV_UP:
        if ( press )
            play.setPaintBright( play.paintBright + PLAY_BRIGHT_STEP );
        return true;
    case IN_NAV_DOWN:
        if ( press )
            play.setPaintBright( play.paintBright - PLAY_BRIGHT_STEP );
        return true;
    case IN_NAV_RIGHT:
        if ( press && play.brushSize < PLAY_BRUSH_MAX - 0.5f )
            play.brushSize += 1.0f;
        return true;
    case IN_NAV_LEFT:
        if ( press && play.brushSize > 0.5f )
            play.brushSize -= 1.0f;
        return true;
    default:
        return false;
    }
}
