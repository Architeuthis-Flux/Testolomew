// SPDX-License-Identifier: MIT
// The LEDs app: the breadboard's LEDs from above, each a square in the
// colour the probe cursor renderer would light it - rows along the panel,
// holes across it, rows 1-30 at the top, 31-60 below the channel.
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
#if MODULE_PROBE_LEDS
#include "ProbeLedService.h"
#endif

#define COLOR_GRID RGB565( 30, 40, 60 )
#define COLOR_TEXT RGB565( 220, 220, 220 )
#define COLOR_TEXT_DIM RGB565( 120, 120, 120 )

static GFXcanvas16* canvas = nullptr;

static void textAt( GFXcanvas16* c, int x, int y, int size, uint16_t color, const char* text ) {
    fastText( c, x, y, size, color, text );
}

// The breadboard's LEDs from above: rows along the panel, holes across it,
// each LED a square in the colour it would be lit, as the probe cursor
// renderer has it. Rows 1-30 at the top, 31-60 below the channel.
static void drawLedScreen( ) {
#if MODULE_PROBE_LEDS
    const LedLayout& layout = probeLeds.layout;
    const ProbeLedFrame& frame = probeLeds.frame;
    const int cell = LCD_WIDTH / ( PROBELED_ROWS + 2 ); // 7 px per row
    const float pxPerMm = cell / 2.54f;
    const int x0 = ( LCD_WIDTH - PROBELED_ROWS * cell ) / 2;
    const int yMid = LCD_HEIGHT / 2 + 8;
    for ( int i = 0; i < layout.count; i++ ) {
        int x = x0 + (int)( ( layout.along[ i ] - 1.0f ) * cell );
        int y = yMid - (int)( layout.acrossMm[ i ] * pxPerMm );
        // The LCD is not an LED: no gamma, so the dim end of the bell shows.
        float level;
        uint8_t cr, cg, cb;
        probeLedShown( &frame, i, &level, &cr, &cg, &cb ); // the LED, or the ring over it
        if ( level > 1.0f )
            level = 1.0f;
        uint8_t r = (uint8_t)( cr * level ), g = (uint8_t)( cg * level ), b = (uint8_t)( cb * level );
        uint16_t colour = level > 0.02f ? RGB565( r, g, b ) : ( layout.kind[ i ] == PROBELED_RAIL ? RGB565( 24, 20, 20 ) : RGB565( 28, 32, 40 ) );
        fastFillRect( canvas, x, y - cell / 2 + 1, cell - 1, cell - 1, colour );
    }
    fastFillRect( canvas, x0, yMid, PROBELED_ROWS * cell, 1, COLOR_GRID );
    textAt( canvas, x0, (int)( yMid - 8 * pxPerMm * 2.54f - 28 ), UI_TEXT, COLOR_TEXT_DIM, "1" );
    textAt( canvas, x0 + 28 * cell, (int)( yMid - 8 * pxPerMm * 2.54f - 28 ), UI_TEXT, COLOR_TEXT_DIM, "30" );

    char line[ 60 ];
    const ProbeLedInput& in = probeLeds.input;
    static const char* names[ 4 ] = { "no probe", "far away", "coasting", "tracking" };
    if ( in.state == PROBELED_NONE ) {
        textAt( canvas, 2, 2, UI_TEXT, COLOR_TEXT_DIM, names[ 0 ] );
    } else {
        int rowNumber = (int)floorf( in.along + 0.5f ) + ( in.acrossMm < 0.0f ? PROBELED_ROWS : 0 );
        snprintf( line, sizeof( line ), "%s row %d", names[ in.state ], rowNumber );
        textAt( canvas, 2, 2, UI_TEXT, COLOR_TEXT, line );
        snprintf( line, sizeof( line ), "+-%.2f %.0f%% up %.0fmm%s", in.sigmaRows, in.confidence * 100.0f, in.heightMm, in.haveUnder ? " *" : "" );
        textAt( canvas, 2, 2 + UI_LINE_H, UI_TEXT, COLOR_TEXT_DIM, line );
    }
    snprintf( line, sizeof( line ), "%s %d LEDs %2.0f fps", probeLeds.v5 ? "V5" : "V6", layout.count, display.fps( ) );
    textAt( canvas, 2, LCD_HEIGHT - UI_LINE_H - 2, UI_TEXT, COLOR_TEXT_DIM, line );
#else
    canvas->setCursor( 4, 4 );
    canvas->setTextColor( COLOR_TEXT_DIM );
    canvas->print( "MODULE_PROBE_LEDS is off" );
#endif
}

void ledsDraw( GFXcanvas16* into ) {
    canvas = into;
    drawLedScreen( );
}
