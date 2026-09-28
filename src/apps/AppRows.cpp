// SPDX-License-Identifier: MIT
// The Rows app: which breadboard row the probe is over, written large,
// with how sure the counter is of it (a bar) and the offset within the
// row. Off with row mode off (the nav press clicked turns it on).
#include <Adafruit_GFX.h>
#include <math.h>

#include "Apps.h"
#include "FastDraw.h"
#include "MagLocator.h"
#include "UiLayout.h"
#include "UiStream.h"
#include "config.h"
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif

#define COLOR_SURE RGB565( 80, 255, 110 )
#define COLOR_UNSURE RGB565( 255, 220, 40 )

void rowsDraw( GFXcanvas16* canvas ) {
#if MODULE_ROW_COUNT
    char line[ 40 ];
    fastText( canvas, 2, 2, UI_TEXT, UI_COLOR_DIM, "rows" );
    if ( !rowCounter.active ) {
        fastText( canvas, 2, LCD_HEIGHT / 2 - UI_LINE_H, UI_TEXT, UI_COLOR_DIM, "row mode is off" );
        fastText( canvas, 2, LCD_HEIGHT / 2, UI_TEXT, UI_COLOR_DIM, "press: turn it on" );
        return;
    }
    const RowReading& r = rowCounter.reading;
    if ( rowCounter.calibrating( ) ) {
        fastText( canvas, 2, LCD_HEIGHT / 2 - UI_LINE_H, UI_TEXT, UI_COLOR_WARNING, "calibrating..." );
        return;
    }
    if ( !r.valid ) {
        fastText( canvas, 2, LCD_HEIGHT / 2 - 2 * UI_LINE_H, 3 * UI_TEXT, UI_COLOR_DIM, "--" );
        fastText( canvas, 2, LCD_HEIGHT - 2 * UI_LINE_H, UI_TEXT, UI_COLOR_DIM, "no probe" );
        return;
    }
    float sure = r.tracked ? r.trackConfidence : r.confidence;
    uint16_t colour = sure > 0.95f ? COLOR_SURE : ( sure > 0.68f ? COLOR_UNSURE : UI_COLOR_WARNING );
    if ( r.row > 0 ) {
        snprintf( line, sizeof( line ), "%d", r.row );
        int w = (int)strlen( line ) * 6 * 8; // size 8: 48 x 64 px digits
        fastText( canvas, ( LCD_WIDTH - w ) / 2, 40, 8, colour, line );
    } else {
        fastText( canvas, 20, 60, 3 * UI_TEXT, colour, "off end" );
    }
    // The bar: how sure.
    int barY = 120, barW = LCD_WIDTH - 40;
    fastRect( canvas, 20, barY, barW, 12, UI_COLOR_DIM );
    fastFillRect( canvas, 20, barY, (int)( barW * sure ), 12, colour );
    snprintf( line, sizeof( line ), "%.0f%% sure  hole %d", sure * 100.0f, r.hole );
    fastText( canvas, 2, barY + 20, UI_TEXT, UI_COLOR_TEXT, line );
    float offset = r.tracked ? r.trackPlace.along - floorf( r.trackPlace.along + 0.5f ) : r.offsetRows;
    float bar = r.tracked ? r.trackSigmaRows : r.sigmaRows;
    snprintf( line, sizeof( line ), "%+.2f +-%.2f rows", offset, bar );
    fastText( canvas, 2, barY + 20 + UI_LINE_H, UI_TEXT, UI_COLOR_DIM, line );
    static const char* trackNames[ 4 ] = { "-", "rough", "coast", "track" };
    snprintf( line, sizeof( line ), "%s  %s", magLocator.track.enabled ? trackNames[ magLocator.track.state ] : ( magLocator.track.smooth ? "smooth" : "raw" ), r.touching ? "touching" : "lifted" );
    fastText( canvas, 2, LCD_HEIGHT - UI_LINE_H - 2, UI_TEXT, UI_COLOR_DIM, line );
#else
    fastText( canvas, 4, 4, UI_TEXT, UI_COLOR_DIM, "MODULE_ROW_COUNT is off" );
#endif
}

bool rowsEvent( const InputEvent* e ) {
#if MODULE_ROW_COUNT
    if ( e->control == IN_NAV_PRESS && e->kind == IN_CLICK ) {
        rowCounter.setActive( !rowCounter.active, &uiStream );
        return true;
    }
#else
    (void)e;
#endif
    return false;
}

// Redrawn when the row, the hole, the bar or the mode changes.
uint32_t rowsGeneration( ) {
#if MODULE_ROW_COUNT
    const RowReading& r = rowCounter.reading;
    float sure = r.tracked ? r.trackConfidence : r.confidence;
    float offset = r.tracked ? r.trackPlace.along - floorf( r.trackPlace.along + 0.5f ) : r.offsetRows;
    return 1u + ( rowCounter.active ? 2u : 0u ) + ( r.valid ? 4u : 0u ) + ( rowCounter.calibrating( ) ? 8u : 0u ) + ( r.touching ? 16u : 0u ) + 32u * (uint32_t)( r.row + 1 ) + 4096u * (uint32_t)r.hole +
           65536u * (uint32_t)( sure * 50.0f ) + 8388608u * (uint32_t)( ( offset + 0.5f ) * 20.0f );
#else
    return 1;
#endif
}
