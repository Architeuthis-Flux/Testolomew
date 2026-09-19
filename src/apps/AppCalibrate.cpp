// SPDX-License-Identifier: MIT
// The Calibrate app: the row counter's guided calibration (twelve taps).
// Going to the app starts it; the screen says which hole is wanted and
// fills a bar while the tap is taken; when the twelve are in it says so.
// Leaving the app cancels a calibration still under way. The nav press
// clicked starts over.
#include <Adafruit_GFX.h>

#include "Apps.h"
#include "FastDraw.h"
#include "UiLayout.h"
#include "UiStream.h"
#include "config.h"
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif

#define COLOR_SURE RGB565( 80, 255, 110 )

static bool started = false; // this visit started a calibration
static bool done = false;    // ...and it finished

void calibrateEnter( ) {
#if MODULE_ROW_COUNT
    done = false;
    if ( !rowCounter.calibrating( ) ) {
        rowCounter.startCalibration( &uiStream );
    }
    started = rowCounter.calibrating( );
#endif
}

void calibrateExit( ) {
#if MODULE_ROW_COUNT
    if ( rowCounter.calibrating( ) ) {
        rowCounter.startCalibration( &uiStream ); // again = cancel
    }
#endif
}

void calibrateDraw( GFXcanvas16* canvas ) {
#if MODULE_ROW_COUNT
    char line[ 40 ];
    fastText( canvas, 2, 2, UI_TEXT, UI_COLOR_DIM, "calibrate" );
    if ( rowCounter.calibrating( ) ) {
        int row, hole;
        rowCounter.calibrationTarget( &row, &hole );
        snprintf( line, sizeof( line ), "tap row %d", row );
        fastText( canvas, 2, 40, 2 * UI_TEXT, UI_COLOR_TEXT, line );
        snprintf( line, sizeof( line ), "%s hole  %d/%d", hole == 1 ? "inner" : "outer", rowCounter.calibrationStepNumber( ) + 1, ROWGRID_CALIBRATION_TARGETS );
        fastText( canvas, 2, 40 + 2 * UI_LINE_H + 6, UI_TEXT, UI_COLOR_TEXT, line );
        int barY = 130;
        fastRect( canvas, 4, barY, LCD_WIDTH - 8, 12, UI_COLOR_DIM );
        fastFillRect( canvas, 4, barY, (int)( ( LCD_WIDTH - 8 ) * rowCounter.tapProgress( ) ), 12, COLOR_SURE );
        fastText( canvas, 2, barY + 24, UI_TEXT, UI_COLOR_DIM, "rest the probe in" );
        fastText( canvas, 2, barY + 24 + UI_LINE_H, UI_TEXT, UI_COLOR_DIM, "the hole, hold still" );
        fastText( canvas, 2, LCD_HEIGHT - UI_LINE_H - 2, UI_TEXT, UI_COLOR_DIM, "B: cancel" );
    } else if ( done ) {
        fastText( canvas, 2, 60, 2 * UI_TEXT, COLOR_SURE, "calibrated" );
        fastText( canvas, 2, 60 + 2 * UI_LINE_H + 8, UI_TEXT, UI_COLOR_TEXT, "the grid is fitted" );
        fastText( canvas, 2, 60 + 3 * UI_LINE_H + 8, UI_TEXT, UI_COLOR_TEXT, "to the twelve taps" );
        fastText( canvas, 2, LCD_HEIGHT - UI_LINE_H - 2, UI_TEXT, UI_COLOR_DIM, "B: back  press: again" );
    } else {
        fastText( canvas, 2, 60, UI_TEXT, UI_COLOR_WARNING, "not calibrating" );
        fastText( canvas, 2, LCD_HEIGHT - UI_LINE_H - 2, UI_TEXT, UI_COLOR_DIM, "press: start  B: back" );
    }
#else
    fastText( canvas, 4, 4, UI_TEXT, UI_COLOR_DIM, "MODULE_ROW_COUNT is off" );
#endif
}

bool calibrateEvent( const InputEvent* e ) {
#if MODULE_ROW_COUNT
    if ( e->control == IN_NAV_PRESS && e->kind == IN_CLICK ) {
        if ( rowCounter.calibrating( ) ) {
            rowCounter.startCalibration( &uiStream ); // cancel...
        }
        rowCounter.startCalibration( &uiStream ); // ...and start over
        done = false;
        started = true;
        return true;
    }
#else
    (void)e;
#endif
    return false;
}

uint32_t calibrateGeneration( ) {
#if MODULE_ROW_COUNT
    bool calibrating = rowCounter.calibrating( );
    if ( started && !calibrating ) {
        done = true; // it finished while we watched
        started = false;
    }
    return 1u + ( calibrating ? 2u : 0u ) + ( done ? 4u : 0u ) + 8u * (uint32_t)( rowCounter.calibrationStepNumber( ) + 1 ) + 256u * (uint32_t)( rowCounter.tapProgress( ) * 40.0f );
#else
    return 1;
#endif
}
