// SPDX-License-Identifier: MIT
// The Terminal app: what the console printed, 40 columns x 28 rows of
// size-1 text, the newest line at the bottom. The nav stick's up/down
// scroll back, its press clicks to the newest.
#include <Adafruit_GFX.h>

#include "Apps.h"
#include "FastDraw.h"
#include "UiLayout.h"
#include "UiStream.h"

static int logScroll = 0; // lines back from the newest that the screen shows

void terminalDraw( GFXcanvas16* canvas ) {
    const int T = UI_LOG_TEXT, H = 8 * UI_LOG_TEXT;
    int count = uiStream.logCount( );
    int rows = count < UI_LOG_ROWS ? count : UI_LOG_ROWS;
    int most = count - rows;
    if ( logScroll > most )
        logScroll = most < 0 ? 0 : most;
    for ( int r = 0; r < rows; r++ ) {
        // Oldest of the shown lines at the top: line (rows - 1 - r + scroll) back.
        int back = rows - 1 - r + logScroll;
        const char* line = uiStream.logLine( back );
        if ( line == nullptr )
            continue;
        fastText( canvas, 0, 2 + r * H, T, back == 0 ? UI_COLOR_TEXT : UI_COLOR_DIM, line );
    }
    fastText( canvas, 0, LCD_HEIGHT - H - 2, T, UI_COLOR_FRAME, logScroll > 0 ? "log (scrolled)  up/down, press: newest" : "log  up/down: scroll" );
}

bool terminalEvent( const InputEvent* e ) {
    bool press = e->kind == IN_PRESS || e->kind == IN_REPEAT;
    int most = uiStream.logCount( ) - UI_LOG_ROWS;
    if ( most < 0 )
        most = 0;
    switch ( e->control ) {
    case IN_NAV_UP:
    case IN_JOY_UP:
        if ( press && logScroll < most )
            logScroll++;
        return true;
    case IN_NAV_DOWN:
    case IN_JOY_DOWN:
        if ( press && logScroll > 0 )
            logScroll--;
        return true;
    case IN_NAV_PRESS:
        if ( e->kind == IN_CLICK )
            logScroll = 0;
        return true;
    default:
        return false;
    }
}

uint32_t terminalGeneration( ) {
    return uiStream.logGeneration( ) + 7919u * (uint32_t)logScroll + 1u;
}
