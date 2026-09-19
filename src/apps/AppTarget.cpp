// SPDX-License-Identifier: MIT
// The Target app: the target game (src/play/Play.h). One LED lights green
// somewhere on the board; touch it. The screen is the board from above
// with the target and the point, and the tally large: hits, the mean time
// to reach, the mean miss. Going to the app starts the game, leaving it
// stops. The nav press clicked skips to another target.
#include <Adafruit_GFX.h>

#include "Apps.h"
#include "FastDraw.h"
#include "Play.h"
#include "ProbeLedService.h"
#include "UiLayout.h"
#include "config.h"

void targetEnter( ) {
    play.mode = PLAY_TARGET;
}

void targetExit( ) {
    if ( play.mode == PLAY_TARGET )
        play.mode = PLAY_OFF;
}

void targetDraw( GFXcanvas16* canvas ) {
    drawBoardMap( canvas, false );
    char line[ 40 ];
    const ProbeLedInput& in = probeLeds.input;
    snprintf( line, sizeof( line ), "target: %s", in.state == PROBELED_NONE ? "no probe" : ( in.heightMm < play.touchMm ? "touching" : "lifted" ) );
    fastText( canvas, 2, 2, UI_TEXT, UI_COLOR_DIM, line );
    int y = LCD_HEIGHT - 5 * UI_LINE_H;
    snprintf( line, sizeof( line ), "%lu hit", (unsigned long)play.hits );
    fastText( canvas, 2, y, 2 * UI_TEXT, UI_COLOR_SELECTED, line );
    y += 2 * UI_LINE_H + 4;
    snprintf( line, sizeof( line ), "%lu missed", (unsigned long)play.misses );
    fastText( canvas, 2, y, UI_TEXT, UI_COLOR_TEXT, line );
    y += UI_LINE_H;
    snprintf( line, sizeof( line ), "mean %.2fs %.1fmm", play.meanMs * 1e-3f, play.meanMissMm );
    fastText( canvas, 2, y, UI_TEXT, UI_COLOR_TEXT, line );
    y += UI_LINE_H;
    snprintf( line, sizeof( line ), "last %.2fs %.1fmm", play.lastMs * 1e-3f, play.lastMissMm );
    fastText( canvas, 2, y, UI_TEXT, UI_COLOR_DIM, line );
}

bool targetEvent( const InputEvent* e ) {
    if ( e->control == IN_NAV_PRESS && e->kind == IN_CLICK ) {
        play.newTarget( ); // another one
        return true;
    }
    return false;
}
