// SPDX-License-Identifier: MIT
#include "Apps.h"

#include <Adafruit_GFX.h>
#include <string.h>

#include "Console.h"
#include "Display.h"
#include "Icons.h"
#include "Input.h"
#include "MagLocator.h"
#include "SettingsMenu.h"
#include "Ui.h"
#include "UiStream.h"
#include "config.h"
#if MODULE_PLAY
#include "Play.h"
#endif
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif

#define APPS_TEXT_REDRAW_MS 200 // a text screen (the terminal, a result panel) no more often than this for the log's sake

UiApp apps[ APP_COUNT ] = {
    { "View", nullptr, nullptr, nullptr, viewTick, viewDraw, viewEvent, nullptr },
    { "LEDs", nullptr, nullptr, nullptr, nullptr, ledsDraw, nullptr, nullptr },
    { "Terminal", nullptr, nullptr, nullptr, nullptr, terminalDraw, terminalEvent, terminalGeneration },
    { "Draw", nullptr, drawEnter, drawExit, drawTick, drawDraw, drawEvent, nullptr },
    { "Target", nullptr, targetEnter, targetExit, nullptr, targetDraw, targetEvent, nullptr },
    { "Rows", nullptr, nullptr, nullptr, nullptr, rowsDraw, rowsEvent, rowsGeneration },
    { "Calibrate", nullptr, calibrateEnter, calibrateExit, nullptr, calibrateDraw, calibrateEvent, calibrateGeneration },
    { "Info", nullptr, nullptr, nullptr, nullptr, infoDraw, nullptr, infoGeneration },
};

// The icons, packed from their ASCII art into bits at boot (a flash table
// read per pixel is what the I-cache does not help with; RAM is cheap here).
static uint8_t iconBits[ ICON_COUNT ][ UIAPP_ICON_BYTES ];
static const char* slotNames[ APP_COUNT ];

static void packIcon( const char* const* art, uint8_t* bits ) {
    memset( bits, 0, UIAPP_ICON_BYTES );
    for ( int row = 0; row < ICON_SIZE; row++ ) {
        for ( int col = 0; col < ICON_SIZE; col++ ) {
            if ( art[ row ][ col ] == '#' )
                bits[ row * 3 + col / 8 ] |= (uint8_t)( 0x80 >> ( col % 8 ) );
        }
    }
}

// Icons.h's order: View, LEDs, Terminal, Draw, Target, Rows, Calibrate, Settings, Info.
static const int iconOfApp[ APP_COUNT ] = { 0, 1, 2, 3, 4, 5, 6, 8 };
#define ICON_SETTINGS_INDEX 7

static uint32_t lastInputStamp = 0, lastFrameStamp = 0;
static uint32_t lastTextDrawMs = 0;
static bool drewOnce = false;

bool appsDrawFrame( GFXcanvas16* canvas, uint32_t nowMs ) {
    display.slot = ui.shell.app < 0 ? 0 : ui.shell.app;
    uint32_t stamp = ui.frameStamp( );
    if ( stamp != 0 && drewOnce ) {
        if ( stamp == lastFrameStamp ) {
            return false; // nothing changed
        }
        uint32_t inputStamp = ui.inputStamp( );
        if ( inputStamp == lastInputStamp && nowMs - lastTextDrawMs < APPS_TEXT_REDRAW_MS ) {
            return false; // only the log changed: not more often than this (a CSV stream would hold the loop)
        }
        lastInputStamp = inputStamp;
    } else {
        lastInputStamp = ui.inputStamp( );
    }
    lastFrameStamp = stamp;
    lastTextDrawMs = nowMs;
    drewOnce = true;
    ui.draw( canvas );
    return true;
}

void appsOpen( int app ) {
    uiShellSelectApp( &ui.shell, app );
}

// `e`: the next app, and what the draws have been costing.
static void onNextApp( Stream* out ) {
    int next = ( ui.shell.app + 1 ) % APP_COUNT;
    appsOpen( next );
    char line[ 80 ];
    snprintf( line, sizeof( line ), "app: %s", apps[ next ].name );
    out->println( line );
    display.printStats( out, slotNames, APP_COUNT );
}

// The modules' lines of :screen.
static void screenExtra( Stream* out ) {
    char line[ 120 ];
    snprintf( line, sizeof( line ), "sim: %s%s%s", magLocator.simProbeActive( ) ? "probe " : "", input.simActive( ) ? "input " : "", !magLocator.simProbeActive( ) && !input.simActive( ) ? "-" : "" );
    out->println( line );
#if MODULE_PLAY
    snprintf( line, sizeof( line ), "play: %s hue %.0f sat %.2f bright %.2f brush %d %s", playModeNames[ play.mode ], play.paintHue, play.paintSat, play.paintBright, (int)( play.brushSize + 0.5f ),
              play.erase ? "erase" : "draw" );
    out->println( line );
#endif
    const MagTrack& t = magLocator.track;
    const MagProbeFix& f = magLocator.fix;
    static const char* names[ 4 ] = { "none", "rough", "coast", "track" };
    if ( t.enabled && t.state != MAGTRACK_NONE ) {
        snprintf( line, sizeof( line ), "probe: %s x %.1f y %.1f z %.1f tilt %.0f cursor %.1f %.1f", names[ t.state ], t.position.x, t.position.y, t.position.z, t.tiltDeg, t.cursor.x, t.cursor.y );
    } else if ( f.valid ) {
        snprintf( line, sizeof( line ), "probe: fix x %.1f y %.1f z %.1f tilt %.0f", f.magnet.x, f.magnet.y, f.magnet.z, f.tiltDeg );
    } else {
        snprintf( line, sizeof( line ), "probe: none" );
    }
    out->println( line );
#if MODULE_ROW_COUNT
    const RowReading& r = rowCounter.reading;
    if ( rowCounter.active && r.valid ) {
        snprintf( line, sizeof( line ), "row: %d hole %d %.0f%%", r.row, r.hole, 100.0f * ( r.tracked ? r.trackConfidence : r.confidence ) );
        out->println( line );
    }
    if ( rowCounter.calibrating( ) ) {
        int row, hole;
        rowCounter.calibrationTarget( &row, &hole );
        snprintf( line, sizeof( line ), "calibrating: tap row %d hole %d, step %d/%d, %.0f%%", row, hole, rowCounter.calibrationStepNumber( ) + 1, ROWGRID_CALIBRATION_TARGETS, 100.0f * rowCounter.tapProgress( ) );
        out->println( line );
    }
    if ( rowCounter.leanCalibrating( ) ) {
        snprintf( line, sizeof( line ), "lean: %d/%d %s, %.0f%%%s%s", rowCounter.leanStepNumber( ) + 1, ROWCOUNT_LEAN_STEPS, rowCounter.leanPrompt( ), 100.0f * rowCounter.tapProgress( ),
                  rowCounter.leanInAir ? ", in the air" : "", rowCounter.leanWaitingForSwing( ) ? ", waiting for the swing" : "" );
        out->println( line );
    }
#endif
    snprintf( line, sizeof( line ), "camera: %s target %.1f %.1f %.1f yaw %.0f el %.0f zoom %.2f", cameraModeNames[ viewCamera.mode ], viewCamera.target.x, viewCamera.target.y, viewCamera.target.z,
              viewCamera.yawDeg, viewCamera.elevationDeg, viewCamera.zoom );
    out->println( line );
}

void appsBegin( ) {
    for ( int a = 0; a < APP_COUNT; a++ ) {
        packIcon( ICONS[ iconOfApp[ a ] ], iconBits[ a ] );
        apps[ a ].icon = iconBits[ a ];
        slotNames[ a ] = apps[ a ].name;
    }
    packIcon( ICONS[ ICON_SETTINGS_INDEX ], iconBits[ APP_COUNT ] );
    viewBegin( );
#if MODULE_PLAY
    drawBegin( );
#endif
    ui.begin( apps, APP_COUNT, APP_VIEW, APPS_SETTINGS_CELL ); // (the shell's init empties the menu: build it after)
    settingsMenuBuild( &ui.shell.menu );
    ui.shell.settingsIcon = iconBits[ APP_COUNT ];
    ui.screenExtra = screenExtra;
    display.drawFn = appsDrawFrame;
    display.slotNames = slotNames;
    display.slotCount = APP_COUNT;
    consoleAddCommand( 'e', "next app (View / LEDs / Terminal / Draw / Target / Rows / Calibrate / Info), with each app's draw time", onNextApp );
}
