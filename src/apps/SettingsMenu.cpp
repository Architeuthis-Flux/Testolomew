// SPDX-License-Identifier: MIT
#include "SettingsMenu.h"

#include <string.h>

#include "Apps.h"
#include "ColourPreview.h"
#include "Console.h"
#include "Input.h"
#include "MagArray.h"
#include "MagLocator.h"
#include "Ui.h"
#include "UiStream.h"
#include "config.h"
#if MODULE_SETTINGS
#include "Settings.h"
#endif
#if MODULE_PLAY
#include "Play.h"
#endif
#if MODULE_PROBE_LEDS
#include "ProbeLedService.h"
#endif
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif

static Menu* menu = nullptr;
static int commandsSubmenu = -1; // the "commands" page, and how many console commands it lists so far
static int commandsAdded = 0;

// Console commands that have a Settings item of their own, or that change
// the app underneath: not listed on the commands page (they stay on the console).
static const char hiddenCommands[] = "evyBNrgukKzolmbpXsRChWn"; // (2026-09-25: the tools and play pages' own commands too, which were listed twice)

// ---- info lines --------------------------------------------------------------------

static void trackInfo( char* buffer, int length ) {
    const MagTrack& t = magLocator.track;
    static const char* names[ 4 ] = { "none", "rough", "coast", "track" };
    snprintf( buffer, length, "%s %lu", names[ t.state ], (unsigned long)t.accepted );
}

static void fixInfo( char* buffer, int length ) {
    const MagProbeFix& f = magLocator.fix;
    if ( f.valid ) {
        snprintf( buffer, length, "%.0f %.0f %.0f", f.magnet.x, f.magnet.y, f.magnet.z );
    } else {
        snprintf( buffer, length, "%s", f.present ? "no fix" : "none" );
    }
}

#if MODULE_ROW_COUNT
static void rowInfo( char* buffer, int length ) {
    const RowReading& r = rowCounter.reading;
    if ( !rowCounter.active ) {
        snprintf( buffer, length, "off" );
    } else if ( !r.valid ) {
        snprintf( buffer, length, "-" );
    } else if ( r.row > 0 ) {
        snprintf( buffer, length, "%d %.0f%%", r.row, 100.0f * ( r.tracked ? r.trackConfidence : r.confidence ) );
    } else {
        snprintf( buffer, length, "off end" );
    }
}
#endif

#if MODULE_PLAY
static void playInfo( char* buffer, int length ) {
    snprintf( buffer, length, "%lu %.1fs %.1fmm", (unsigned long)play.hits, play.meanMs * 1e-3f, play.meanMissMm );
}
#endif

static void sensorsInfo( char* buffer, int length ) {
    snprintf( buffer, length, "%d/%d %lu rst", magArray.sensorsOk( ), magArray.sensorCount( ), (unsigned long)magArray.busResetCount( ) );
}

static void magnetInfo( char* buffer, int length ) {
    if ( magLocator.knownStrength > 0.0f ) {
        snprintf( buffer, length, "%.0f held", magLocator.knownStrength );
    } else {
        snprintf( buffer, length, "free" );
    }
}

// ---- accessors: the values that live in the modules ------------------------------

static bool getTrackerOn( ) {
    return magLocator.track.enabled;
}
static void setTrackerOn( bool on ) {
    magLocator.track.enabled = on;
    magTrackReset( &magLocator.track );
}

static const char* const cursorModeNames[ 2 ] = { "under", "aim" };
static int getCursorMode( ) {
    return magLocator.track.cursorMode == MAGCURSOR_UNDER ? 0 : 1;
}
static void setCursorMode( int choice ) {
    magLocator.track.cursorMode = choice == 0 ? MAGCURSOR_UNDER : MAGCURSOR_POINTED;
}

static int getCameraMode( ) {
    return viewCamera.mode;
}
static void setCameraMode( int choice ) {
    cameraSetMode( &viewCamera, (CameraMode)choice );
}

#if MODULE_PROBE_LEDS
static bool getLayoutV5( ) {
    return probeLeds.v5;
}
static void setLayoutV5( bool v5 ) {
    probeLeds.useV5( v5 );
}
static bool getStripOn( ) {
    return probeLeds.strip;
}
static void setStripOn( bool on ) {
    probeLeds.setStrip( on, &uiStream );
}
#endif

#if MODULE_ROW_COUNT
static bool getRowMode( ) {
    return rowCounter.active;
}
static void setRowMode( bool on ) {
    rowCounter.setActive( on, &uiStream );
}
#endif

// ---- actions -----------------------------------------------------------------------

// A console command from the menu: its output goes to the log (and the
// serial port) and, if it said anything, into a Result panel over the page.
static void runConsoleKey( int key, float argument ) {
    (void)argument;
    uint32_t before = uiStream.logGeneration( );
    consoleRunCommand( (char)key, &uiStream );
    if ( uiStream.logGeneration( ) != before ) {
        char title[ 24 ];
        const char* help = "";
        for ( int i = 0; i < consoleCommandCount( ); i++ ) {
            char k;
            const char* h;
            if ( consoleCommandAt( i, &k, &h ) && k == (char)key ) {
                help = h;
                break;
            }
        }
        snprintf( title, sizeof( title ), "%c %.20s", (char)key, help );
        ui.showResult( title );
    }
}

// ...one that asks for a number gets the item's argument typed in ahead.
static void runConsoleKeyWithNumber( int key, float argument ) {
    char digits[ 12 ];
    snprintf( digits, sizeof( digits ), "%ld\n", (long)( argument + 0.5f ) );
    uiStream.inject( digits );
    runConsoleKey( key, argument );
}

// An app from the menu (the menu closes over it).
static void runOpenApp( int app, float ) {
    appsOpen( app );
}

static void runResetView( int, float ) {
    cameraReset( &viewCamera );
}

static void runResetSettings( int, float ) {
#if MODULE_SETTINGS
    settings.reset( &uiStream ); // every value back to its default (through the setters) and saved so
    ui.showResult( "reset settings" );
#endif
}

// ---- the menu ----------------------------------------------------------------------

void settingsMenuBuild( Menu* m ) {
    menu = m;
    menuInit( m );

    // One page for everything continuous about the probe's pose - the
    // tracker, the cursor, the smoothing (2026-09-25: three pages until
    // then; Kevin: "one menu item that contains all the tracker, cursor,
    // smoothing options"). The order: what the cursor is, then the levers
    // from the most felt (accel, rest jitter) to the least, then the fit's.
    int tracking = menuAddSubmenu( m, MENU_ROOT, "tracking" );
    menuAddInfo( m, tracking, "state", trackInfo );
    menuAddInfo( m, tracking, "fix", fixInfo );
    menuAddToggleAccessor( m, tracking, "tracker on", getTrackerOn, setTrackerOn );
    menuAddChoiceAccessor( m, tracking, "cursor", getCursorMode, setCursorMode, cursorModeNames, 2 );
    menuAddNumber( m, tracking, "surface", &magLocator.boardZ, 0.0f, 60.0f, 0.5f, "mm" );
    menuAddNumber( m, tracking, "tip", &magLocator.tipOffsetMm, 0.0f, 60.0f, 0.5f, "mm" );
    menuAddNumber( m, tracking, "reach", &magLocator.track.maxReachMm, 5.0f, 60.0f, 2.5f, "mm" ); // (5-100 until 2026-09-25: the reach is the drop x tan 70 deg at most - 55 mm from a 20 mm hover - so 60 is the useful end)
    // The smoothing, the levers that are felt first (tools/hostsim/pencil.cpp,
    // 2026-09-25: the ranges are where the pencil measured a change):
    // accel sets the Kalman's rest jitter against its lag (0.075 mm at 500
    // to 0.5 at 20000 in the bench-like world, 40 to 20 ms), rest jitter
    // the field EMA's (0: three times the jitter; 2: half, +10 ms), the
    // betas how fast the 1-Euro filters open with speed (the lag's whole
    // effect is under 0.5, the jitter's runs on to 2), the Hz where they sit at
    // rest - which only shows with the beta low, since the rest jitter's
    // own speed opens the cutoff by beta x 2-5 mm/s.
    menuAddNumber( m, tracking, "accel", &magLocator.track.accelSigma, 250.0f, 20000.0f, 250.0f, "" );
    menuAddNumber( m, tracking, "rest jitter", &magLocator.speedJitterK, 0.0f, 3.0f, 0.1f, "" );
    menuAddNumber( m, tracking, "view Hz", &magLocator.track.viewMinCutoff, 0.1f, 5.0f, 0.1f, "" );
    menuAddNumber( m, tracking, "view beta", &magLocator.track.viewBeta, 0.0f, 2.0f, 0.02f, "" );
    menuAddNumber( m, tracking, "cursor Hz", &magLocator.track.oneEuroMinCutoff, 0.1f, 5.0f, 0.1f, "" );
    menuAddNumber( m, tracking, "cursor beta", &magLocator.track.oneEuroBeta, 0.0f, 2.0f, 0.02f, "" );
    menuAddNumber( m, tracking, "shaft Hz", &magLocator.track.shaftMinCutoff, 0.1f, 5.0f, 0.1f, "" );
    menuAddNumber( m, tracking, "shaft beta", &magLocator.track.shaftBeta, 0.0f, 10.0f, 0.25f, "" );
    menuAddNumber( m, tracking, "floor", &magLocator.track.sigmaFloorMm, 0.1f, 3.0f, 0.1f, "mm" );
    menuAddNumber( m, tracking, "gate", &magLocator.track.gate, 2.0f, 8.0f, 0.5f, "sd" ); // (to 10 until 2026-09-25: the drop line is 11.3, past 8 the soft zone is nothing)
    menuAddNumber( m, tracking, "presence", &magLocator.presentMt, 0.02f, 0.20f, 0.01f, "mT" ); // (0.04-0.20 on 2026-09-25: below the TMAGs' plain level the lever did nothing, presence needing two of them at 0.04 anyway; since 2026-09-26 the two TMAGs are asked for the lever's level, so below 0.04 it reaches further out - as far as the zeros' drift allows)
    menuAddNumber( m, tracking, "fit chi", &magLocator.fitMaxChi, 1.0f, 6.0f, 0.5f, "" );
    menuAddNumber( m, tracking, "far hold", &magLocator.track.roughHoldS, 0.5f, 10.0f, 0.5f, "s" );
    int fitLoad = menuAddToggle( m, tracking, "fit load", &magLocator.steadyFit ); // steady: the same work every frame (the supply shows bursts)
    menuSetToggleText( m, fitLoad, "steady", "burst" );
    menuAddNumber( m, tracking, "fit iters", &magLocator.steadyIterations, 2.0f, 16.0f, 1.0f, "" ); // ...how much, a frame (~0.85 ms each); read only under steady

    int camera = menuAddSubmenu( m, MENU_ROOT, "camera" );
    menuAddChoiceAccessor( m, camera, "mode", getCameraMode, setCameraMode, cameraModeNames, CAMERA_MODE_COUNT );
    menuAddAction( m, camera, "reset view", 0, runResetView, false );
    menuAddNumber( m, camera, "glide", &viewCamera.tauS, 0.02f, 2.0f, 0.02f, "s" ); // (the smoothing page's "camera" until 2026-09-25)
    menuAddNumber( m, camera, "POV turn", &viewCamera.povTurnTauS, 0.02f, 3.0f, 0.05f, "s" );
    menuAddNumber( m, camera, "POV move", &viewCamera.povMoveTauS, 0.02f, 3.0f, 0.05f, "s" );

#if MODULE_PROBE_LEDS
    int leds = menuAddSubmenu( m, MENU_ROOT, "LEDs" );
    int layout = menuAddToggleAccessor( m, leds, "layout", getLayoutV5, setLayoutV5 ); // saved as 0/1 as the choice was
    menuSetToggleText( m, layout, "V5", "V6" );
    if ( probeLeds.strip ) {
        menuAddToggleAccessor( m, leds, "chain on", getStripOn, setStripOn );
        menuAddAction( m, leds, "test chain", 'n', runConsoleKey, false );
    }
    menuAddNumber( m, leds, "bright", &probeLeds.style.peak, 0.05f, 1.0f, 0.05f, "" );
    menuAddNumber( m, leds, "strip", &probeLeds.stripBrightness, 0.02f, 1.0f, 0.02f, "" );
    menuAddNumber( m, leds, "budget mA", &probeLeds.stripMaxMa, 100.0f, PROBELED_STRIP_HARD_MAX_MA, 100.0f, "" ); // the ceiling is the most it can be
    menuAddToggle( m, leds, "full peak", &probeLeds.style.fullPeak );
    menuAddNumber( m, leds, "spot", &probeLeds.style.spot, 0.5f, 4.0f, 0.1f, "" );             // the spot's size, 1 = one hole
    menuAddNumber( m, leds, "spot by height", &probeLeds.style.spotByHeight, 0.0f, 4.0f, 0.25f, "" ); // ...and how much wider at the height scale
    menuAddNumber( m, leds, "bloom", &probeLeds.style.bloom, 0.0f, 1.0f, 0.1f, "" );
    menuAddNumber( m, leds, "sparkle", &probeLeds.style.sparkle, 0.0f, 1.0f, 0.1f, "" );
    menuAddNumber( m, leds, "pulse", &probeLeds.style.pulse, 0.0f, 1.0f, 0.1f, "" );
    menuAddNumber( m, leds, "fade", &probeLeds.style.decayS, 0.05f, 2.0f, 0.05f, "s" );
    menuAddToggle( m, leds, "touch ring", &probeLeds.style.touchRing );
    menuAddToggle( m, leds, "V5 stream", &probeLeds.streaming );

    // The colours (2026-09-25): the scheme (what drives the hue), how the
    // wheel is mapped onto it, what dims the cursor and what raises the
    // sparkle (ProbeLeds.h), and the View's poles and field arrows (Apps.h);
    // the page previews the mapping under its items (ColourPreview.h).
    ui.shell.pageRowsTaken = colourPreviewRows; // the shell steers by the rows left (the absolute stick)
    ui.menuPreview = colourPreviewDraw;
    int colours = menuAddSubmenu( m, MENU_ROOT, "colours" );
    menuAddChoice( m, colours, "scheme", &probeLeds.style.scheme, probeLedSchemeNames, PROBELED_SCHEME_COUNT );
    menuAddNumber( m, colours, "turns", &probeLeds.style.hueTurns, 0.1f, 4.0f, 0.05f, "" ); // of the wheel over the scale: under 1 a chunk of the spectrum, over 1 several rainbows
    menuAddNumber( m, colours, "hue start", &probeLeds.style.hueStartDeg, 0.0f, 355.0f, 5.0f, "deg" );
    menuAddNumber( m, colours, "height scale", &probeLeds.style.liftFullMm, 5.0f, 60.0f, 1.0f, "mm" );
    menuAddNumber( m, colours, "colour from", &probeLeds.style.colourByMm, 2.0f, 30.0f, 0.5f, "mm" ); // white on the board below 1.5 mm, all the colour from here up
    menuAddChoice( m, colours, "bright by", &probeLeds.style.brightBy, probeLedDataNames, PROBELED_DATA_COUNT );
    menuAddNumber( m, colours, "bright amount", &probeLeds.style.brightAmount, 0.0f, 1.0f, 0.05f, "" );
    menuAddChoice( m, colours, "sparkle by", &probeLeds.style.sparkleBy, probeLedDataNames, PROBELED_DATA_COUNT );
    menuAddNumber( m, colours, "tail length", &probeLeds.style.tailLength, 0.1f, 1.0f, 0.05f, "" ); // in pointed mode: how far back toward the point the tail runs (2026-09-26)
    menuAddNumber( m, colours, "tail bright", &probeLeds.style.tailBright, 0.1f, 1.0f, 0.05f, "" );
    menuAddNumber( m, colours, "tail hue", &probeLeds.style.tailHueDeg, 0.0f, 355.0f, 5.0f, "deg" );
#else
    int colours = menuAddSubmenu( m, MENU_ROOT, "colours" );
#endif
    menuAddToggle( m, colours, "poles", &viewStyle.poles ); // the View's magnet bar: north and south coloured, or a plain bar
    menuAddNumber( m, colours, "pole size", &viewStyle.poleMm, 1.0f, 15.0f, 0.5f, "mm" );
    menuAddNumber( m, colours, "north hue", &viewStyle.northHueDeg, 0.0f, 355.0f, 5.0f, "deg" );
    menuAddNumber( m, colours, "south hue", &viewStyle.southHueDeg, 0.0f, 355.0f, 5.0f, "deg" );
    menuAddNumber( m, colours, "field arrows", &viewStyle.fieldArrows, 0.0f, 1.0f, 0.1f, "" ); // the sensors' arrows in the View: their length, 0 = none

    // The one-shot things: the magnet (k, K, z, o, l), the sensors and the
    // machine (m, b, p, X, s), the rows (r, c, R, C, h).
    int tools = menuAddSubmenu( m, MENU_ROOT, "tools" );
    menuAddInfo( m, tools, "strength", magnetInfo );
    menuAddAction( m, tools, "learn strength", 'k', runConsoleKey, false );
    menuAddAction( m, tools, "forget strength", 'K', runConsoleKey, true );
    menuAddAction( m, tools, "re-zero (away!)", 'z', runConsoleKey, true );
    menuAddAction( m, tools, "orientation check", 'o', runConsoleKey, false );
    menuAddAction( m, tools, "latest fix", 'l', runConsoleKey, false );
    menuAddInfo( m, tools, "sensors", sensorsInfo );
    menuAddToggle( m, tools, "use MMC", &magArray.useMmc ); // the MMC56x3 in the fit or ignored (MAG_USE_MMC_AT_BOOT: off)
    menuAddAction( m, tools, "array status", 'm', runConsoleKey, false );
    menuAddAction( m, tools, "bus check", 'b', runConsoleKey, false );
    menuAddAction( m, tools, "power-cycle", 'p', runConsoleKey, true );
    menuAddAction( m, tools, "service table", 'X', runConsoleKey, false );
#if MODULE_SETTINGS
    menuAddAction( m, tools, "saved settings", 's', runConsoleKey, false );
#endif
#if MODULE_ROW_COUNT
    menuAddToggleAccessor( m, tools, "row mode", getRowMode, setRowMode );
    menuAddInfo( m, tools, "row", rowInfo );
    menuAddAction( m, tools, "calibrate 12 taps", APP_CALIBRATE, runOpenApp, false );
    menuAddNumberAction( m, tools, "anchor at row", 'R', runConsoleKeyWithNumber, 1.0f, 60.0f, 1.0f, 1.0f );
    menuAddAction( m, tools, "forget anchors", 'C', runConsoleKey, true );
    menuAddAction( m, tools, "hold-still test", 'h', runConsoleKey, false );
#endif

#if MODULE_PLAY
    int playPage = menuAddSubmenu( m, MENU_ROOT, "play" );
    menuAddChoice( m, playPage, "mode", &play.mode, playModeNames, PLAY_MODE_COUNT );
    menuAddNumber( m, playPage, "hue", &play.paintHue, 0.0f, 359.0f, 5.0f, "deg" ); // the Draw app's colour wheel sets these too
    menuAddNumber( m, playPage, "sat", &play.paintSat, 0.0f, 1.0f, 0.05f, "" );
    menuAddNumber( m, playPage, "paint bright", &play.paintBright, PLAY_BRIGHT_STEP, 1.0f, PLAY_BRIGHT_STEP, "" ); // nav up/down in the Draw app
    menuAddNumber( m, playPage, "brush", &play.brushSize, 0.0f, (float)PLAY_BRUSH_MAX, 1.0f, "rows" );             // nav left/right there
    menuAddNumber( m, playPage, "touch", &play.touchMm, 0.2f, 5.0f, 0.2f, "mm" );                                  // the point paints below this height
    menuAddAction( m, playPage, "clear", 'W', runConsoleKey, false );
    menuAddInfo( m, playPage, "target", playInfo );
#endif

    // The feel of the controls (Input.h): how long a tilt of the nav stick
    // must hold before it counts, longer while its push contact is closed
    // (a centre push wobbles into a direction first), and how far the
    // joystick goes before it is a menu direction.
    int controls = menuAddSubmenu( m, MENU_ROOT, "controls" );
    menuAddNumber( m, controls, "direction ms", &input.navDirectionMs, 0.0f, 150.0f, 5.0f, "" );
    menuAddNumber( m, controls, "push guard ms", &input.navPushGuardMs, 0.0f, 200.0f, 5.0f, "" );
    menuAddNumber( m, controls, "joy menu at", &input.joyMenuAt, 0.1f, 0.9f, 0.05f, "" );
    menuAddNumber( m, controls, "joy menu off", &input.joyMenuOff, 0.05f, 0.9f, 0.05f, "" ); // ...and over inside this (kept under 'at')
    menuAddNumber( m, controls, "joy dead", &input.joyDead, 0.02f, 0.30f, 0.02f, "" ); // the analog stick's inner dead zone (2026-09-25)
    menuAddNumber( m, controls, "cursor repeat", &ui.shell.stepRepeatMs, 100.0f, 800.0f, 50.0f, "ms" ); // a held up/down (Home: any way) steps the cursor no faster than this (2026-09-25)
    int joystick = menuAddToggle( m, controls, "joystick", &ui.shell.absoluteJoystick ); // absolute: the stick's position is the cursor on Home and the menu pages
    menuSetToggleText( m, joystick, "absolute", "relative" );

    // Every console command, as it is (the ones registered after this menu
    // is built are added by settingsMenuAddNewCommands()).
    commandsSubmenu = menuAddSubmenu( m, MENU_ROOT, "commands" );
    settingsMenuAddNewCommands( );
#if MODULE_SETTINGS
    if ( menuAddAction( m, MENU_ROOT, "reset settings", 0, runResetSettings, true ) < 0 || m->count >= MENU_MAX_ITEMS ) {
        uiStream.println( "menu: the item table is full - raise MENU_MAX_ITEMS (items are being dropped)" );
    }
#endif
}

void settingsMenuAddNewCommands( ) {
    static char labels[ CONSOLE_MAX_COMMANDS ][ 36 ];
    Menu* m = menu;
    if ( m == nullptr )
        return;
    for ( int i = commandsAdded; i < consoleCommandCount( ) && i < CONSOLE_MAX_COMMANDS; i++ ) {
        char key;
        const char* help;
        if ( !consoleCommandAt( i, &key, &help ) )
            break;
        commandsAdded = i + 1;
        if ( strchr( hiddenCommands, key ) != nullptr )
            continue; // a Settings item of its own, or it changes the app: the console's
        snprintf( labels[ i ], sizeof( labels[ i ] ), "%c %s", key, help );
        bool number = strstr( help, "<number>" ) != nullptr;
        int added = number ? menuAddNumberAction( m, commandsSubmenu, labels[ i ], key, runConsoleKeyWithNumber, 0.0f, 999.0f, 1.0f, 0.0f )
                           : menuAddAction( m, commandsSubmenu, labels[ i ], key, runConsoleKey, false );
        if ( added < 0 ) {
            uiStream.println( "menu: the item table is full - raise MENU_MAX_ITEMS (commands are being dropped)" );
            break;
        }
    }
}
