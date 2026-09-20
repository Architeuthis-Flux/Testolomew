// SPDX-License-Identifier: MIT
#include "SettingsMenu.h"

#include <string.h>

#include "Apps.h"
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
static const char hiddenCommands[] = "evyBNrgu";

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
    if ( magLocator.learning( ) ) {
        snprintf( buffer, length, "learning" );
    } else if ( magLocator.knownStrength > 0.0f ) {
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

    int tracker = menuAddSubmenu( m, MENU_ROOT, "tracker" );
    menuAddInfo( m, tracker, "state", trackInfo );
    menuAddInfo( m, tracker, "fix", fixInfo );
    menuAddToggleAccessor( m, tracker, "tracker on", getTrackerOn, setTrackerOn );
    menuAddNumber( m, tracker, "floor", &magLocator.track.sigmaFloorMm, 0.1f, 3.0f, 0.1f, "mm" );
    menuAddNumber( m, tracker, "gate", &magLocator.track.gate, 2.0f, 10.0f, 0.5f, "sd" );
    menuAddNumber( m, tracker, "presence", &magLocator.presentMt, 0.02f, 0.20f, 0.01f, "mT" );
    menuAddNumber( m, tracker, "far hold", &magLocator.track.roughHoldS, 0.5f, 10.0f, 0.5f, "s" );
    int fitLoad = menuAddToggle( m, tracker, "fit load", &magLocator.steadyFit ); // steady: the same work every frame (the supply shows bursts)
    menuSetToggleText( m, fitLoad, "steady", "burst" );
    menuAddNumber( m, tracker, "fit iters", &magLocator.steadyIterations, 2.0f, 16.0f, 1.0f, "" ); // ...how much, a frame (~0.85 ms each)

    int smoothing = menuAddSubmenu( m, MENU_ROOT, "smoothing" );
    menuAddNumber( m, smoothing, "view Hz", &magLocator.track.viewMinCutoff, 0.2f, 10.0f, 0.1f, "" );
    menuAddNumber( m, smoothing, "view beta", &magLocator.track.viewBeta, 0.0f, 1.0f, 0.01f, "" );
    menuAddNumber( m, smoothing, "cursor Hz", &magLocator.track.oneEuroMinCutoff, 0.2f, 10.0f, 0.1f, "" );
    menuAddNumber( m, smoothing, "cursor beta", &magLocator.track.oneEuroBeta, 0.0f, 1.0f, 0.01f, "" );
    menuAddNumber( m, smoothing, "shaft Hz", &magLocator.track.shaftMinCutoff, 0.2f, 10.0f, 0.1f, "" );
    menuAddNumber( m, smoothing, "shaft beta", &magLocator.track.shaftBeta, 0.0f, 20.0f, 0.5f, "" );
    menuAddNumber( m, smoothing, "camera", &viewCamera.tauS, 0.02f, 2.0f, 0.02f, "s" );
    menuAddNumber( m, smoothing, "POV turn", &viewCamera.povTurnTauS, 0.02f, 3.0f, 0.05f, "s" );
    menuAddNumber( m, smoothing, "POV move", &viewCamera.povMoveTauS, 0.02f, 3.0f, 0.05f, "s" );
    menuAddNumber( m, smoothing, "accel", &magLocator.track.accelSigma, 500.0f, 20000.0f, 500.0f, "" );

    int cursor = menuAddSubmenu( m, MENU_ROOT, "cursor" );
    menuAddChoiceAccessor( m, cursor, "cursor", getCursorMode, setCursorMode, cursorModeNames, 2 );
    menuAddNumber( m, cursor, "surface", &magLocator.boardZ, 0.0f, 60.0f, 0.5f, "mm" );
    menuAddNumber( m, cursor, "tip", &magLocator.tipOffsetMm, 0.0f, 60.0f, 0.5f, "mm" );
    menuAddNumber( m, cursor, "reach", &magLocator.track.maxReachMm, 5.0f, 100.0f, 5.0f, "mm" );

    int camera = menuAddSubmenu( m, MENU_ROOT, "camera" );
    menuAddChoiceAccessor( m, camera, "mode", getCameraMode, setCameraMode, cameraModeNames, CAMERA_MODE_COUNT );
    menuAddAction( m, camera, "reset view", 0, runResetView, false );

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
    menuAddChoice( m, leds, "colours", &probeLeds.style.scheme, probeLedSchemeNames, PROBELED_SCHEME_COUNT );
    menuAddToggle( m, leds, "full peak", &probeLeds.style.fullPeak );
    menuAddNumber( m, leds, "bloom", &probeLeds.style.bloom, 0.0f, 1.0f, 0.1f, "" );
    menuAddNumber( m, leds, "sparkle", &probeLeds.style.sparkle, 0.0f, 1.0f, 0.1f, "" );
    menuAddNumber( m, leds, "pulse", &probeLeds.style.pulse, 0.0f, 1.0f, 0.1f, "" );
    menuAddNumber( m, leds, "fade", &probeLeds.style.decayS, 0.05f, 2.0f, 0.05f, "s" );
    menuAddToggle( m, leds, "touch ring", &probeLeds.style.touchRing );
    menuAddToggle( m, leds, "V5 stream", &probeLeds.streaming );
#endif

#if MODULE_ROW_COUNT
    // The row counter's tools: what r, c, R, C and h do on the console.
    int rows = menuAddSubmenu( m, MENU_ROOT, "rows" );
    menuAddToggleAccessor( m, rows, "row mode", getRowMode, setRowMode );
    menuAddInfo( m, rows, "row", rowInfo );
    menuAddAction( m, rows, "calibrate 12 taps", APP_CALIBRATE, runOpenApp, false );
    menuAddNumberAction( m, rows, "anchor at row", 'R', runConsoleKeyWithNumber, 1.0f, 60.0f, 1.0f, 1.0f );
    menuAddAction( m, rows, "forget anchors", 'C', runConsoleKey, true );
    menuAddAction( m, rows, "hold-still test", 'h', runConsoleKey, false );
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
    int joystick = menuAddToggle( m, controls, "joystick", &ui.shell.absoluteJoystick ); // absolute: the stick's position is the cursor on Home and the menu pages
    menuSetToggleText( m, joystick, "absolute", "relative" );

    // The magnet: k, K, z, o, l.
    int magnet = menuAddSubmenu( m, MENU_ROOT, "magnet" );
    menuAddInfo( m, magnet, "strength", magnetInfo );
    menuAddAction( m, magnet, "learn strength", 'k', runConsoleKey, false );
    menuAddAction( m, magnet, "forget strength", 'K', runConsoleKey, true );
    menuAddAction( m, magnet, "re-zero (away!)", 'z', runConsoleKey, true );
    menuAddAction( m, magnet, "orientation check", 'o', runConsoleKey, false );
    menuAddAction( m, magnet, "latest fix", 'l', runConsoleKey, false );

    // The sensors and the machine: m, b, p, X, s.
    int sensors = menuAddSubmenu( m, MENU_ROOT, "sensors" );
    menuAddInfo( m, sensors, "sensors", sensorsInfo );
    menuAddAction( m, sensors, "array status", 'm', runConsoleKey, false );
    menuAddAction( m, sensors, "bus check", 'b', runConsoleKey, false );
    menuAddAction( m, sensors, "power-cycle", 'p', runConsoleKey, true );
    menuAddAction( m, sensors, "service table", 'X', runConsoleKey, false );
#if MODULE_SETTINGS
    menuAddAction( m, sensors, "saved settings", 's', runConsoleKey, false );
#endif

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
