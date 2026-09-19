// SPDX-License-Identifier: MIT
#include "Ui.h"

#include <Adafruit_GFX.h>
#include <string.h>

#include "Console.h"
#include "FastDraw.h"
#include "Input.h"
#include "MagArray.h"
#include "MagLocator.h"
#include "MagView.h"
#include "ST7789.h"
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

#define UI_COLOR_PANEL RGB565( 12, 14, 22 )
#define UI_COLOR_FRAME RGB565( 90, 110, 150 )
#define UI_COLOR_TEXT RGB565( 220, 220, 220 )
#define UI_COLOR_DIM RGB565( 120, 120, 120 )
#define UI_COLOR_SELECTED RGB565( 255, 200, 60 )
#define UI_COLOR_EDIT RGB565( 80, 255, 110 )

// Actions above the console's keys: the menu's own.
#define ACTION_RESET_VIEW 1000
#define ACTION_CLOSE 1001
#define ACTION_RESET_SETTINGS 1002

Ui& ui = Ui::getInstance( );

Ui& Ui::getInstance( ) {
    static Ui instance;
    return instance;
}

static const char* const cursorModeNames[ 2 ] = { "under", "aim" };
static const char* const ledLayoutNames[ 2 ] = { "V6", "V5" };

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

void Ui::buildMenu( ) {
    Menu* m = &menu;
    menuInit( m );

    int tracker = menuAddSubmenu( m, MENU_ROOT, "tracker" );
    menuAddInfo( m, tracker, "state", trackInfo );
    menuAddInfo( m, tracker, "fix", fixInfo );
    menuAddToggle( m, tracker, "tracker on", &trackerOn );
    menuAddNumber( m, tracker, "floor", &magLocator.track.sigmaFloorMm, 0.1f, 3.0f, 0.1f, "mm" );
    menuAddNumber( m, tracker, "gate", &magLocator.track.gate, 2.0f, 10.0f, 0.5f, "sd" );
    menuAddNumber( m, tracker, "presence", &magLocator.presentMt, 0.02f, 0.20f, 0.01f, "mT" );
    menuAddNumber( m, tracker, "far hold", &magLocator.track.roughHoldS, 0.5f, 10.0f, 0.5f, "s" );

    int smoothing = menuAddSubmenu( m, MENU_ROOT, "smoothing" );
    menuAddNumber( m, smoothing, "view Hz", &magLocator.track.viewMinCutoff, 0.2f, 10.0f, 0.1f, "" );
    menuAddNumber( m, smoothing, "view beta", &magLocator.track.viewBeta, 0.0f, 1.0f, 0.01f, "" );
    menuAddNumber( m, smoothing, "cursor Hz", &magLocator.track.oneEuroMinCutoff, 0.2f, 10.0f, 0.1f, "" );
    menuAddNumber( m, smoothing, "cursor beta", &magLocator.track.oneEuroBeta, 0.0f, 1.0f, 0.01f, "" );
    menuAddNumber( m, smoothing, "shaft Hz", &magLocator.track.shaftMinCutoff, 0.2f, 10.0f, 0.1f, "" );
    menuAddNumber( m, smoothing, "shaft beta", &magLocator.track.shaftBeta, 0.0f, 20.0f, 0.5f, "" );
    menuAddNumber( m, smoothing, "camera", &magView.cam.tauS, 0.02f, 2.0f, 0.02f, "s" );
    menuAddNumber( m, smoothing, "POV turn", &magView.cam.povTurnTauS, 0.02f, 3.0f, 0.05f, "s" );
    menuAddNumber( m, smoothing, "POV move", &magView.cam.povMoveTauS, 0.02f, 3.0f, 0.05f, "s" );
    menuAddNumber( m, smoothing, "accel", &magLocator.track.accelSigma, 500.0f, 20000.0f, 500.0f, "" );

    int cursor = menuAddSubmenu( m, MENU_ROOT, "cursor" );
    menuAddChoice( m, cursor, "cursor", &cursorModeChoice, cursorModeNames, 2 );
    menuAddNumber( m, cursor, "surface", &magLocator.boardZ, 0.0f, 60.0f, 0.5f, "mm" );
    menuAddNumber( m, cursor, "tip", &magLocator.tipOffsetMm, 0.0f, 60.0f, 0.5f, "mm" );
    menuAddNumber( m, cursor, "reach", &magLocator.track.maxReachMm, 5.0f, 100.0f, 5.0f, "mm" );

    int camera = menuAddSubmenu( m, MENU_ROOT, "camera" );
    menuAddChoice( m, camera, "mode", &cameraModeChoice, cameraModeNames, CAMERA_MODE_COUNT );
    menuAddAction( m, camera, "reset view", ACTION_RESET_VIEW, false );

#if MODULE_PROBE_LEDS
    int leds = menuAddSubmenu( m, MENU_ROOT, "LEDs" );
    menuAddChoice( m, leds, "layout", &ledLayoutChoice, ledLayoutNames, 2 );
    if ( probeLeds.strip ) {
        menuAddToggle( m, leds, "chain on", &stripOn );
        menuAddAction( m, leds, "test chain", 'n', false );
    }
    menuAddNumber( m, leds, "bright", &probeLeds.style.peak, 0.05f, 1.0f, 0.05f, "" );
    menuAddNumber( m, leds, "strip", &probeLeds.stripBrightness, 0.02f, 1.0f, 0.02f, "" );
    menuAddNumber( m, leds, "budget mA", &probeLeds.stripMaxMa, 100.0f, PROBELED_STRIP_HARD_MAX_MA, 100.0f, "" ); // the ceiling is the most it can be (renamed from "max mA" so the saved 300 gives way to the new default)
    menuAddChoice( m, leds, "colours", &probeLeds.style.scheme, probeLedSchemeNames, PROBELED_SCHEME_COUNT );
    menuAddToggle( m, leds, "full peak", &probeLeds.style.fullPeak );
    menuAddNumber( m, leds, "bloom", &probeLeds.style.bloom, 0.0f, 1.0f, 0.1f, "" );
    menuAddNumber( m, leds, "sparkle", &probeLeds.style.sparkle, 0.0f, 1.0f, 0.1f, "" );
    menuAddNumber( m, leds, "pulse", &probeLeds.style.pulse, 0.0f, 1.0f, 0.1f, "" );
    menuAddNumber( m, leds, "fade", &probeLeds.style.decayS, 0.05f, 2.0f, 0.05f, "s" );
    menuAddToggle( m, leds, "touch ring", &probeLeds.style.touchRing );
    menuAddToggle( m, leds, "V5 stream", &ledStream );
#endif

#if MODULE_ROW_COUNT
    // The row counter's tools: what r, c, R, C and h do on the console.
    int rows = menuAddSubmenu( m, MENU_ROOT, "rows" );
    menuAddToggle( m, rows, "row mode", &rowMode );
    menuAddInfo( m, rows, "row", rowInfo );
    menuAddAction( m, rows, "calibrate 12 taps", 'c', false );
    menuAddAction( m, rows, "anchor at row", 'R', true );
    menuAddAction( m, rows, "forget anchors", 'C', false );
    menuAddAction( m, rows, "hold-still test", 'h', false );
#endif

#if MODULE_PLAY
    int playPage = menuAddSubmenu( m, MENU_ROOT, "play" );
    menuAddChoice( m, playPage, "mode", &play.mode, playModeNames, PLAY_MODE_COUNT );
    menuAddNumber( m, playPage, "hue", &play.paintHue, 0.0f, 359.0f, 5.0f, "deg" ); // the draw screen's colour wheel sets these too
    menuAddNumber( m, playPage, "sat", &play.paintSat, 0.0f, 1.0f, 0.05f, "" );
    menuAddNumber( m, playPage, "paint bright", &play.paintBright, PLAY_BRIGHT_STEP, 1.0f, PLAY_BRIGHT_STEP, "" ); // nav up/down on the draw screen
    menuAddNumber( m, playPage, "brush", &play.brushSize, 0.0f, (float)PLAY_BRUSH_MAX, 1.0f, "rows" );             // nav left/right there
    menuAddNumber( m, playPage, "touch", &play.touchMm, 0.2f, 5.0f, 0.2f, "mm" );                                  // the point paints below this height
    menuAddAction( m, playPage, "clear", 'W', false );
    menuAddInfo( m, playPage, "target", playInfo );
#endif

    // The magnet: k, K, z, o, l.
    int magnet = menuAddSubmenu( m, MENU_ROOT, "magnet" );
    menuAddInfo( m, magnet, "strength", magnetInfo );
    menuAddAction( m, magnet, "learn strength", 'k', false );
    menuAddAction( m, magnet, "forget strength", 'K', false );
    menuAddAction( m, magnet, "re-zero (away!)", 'z', false );
    menuAddAction( m, magnet, "orientation check", 'o', false );
    menuAddAction( m, magnet, "latest fix", 'l', false );

    // The sensors and the machine: m, b, p, X, s.
    int sensors = menuAddSubmenu( m, MENU_ROOT, "sensors" );
    menuAddInfo( m, sensors, "sensors", sensorsInfo );
    menuAddAction( m, sensors, "array status", 'm', false );
    menuAddAction( m, sensors, "bus check", 'b', false );
    menuAddAction( m, sensors, "power-cycle", 'p', false );
    menuAddAction( m, sensors, "service table", 'X', false );
#if MODULE_SETTINGS
    menuAddAction( m, sensors, "saved settings", 's', false );
#endif

#if MODULE_ROW_COUNT
    menuAddToggle( m, MENU_ROOT, "row mode", &rowMode );
#endif

    // Every console command, as it is (the ones registered after this menu
    // is built are added by addNewCommands()).
    commandsSubmenu = menuAddSubmenu( m, MENU_ROOT, "commands" );
    addNewCommands( );
#if MODULE_SETTINGS
    menuAddAction( m, MENU_ROOT, "reset settings", ACTION_RESET_SETTINGS, false );
#endif
    if ( menuAddAction( m, MENU_ROOT, "close", ACTION_CLOSE, false ) < 0 || m->count >= MENU_MAX_ITEMS ) {
        uiStream.println( "menu: the item table is full - raise MENU_MAX_ITEMS (items are being dropped)" );
    }
}

// Console commands not yet in the commands submenu, appended to it: called
// from buildMenu(), and again from main() once every module has registered
// (the settings module registers after the menu is built, because it needs
// the menu).
void Ui::addNewCommands( ) {
    static char labels[ CONSOLE_MAX_COMMANDS ][ 36 ];
    Menu* m = &menu;
    for ( int i = commandsAdded; i < consoleCommandCount( ) && i < CONSOLE_MAX_COMMANDS; i++ ) {
        char key;
        const char* help;
        if ( !consoleCommandAt( i, &key, &help ) )
            break;
        snprintf( labels[ i ], sizeof( labels[ i ] ), "%c %s", key, help );
        if ( menuAddAction( m, commandsSubmenu, labels[ i ], key, strstr( help, "<number>" ) != nullptr ) < 0 ) {
            uiStream.println( "menu: the item table is full - raise MENU_MAX_ITEMS (commands are being dropped)" );
            break;
        }
        commandsAdded = i + 1;
    }
}

void Ui::begin( ) {
    trackerOn = magLocator.track.enabled;
    cursorModeChoice = magLocator.track.cursorMode == MAGCURSOR_UNDER ? 0 : 1;
    cameraModeChoice = magView.cam.mode;
    buildMenu( );
}

void Ui::settingsLoaded( ) {
#if MODULE_PROBE_LEDS
    stripOn = probeLeds.strip;
    ledStream = probeLeds.streaming;
#endif
#if MODULE_ROW_COUNT
    rowMode = rowCounter.active;
#endif
    applyChoices( );
}

// The choice/toggle items write plain variables; this carries them into the
// modules (and back, so a console command that changed one shows right).
void Ui::applyChoices( ) {
    MagTrack& t = magLocator.track;
    if ( trackerOn != t.enabled ) {
        t.enabled = trackerOn;
        magTrackReset( &t );
    }
    MagCursorMode wanted = cursorModeChoice == 0 ? MAGCURSOR_UNDER : MAGCURSOR_POINTED;
    if ( wanted != t.cursorMode ) {
        t.cursorMode = wanted;
    }
    if ( cameraModeChoice != (int)magView.cam.mode ) {
        cameraSetMode( &magView.cam, (CameraMode)cameraModeChoice );
    }
#if MODULE_PROBE_LEDS
    if ( ( ledLayoutChoice == 1 ) != probeLeds.v5 ) {
        probeLeds.useV5( ledLayoutChoice == 1 );
    }
    probeLeds.streaming = ledStream;
    if ( stripOn != probeLeds.strip ) {
        consoleRunCommand( 'N', &uiStream ); // toggles, and clears the chain when switching it off
        stripOn = probeLeds.strip;
    }
#endif
#if MODULE_ROW_COUNT
    if ( rowMode != rowCounter.active ) {
        consoleRunCommand( 'r', &uiStream );
        rowMode = rowCounter.active;
    }
#endif
}

// What the modules show back into the menu's variables (a console command
// may have changed them).
static void readBack( Ui* u, bool* trackerOn, int* cursorChoice, int* cameraChoice, int* ledChoice, bool* ledStream, bool* rowMode ) {
    *trackerOn = magLocator.track.enabled;
    *cursorChoice = magLocator.track.cursorMode == MAGCURSOR_UNDER ? 0 : 1;
    *cameraChoice = magView.cam.mode;
#if MODULE_PROBE_LEDS
    *ledChoice = probeLeds.v5 ? 1 : 0;
    *ledStream = probeLeds.streaming;
    u->stripOn = probeLeds.strip;
#else
    (void)u;
    (void)ledChoice;
    (void)ledStream;
#endif
#if MODULE_ROW_COUNT
    *rowMode = rowCounter.active;
#else
    (void)rowMode;
#endif
}

void Ui::runAction( int action, float number, bool withNumber ) {
    if ( action == ACTION_RESET_VIEW ) {
        cameraReset( &magView.cam );
        return;
    }
    if ( action == ACTION_CLOSE ) {
        ::menuClose( &menu );
        return;
    }
#if MODULE_SETTINGS
    if ( action == ACTION_RESET_SETTINGS ) {
        // Every menu value back to its default and saved so (the reset
        // carries the choice items into the modules itself).
        settings.reset( &uiStream );
        magView.screen = MAGVIEW_SCREEN_LOG; // it said what it did
        logScroll = 0;
        return;
    }
#endif
    if ( action >= 0 && action < 128 ) {
        // A console command, output to the log (and the serial port). One that
        // asks for a number gets it typed in ahead of time.
        if ( withNumber ) {
            char digits[ 12 ];
            snprintf( digits, sizeof( digits ), "%ld\n", (long)( number + 0.5f ) );
            uiStream.inject( digits );
        }
        uint32_t before = uiStream.logGeneration( );
        MagViewScreen screenBefore = magView.screen;
        consoleRunCommand( (char)action, &uiStream );
        if ( uiStream.logGeneration( ) != before && magView.screen == screenBefore ) {
            uiStream.print( "^ " );
            uiStream.println( (char)action );
            magView.screen = MAGVIEW_SCREEN_LOG; // it said something (and did not pick a screen itself): show it
            logScroll = 0;
        }
    }
}

void Ui::handleMenuKey( MenuKey key, bool repeat ) {
    float number = 0.0f;
    bool wasEditingAction = menu.editing;
    int action = menuKey( &menu, key, repeat, &number );
    menuEdits++;
    if ( action >= 0 ) {
        // A command may have changed what the choice items mirror (g, u, r,
        // B, L, v, e...): read the modules back rather than write over them.
        runAction( action, number, wasEditingAction );
        readBack( this, &trackerOn, &cursorModeChoice, &cameraModeChoice, &ledLayoutChoice, &ledStream, &rowMode );
    } else {
        applyChoices( );
    }
}

void Ui::handleScreenEvent( int control, int kind ) {
    bool press = kind == IN_PRESS || kind == IN_REPEAT;
    switch ( magView.screen ) {
    case MAGVIEW_SCREEN_SCENE:
        if ( press ) {
            switch ( control ) {
            case IN_NAV_LEFT:
                cameraPan( &magView.cam, -UI_PAN_MM, 0.0f );
                break;
            case IN_NAV_RIGHT:
                cameraPan( &magView.cam, UI_PAN_MM, 0.0f );
                break;
            case IN_NAV_UP:
                cameraPan( &magView.cam, 0.0f, UI_PAN_MM );
                break;
            case IN_NAV_DOWN:
                cameraPan( &magView.cam, 0.0f, -UI_PAN_MM );
                break;
            case IN_NAV_PRESS:
                if ( kind == IN_PRESS )
                    cameraNextMode( &magView.cam );
                break;
            case IN_JOY_PRESS:
                if ( kind == IN_PRESS )
                    cameraReset( &magView.cam );
                break;
            default:
                break;
            }
        }
        if ( control == IN_JOY_PRESS && kind == IN_HOLD ) {
            cameraReset( &magView.cam );
            cameraSetMode( &magView.cam, CAMERA_FIXED );
        }
        break;
    case MAGVIEW_SCREEN_DRAW:
#if MODULE_PLAY
        // The paint's controls (in paint mode: the wheel is only drawn then):
        // the joystick's click draw/erase, the nav stick's up/down the
        // brightness, left/right the brush (the wheel is the joystick
        // itself, in service()).
        if ( play.mode != PLAY_PAINT ) {
            break;
        }
        if ( control == IN_JOY_PRESS && kind == IN_PRESS ) {
            play.erase = !play.erase;
        } else if ( control == IN_NAV_PRESS && kind == IN_HOLD ) {
            play.clearPaint( ); // the centre held: the drawing gone
        } else if ( press ) {
            switch ( control ) {
            case IN_NAV_UP:
                play.setPaintBright( play.paintBright + PLAY_BRIGHT_STEP );
                break;
            case IN_NAV_DOWN:
                play.setPaintBright( play.paintBright - PLAY_BRIGHT_STEP );
                break;
            case IN_NAV_RIGHT:
                if ( play.brushSize < PLAY_BRUSH_MAX - 0.5f )
                    play.brushSize += 1.0f;
                break;
            case IN_NAV_LEFT:
                if ( play.brushSize > 0.5f )
                    play.brushSize -= 1.0f;
                break;
            default:
                break;
            }
        }
#endif
        break;
    case MAGVIEW_SCREEN_LOG:
        if ( press ) {
            int most = uiStream.logCount( ) - UI_LOG_ROWS;
            if ( most < 0 )
                most = 0;
            if ( control == IN_NAV_UP && logScroll < most )
                logScroll++;
            if ( control == IN_NAV_DOWN && logScroll > 0 )
                logScroll--;
            if ( control == IN_NAV_PRESS )
                logScroll = 0;
        }
        break;
    default:
        break;
    }
}

ServiceStatus Ui::service( ) {
    uint32_t now = micros( );
    float dtS = lastUs == 0 ? 0.01f : ( now - lastUs ) * 1e-6f;
    lastUs = now;

    input.uiWantsEnter = menu.open;
    // The choice items mirror module state that console commands change too
    // (g, u, v, B, L, r): keep the mirrors current, so the menu shows the
    // truth and the settings module saves it. Not while the menu is being
    // edited: the mirror is then the value on its way in.
    if ( !menu.open ) {
        readBack( this, &trackerOn, &cursorModeChoice, &cameraModeChoice, &ledLayoutChoice, &ledStream, &rowMode );
    }
    InputEvent e;
    bool anything = false;
    while ( input.next( &e ) ) {
        anything = true;
        // Button A opens and closes the menu from anywhere.
        if ( e.control == IN_BTN_A && e.kind == IN_PRESS ) {
            if ( menu.open ) {
                ::menuClose( &menu );
            } else {
                readBack( this, &trackerOn, &cursorModeChoice, &cameraModeChoice, &ledLayoutChoice, &ledStream, &rowMode );
                ::menuOpen( &menu );
            }
            continue;
        }
        if ( menu.open ) {
            bool repeat = e.kind == IN_REPEAT;
            if ( e.kind != IN_PRESS && e.kind != IN_REPEAT )
                continue;
            switch ( e.control ) {
            case IN_NAV_UP:
            case IN_JOY_UP:
                handleMenuKey( MENUKEY_UP, repeat );
                break;
            case IN_NAV_DOWN:
            case IN_JOY_DOWN:
                handleMenuKey( MENUKEY_DOWN, repeat );
                break;
            case IN_NAV_LEFT:
            case IN_JOY_LEFT:
                handleMenuKey( MENUKEY_LEFT, repeat );
                break;
            case IN_NAV_RIGHT:
            case IN_JOY_RIGHT:
                handleMenuKey( MENUKEY_RIGHT, repeat );
                break;
            case IN_NAV_PRESS:
            case IN_JOY_PRESS:
                if ( !repeat )
                    handleMenuKey( MENUKEY_ENTER, false );
                break;
            case IN_BTN_B:
                if ( !repeat )
                    handleMenuKey( MENUKEY_BACK, false );
                break;
            default:
                break;
            }
            continue;
        }
        if ( e.control == IN_BTN_B && e.kind == IN_PRESS ) {
            magView.nextScreen( );
            continue;
        }
        handleScreenEvent( e.control, e.kind );
    }

    // The joystick, continuously: on the draw screen it moves the colour
    // wheel's marker; on the 3D screen it orbits, or zooms with the stick
    // pressed.
#if MODULE_PLAY
    if ( !menu.open && magView.screen == MAGVIEW_SCREEN_DRAW && play.mode == PLAY_PAINT && ( input.joyX != 0.0f || input.joyY != 0.0f ) ) {
        play.movePicker( input.joyX * PLAY_PICK_PER_S * dtS, input.joyY * PLAY_PICK_PER_S * dtS );
        anything = true;
    }
#endif
    if ( !menu.open && magView.screen == MAGVIEW_SCREEN_SCENE && ( input.joyX != 0.0f || input.joyY != 0.0f ) ) {
        if ( input.held( IN_JOY_PRESS ) ) {
            float f = 1.0f + input.joyY * UI_ZOOM_PER_S * dtS;
            cameraZoom( &magView.cam, f );
        } else {
            cameraOrbit( &magView.cam, input.joyX * UI_ORBIT_DEG_PER_S * dtS, input.joyY * UI_ORBIT_DEG_PER_S * dtS );
        }
        anything = true;
    }

    lastStatus = anything ? ServiceStatus::BUSY : ServiceStatus::IDLE;
    return lastStatus;
}

// ---- drawing ---------------------------------------------------------------------

void Ui::drawMenu( GFXcanvas16* canvas ) {
    // Size-2 text: 12 px characters, 20 to a line; a panel of UI_MENU_ROWS
    // rows of 18 px with a title above and a one-line hint below.
    const int T = MAGVIEW_TEXT, charW = MAGVIEW_CHAR_W, rowH = MAGVIEW_LINE_H + 2;
    const int x0 = 4, y0 = 4, w = LCD_WIDTH - 8;
    const int columns = ( w - 8 ) / charW; // characters across the panel
    int visible = menuVisibleCount( &menu );
    int rows = visible < UI_MENU_ROWS ? visible : UI_MENU_ROWS;
    int h = rowH + 2 + rows * rowH + 12;
    fastFillRect( canvas, x0, y0, w, h, UI_COLOR_PANEL );
    fastRect( canvas, x0, y0, w, h, UI_COLOR_FRAME );
    fastText( canvas, x0 + 4, y0 + 3, T, UI_COLOR_FRAME, menuTitle( &menu ) );

    if ( menu.cursor < scrollTop )
        scrollTop = menu.cursor;
    if ( menu.cursor >= scrollTop + rows )
        scrollTop = menu.cursor - rows + 1;
    if ( scrollTop > visible - rows )
        scrollTop = visible - rows < 0 ? 0 : visible - rows;

    char text[ 48 ];
    for ( int r = 0; r < rows; r++ ) {
        int n = scrollTop + r;
        int index = menuVisibleItem( &menu, n );
        if ( index < 0 )
            break;
        const MenuItem& item = menu.items[ index ];
        bool selected = n == menu.cursor;
        int y = y0 + rowH + 2 + r * rowH;
        uint16_t colour = selected ? ( menu.editing ? UI_COLOR_EDIT : UI_COLOR_SELECTED ) : UI_COLOR_TEXT;
        if ( selected ) {
            fastText( canvas, x0 + 4, y, T, colour, ">" );
        }
        char value[ 24 ] = "";
        switch ( item.kind ) {
        case MENU_SUBMENU:
            snprintf( value, sizeof( value ), ">" );
            break;
        case MENU_TOGGLE:
            snprintf( value, sizeof( value ), *item.flag ? "on" : "off" );
            break;
        case MENU_NUMBER:
            snprintf( value, sizeof( value ), item.step >= 1.0f ? "%.0f%s" : ( item.step >= 0.1f ? "%.1f%s" : "%.2f%s" ), *item.value, item.unit );
            break;
        case MENU_CHOICE:
            snprintf( value, sizeof( value ), "%s", item.names[ *item.choice ] );
            break;
        case MENU_ACTION:
            if ( item.takesNumber && selected && menu.editing ) {
                snprintf( value, sizeof( value ), "%.0f", menu.editValue );
            }
            break;
        case MENU_INFO:
            item.info( value, sizeof( value ) );
            break;
        }
        // The label gets what the value leaves: the value is right-aligned.
        int valueChars = (int)strlen( value );
        int labelChars = columns - 1 - valueChars - ( valueChars > 0 ? 1 : 0 );
        if ( labelChars < 4 )
            labelChars = 4;
        snprintf( text, sizeof( text ), "%.*s", labelChars, item.label );
        fastText( canvas, x0 + 4 + charW, y, T, colour, text );
        if ( value[ 0 ] ) {
            fastText( canvas, x0 + w - 4 - charW * valueChars, y, T, colour, value );
        }
    }
    if ( visible > rows ) {
        snprintf( text, sizeof( text ), "%d/%d", menu.cursor + 1, visible );
        fastText( canvas, x0 + w - 4 - charW * (int)strlen( text ), y0 + 3, T, UI_COLOR_DIM, text );
    }
    fastText( canvas, x0 + 4, y0 + h - 10, 1, UI_COLOR_DIM, menu.editing ? "up/down: change  press: keep  B: cancel" : "press: enter  B: back  A: close" );
}

void Ui::drawLog( GFXcanvas16* canvas ) {
    const int T = MAGVIEW_TEXT, H = MAGVIEW_LINE_H;
    int count = uiStream.logCount( );
    int rows = count < UI_LOG_ROWS ? count : UI_LOG_ROWS;
    for ( int r = 0; r < rows; r++ ) {
        // Oldest of the shown lines at the top: line (rows - 1 - r + scroll) back.
        int back = rows - 1 - r + logScroll;
        const char* line = uiStream.logLine( back );
        if ( line == nullptr )
            continue;
        fastText( canvas, 0, 2 + r * H, T, back == 0 ? UI_COLOR_TEXT : UI_COLOR_DIM, line );
    }
    fastText( canvas, 0, LCD_HEIGHT - H - 2, T, UI_COLOR_FRAME, logScroll > 0 ? "log (scrolled)" : "log  up/down, press" );
}
