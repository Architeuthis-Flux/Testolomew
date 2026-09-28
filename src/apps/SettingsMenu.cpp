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
static void runForgetLean( int, float ) {
    magLocator.forgetTipModel( );
    uiStream.println( "lean forgotten: the point is the magnet less tracking/tip along the shaft, whatever the lean" );
    ui.showResult( "forget lean" );
}

static void runOpenApp( int app, float ) {
    appsOpen( app );
}

static void runOpenRows( int app, float arg ) {
    calibrateSetKind( 0 );
    runOpenApp( app, arg );
}

static void runOpenLean( int app, float arg ) {
    calibrateSetKind( 1 );
    runOpenApp( app, arg );
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
    menuAddInfo( m, tracking, "state", trackInfo,
                 "The track's state: none; rough (a far fix with a wide bar); coasting (no fix for a moment, carried on from the last ones); tracking. Info only." );
    menuAddInfo( m, tracking, "fix", fixInfo,
                 "The latest raw fix: where the magnet is and its error bar, before any smoothing. Info only." );
    menuAddToggleAccessor( m, tracking, "tracker on", getTrackerOn, setTrackerOn,
                 "The Kalman tracker between the fits and everything shown. On: fixes are weighed by their error bars, a jump is gated, a gap is coasted through for 0.4 s. Off: every fix passes straight through (for comparing). The smoothing levers below apply either way." );
    menuAddChoiceAccessor( m, tracking, "cursor", getCursorMode, setCursorMode, cursorModeNames, 2,
                 "What the LED cursor marks. under: straight below the tip. aim: where the tip points on the surface, down the shaft, never further than reach." );
    menuAddNumber( m, tracking, "surface", &magLocator.boardZ, 0.0f, 60.0f, 0.5f, "mm",
                 "How high the breadboard's surface sits above the sensors at the taps' centre, mm. The height shown, where the aim cursor lands and the touch come from it. The 12-tap calibration also learns how the surface runs across the board (a tilt, a bow), so the height reads the same in every hole; forget anchors flattens it." );
    menuAddNumber( m, tracking, "tip", &magLocator.tipOffsetMm, 0.0f, 60.0f, 0.5f, "mm",
                 "The magnet's centre sits this far up the shaft from the probe's point, mm. Wrong here and the point lands off along the lean - a row off at 40 degrees for 3 mm. The lean calibration (tools) measures it, with the fix's own lean bias; editing this by hand drops that model." );
    menuAddNumber( m, tracking, "reach", &magLocator.track.maxReachMm, 5.0f, 60.0f, 2.5f, "mm",
                 "The aim cursor never lands further from under the tip than this, mm. Lower it if a flat lean throws the cursor across the board." ); // (5-100 until 2026-09-25: the reach is the drop x tan 70 deg at most - 55 mm from a 20 mm hover - so 60 is the useful end)
    // The smoothing, the levers that are felt first (tools/hostsim/pencil.cpp,
    // 2026-09-25: the ranges are where the pencil measured a change):
    // accel sets the Kalman's rest jitter against its lag (0.075 mm at 500
    // to 0.5 at 20000 in the bench-like world, 40 to 20 ms), rest jitter
    // the field EMA's (0: three times the jitter; 2: half, +10 ms), the
    // betas how fast the 1-Euro filters open with speed (the lag's whole
    // effect is under 0.5, the jitter's runs on to 2), the Hz where they sit at
    // rest - which only shows with the beta low, since the rest jitter's
    // own speed opens the cutoff by beta x 2-5 mm/s.
    menuAddNumber( m, tracking, "accel", &magLocator.track.accelSigma, 250.0f, 20000.0f, 250.0f, "",
                 "The tracker's process noise, mm/s2: how jerky a hand may be. Lower: steadier at rest, more lag when moving. Higher: follows a fast hand, jitters more at rest. Tracker on only." );
    menuAddNumber( m, tracking, "rest jitter", &magLocator.speedJitterK, 0.0f, 3.0f, 0.1f, "",
                 "How much of the fix's own error bar is taken out of the speed the readings' smoothing follows. 0: the smoothing opens at rest and the fix jitters. 2: it stays closed at rest, for 10 ms more lag." );
    menuAddNumber( m, tracking, "view Hz", &magLocator.track.viewMinCutoff, 0.1f, 5.0f, 0.1f, "",
                 "The 3D scene's smoothing of the magnet: where its filter sits at rest, Hz. Lower is calmer and later - 0.1 is well over 100 ms behind." );
    menuAddNumber( m, tracking, "view beta", &magLocator.track.viewBeta, 0.0f, 2.0f, 0.02f, "",
                 "How fast the scene's filter opens with speed, per mm/s. 0: the same lag at any speed. Higher: fast moves are followed at once, and the rest jitter opens it too." );
    menuAddNumber( m, tracking, "cursor Hz", &magLocator.track.oneEuroMinCutoff, 0.1f, 5.0f, 0.1f, "",
                 "The LED cursor's smoothing: where its filter sits at rest, Hz. Lower: a calmer cursor that lags more. 1 Hz is about 160 ms to settle." );
    menuAddNumber( m, tracking, "cursor beta", &magLocator.track.oneEuroBeta, 0.0f, 2.0f, 0.02f, "",
                 "How fast the cursor's filter opens with speed, per mm/s. Higher: a fast hand is followed with less lag - but the rest jitter's own speed opens it too, so more jitter at rest." );
    menuAddToggle( m, tracking, "smoothing", &magLocator.track.smooth,
                 "The cursor's, the scene's and the shaft's smoothing filters (the Hz, beta and height levers around it). Off: what is shown is the track's own output - with the tracker off too, the bare fix, raw, for comparing." );
    menuAddNumber( m, tracking, "height Hz", &magLocator.track.heightMinCutoff, 0.1f, 5.0f, 0.1f, "",
                   "The height's own smoothing (what the LEDs color and size by, and the View shows): where its filter sits at rest, Hz. Lower: a calmer height that lags more." );
    menuAddNumber( m, tracking, "height beta", &magLocator.track.heightBeta, 0.0f, 2.0f, 0.02f, "",
                   "How fast the height's filter opens with speed, per mm/s. Higher follows a lift or a landing at once; more jitter at rest." );
    menuAddNumber( m, tracking, "Hz halved at", &magLocator.track.hzHalfMm, 0.0f, 150.0f, 5.0f, "mm",
                 "The height above the surface, mm, at which the cursor's, the height's and the scene's Hz are halved; a third at twice it. A far fix is a noisy one. 0: the same at any height." ); // the cursor's and view's Hz halved at this height (0 = the same at any height)...
    menuAddNumber( m, tracking, "beta halved at", &magLocator.track.betaHalfMm, 0.0f, 150.0f, 5.0f, "mm",
                 "The height, mm, at which the cursor's, the height's and the scene's beta is halved; a third at twice it. Low: responsive close to the board, very smooth far up. 0: the same at any height." ); // ...and their betas: responsive close, smooth far
    menuAddNumber( m, tracking, "shaft Hz", &magLocator.track.shaftMinCutoff, 0.1f, 5.0f, 0.1f, "",
                 "The shaft direction's own filter at rest, Hz. The aim cursor and the tail follow the shaft; lower is calmer and later." );
    menuAddNumber( m, tracking, "shaft beta", &magLocator.track.shaftBeta, 0.0f, 10.0f, 0.25f, "",
                 "How fast the shaft's filter opens as the pencil turns. Higher follows a turn at once." );
    menuAddNumber( m, tracking, "floor", &magLocator.track.sigmaFloorMm, 0.1f, 3.0f, 0.1f, "mm",
                 "The array's systematic error, mm, added to every fix's error bar. The tracker's gate and weights use it, and the LED spot's width." );
    menuAddNumber( m, tracking, "gate", &magLocator.track.gate, 2.0f, 8.0f, 0.5f, "sd",
                 "A fix further than this many error bars from the track counts for less; past 11 it is dropped. Lower: more fixes ignored on a jump. Tracker on only." ); // (to 10 until 2026-09-25: the drop line is 11.3, past 8 the soft zone is nothing)
    menuAddNumber( m, tracking, "presence", &magLocator.presentMt, 0.02f, 0.20f, 0.01f, "mT",
                 "The strongest reading, mT, above which a magnet is there - two TMAGs must read it. Lower reaches further out; too low and a sensor's zero drift passes as a magnet (the toggles on :load say)." ); // (0.04-0.20 on 2026-09-25: below the TMAGs' plain level the lever did nothing, presence needing two of them at 0.04 anyway; since 2026-09-26 the two TMAGs are asked for the lever's level, so below 0.04 it reaches further out - as far as the zeros' drift allows)
    menuAddNumber( m, tracking, "fit chi", &magLocator.fitMaxChi, 1.0f, 6.0f, 0.5f, "",
                 "The fit's acceptance: its leftover residuals over the sensors' own errors. About 1 is a fit as good as the readings. Higher accepts more frames, wrong ones included; lower refuses more, and a refused frame is a gap." );
    menuAddNumber( m, tracking, "far hold", &magLocator.track.roughHoldS, 0.5f, 10.0f, 0.5f, "s",
                 "How long a rough (far) track outlives its last rough fix, s. Tracker on only." );
    int fitLoad = menuAddToggle( m, tracking, "fit load", &magLocator.steadyFit,
                 "burst: the fit costs what it costs each frame. steady: exactly fit iters iterations every frame, so the supply current does not swing (the backlight showed the bursts)." ); // steady: the same work every frame (the supply shows bursts)
    menuSetToggleText( m, fitLoad, "steady", "burst" );
    menuAddNumber( m, tracking, "fit iters", &magLocator.steadyIterations, 2.0f, 16.0f, 1.0f, "",
                 "Under the steady load, iterations a frame, about 0.85 ms each. 2 tracks as well as burst; more only makes the pulse bigger." ); // ...how much, a frame (~0.85 ms each); read only under steady

    int camera = menuAddSubmenu( m, MENU_ROOT, "camera" );
    menuAddChoiceAccessor( m, camera, "mode", getCameraMode, setCameraMode, cameraModeNames, CAMERA_MODE_COUNT,
                 "fixed, sway, spin, top, follow, POV: how the scene's camera moves. In the View the joystick orbits and zooms, the nav press steps this." );
    menuAddAction( m, camera, "reset view", 0, runResetView, false );
    menuAddNumber( m, camera, "glide", &viewCamera.tauS, 0.02f, 2.0f, 0.02f, "s",
                 "The camera's time constant for every change of view, s. Lower snaps, higher glides." ); // (the smoothing page's "camera" until 2026-09-25)
    menuAddNumber( m, camera, "POV turn", &viewCamera.povTurnTauS, 0.02f, 3.0f, 0.05f, "s",
                 "In POV mode, how slowly the view's direction follows the shaft, s." );
    menuAddNumber( m, camera, "POV move", &viewCamera.povMoveTauS, 0.02f, 3.0f, 0.05f, "s",
                 "In POV mode, how slowly the view's position follows the point, s." );

#if MODULE_PROBE_LEDS
    int leds = menuAddSubmenu( m, MENU_ROOT, "LEDs" );
    int layout = menuAddToggleAccessor( m, leds, "layout", getLayoutV5, setLayoutV5,
                 "V6: 16 x 30 holes and rails. V5: 5 + 5 holes across the channel, 4 x 25 rail LEDs. A wired V5 chain forces V5." ); // saved as 0/1 as the choice was
    menuSetToggleText( m, layout, "V5", "V6" );
    if ( probeLeds.strip ) {
        menuAddToggleAccessor( m, leds, "chain on", getStripOn, setStripOn,
                 "The V5 chain streaming; off darkens it. A mode: not saved." );
        menuAddAction( m, leds, "test chain", 'n', runConsoleKey, false );
    }
    menuAddNumber( m, leds, "bright", &probeLeds.style.peak, 0.05f, 1.0f, 0.05f, "",
                 "The cursor's peak level, 0-1. Everything else scales under it." );
    menuAddNumber( m, leds, "strip", &probeLeds.stripBrightness, 0.02f, 1.0f, 0.02f, "",
                 "A lever on everything sent to the chain, 0-1, after the cursor's own bright." );
    menuAddNumber( m, leds, "budget mA", &probeLeds.stripMaxMa, 100.0f, PROBELED_STRIP_HARD_MAX_MA, 100.0f, "",
                 "The chain's current budget per frame. A frame that would draw more is dimmed whole. Never above the compiled ceiling of 800: the rail browned out at 900-1100." ); // the ceiling is the most it can be
    menuAddToggle( m, leds, "full peak", &probeLeds.style.fullPeak,
                 "On: the brightest LED is always the peak, whatever the spot's width. Off: the total light is held, so a wide spot is a dim one." );
    menuAddNumber( m, leds, "spot", &probeLeds.style.spot, 0.1f, 4.0f, 0.1f, "",
                 "The size of the spot: 1 is one hole lit with its neighbours faint, 0.3 a pin, 2 twice as wide. The fix's error bar can widen it (error width). The LED nearest the cursor is always lit." );             // the spot's size, 1 = one hole
    menuAddNumber( m, leds, "spot by height", &probeLeds.style.spotByHeight, 0.0f, 4.0f, 0.1f, "",
                 "How much wider the spot grows per 10 mm of the point's lift: at 1 it is twice as wide 10 mm up and three times at 20. 0: the same at any height. A cone, the peak kept." );
    menuAddNumber( m, leds, "error width", &probeLeds.style.errorWidth, 0.0f, 3.0f, 0.1f, "",
                   "How much of the fix's error bar shows as the spot's width: 0 none (the spot is always spot wide, whatever the fit knows), 1 the bar as it is, 2 twice. The wider of this and spot is drawn." );
    menuAddNumber( m, leds, "falloff", &probeLeds.style.falloff, 0.3f, 3.0f, 0.1f, "",
                   "The spot's edge: 1 is a Gaussian bell; higher a flatter top with a sharper edge (3 is nearly a disc); lower a bright centre with a wide dim skirt." ); // ...and how much wider at the height scale
    menuAddNumber( m, leds, "bloom", &probeLeds.style.bloom, 0.0f, 1.0f, 0.1f, "",
                 "A soft halo three times as wide as the spot, at this fraction of its light." );
    menuAddNumber( m, leds, "sparkle", &probeLeds.style.sparkle, 0.0f, 1.0f, 0.1f, "",
                 "Random near-white flashes in the glow, 0-1. What raises their density is the colors page's sparkle by." );
    menuAddNumber( m, leds, "pulse", &probeLeds.style.pulse, 0.0f, 1.0f, 0.1f, "",
                 "The cursor breathes: how deeply, 0-1." );
    menuAddNumber( m, leds, "fade", &probeLeds.style.decayS, 0.05f, 2.0f, 0.05f, "s",
                 "How long a lit LED takes to go dark, s. Longer: a longer comet behind a moving cursor." );
    menuAddToggle( m, leds, "touch ring", &probeLeds.style.touchRing,
                 "A ring runs out from the point when it lands on the board." );
    menuAddToggle( m, leds, "ring repeat", &probeLeds.style.ringRepeat,
                 "...and again at every new hole the point slides to while it is down." ); // ...again at every new hole while down
    menuAddToggle( m, leds, "V5 stream", &probeLeds.streaming,
                 "The cursor as CSV on the console, 25 a second, for a V5 to follow. A mode: not saved." );

    // The colours (2026-09-25): the scheme (what drives the hue), how the
    // wheel is mapped onto it, what dims the cursor and what raises the
    // sparkle (ProbeLeds.h), and the View's poles and field arrows (Apps.h);
    // the page previews the mapping under its items (ColourPreview.h).
    ui.shell.pageRowsTaken = colourPreviewRows; // the shell steers by the rows left (the absolute stick)
    ui.menuPreview = colourPreviewDraw;
    int colours = menuAddSubmenu( m, MENU_ROOT, "colors" ); // (US spelling since 2026-09-28: "colors" until then, whose saved values retired to the defaults - the bench's own)
    menuAddChoice( m, colours, "scheme", &probeLeds.style.scheme, probeLedSchemeNames, PROBELED_SCHEME_COUNT,
                 "What drives the hue. classic: white on the board, blue in the air. height: the wheel from the board to height scale. sure: from a toss-up row to a certain one. aim: the direction the probe leans. rainbow: along the board, turning with time. Every scheme is white with the point on the board." );
    menuAddNumber( m, colours, "turns", &probeLeds.style.hueTurns, 0.1f, 4.0f, 0.05f, "",
                 "Turns of the wheel over the scheme's scale. Under 1 a chunk of the spectrum, 1 the whole spectrum once, over 1 several rainbows." ); // of the wheel over the scale: under 1 a chunk of the spectrum, over 1 several rainbows
    menuAddNumber( m, colours, "hue start", &probeLeds.style.hueStartDeg, 0.0f, 355.0f, 5.0f, "deg",
                 "Where the wheel starts, degrees: 0 red, 120 green, 240 blue." );
    menuAddNumber( m, colours, "height scale", &probeLeds.style.liftFullMm, 5.0f, 150.0f, 1.0f, "mm",
                 "The height, mm, the height scheme's wheel runs to. Classic's color is all blue there, and the spot's, the sparkle's and the dimming's height data end there too." ); // to the array's range: a far probe is drawn by the same mapping since 2026-09-27 (60 until then)
    menuAddChoice( m, colours, "on board", &probeLeds.style.onBoard, probeLedOnBoardNames, PROBELED_ONBOARD_COUNT,
                   "How a wheel scheme meets the board. white: white below 1.5 mm, then the color comes in by color from. fade: the color at color from fading smoothly to white at the board itself. color: the scheme's color at every height. The tap ring is white whichever; classic keeps its own ramp." );
    menuAddNumber( m, colours, "color from", &probeLeds.style.colourByMm, 0.0f, 100.0f, 0.5f, "mm",
                 "The height, mm, at which a wheel scheme's color is all in (on board: white or fade); below it the cursor is white, or blended toward white." );  // white on the board below 1.5 mm, all the colour from here up
    menuAddChoice( m, colours, "bright by", &probeLeds.style.brightBy, probeLedDataNames, PROBELED_DATA_COUNT,
                 "What dims the cursor: unsure (how far the row is from certain), height, tilt, speed, or none." );
    menuAddNumber( m, colours, "bright amount", &probeLeds.style.brightAmount, -1.0f, 1.0f, 0.05f, "",
                 "How much the chosen data changes the cursor at its far end, signed: +1 doubles the peak there (by speed: twice as bright when fast - never past full), -1 takes it to nothing. -0.5 by unsure: a coin-toss row at half." );
    menuAddChoice( m, colours, "sparkle by", &probeLeds.style.sparkleBy, probeLedDataNames, PROBELED_DATA_COUNT,
                 "What raises the sparkle's density from a tenth of the lever to all of it: height, unsure, tilt, speed, or none." );
    menuAddToggle( m, colours, "tail", &probeLeds.style.tail,
                   "In aim mode, a tail from the cursor back toward the point, along the lean, in the cursor's color fading to tail hue. Off: the spot alone (at full length and brightness it read as a bar)." );
    menuAddNumber( m, colours, "tail length", &probeLeds.style.tailLength, 0.1f, 1.0f, 0.05f, "",
                 "In aim mode a tail runs from the cursor back toward the point, this fraction of the way. 1 reaches the point." ); // in pointed mode: how far back toward the point the tail runs (2026-09-26)
    menuAddNumber( m, colours, "tail bright", &probeLeds.style.tailBright, 0.1f, 1.0f, 0.05f, "",
                 "The tail's near end, as a fraction of the cursor's peak." );
    menuAddNumber( m, colours, "tail hue", &probeLeds.style.tailHueDeg, 0.0f, 355.0f, 5.0f, "deg",
                 "The tail's color at its far end, degrees round the wheel. Its near end is the cursor's own color." );
#else
    int colours = menuAddSubmenu( m, MENU_ROOT, "colors" ); // (US spelling since 2026-09-28: "colors" until then, whose saved values retired to the defaults - the bench's own)
#endif
    menuAddToggle( m, colours, "poles", &viewStyle.poles,
                 "The View's magnet bar: its north and south halves colored, or a thin plain bar." ); // the View's magnet bar: north and south coloured, or a plain bar
    menuAddNumber( m, colours, "pole size", &viewStyle.poleMm, 1.0f, 15.0f, 0.5f, "mm",
                 "The bar's half length, mm." );
    menuAddNumber( m, colours, "north hue", &viewStyle.northHueDeg, 0.0f, 355.0f, 5.0f, "deg",
                 "The bar's north half, degrees round the wheel." );
    menuAddNumber( m, colours, "south hue", &viewStyle.southHueDeg, 0.0f, 355.0f, 5.0f, "deg",
                 "The bar's south half, degrees round the wheel." );
    menuAddNumber( m, colours, "field arrows", &viewStyle.fieldArrows, 0.0f, 1.0f, 0.1f, "",
                 "The sensors' field arrows in the View: their length as a fraction, 0 for none. They grow with the field." ); // the sensors' arrows in the View: their length, 0 = none

    // The one-shot things: the magnet (k, K, z, o, l), the sensors and the
    // machine (m, b, p, X, s), the rows (r, c, R, C, h).
    int tools = menuAddSubmenu( m, MENU_ROOT, "tools" );
    menuAddInfo( m, tools, "strength", magnetInfo,
                 "The magnet's strength the fit holds, mT*mm3: learned from the near fixes and kept across boots. Info only." );
    menuAddAction( m, tools, "learn strength", 'k', runConsoleKey, false );
    menuAddAction( m, tools, "forget strength", 'K', runConsoleKey, true );
    menuAddAction( m, tools, "re-zero (away!)", 'z', runConsoleKey, true );
    menuAddAction( m, tools, "orientation check", 'o', runConsoleKey, false );
    menuAddAction( m, tools, "latest fix", 'l', runConsoleKey, false );
    menuAddInfo( m, tools, "sensors", sensorsInfo,
                 "How many sensors answer, and the frame rate. Info only." );
    menuAddToggle( m, tools, "use MMC", &magArray.useMmc,
                 "The MMC56x3 at the centre in the fit, or ignored. It is the far sensor, 30-50x quieter than a TMAG, but near the board its readings have been sketchy." ); // the MMC56x3 in the fit or ignored (MAG_USE_MMC_AT_BOOT: off)
    menuAddAction( m, tools, "array status", 'm', runConsoleKey, false );
    menuAddAction( m, tools, "bus check", 'b', runConsoleKey, false );
    menuAddAction( m, tools, "power-cycle", 'p', runConsoleKey, true );
    menuAddAction( m, tools, "service table", 'X', runConsoleKey, false );
#if MODULE_SETTINGS
    menuAddAction( m, tools, "saved settings", 's', runConsoleKey, false );
#endif
#if MODULE_ROW_COUNT
    menuAddToggleAccessor( m, tools, "row mode", getRowMode, setRowMode,
                 "Which breadboard row the probe is over, on the LCD and the console. A mode: not saved." );
    menuAddInfo( m, tools, "row", rowInfo,
                 "The counted row and how sure. Info only." );
    menuAddAction( m, tools, "calibrate 12 taps", APP_CALIBRATE, runOpenRows, false );
    menuAddAction( m, tools, "calibrate lean", APP_CALIBRATE, runOpenLean, false ); // the point in one hole, six holds: the tip model (2026-09-28)
    menuAddAction( m, tools, "forget lean", 0, runForgetLean, true );
    menuAddNumberAction( m, tools, "anchor at row", 'R', runConsoleKeyWithNumber, 1.0f, 60.0f, 1.0f, 1.0f );
    menuAddAction( m, tools, "forget anchors", 'C', runConsoleKey, true );
    menuAddAction( m, tools, "hold-still test", 'h', runConsoleKey, false );
#endif

#if MODULE_PLAY
    int playPage = menuAddSubmenu( m, MENU_ROOT, "play" );
    menuAddChoice( m, playPage, "mode", &play.mode, playModeNames, PLAY_MODE_COUNT,
                 "off, paint, target. The Draw and Target apps set this on entry and clear it on leaving." );
    menuAddNumber( m, playPage, "hue", &play.paintHue, 0.0f, 359.0f, 5.0f, "deg",
                 "The brush color: degrees round the wheel." ); // the Draw app's colour wheel sets these too
    menuAddNumber( m, playPage, "sat", &play.paintSat, 0.0f, 1.0f, 0.05f, "",
                 "The brush color's saturation: 0 white, 1 the rim of the wheel." );
    menuAddNumber( m, playPage, "paint bright", &play.paintBright, PLAY_BRIGHT_STEP, 1.0f, PLAY_BRIGHT_STEP, "",
                 "The brush's level. What is painted keeps the level it got." ); // nav up/down in the Draw app
    menuAddNumber( m, playPage, "brush", &play.brushSize, 0.0f, (float)PLAY_BRUSH_MAX, 1.0f, "rows",
                 "0-3 rows around the LED under the point, with a soft edge." );             // nav left/right there
    menuAddNumber( m, playPage, "touch", &play.touchMm, 0.2f, 5.0f, 0.2f, "mm",
                 "The height, mm, below which the point paints, with 0.7 mm of hysteresis." );                                  // the point paints below this height
    menuAddAction( m, playPage, "clear", 'W', runConsoleKey, false );
    menuAddInfo( m, playPage, "target", playInfo,
                 "The target game's tally. Info only." );
#endif

    // The feel of the controls (Input.h): how long a tilt of the nav stick
    // must hold before it counts, longer while its push contact is closed
    // (a centre push wobbles into a direction first), and how far the
    // joystick goes before it is a menu direction.
    int controls = menuAddSubmenu( m, MENU_ROOT, "controls" );
    menuAddNumber( m, controls, "direction ms", &input.navDirectionMs, 0.0f, 150.0f, 5.0f, "",
                 "How long a nav contact must hold before it is a direction, and the push contact alone before it is the press. Lower is quicker, with more false directions." );
    menuAddNumber( m, controls, "push guard ms", &input.navPushGuardMs, 0.0f, 200.0f, 5.0f, "",
                 "The same for a direction while the push contact is closed too - it closes on every tilt. Raise it if a press ever moves the cursor, lower it for speed." );
    menuAddNumber( m, controls, "joy menu at", &input.joyMenuAt, 0.1f, 0.9f, 0.05f, "",
                 "How far the joystick goes, 0-1 of its travel, before it counts as a menu direction." );
    menuAddNumber( m, controls, "joy menu off", &input.joyMenuOff, 0.05f, 0.9f, 0.05f, "",
                 "...and how far back before that direction is over. Kept under joy menu at." ); // ...and over inside this (kept under 'at')
    menuAddNumber( m, controls, "joy dead", &input.joyDead, 0.02f, 0.30f, 0.02f, "",
                 "The analog stick's inner dead zone, as a fraction of its travel. Inside it the stick is nothing." ); // the analog stick's inner dead zone (2026-09-25)
    menuAddNumber( m, controls, "cursor repeat", &ui.shell.stepRepeatMs, 100.0f, 800.0f, 50.0f, "ms",
                 "A held up/down on a page steps the cursor no faster than this, ms." ); // a held up/down (Home: any way) steps the cursor no faster than this (2026-09-25)
    int joystick = menuAddToggle( m, controls, "joystick", &ui.shell.absoluteJoystick,
                 "relative: the stick steps the cursor. absolute: while the stick is going out, its position is the cursor on Home and the pages." ); // absolute: the stick's position is the cursor on Home and the menu pages
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
