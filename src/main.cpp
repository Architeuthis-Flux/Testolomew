// SPDX-License-Identifier: MIT
// ---------------------------------------------------------------------------
// Testolomew - the Jumperless V6 idea test bed.
//
// setup() brings up the board, then each module that config.h switches on:
// begin() it, register its Service with jOS. loop() is the scheduler. That is
// the whole program - everything interesting is in a module folder.
//
// To add a module: make a folder under src/, write a Service in it (copy the
// shape of MagLocator - the smallest one), add a MODULE_ flag to config.h and a
// block below. Console commands are registered by the module itself in its
// begin(), so nothing else changes.
//
// Both CH32H417 cores are available (setup1()/loop1() run on the 100 MHz V3F,
// as in arduino-pico); everything here runs on the 400 MHz V5F for now.
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "BoardPins.h"
#include "Console.h"
#include "DumpService.h"
#include "JumperlOS.h"
#include "config.h"

#if MODULE_MAG_ARRAY
#include "MagArray.h"
#include "MagSampler.h"
#include "FastDraw.h"
#endif
#if MODULE_MAG_LOCATOR
#include "MagLocator.h"
#endif
#if MODULE_MAG_VIEW
#include "Display.h"
#include "ST7789.h"
#endif
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif
#if MODULE_PROBE_LEDS
#include "LedStrip.h"
#include "ProbeLedService.h"
#endif
#if MODULE_UI
#include "Apps.h"
#include "Input.h"
#include "SettingsMenu.h"
#include "Ui.h"
#include "UiStream.h"
#endif
#if MODULE_SETTINGS
#include "Settings.h"
#endif
#if MODULE_PLAY
#include "Play.h"
#endif

#if MODULE_MAG_LOCATOR && !MODULE_MAG_ARRAY
#error "MODULE_MAG_LOCATOR needs MODULE_MAG_ARRAY"
#endif
#if MODULE_MAG_VIEW && !MODULE_MAG_LOCATOR
#error "MODULE_MAG_VIEW needs MODULE_MAG_LOCATOR"
#endif
#if MODULE_UI && !MODULE_PROBE_LEDS
#error "MODULE_UI needs MODULE_PROBE_LEDS (the LEDs and Draw apps)"
#endif
#if MODULE_ROW_COUNT && !MODULE_MAG_LOCATOR
#error "MODULE_ROW_COUNT needs MODULE_MAG_LOCATOR"
#endif
#if MODULE_PROBE_LEDS && !MODULE_MAG_LOCATOR
#error "MODULE_PROBE_LEDS needs MODULE_MAG_LOCATOR"
#endif
#if MODULE_UI && !MODULE_MAG_VIEW
#error "MODULE_UI needs MODULE_MAG_VIEW"
#endif
#if MODULE_SETTINGS && !MODULE_UI
#error "MODULE_SETTINGS needs MODULE_UI (it saves what the menu can edit)"
#endif
#if MODULE_PLAY && !MODULE_PROBE_LEDS
#error "MODULE_PLAY needs MODULE_PROBE_LEDS"
#endif

// A heartbeat on the green LED, so a glance says the scheduler is alive.
class HeartbeatService : public Service {
  public:
    ServiceStatus service( ) override {
        on = enabled && !on;
        boardLed( PIN_LED_GREEN, on );
        return ServiceStatus::IDLE;
    }
    const char* getName( ) const override { return "Heartbeat"; }
    ServicePriority getPriority( ) const override { return ServicePriority::LOW; }
    uint32_t periodUs( ) const override { return 500000; }
    bool enabled = true; // :load heartbeat off: the LED is on VDDIO through 1 k, 2 mA at 1 Hz

  private:
    bool on = false;
};

static HeartbeatService heartbeat;

#if MODULE_SETTINGS && MODULE_PROBE_LEDS
// The board in the dark for a settings write (Settings.h: aroundWrite).
// The page erase behind a save is a few tens of ms of extra current, and
// with the probe near the board - the chain lit around its cursor, up to
// the budget - it browned the board out (2026-09-21: using the menu with
// the probe on the board rebooted it; two saves with the probe off in a
// corner did not). So the chain is cleared first and the cleared frame
// waited out on the wire: a WS2812 keeps what it was last given, so the
// LEDs are not off until that frame has landed (12 ms for 400 of them).
// The chain comes back right after: the next tick sends the live frame. A
// save shows as one blink of the LEDs. Nothing else needs holding: the
// loop itself stalls on the erase (the fit and the display run in it),
// and the sampler on the other core draws next to nothing.
static bool quietStripWas = false;
static bool flashWriting = false;    // a settings page erase/program is under way (the flight recorder)
static uint32_t ledRestoreMs = 0;    // when the chain was last re-lit after one (0 = never)
static void quietForFlash( bool starting ) {
    flashWriting = starting;
    if ( starting ) {
        quietStripWas = probeLeds.strip;
        if ( quietStripWas ) {
            probeLeds.setStrip( false, nullptr ); // waits for the frame in flight, then starts the cleared one
            ledStripWait( &probeLeds.chain, 50 );
            if ( probeLeds.topStrip ) {
                ledStripWait( &probeLeds.top, 50 );
            }
            delayMicroseconds( 300 ); // the chain's latch: the frame shows once the wire has been low this long
        }
    } else if ( quietStripWas ) {
        probeLeds.setStrip( true, nullptr );
        ledRestoreMs = millis( ) == 0 ? 1 : millis( );
    }
}
#endif

// The flight recorder: what the board was doing when it last reset. The
// brown-outs (2026-09-21: the menu used with the probe on the board; the
// core's "rst=... por" line at boot says the rail went down, not what
// pulled it) leave nothing in RAM that the firmware can read - except this
// region, .xcore, which the linker does not load or clear and a reset does
// not touch (the core keeps its own fault log there). Every tick the state
// that draws from the rail is written here; at boot, if the record checks
// out, it is printed under the core's rst= line. A power-cycle leaves
// noise in it: the magic word (with the record's size in it, so an older
// build's bytes are not read through this layout), a sum over the fields
// and a range check tell noise from a record.
#ifndef CH32H4_XCORE
#define CH32H4_XCORE // the host build: ordinary RAM
#endif
#define FLIGHT_LABEL 24
struct FlightRecord {
    uint32_t magic;
    uint32_t uptimeMs;
    float stripMa, stripStepMa, budgetMa; // the chain's last frame, the biggest step in the report window, the budget
    uint32_t ledRestoreAgoMs;             // since the chain was re-lit after a settings write (0xFFFFFFFF = not since boot)
    uint32_t coldStarts;
    float peakMt;
    uint8_t fitValid, fitRough, present, settingsWriting;
    int8_t pane;    // PaneKind on top of the screen (-1 = no UI)
    uint8_t depth;  // overlays open
    uint16_t fps;
    char label[ FLIGHT_LABEL ]; // the menu item under the cursor, when a menu page is open
    uint32_t check;
};
#define FLIGHT_MAGIC ( 0x464C5400u ^ (uint32_t)sizeof( FlightRecord ) )
static FlightRecord CH32H4_XCORE flight;

static uint32_t flightSum( const FlightRecord* r ) {
    const uint8_t* p = (const uint8_t*)r;
    uint32_t s = 0x2468;
    for ( size_t i = 0; i < offsetof( FlightRecord, check ); i++ ) {
        s = ( ( s << 5 ) | ( s >> 27 ) ) ^ p[ i ];
    }
    return s;
}

class FlightRecorderService : public Service {
  public:
    ServiceStatus service( ) override {
        FlightRecord r = { };
        r.magic = FLIGHT_MAGIC;
        uint32_t now = millis( );
        r.uptimeMs = now;
#if MODULE_PROBE_LEDS
        r.stripMa = probeLeds.strip ? probeLeds.stripLastMa : 0.0f;
        r.stripStepMa = probeLeds.stripMaxStepMa;
        r.budgetMa = probeLeds.stripBudgetMa( );
#if MODULE_SETTINGS
        r.ledRestoreAgoMs = ledRestoreMs == 0 ? 0xFFFFFFFFu : now - ledRestoreMs;
        r.settingsWriting = flashWriting ? 1 : 0;
#else
        r.ledRestoreAgoMs = 0xFFFFFFFFu;
#endif
#else
        r.ledRestoreAgoMs = 0xFFFFFFFFu;
#endif
#if MODULE_MAG_LOCATOR
        r.coldStarts = magLocator.coldStarts;
        r.peakMt = magLocator.fix.peakMt;
        r.fitValid = magLocator.fix.valid ? 1 : 0;
        r.fitRough = magLocator.fix.rough ? 1 : 0;
        r.present = magLocator.fix.present ? 1 : 0;
#endif
        r.pane = -1;
#if MODULE_UI
        r.depth = (uint8_t)ui.shell.depth;
        r.pane = (int8_t)( ui.shell.depth > 0 ? ui.shell.stack[ ui.shell.depth - 1 ] : PANE_APP );
        if ( r.pane == PANE_MENU ) {
            const Menu& m = ui.shell.menu;
            int n = menuVisibleItem( &m, m.cursor );
            if ( n >= 0 && n < m.count && m.items[ n ].label != nullptr ) {
                strncpy( r.label, m.items[ n ].label, FLIGHT_LABEL - 1 );
            }
        }
#endif
#if MODULE_MAG_VIEW
        r.fps = (uint16_t)( display.fps( ) + 0.5f );
#endif
        r.check = flightSum( &r );
        flight = r;
        return ServiceStatus::IDLE;
    }
    const char* getName( ) const override { return "Flight"; }
    ServicePriority getPriority( ) const override { return ServicePriority::LOW; }
    uint32_t periodUs( ) const override { return 10000; }
};

static FlightRecorderService flightRecorder;

static const char* paneName( int pane ) {
    switch ( pane ) {
    case PANE_APP:
        return "the app";
    case PANE_HOME:
        return "Home";
    case PANE_MENU:
        return "a menu page";
    case PANE_CONFIRM:
        return "a confirm pane";
    case PANE_RESULT:
        return "a result pane";
    default:
        return "no UI";
    }
}

// At boot: the record of the run that ended in the reset, if there is one.
static void flightReport( Stream& out ) {
    FlightRecord r = flight;
    bool valid = r.magic == FLIGHT_MAGIC && r.check == flightSum( &r ) && r.uptimeMs < 7u * 24u * 3600000u && r.stripMa >= 0.0f && r.stripMa < 5000.0f &&
                 r.budgetMa >= 0.0f && r.budgetMa < 5000.0f && r.depth < 8 && r.fps < 1000;
    if ( !valid ) {
        out.println( "flight recorder: no record of a run before this one (a power-up, or the first boot of this build)" );
    } else {
        r.label[ FLIGHT_LABEL - 1 ] = '\0';
        char line[ 320 ];
        char relit[ 40 ];
        if ( r.ledRestoreAgoMs == 0xFFFFFFFFu )
            snprintf( relit, sizeof( relit ), "not re-lit since boot" );
        else
            snprintf( relit, sizeof( relit ), "re-lit %lu ms before", (unsigned long)r.ledRestoreAgoMs );
        snprintf( line, sizeof( line ),
                  "flight recorder: the last run ended after %lu.%lu s (the rst= line above says how). Then: strip %.0f mA (budget %.0f, biggest step %.0f, %s); fit %s, %lu cold starts, strongest %.3f mT; "
                  "screen %s%s%s, %u fps; settings write %s",
                  (unsigned long)( r.uptimeMs / 1000 ), (unsigned long)( r.uptimeMs % 1000 / 100 ), r.stripMa, r.budgetMa, r.stripStepMa, relit,
                  r.fitValid ? ( r.fitRough ? "rough" : "tracking" ) : ( r.present ? "present, no fix" : "nothing" ), (unsigned long)r.coldStarts, r.peakMt, paneName( r.pane ),
                  r.pane == PANE_MENU && r.label[ 0 ] != '\0' ? " at " : "", r.pane == PANE_MENU ? r.label : "", (unsigned)r.fps, r.settingsWriting ? "IN PROGRESS" : "no" );
        out.println( line );
    }
    flight.magic = 0; // read once; the recorder rewrites it from the first tick
}

// :load - what draws from the 3.3 V rail and changes, and switches to take
// each away for a bisect (the LCD's backlight hangs straight off that rail
// with no resistor of its own, so it shows every millivolt: the board's
// VDDIO is the ME6211 LDO's 3.3 V through R2, shared with VDD33, the LCD
// and the header's 3V3).
static void onLoadVerb( int argc, char** argv, Stream* out ) {
    char line[ 200 ];
    if ( argc >= 3 ) {
        bool on = strcmp( argv[ 2 ], "on" ) == 0;
        bool off = strcmp( argv[ 2 ], "off" ) == 0;
        if ( strcmp( argv[ 1 ], "strip" ) == 0 && ( on || off ) ) {
#if MODULE_PROBE_LEDS
            probeLeds.setStrip( on, nullptr );
            consoleOk( out, on ? "strip on" : "strip off (cleared)" );
#else
            consoleErr( out, "no strip in this build" );
#endif
            return;
        }
        if ( strcmp( argv[ 1 ], "display" ) == 0 && ( on || off ) ) {
#if MODULE_MAG_VIEW
            display.hold = off; // held: no frames go to the panel (the picture stays)
            consoleOk( out, on ? "display running" : "display held (no pushes)" );
#else
            consoleErr( out, "no display in this build" );
#endif
            return;
        }
        if ( strcmp( argv[ 1 ], "heartbeat" ) == 0 && ( on || off ) ) {
            heartbeat.enabled = on;
            consoleOk( out, on ? "heartbeat on" : "heartbeat off" );
            return;
        }
        if ( strcmp( argv[ 1 ], "sensors" ) == 0 ) {
#if MODULE_MAG_ARRAY
            if ( on || off ) {
                if ( off )
                    magArray.powerOff( );
                else
                    magArray.powerOn( );
                consoleOk( out, on ? "sensors powered and re-addressed" : "sensors off (their VCC pins low, the sampler paused)" );
            } else if ( strcmp( argv[ 2 ], "lp" ) == 0 || strcmp( argv[ 2 ], "ln" ) == 0 ) {
                bool ln = argv[ 2 ][ 1 ] == 'n';
                magArray.setLowNoise( ln );
                consoleOk( out, ln ? "sensors in low-noise conversion (3.0 mA each)" : "sensors in low-power conversion (2.3 mA each; a recovery puts low-noise back)" );
            } else if ( strcmp( argv[ 2 ], "only" ) == 0 && argc >= 4 ) {
                // A list of sensor numbers, 0-7, comma-separated: :load sensors only 0,3,5
                uint32_t mask = 0;
                for ( const char* q = argv[ 3 ]; *q != '\0'; q++ ) {
                    if ( *q >= '0' && *q <= '7' )
                        mask |= 1u << ( *q - '0' );
                }
                magArray.powerOnly( mask );
                snprintf( line, sizeof( line ), "sensors %s powered, the rest off (:load sensors on brings them back)", argv[ 3 ] );
                consoleOk( out, line );
            } else if ( argv[ 2 ][ 0 ] >= '0' && argv[ 2 ][ 0 ] <= '9' ) {
                int n = atoi( argv[ 2 ] );
                magArray.powerOffFrom( n );
                snprintf( line, sizeof( line ), "the first %d sensor%s powered, the rest off (:load sensors on brings them back)", n, n == 1 ? "" : "s" );
                consoleOk( out, line );
            } else {
                consoleErr( out, "usage: :load sensors on|off|lp|ln|<n>|only <list, e.g. 0,3,5>" );
            }
#else
            consoleErr( out, "no array in this build" );
#endif
            return;
        }
        if ( strcmp( argv[ 1 ], "fit" ) == 0 && ( on || off || strcmp( argv[ 2 ], "steady" ) == 0 || strcmp( argv[ 2 ], "burst" ) == 0 ) ) {
#if MODULE_MAG_LOCATOR
            if ( on || off ) {
                magLocator.fitHeld = off;
                consoleOk( out, on ? "fit running" : "fit held (the frames flow; nothing is fitted, so no fix, no tracking)" );
            } else {
                magLocator.steadyFit = argv[ 2 ][ 0 ] == 's';
                consoleOk( out, magLocator.steadyFit ? "fit load steady: 'fit iters' iterations every frame (the tracker page, saved)" : "fit load burst: the natural caps, cold-start slices every 25 ms" );
            }
#else
            consoleErr( out, "no locator in this build" );
#endif
            return;
        }
        if ( strcmp( argv[ 1 ], "sampler" ) == 0 ) {
#if MODULE_MAG_ARRAY
            if ( on || off ) {
                magArray.holdSampler( off );
                consoleOk( out, on ? "sampler running" : "sampler held (the other core off the bus; the sensors stay powered and converting)" );
            } else if ( argv[ 2 ][ 0 ] >= '0' && argv[ 2 ][ 0 ] <= '9' ) {
                uint32_t ms = (uint32_t)atol( argv[ 2 ] );
                magArray.setSamplerPassPeriodMs( ms );
                snprintf( line, sizeof( line ), ms == 0 ? "sampler free-running" : "sampler paced: one pass over the sensors every %lu ms", (unsigned long)ms );
                consoleOk( out, line );
            } else {
                consoleErr( out, "usage: :load sampler on|off|<ms> (a pass every that many ms; 0 = free-running)" );
            }
#else
            consoleErr( out, "no array in this build" );
#endif
            return;
        }
        if ( strcmp( argv[ 1 ], "lcd" ) == 0 ) {
#if MODULE_MAG_VIEW
            long mhz = atol( argv[ 2 ] );
            if ( mhz < 1 || mhz > 100 ) {
                consoleErr( out, "usage: :load lcd <MHz 1-100> (50 at boot; the core rounds down to 100 MHz / 2^n)" );
                return;
            }
            st7789SetSpiHz( (uint32_t)mhz * 1000000u );
            snprintf( line, sizeof( line ), "LCD SPI asked for %ld MHz", mhz );
            consoleOk( out, line );
#else
            consoleErr( out, "no display in this build" );
#endif
            return;
        }
        consoleErr( out, "usage: :load [strip|display|heartbeat on|off] [fit on|off|steady|burst] [sampler on|off|<ms>] [sensors on|off|lp|ln|<n>|only <list>] [lcd <MHz>]" );
        return;
    }
    out->println( "load{" );
#if MODULE_PROBE_LEDS
    if ( probeLeds.strip ) {
        snprintf( line, sizeof( line ), "strip: on, %lu frames since the last report, %.0f mA now, %.0f-%.0f mA, the biggest change from one frame to the next %.0f mA (budget %.0f, by the %.0f mA/channel model)",
                  (unsigned long)probeLeds.stripFramesSent, probeLeds.stripLastMa, probeLeds.stripLeastMa, probeLeds.stripMostMa, probeLeds.stripMaxStepMa, probeLeds.stripBudgetMa( ), PROBELED_MA_PER_CHANNEL );
        probeLeds.resetStripWindow( );
    } else {
        snprintf( line, sizeof( line ), "strip: off%s", PIN_LED_STRIP >= 0 ? " (wired: :load strip on)" : "" );
    }
    out->println( line );
#endif
#if MODULE_MAG_VIEW
    snprintf( line, sizeof( line ), "display: %s, %.0f fps, SPI %lu MHz, two DMA bands a frame (115 KB); the panel's own current follows the picture", display.hold ? "HELD" : "running", display.fps( ),
              (unsigned long)( st7789SpiHz( ) / 1000000u ) );
    out->println( line );
#endif
#if MODULE_MAG_ARRAY
    snprintf( line, sizeof( line ), "sensors: %s, %d of %d powered, %d answering, continuous %s conversion (~%.1f mA each, ~400 results/s each on its own clock) from their GPIO supplies",
              magArray.poweredOff ? "OFF" : "on", magArray.enabledCount( ), magArray.sensorCount( ), magArray.sensorsOk( ), magArray.lowNoise ? "low-noise" : "low-power", magArray.lowNoise ? 3.0f : 2.3f );
    out->println( line );
    float passes = magArray.samplerPassesPerSecond( );
    if ( magArray.samplerPassPeriodMs != 0 ) {
        snprintf( line, sizeof( line ), "sampler: %s, paced to a pass every %lu ms (%.0f passes/s since the last look; 0 = free-running)", magArray.samplerHeld ? "HELD" : "running", (unsigned long)magArray.samplerPassPeriodMs, passes );
    } else {
        snprintf( line, sizeof( line ), "sampler: %s, free-running (%.0f passes/s since the last look; :load sampler <ms> paces it)", magArray.samplerHeld ? "HELD" : "running", passes );
    }
    out->println( line );
#endif
#if MODULE_MAG_LOCATOR
    snprintf( line, sizeof( line ), "fit: %s, %s load, the last %lu us; %lu cold starts since boot, the last %lu us in all, its longest slice %lu us - the V5F's heaviest work",
              magLocator.fitHeld ? "HELD" : "running", magLocator.steadyFit ? "steady (fit iters a frame)" : "burst (slices every 25 ms)", (unsigned long)magLocator.fix.fitUs,
              (unsigned long)magLocator.coldStarts, (unsigned long)magLocator.coldStartUs, (unsigned long)magLocator.coldSliceMaxUs );
    out->println( line );
    snprintf( line, sizeof( line ), "  the track ended before them: %lu times nothing present, %lu too few noticing, %lu the track coasted out; %lu cold starts rejected (chi over %.1f, a bar over %.0f mm, or beyond the array)",
              (unsigned long)magLocator.coldWhyAbsent, (unsigned long)magLocator.coldWhyFew, (unsigned long)magLocator.coldWhyCoasted, (unsigned long)magLocator.coldWhyRejected, magLocator.fitMaxChi, MAGLOC_MAX_ERROR_MM );
    out->println( line );
    snprintf( line, sizeof( line ), "  frames a sensor was not read in: 0:%lu 1:%lu 2:%lu 3:%lu 4:%lu 5:%lu 6:%lu 7:%lu 8:%lu; a missing sensor's last reading carried presence %lu frames; magnet strength held at %.0f (measured by %lu frames, another magnet taken %lu times, the settings hold %.0f)",
              (unsigned long)magLocator.staleFrames[ 0 ], (unsigned long)magLocator.staleFrames[ 1 ], (unsigned long)magLocator.staleFrames[ 2 ], (unsigned long)magLocator.staleFrames[ 3 ], (unsigned long)magLocator.staleFrames[ 4 ],
              (unsigned long)magLocator.staleFrames[ 5 ], (unsigned long)magLocator.staleFrames[ 6 ], (unsigned long)magLocator.staleFrames[ 7 ], (unsigned long)magLocator.staleFrames[ 8 ], (unsigned long)magLocator.presenceHeldFrames,
              magLocator.knownStrength, (unsigned long)magLocator.strengthMeasured, (unsigned long)magLocator.strengthJumps, magLocator.strengthSaved );
    out->println( line );
    snprintf( line, sizeof( line ), "  presence toggled %lu times; %lu frames missed while tracking; the strongest reading since the last look %.4f (TMAG terms; the presence level is %.3f)",
              (unsigned long)magLocator.presenceToggles, (unsigned long)magLocator.missFrames, magLocator.maxPeakLevel, magLocator.presentMt );
    out->println( line );
    magLocator.maxPeakLevel = 0.0f;
#endif
    snprintf( line, sizeof( line ), "heartbeat: %s (green LED, 1 k to VDDIO, ~2 mA at 1 Hz)", heartbeat.enabled ? "on" : "off" );
    out->println( line );
    out->println( "fixed: the LCD backlight (LEDA straight to VDDIO, no series resistor on the board), the LCD's VDD, the cores from VDD33 (the same LDO), the joystick's pots" );
    out->println( "}" );
}

#if MODULE_MAG_ARRAY && MODULE_MAG_VIEW
// The sensor watch banner (:watch <i>): painted over any app while a
// sensor is being wired. Red with a running clock while it does not
// answer; green, blinking, with what it said once it does.
static bool sensorWatchOverlay( GFXcanvas16* canvas, uint32_t nowMs ) {
    uint32_t mask = magArray.watchMask;
    if ( mask == 0 ) {
        return false;
    }
    char line[ 32 ];
    int y = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT && y + 20 <= 240; i++ ) {
        if ( !( mask & ( 1u << i ) ) ) {
            continue;
        }
        const MagSensorState& s = magArray.sensor( i );
        bool seen = s.ok;
        uint16_t box = seen ? ( ( nowMs / 400 ) & 1 ? 0x07E0 : 0x0320 ) : 0xF800; // green / dark green blinking, red
        uint16_t text = seen ? 0x0000 : 0xFFFF;
        fastFillRect( canvas, 0, y, 240, 20, box );
        if ( seen ) {
            snprintf( line, sizeof( line ), "S%d b%d SEEN id%02X r%lu", i, s.bus, s.type == MAG_MMC56X3 ? s.mmc.productId : s.dev.version, (unsigned long)magSampler.reads[ i ] );
        } else {
            snprintf( line, sizeof( line ), "S%d b%d 0x%02X NO %lus", i, s.bus, magSensorPlaces[ i ].address, (unsigned long)( ( nowMs - magArray.watchSinceMs ) / 1000 ) );
        }
        fastText( canvas, 4, y + 2, 2, text, line );
        y += 20;
    }
    return true;
}
#endif

void setup( ) {
#if MODULE_PROBE_LEDS
    ledStripDarkAll( ); // before anything else draws: the chain may hold a lit frame from before the reset (LedStrip.h)
#endif
    Serial.begin( CONSOLE_BAUD );
    boardLedsInit( );
    boardLed( PIN_LED_BLUE, true ); // blue while setup runs

    Serial.println( );
    Serial.println( "Testolomew - Jumperless V6 test bed" );
    if ( !boardVioIs3V3( ) ) {
        Serial.println( "WARNING: the VIO18 rail is not at 3.3 V - most pins will not reach 3.3 V logic levels" );
    }

#if MODULE_UI
    // The console talks through the UI's stream: the log screen sees every
    // line, and the menu can type into it.
    uiStream.begin( &Serial );
    console.begin( &uiStream );
#else
    console.begin( &Serial );
#endif
    jOS.registerService( &console );
    jOS.registerService( &heartbeat );
    // Everything setup() says from here goes through the console's port: with
    // the UI on that is the DMA-driven stream, and a plain Serial.print
    // beside it would interleave with what the modules print.
    Stream& boot = *console.port( );
    flightReport( boot );
    jOS.registerService( &flightRecorder );

#if MODULE_MAG_ARRAY
    int found = magArray.begin( );
    jOS.registerService( &magArray );
#if MODULE_MAG_VIEW
    display.overlayFn = sensorWatchOverlay;
#endif
    magArray.printStatus( &boot );
    if ( found < magArray.sensorCount( ) ) {
        boot.println( "some sensors did not answer: check that sensor's VCC pin in src/board/BoardPins.h, and the pull-ups on SDA/SCL" );
    }
#endif

#if MODULE_MAG_LOCATOR
    magLocator.begin( );
    jOS.registerService( &magLocator );
#endif

#if MODULE_ROW_COUNT
    rowCounter.begin( );
    jOS.registerService( &rowCounter );
#endif

#if MODULE_PROBE_LEDS
    probeLeds.begin( );
    jOS.registerService( &probeLeds );
#endif

#if MODULE_PLAY
    play.begin( );
    jOS.registerService( &play );
#endif

#if MODULE_MAG_VIEW
    if ( display.begin( ) ) {
        jOS.registerService( &display );
    } else {
        boot.println( "Display: LCD or framebuffer setup failed - running without the display" );
    }
#endif

#if MODULE_UI
    input.begin( );
    jOS.registerService( &input );
    appsBegin( ); // after every module: the menu lists the console commands they registered
    jOS.registerService( &ui );
#endif

#if MODULE_SETTINGS
    // After the menu exists and every module has begun: the saved values go
    // in through the items (an accessor item's module follows), and what
    // the items read before that is the default.
#if MODULE_PROBE_LEDS
    settings.aroundWrite = quietForFlash; // the LEDs off for the erase (above)
#endif
    int loaded = settings.begin( &ui.shell.menu );
    settingsMenuAddNewCommands( ); // s and Z into the menu's commands page
    jOS.registerService( &settings );
    boot.print( "menu: " );
    boot.print( ui.shell.menu.count );
    boot.print( "/" );
    boot.print( MENU_MAX_ITEMS );
    boot.println( " items" );
#if MODULE_MAG_ARRAY
    if ( !magArray.baselineRestored ) {
        // Nothing saved: the compiled-in zero rather than the blind one under
        // way (the probe may be lying on the board; see MAG_ZERO_AT_BOOT).
        static const Vec3 compiledZero[ MAG_ZERO_AT_BOOT_COUNT ] = MAG_ZERO_AT_BOOT;
        magArray.restoreBaseline( compiledZero, MAG_ZERO_AT_BOOT_COUNT, "no zero saved - the compiled-in one" );
    }
#endif
    if ( loaded >= 0 ) {
        boot.print( "settings: " );
        boot.print( loaded );
        boot.println( " saved values put back (s lists them, Z resets them all)" );
    } else {
        boot.println( "settings: none saved, defaults (the menu's values are kept in flash from now on)" );
    }
#endif

    // The dumps (:screen:ascii, :leds...) go out a row a tick from here.
    dump.begin( );
    jOS.registerService( &dump );
    consoleAddVerb( "load", "[strip|display|heartbeat on|off] [fit on|off|steady|burst] [sampler on|off|<ms>] [sensors on|off|lp|ln|<n>|only <list>] [lcd <MHz>]", "what draws from the 3.3 V rail and changes; switches for a bisect", CONSOLE_CHANGES, onLoadVerb );

    console.printHelp( );
    boardLed( PIN_LED_BLUE, false );
}

void loop( ) {
    jOS.serviceAll( );
}
