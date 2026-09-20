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
#include <stdlib.h>
#include <string.h>

#include "BoardPins.h"
#include "Console.h"
#include "DumpService.h"
#include "JumperlOS.h"
#include "config.h"

#if MODULE_MAG_ARRAY
#include "MagArray.h"
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
        if ( strcmp( argv[ 1 ], "fit" ) == 0 && ( on || off ) ) {
#if MODULE_MAG_LOCATOR
            magLocator.fitHeld = off;
            consoleOk( out, on ? "fit running" : "fit held (the frames flow; nothing is fitted, so no fix, no tracking)" );
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
        consoleErr( out, "usage: :load [strip|display|heartbeat|fit on|off] [sampler on|off|<ms>] [sensors on|off|lp|ln|<n>|only <list>] [lcd <MHz>]" );
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
    snprintf( line, sizeof( line ), "fit: %s, the last %lu us; %lu cold starts since boot, the last %lu us in all, its longest slice %lu us (one slice a frame) - the V5F's heaviest work",
              magLocator.fitHeld ? "HELD" : "running", (unsigned long)magLocator.fix.fitUs, (unsigned long)magLocator.coldStarts, (unsigned long)magLocator.coldStartUs, (unsigned long)magLocator.coldSliceMaxUs );
    out->println( line );
#endif
    snprintf( line, sizeof( line ), "heartbeat: %s (green LED, 1 k to VDDIO, ~2 mA at 1 Hz)", heartbeat.enabled ? "on" : "off" );
    out->println( line );
    out->println( "fixed: the LCD backlight (LEDA straight to VDDIO, no series resistor on the board), the LCD's VDD, the cores from VDD33 (the same LDO), the joystick's pots" );
    out->println( "}" );
}

void setup( ) {
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

#if MODULE_MAG_ARRAY
    int found = magArray.begin( );
    jOS.registerService( &magArray );
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
        static const Vec3 compiledZero[ MAG_SENSOR_COUNT ] = MAG_ZERO_AT_BOOT;
        magArray.restoreBaseline( compiledZero, MAG_SENSOR_COUNT, "no zero saved - the compiled-in one" );
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
    consoleAddVerb( "load", "[strip|display|heartbeat|fit on|off] [sampler on|off|<ms>] [sensors on|off|lp|ln|<n>|only <list>] [lcd <MHz>]", "what draws from the 3.3 V rail and changes; switches for a bisect", CONSOLE_CHANGES, onLoadVerb );

    console.printHelp( );
    boardLed( PIN_LED_BLUE, false );
}

void loop( ) {
    jOS.serviceAll( );
}
