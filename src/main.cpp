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
#include "MagView.h"
#endif
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif
#if MODULE_PROBE_LEDS
#include "ProbeLedService.h"
#endif
#if MODULE_UI
#include "Input.h"
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
        on = !on;
        boardLed( PIN_LED_GREEN, on );
        return ServiceStatus::IDLE;
    }
    const char* getName( ) const override { return "Heartbeat"; }
    ServicePriority getPriority( ) const override { return ServicePriority::LOW; }
    uint32_t periodUs( ) const override { return 500000; }

  private:
    bool on = false;
};

static HeartbeatService heartbeat;

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
    if ( magView.begin( ) ) {
        jOS.registerService( &magView );
    } else {
        boot.println( "MagView: LCD or framebuffer setup failed - running without the display" );
    }
#endif

#if MODULE_UI
    input.begin( );
    jOS.registerService( &input );
    ui.begin( ); // after every module: it lists the console commands they registered
    jOS.registerService( &ui );
#endif

#if MODULE_SETTINGS
    // After the menu exists: the saved values go into its variables, and the
    // UI carries the choice ones into the modules.
    int loaded = settings.begin( &ui.menu );
    ui.settingsLoaded( );
    ui.addNewCommands( ); // s and Z into the menu's commands page
    jOS.registerService( &settings );
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

    console.printHelp( );
    boardLed( PIN_LED_BLUE, false );
}

void loop( ) {
    jOS.serviceAll( );
}
