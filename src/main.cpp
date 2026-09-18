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

#if MODULE_MAG_LOCATOR && !MODULE_MAG_ARRAY
#error "MODULE_MAG_LOCATOR needs MODULE_MAG_ARRAY"
#endif
#if MODULE_MAG_VIEW && !MODULE_MAG_LOCATOR
#error "MODULE_MAG_VIEW needs MODULE_MAG_LOCATOR"
#endif
#if MODULE_ROW_COUNT && !MODULE_MAG_LOCATOR
#error "MODULE_ROW_COUNT needs MODULE_MAG_LOCATOR"
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

    console.begin( &Serial );
    jOS.registerService( &console );
    jOS.registerService( &heartbeat );

#if MODULE_MAG_ARRAY
    int found = magArray.begin( );
    jOS.registerService( &magArray );
    magArray.printStatus( &Serial );
    if ( found < magArray.sensorCount( ) ) {
        Serial.println( "some sensors did not answer: check that sensor's VCC pin in src/board/BoardPins.h, and the pull-ups on SDA/SCL" );
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

#if MODULE_MAG_VIEW
    if ( magView.begin( ) ) {
        jOS.registerService( &magView );
    } else {
        Serial.println( "MagView: LCD or framebuffer setup failed - running without the display" );
    }
#endif

    console.printHelp( );
    boardLed( PIN_LED_BLUE, false );
}

void loop( ) {
    jOS.serviceAll( );
}
