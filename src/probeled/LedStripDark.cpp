// SPDX-License-Identifier: MIT
// The dark frame by polling (LedStrip.h: ledStripDarkAll), and the V3F's
// hook before the wake. This file runs on BOTH cores: on the V3F there is
// no Arduino runtime yet (no millis, no heap, no constructed globals), so
// everything here is on the stack and talks to the peripheral through the
// SPI class's polled transfer and the SDK.
#include "LedStrip.h"

#include <SPI.h>
#include <ch32h4_gpio.h> // g_pins
#include <ch32h4_rcc.h>  // ch32h4_pin_af
#include <ch32h4_spi.h>  // ch32h4_spi_regs

#include "BoardPins.h"
#include "ch32h4_console.h"
#include "config.h"

static void darkOne( int mosi, int sck, int miso ) {
    if ( mosi < 0 ) {
        return;
    }
    SPIClassCH32H4 bus( sck, miso, mosi );
    bus.begin( );
    if ( bus.peripheral( ) == 0 ) {
        return; // no SPI has these pins
    }
    // As ledStripBegin: the clock pad back to analog (it would be a 3 MHz
    // square wave on a header wire), MOSI on the slower driver.
    ch32h4_pin_af( g_pins[ sck ].port, g_pins[ sck ].bit, CH32H4_AF_NONE, CH32H4_CFG_IN_ANALOG );
    ch32h4_pin_af( g_pins[ mosi ].port, g_pins[ mosi ].bit, CH32H4_AF_NONE, LEDSTRIP_MOSI_CFG );
    bus.beginTransaction( SPISettings( LEDSTRIP_SPI_HZ, MSBFIRST, SPI_MODE0 ) );
    for ( int i = 0; i < LEDSTRIP_MAX * LEDSTRIP_BYTES_PER_LED; i++ ) {
        bus.transfer( 0x88 ); // two zero bits: 1000 1000
    }
    SPI_TypeDef* dev = ch32h4_spi_regs( bus.peripheral( ) );
    while ( SPI_I2S_GetFlagStatus( dev, SPI_I2S_FLAG_BSY ) != RESET ) {
    }
    // The latch: the line low for 50 us and more. No clock to count by on
    // the V3F before the wake, so a loop: about 300 us at 100 MHz from
    // flash, longer anywhere slower, and the length is not critical.
    for ( volatile uint32_t k = 0; k < 20000; k++ ) {
    }
    bus.endTransaction( );
}

extern "C" void ledStripDarkAll( void ) {
#if MODULE_PROBE_LEDS
    darkOne( PIN_LED_STRIP, PIN_LED_STRIP_SCK, PIN_LED_STRIP_MISO );
#ifdef PIN_LED_STRIP_TOP
    darkOne( PIN_LED_STRIP_TOP, PIN_LED_STRIP_TOP_SCK, PIN_LED_STRIP_TOP_MISO );
#endif
#endif
}

// The V3F, before it wakes the V5F (the Arduino core's main_v3f.c, patched
// by scripts/patch_core.py to call this if the sketch defines it): the
// chain dark, so that a boot with the chain lit does not fall over at the
// wake, again and again, until someone pulls a plug.
extern "C" void ch32h4_v3f_before_wake( void ) {
    ledStripDarkAll( );
    ch32h4_console_puts( "V3F: the LED chain darkened before the wake\n" );
}
