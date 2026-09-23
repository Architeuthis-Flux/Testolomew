// SPDX-License-Identifier: MIT
// The LED strip driver for the host: no SPI, no DMA, nothing wired. The
// service then reports "NOT set up" and renders for the LCD preview only.
#include "LedStrip.h"

bool ledStripBegin( LedStrip* s, int, int, int, int count, int ) {
    s->count = count;
    return false;
}
void ledStripSet( LedStrip*, int, uint8_t, uint8_t, uint8_t ) {}
void ledStripClear( LedStrip* ) {}
void ledStripTouch( LedStrip* ) {}
bool ledStripShow( LedStrip* ) { return false; }
bool ledStripBusy( LedStrip* ) { return false; }
bool ledStripWait( LedStrip*, uint32_t ) { return true; }
int ledStripPeripheral( const LedStrip* ) { return 0; }
uint32_t ledStripBytesLeft( const LedStrip* ) { return 0; }
void ledStripReport( LedStrip*, char* line, size_t size ) { snprintf( line, size, "no strip on the host" ); }

extern "C" void ledStripDarkAll( void ) {} // the polled dark frame: nothing to darken on the host
