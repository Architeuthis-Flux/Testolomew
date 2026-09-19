// SPDX-License-Identifier: MIT
// The LCD on the host: the same contract as the real driver (ST7789.h) - a
// pushed band is byte-swapped IN PLACE in the frame buffer for the wire, and
// takes as long on the (pretend) wire as it would at LCD_SPI_HZ - and a copy
// of what the panel holds, native RGB565, for the simulator's PNGs and for
// :screen:verify (which checks the dump path un-swaps exactly what went out).
#include "ST7789.h"

#include <string.h>

uint16_t simPanel[ LCD_WIDTH * LCD_HEIGHT ];
uint32_t simPushes = 0;
static bool inFlight = false;
static uint64_t doneUs = 0;

static void pushBand( uint16_t* frame, int y0, int rows ) {
    uint16_t* band = frame + (size_t)y0 * LCD_WIDTH;
    size_t pixels = (size_t)rows * LCD_WIDTH;
    for ( size_t i = 0; i < pixels; i++ ) {
        simPanel[ (size_t)y0 * LCD_WIDTH + i ] = band[ i ]; // what the panel shows: the native value...
        band[ i ] = (uint16_t)( ( band[ i ] << 8 ) | ( band[ i ] >> 8 ) ); // ...and the buffer is left swapped, as on the device
    }
    simPushes++;
}

bool st7789Begin( void ) {
    memset( simPanel, 0, sizeof( simPanel ) );
    return true;
}

void st7789PushRows( uint16_t* frame, int y0, int rows ) {
    pushBand( frame, y0, rows );
}

bool st7789PushRowsStart( uint16_t* frame, int y0, int rows ) {
    if ( inFlight ) {
        return false;
    }
    pushBand( frame, y0, rows );
    inFlight = true;
    // rows x 240 px x 16 bits at LCD_SPI_HZ: a 120-row band is ~9 ms.
    doneUs = simMicros + (uint64_t)rows * LCD_WIDTH * 16 * 1000000ull / LCD_SPI_HZ;
    return true;
}

bool st7789PushBusy( void ) {
    return inFlight && simMicros < doneUs;
}

void st7789PushFinish( void ) {
    inFlight = false;
}
