// SPDX-License-Identifier: MIT
#include "BoardPins.h"

extern "C" {
#include "ch32h417.h"
}

bool boardVioIs3V3( void ) {
    // PWR_CTLR bits [12:10] select the VIO18 rail: 0 = 1.2 V (reset), 1 = 1.8 V,
    // 2 = 2.5 V, 3 = 3.3 V. The PWR block reads back as zeroes with its clock off.
    RCC_HB1PeriphClockCmd( RCC_HB1Periph_PWR, ENABLE );
    (void)RCC->HB1PCENR;
    return ( ( PWR->CTLR >> 10 ) & 0x7u ) == 3u;
}

void boardLedsInit( void ) {
    pinMode( PIN_LED_BLUE, OUTPUT );
    pinMode( PIN_LED_GREEN, OUTPUT );
    boardLed( PIN_LED_BLUE, false );
    boardLed( PIN_LED_GREEN, false );
}

void boardLed( int pin, bool on ) {
    digitalWrite( pin, on ? LOW : HIGH );
}
