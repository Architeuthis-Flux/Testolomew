// SPDX-License-Identifier: MIT
// What every host program needs once: the clock, the pins, the serial port,
// the bus and EEPROM stand-ins, and the board's little helpers.
#include "Arduino.h"
#include "EEPROM.h"
#include "SPI.h"
#include "Wire.h"

uint64_t simMicros = 0;
int simPinLevel[ 64 ] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                          1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
int simAnalog[ 64 ] = { 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,
                        2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048,
                        2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048 };
HardwareSerialStub Serial;
TwoWire Wire;
SPIClass SPI;
EEPROMClass EEPROM;

bool boardVioIs3V3( void ) {
    return true;
}
void boardLedsInit( void ) {}
void boardLed( int, bool ) {}
