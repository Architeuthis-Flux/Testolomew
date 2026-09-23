// SPDX-License-Identifier: MIT
// The other-core sampler for the host: there is no other core, so the
// array reads its (simulated) bus itself.
#include "MagSampler.h"

MagSamplerShared magSampler;

void magSamplerSetup( int ) {}
void magSamplerSetBusHz( uint32_t ) {}
void magSamplerSetSensor( int, int, void*, uint8_t, int, int, bool ) {}
bool magSamplerCommand( MagSamplerCommand ) { return false; }
bool magSamplerRunning( void ) { return false; }
bool magSamplerTake( int, uint32_t*, uint8_t*, uint32_t* ) { return false; }
bool magSamplerParkForFlash( void ) { return false; }
bool magSamplerParked( void ) { return false; }
void magSamplerSetPassPeriodMs( uint32_t ) {}
