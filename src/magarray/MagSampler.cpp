// SPDX-License-Identifier: MIT
#include "MagSampler.h"

#include "MagI2c.h"

extern "C" {
#include "ch32h4_itcm.h"
#include "ch32h4_park.h"
#include "ch32h4_xcore.h"
}

MagSamplerShared magSampler CH32H4_XCORE;

// The shared block is NOLOAD: after a reset it still says whatever it said
// before, and the V3F, which starts loop1() the moment this core's runtime
// is ready, would read the old magic and the old command and be on the bus
// at the old addresses while this core is still addressing the sensors
// (2026-09-19: every warm boot found "0 of 8 sensors" until recovery). A
// constructor runs before the runtime is declared ready: the block is
// silenced here, before the V3F can look.
struct MagSamplerSilencer {
    MagSamplerSilencer( ) {
        magSampler.magic = 0;
        magSampler.command = MAGSAMPLER_STOP;
    }
};
static MagSamplerSilencer silencer;

// ---- the V3F: loop1() -----------------------------------------------------
// One pass over the enabled sensors per call; the core's wrapper calls it
// again at once (and checks whether the V5F wants this core parked for a
// flash write in between). Nothing here waits on anything but the bus.
__itcm_func void loop1( ) {
    MagSamplerShared* s = &magSampler;
    if ( s->magic != MAGSAMPLER_MAGIC ) {
        return;
    }
    uint32_t command = s->command;
    if ( command != MAGSAMPLER_RUN ) {
        s->state = command; // paused or stopped: say so, touch nothing
        return;
    }
    s->state = MAGSAMPLER_RUN;
    uint32_t period = s->passPeriodMs;
    if ( period != 0 ) {
        uint32_t now = millis( );
        if ( now - s->lastPassMs < period ) {
            return; // paced: not yet
        }
        s->lastPassMs = now;
    }
    uint32_t count = s->count;
    if ( count > MAGSAMPLER_MAX_SENSORS ) {
        count = MAGSAMPLER_MAX_SENSORS;
    }
    uint32_t tried[ MAGSAMPLER_MAX_BUSES ] = { 0, 0, 0, 0 }, failed[ MAGSAMPLER_MAX_BUSES ] = { 0, 0, 0, 0 };
    uint32_t wanted = s->resetWanted;
    for ( uint32_t i = 0; i < count; i++ ) {
        if ( !s->enabled[ i ] || s->command != MAGSAMPLER_RUN ) {
            continue;
        }
        uint32_t b = s->bus[ i ] & ( MAGSAMPLER_MAX_BUSES - 1 );
        if ( wanted & ( 1u << b ) ) {
            continue; // the V5F is resetting that bus's block: off it until it says so
        }
        uint8_t raw[ MAGSAMPLER_MAX_BYTES ];
        uint32_t bytes = s->bytes[ i ];
        if ( bytes > MAGSAMPLER_MAX_BYTES ) {
            bytes = MAGSAMPLER_MAX_BYTES;
        }
        tried[ b ]++;
        int code = magI2cBurstReadOn( (void*)s->registers[ i ], s->address[ i ], s->reg[ i ], raw, bytes );
        if ( code == MAGI2C_OK ) {
            s->seq[ i ]++; // odd: being written
            for ( uint32_t k = 0; k < bytes; k++ ) {
                s->raw[ i ][ k ] = raw[ k ];
            }
            s->stampUs[ i ] = millis( ); // (micros() has no sub-ms part on this core)
            s->seq[ i ]++;               // even: whole
            s->reads[ i ]++;
        } else {
            s->fails[ i ]++;
            s->lastFail[ i ] = (uint8_t)code;
            failed[ b ]++;
        }
    }
    s->passes++;
    // Every sensor failing in one pass is the block(s), not the sensors (a
    // glitch on the lines leaves a block wedged in ways the read's own
    // checks do not all catch): have them reset here and now, as the main
    // core did when it read the bus itself, rather than fail on until that
    // core notices the sensors "lost" and power-cycles them. (One bus of
    // two wedged shows as its sensors lost instead; MagArray's recovery
    // resets that bus.)
    for ( uint32_t b = 0; b < MAGSAMPLER_MAX_BUSES; b++ ) {
        if ( tried[ b ] > 1 && failed[ b ] == tried[ b ] ) {
            if ( ++s->failedPasses[ b ] >= MAGSAMPLER_RESET_AFTER_PASSES ) {
                // The V5F does the reset (Wire's end/begin): the SDK's I2C_Init
                // on this core takes this core's SystemCoreClock for the bus
                // clock and set SCL four times slower (2026-09-19: 158 passes
                // a second instead of 274 after the first reset). Until it has,
                // no reads on that bus. (A bus with one sensor never trips
                // this: one part not answering is not a wedge.)
                s->resetWanted |= 1u << b;
                s->resets++;
                s->failedPasses[ b ] = 0;
            }
        } else if ( tried[ b ] > 0 ) {
            s->failedPasses[ b ] = 0;
        }
    }
}

// ---- the V5F ----------------------------------------------------------------
void magSamplerSetBusHz( uint32_t hz ) {
    magSampler.busHz = hz;
}

void magSamplerSetPassPeriodMs( uint32_t ms ) {
    magSampler.passPeriodMs = ms;
}

void magSamplerSetup( int count ) {
    MagSamplerShared* s = &magSampler;
    s->magic = 0;
    s->command = MAGSAMPLER_STOP;
    s->state = MAGSAMPLER_STOP;
    s->count = (uint32_t)( count > MAGSAMPLER_MAX_SENSORS ? MAGSAMPLER_MAX_SENSORS : count );
    for ( int i = 0; i < MAGSAMPLER_MAX_SENSORS; i++ ) {
        s->registers[ i ] = 0;
        s->bus[ i ] = 0;
        s->address[ i ] = 0;
        s->reg[ i ] = MAGI2C_NO_REGISTER;
        s->bytes[ i ] = 0;
        s->enabled[ i ] = 0;
        s->seq[ i ] = 0;
        s->stampUs[ i ] = 0;
        s->reads[ i ] = 0;
        s->fails[ i ] = 0;
        s->lastFail[ i ] = 0;
        for ( int k = 0; k < MAGSAMPLER_MAX_BYTES; k++ ) {
            s->raw[ i ][ k ] = 0;
        }
    }
    s->passes = 0;
    for ( int b = 0; b < MAGSAMPLER_MAX_BUSES; b++ ) {
        s->failedPasses[ b ] = 0;
    }
    s->resets = 0;
    s->resetWanted = 0;
    s->busHz = 400000;
    s->magic = MAGSAMPLER_MAGIC;
}

void magSamplerSetSensor( int i, int bus, void* registers, uint8_t address, int reg, int bytes, bool enabled ) {
    if ( i < 0 || i >= MAGSAMPLER_MAX_SENSORS ) {
        return;
    }
    magSampler.bus[ i ] = (uint8_t)( bus < 0 ? 0 : ( bus >= MAGSAMPLER_MAX_BUSES ? MAGSAMPLER_MAX_BUSES - 1 : bus ) );
    magSampler.registers[ i ] = (uint32_t)(uintptr_t)registers;
    magSampler.address[ i ] = address;
    magSampler.reg[ i ] = (int16_t)reg;
    magSampler.bytes[ i ] = (uint8_t)bytes;
    magSampler.enabled[ i ] = ( enabled && registers != nullptr ) ? 1 : 0;
}

bool magSamplerCommand( MagSamplerCommand command ) {
    magSampler.command = (uint32_t)command;
    uint32_t start = millis( );
    while ( magSampler.state != (uint32_t)command ) {
        if ( millis( ) - start > 20 ) {
            return false; // the other core is not running the sampler
        }
    }
    return true;
}

static bool parkedForFlash = false;

bool magSamplerParkForFlash( void ) {
    if ( parkedForFlash ) {
        return true; // F twice: still parked
    }
    magSamplerCommand( MAGSAMPLER_STOP );
    parkedForFlash = ch32h4_park_other( 1000 ); // and it is never released: the reset after the flash restarts both cores
    return parkedForFlash;
}

bool magSamplerParked( void ) {
    return parkedForFlash;
}

bool magSamplerRunning( void ) {
    return magSampler.magic == MAGSAMPLER_MAGIC && magSampler.state == MAGSAMPLER_RUN;
}

bool magSamplerTake( int i, uint32_t* lastSeq, uint8_t* out, uint32_t* stampUs ) {
    if ( i < 0 || i >= MAGSAMPLER_MAX_SENSORS ) {
        return false;
    }
    MagSamplerShared* s = &magSampler;
    for ( int attempt = 0; attempt < 3; attempt++ ) {
        uint32_t before = s->seq[ i ];
        if ( before & 1u ) {
            continue; // being written: the writer is a few microseconds from done
        }
        if ( before == *lastSeq ) {
            return false; // nothing new
        }
        uint32_t bytes = s->bytes[ i ];
        if ( bytes > MAGSAMPLER_MAX_BYTES ) {
            bytes = MAGSAMPLER_MAX_BYTES;
        }
        for ( uint32_t k = 0; k < bytes; k++ ) {
            out[ k ] = s->raw[ i ][ k ];
        }
        uint32_t stamp = s->stampUs[ i ];
        if ( s->seq[ i ] == before ) {
            *lastSeq = before;
            *stampUs = stamp;
            return true;
        }
    }
    return false;
}
