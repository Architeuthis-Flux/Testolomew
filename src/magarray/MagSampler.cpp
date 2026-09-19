// SPDX-License-Identifier: MIT
#include "MagSampler.h"

#include "TMAG5273.h"

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
    if ( s->resetWanted ) {
        return; // the V5F is resetting the block: off the bus until it says so
    }
    s->state = MAGSAMPLER_RUN;
    uint32_t count = s->count;
    if ( count > MAGSAMPLER_MAX_SENSORS ) {
        count = MAGSAMPLER_MAX_SENSORS;
    }
    uint32_t tried = 0, failed = 0;
    for ( uint32_t i = 0; i < count; i++ ) {
        if ( !s->enabled[ i ] || s->command != MAGSAMPLER_RUN ) {
            continue;
        }
        uint8_t raw[ MAGSAMPLER_MAX_BYTES ];
        uint32_t bytes = s->bytes[ i ];
        if ( bytes > MAGSAMPLER_MAX_BYTES ) {
            bytes = MAGSAMPLER_MAX_BYTES;
        }
        tried++;
        if ( tmag5273BurstRead( (int)s->peripheral, s->address[ i ], raw, bytes ) ) {
            s->seq[ i ]++; // odd: being written
            for ( uint32_t k = 0; k < bytes; k++ ) {
                s->raw[ i ][ k ] = raw[ k ];
            }
            s->stampUs[ i ] = millis( ); // (micros() has no sub-ms part on this core)
            s->seq[ i ]++;               // even: whole
            s->reads[ i ]++;
        } else {
            s->fails[ i ]++;
            failed++;
        }
    }
    s->passes++;
    // Every sensor failing in one pass is the block, not the sensors (a
    // glitch on the lines leaves it wedged in ways the read's own checks do
    // not all catch): reset it here and now, as the main core did when it
    // read the bus itself, rather than fail on until that core notices the
    // sensors "lost" and power-cycles them.
    if ( tried > 1 && failed == tried ) {
        if ( ++s->failedPasses >= MAGSAMPLER_RESET_AFTER_PASSES ) {
            // The V5F does the reset (Wire's end/begin): the SDK's I2C_Init
            // on this core takes this core's SystemCoreClock for the bus
            // clock and set SCL four times slower (2026-09-19: 158 passes
            // a second instead of 274 after the first reset). Until it has,
            // no reads.
            s->resetWanted = 1;
            s->resets++;
            s->failedPasses = 0;
        }
    } else {
        s->failedPasses = 0;
    }
}

// ---- the V5F ----------------------------------------------------------------
void magSamplerSetBusHz( uint32_t hz ) {
    magSampler.busHz = hz;
}

void magSamplerSetup( int peripheral, int count ) {
    MagSamplerShared* s = &magSampler;
    s->magic = 0;
    s->command = MAGSAMPLER_STOP;
    s->state = MAGSAMPLER_STOP;
    s->peripheral = (uint32_t)peripheral;
    s->count = (uint32_t)( count > MAGSAMPLER_MAX_SENSORS ? MAGSAMPLER_MAX_SENSORS : count );
    for ( int i = 0; i < MAGSAMPLER_MAX_SENSORS; i++ ) {
        s->address[ i ] = 0;
        s->bytes[ i ] = 0;
        s->enabled[ i ] = 0;
        s->seq[ i ] = 0;
        s->stampUs[ i ] = 0;
        s->reads[ i ] = 0;
        s->fails[ i ] = 0;
        for ( int k = 0; k < MAGSAMPLER_MAX_BYTES; k++ ) {
            s->raw[ i ][ k ] = 0;
        }
    }
    s->passes = 0;
    s->failedPasses = 0;
    s->resets = 0;
    s->resetWanted = 0;
    s->busHz = 400000;
    s->magic = MAGSAMPLER_MAGIC;
}

void magSamplerSetSensor( int i, uint8_t address, int bytes, bool enabled ) {
    if ( i < 0 || i >= MAGSAMPLER_MAX_SENSORS ) {
        return;
    }
    magSampler.address[ i ] = address;
    magSampler.bytes[ i ] = (uint8_t)bytes;
    magSampler.enabled[ i ] = enabled ? 1 : 0;
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
