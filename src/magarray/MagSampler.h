// SPDX-License-Identifier: MIT
#ifndef MAGSAMPLER_H
#define MAGSAMPLER_H
// ---------------------------------------------------------------------------
// The sensor round-robin on the CH32H417's other core.
//
// Reading eight TMAG5273s over I2C is a polled affair - a byte every 20 us at
// 400 kHz, ~190 us a sensor, 1.5 ms a pass - and on one core it was 65 % of
// the main loop and a 1.5 ms hole in everything else's timing several
// hundred times a second. The V3F (100 MHz, in-order, otherwise asleep)
// has nothing better to do: loop1() below runs the passes back to back and
// leaves each sensor's latest bytes in shared RAM with a sequence number;
// MagArray on the V5F picks them up at its own pace, decodes and averages
// them into frames as before, and never touches the bus while the sampler
// runs. The sampler is started once the V5F has addressed and configured
// the sensors (through Wire), and paused - it acknowledges - whenever the
// V5F needs the bus itself (recovery, a power-cycle, the bus check).
//
// The shared block lives in the .xcore section (the one memory both cores
// reach at speed, uninitialised at reset: the V5F clears it before the
// start). Each sensor's slot is written under a seqlock - the sequence
// number is odd while the bytes are being written - so a reader that sees
// an even, unchanged sequence around its copy has a whole sample.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#define MAGSAMPLER_MAX_SENSORS 8
#define MAGSAMPLER_MAX_BYTES 10
#define MAGSAMPLER_MAGIC 0x53414D50u // "SAMP"

enum MagSamplerCommand {
    MAGSAMPLER_STOP = 0,
    MAGSAMPLER_RUN = 1,
    MAGSAMPLER_PAUSE = 2
};

struct MagSamplerShared {
    volatile uint32_t magic;   // MAGSAMPLER_MAGIC once the fields below are set (the V5F writes)
    volatile uint32_t command; // MagSamplerCommand (the V5F writes)
    volatile uint32_t state;   // what the sampler is doing: the command it last obeyed (the V3F writes)
    volatile uint32_t peripheral;
    volatile uint32_t count;
    volatile uint8_t address[ MAGSAMPLER_MAX_SENSORS ];
    volatile uint8_t bytes[ MAGSAMPLER_MAX_SENSORS ];
    volatile uint8_t enabled[ MAGSAMPLER_MAX_SENSORS ]; // 0 = skip this one (it does not answer)
    // The results.
    volatile uint32_t seq[ MAGSAMPLER_MAX_SENSORS ]; // odd while raw[] is being written
    volatile uint8_t raw[ MAGSAMPLER_MAX_SENSORS ][ MAGSAMPLER_MAX_BYTES ];
    volatile uint32_t stampUs[ MAGSAMPLER_MAX_SENSORS ]; // millis() when the read finished (the V3F has no finer clock)
    volatile uint32_t reads[ MAGSAMPLER_MAX_SENSORS ];
    volatile uint32_t fails[ MAGSAMPLER_MAX_SENSORS ];
    volatile uint32_t passes;
    volatile uint32_t failedPasses; // passes in a row in which every sensor failed
    volatile uint32_t resets;       // times the sampler asked for the I2C block to be reset for that
    volatile uint32_t resetWanted;  // set by the V3F (it stays off the bus), cleared by the V5F once the block is reset
    volatile uint32_t busHz;        // what to set it up at again
    // Pacing (the V5F writes): no pass starts within passPeriodMs of the
    // last one's start (0 = free-running, ~484 passes a second on the
    // bench). The reads then come in a burst once a period rather than
    // drifting against the sensors' own conversion clocks (:load sampler
    // <ms>; the backlight showed the drift, 2026-09-20).
    volatile uint32_t passPeriodMs;
    volatile uint32_t lastPassMs;
};
#define MAGSAMPLER_RESET_AFTER_PASSES 2

extern MagSamplerShared magSampler;

// V5F side. Clear the block and describe the bus (before the V3F is told to
// run); then run / pause / stop, each waiting (up to 20 ms) for the sampler
// to say it has obeyed - false if it did not, which means the V3F is not
// running loop1() at all.
void magSamplerSetup( int peripheral, int count );
void magSamplerSetBusHz( uint32_t hz ); // for the sampler's own block resets (after setup)
void magSamplerSetPassPeriodMs( uint32_t ms ); // 0 = free-running
void magSamplerSetSensor( int i, uint8_t address, int bytes, bool enabled );
bool magSamplerCommand( MagSamplerCommand command );
bool magSamplerRunning( void );
// A whole sample of sensor i, if there is one newer than `lastSeq` (updated).
bool magSamplerTake( int i, uint32_t* lastSeq, uint8_t* out, uint32_t* stampUs );
// Before a reflash from outside (wlink): stop the sampler and park the V3F
// in ITCM for good. That core has no instruction cache and fetches every
// instruction from the flash being programmed, and a page program with it
// running does not complete (the core's ch32h4_park.c; seen here as
// "Error while fastprogram" on every other flash, and one that bricked the
// board, 2026-09-19). A reset brings it back. false if it did not park.
bool magSamplerParkForFlash( void );
bool magSamplerParked( void ); // F has been given: the array leaves the sampler and the bus alone

#endif // MAGSAMPLER_H
