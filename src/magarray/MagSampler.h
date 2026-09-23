// SPDX-License-Identifier: MIT
#ifndef MAGSAMPLER_H
#define MAGSAMPLER_H
// ---------------------------------------------------------------------------
// The sensor round-robin on the CH32H417's other core.
//
// Reading eight TMAG5273s over I2C is a polled affair - a byte every 20 us at
// 400 kHz, ~190 us a sensor, 1.5 ms a pass (an MMC56x3's register read is
// ~320 us) - and on one core it was 65 % of
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

#define MAGSAMPLER_MAX_SENSORS 16
#define MAGSAMPLER_MAX_BUSES 4
#define MAGSAMPLER_MAX_BYTES 10
#define MAGSAMPLER_MAGIC 0x53414D53u // "SAMS" (the block's layout changed with the second bus and the per-bus wedge rule, 2026-09-21)

enum MagSamplerCommand {
    MAGSAMPLER_STOP = 0,
    MAGSAMPLER_RUN = 1,
    MAGSAMPLER_PAUSE = 2
};

struct MagSamplerShared {
    volatile uint32_t magic;   // MAGSAMPLER_MAGIC once the fields below are set (the V5F writes)
    volatile uint32_t command; // MagSamplerCommand (the V5F writes)
    volatile uint32_t state;   // what the sampler is doing: the command it last obeyed (the V3F writes)
    volatile uint32_t count;
    // Per sensor: the I2C block it is on (its registers, magI2cRegisters():
    // sensors may be spread over the chip's four blocks), its address, the
    // data register (MAGI2C_NO_REGISTER = a bare read, the TMAG5273's
    // 1-byte-read mode) and how many bytes one read is.
    volatile uint32_t registers[ MAGSAMPLER_MAX_SENSORS ];
    volatile uint8_t bus[ MAGSAMPLER_MAX_SENSORS ]; // MagArray's bus index (0-3): the wedge rule below counts per bus
    volatile uint8_t address[ MAGSAMPLER_MAX_SENSORS ];
    volatile int16_t reg[ MAGSAMPLER_MAX_SENSORS ];
    volatile uint8_t bytes[ MAGSAMPLER_MAX_SENSORS ];
    volatile uint8_t enabled[ MAGSAMPLER_MAX_SENSORS ]; // 0 = skip this one (it does not answer)
    // The results.
    volatile uint32_t seq[ MAGSAMPLER_MAX_SENSORS ]; // odd while raw[] is being written
    volatile uint8_t raw[ MAGSAMPLER_MAX_SENSORS ][ MAGSAMPLER_MAX_BYTES ];
    volatile uint32_t stampUs[ MAGSAMPLER_MAX_SENSORS ]; // millis() when the read finished (the V3F has no finer clock)
    volatile uint32_t reads[ MAGSAMPLER_MAX_SENSORS ];
    volatile uint32_t fails[ MAGSAMPLER_MAX_SENSORS ];
    volatile uint8_t lastFail[ MAGSAMPLER_MAX_SENSORS ]; // where the last failed read broke (MAGI2C_FAIL_*), for `m`
    volatile uint32_t passes;
    // The wedge rule, per bus: every sensor on a bus (more than one of
    // them) failing MAGSAMPLER_RESET_AFTER_PASSES passes running is that
    // bus's block wedged, and bit b of resetWanted asks the V5F to reset it
    // (the V3F stays off that bus until the bit is cleared; the other buses
    // go on). One rule over every sensor let a wedged first bus fail on
    // while the second bus's sensor kept the count from being "all"
    // (2026-09-21: eight TMAGs lost for seconds, twice).
    volatile uint32_t failedPasses[ MAGSAMPLER_MAX_BUSES ];
    volatile uint32_t resets;      // times the sampler asked for a block to be reset
    volatile uint32_t resetWanted; // bit b: bus b (set by the V3F, cleared by the V5F once the block is reset)
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

// V5F side. Clear the block and describe the sensors (before the V3F is
// told to run); then run / pause / stop, each waiting (up to 20 ms) for the
// sampler to say it has obeyed - false if it did not, which means the V3F
// is not running loop1() at all.
void magSamplerSetup( int count );
void magSamplerSetBusHz( uint32_t hz ); // for the block resets (after setup)
void magSamplerSetPassPeriodMs( uint32_t ms ); // 0 = free-running
// Sensor i: on the block whose registers these are (magI2cRegisters()), at
// this address, read from this register (MAGI2C_NO_REGISTER = bare), this
// many bytes; enabled = read it at all.
void magSamplerSetSensor( int i, int bus, void* registers, uint8_t address, int reg, int bytes, bool enabled );
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
