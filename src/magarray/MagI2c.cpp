// SPDX-License-Identifier: MIT
#include "MagI2c.h"

#if __has_include( <ch32h4_i2c.h> )
#include <ch32h4_i2c.h> // the I2C block's registers
extern "C" {
#include <ch32h4_itcm.h> // __itcm_func
}
#define MAGI2C_DIRECT 1
#else
#define MAGI2C_DIRECT 0 // the host simulation has no I2C block
#endif

const char* magI2cFailureName( int code ) {
    switch ( code ) {
    case MAGI2C_OK:
        return "none";
    case MAGI2C_FAIL_NO_START:
        return "no START (the block wedged, or a line held)";
    case MAGI2C_FAIL_ADDRESS:
        return "address not acknowledged (absent, resetting or unpowered)";
    case MAGI2C_FAIL_REGISTER:
        return "register number not taken (NACK, or dropped out of master mode)";
    case MAGI2C_FAIL_DATA:
        return "a data byte never came (a glitch mid-read)";
    default:
        return "?";
    }
}

#if !MAGI2C_DIRECT
void* magI2cRegisters( int ) {
    return nullptr;
}
bool magI2cBurstRead( int, uint8_t, int, uint8_t*, size_t ) {
    return false;
}
int magI2cBurstReadOn( void*, uint8_t, int, uint8_t*, size_t ) {
    return MAGI2C_FAIL_NO_START;
}
void magI2cBusReset( int, uint32_t ) {}
#else

void* magI2cRegisters( int peripheral ) {
    return (void*)ch32h4_i2c_regs( (uint8_t)peripheral );
}

// The I2C block reset and set up again as a master at `hz`, the way Wire's
// begin() does it (minus the pins, which stay as they are): what gets a
// block out of a wedge a glitch put it in.
void magI2cBusReset( int peripheral, uint32_t hz ) {
    uint8_t id = (uint8_t)peripheral;
    I2C_TypeDef* dev = ch32h4_i2c_regs( id );
    if ( dev == nullptr ) {
        return;
    }
    ch32h4_i2c_reset( id );
    ch32h4_i2c_clock_enable( id );
    I2C_InitTypeDef init = { };
    init.I2C_Mode = I2C_Mode_I2C;
    init.I2C_DutyCycle = I2C_DutyCycle_2;
    init.I2C_OwnAddress1 = 0x00;
    init.I2C_Ack = I2C_Ack_Enable;
    init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    init.I2C_ClockSpeed = hz;
    I2C_Init( dev, &init );
    I2C_Cmd( dev, ENABLE );
}

// A wait ends the moment the block is no longer master or reports a bus
// error, and in MAGI2C_READ_TIMEOUT_US anyway (a 9-byte bare read at
// 400 kHz is 225 us; a register read of 9 bytes about 320 us). The STOP
// request an aborted read leaves pending would stop the next START ever
// happening; it is cleared first.
#define MAGI2C_READ_TIMEOUT_US 1000
#define MAGI2C_ERRORS ( I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_AF )
#define MAGI2C_IDLE_TURNS 2000 // register polls waited for the last STOP to finish (~20 us on the V5F, ~80 on the V3F)

// The clock for the timeout. micros() is right on the V5F only: its
// sub-millisecond part is that core's own SysTick, which the V3F does not
// have (there it reads as noise, and a timeout of noise fails one read in
// eight - seen 2026-09-18). On the V3F, where the sampler runs, millis()
// (kept by the V5F in shared memory) is used instead, with a coarser limit.
// Which core is reading is the CALLER's to say (magI2cBurstReadOn is the
// V3F's entry, magI2cBurstRead the V5F's): asking ch32h4_core_num() from
// here is a call from ITCM into flash, which the build's ITCM check
// refuses - reached during a flash write it would execute garbage
// (2026-09-21: it passed once only because the link-time optimiser
// happened to inline it).
#define MAGI2C_READ_TIMEOUT_MS 3

struct ReadClock {
    bool coarse;
    uint32_t start;
};

__itcm_func static ReadClock clockStart( bool coarse ) {
    ReadClock c;
    c.coarse = coarse;
    c.start = coarse ? millis( ) : micros( );
    return c;
}

__itcm_func static bool clockExpired( const ReadClock* c ) {
    return c->coarse ? ( millis( ) - c->start > MAGI2C_READ_TIMEOUT_MS ) : ( micros( ) - c->start > MAGI2C_READ_TIMEOUT_US );
}

// The read's hot path is __itcm_func: the V3F, which runs it most, has no
// instruction cache and fetches flash at a crawl (a pass over eight sensors
// took 6 ms against 1.6 ms of bus time: the block stretched the clock every
// byte waiting for the core to notice). ITCM it reaches over the bus in a
// few cycles. The SDK's calls (in flash) are replaced by the register
// writes they make.
__itcm_func static bool waitFlag( I2C_TypeDef* dev, uint16_t flag, const ReadClock* clock, bool mustBeMaster ) {
    // The clock is looked at every 32nd turn only: it is a safety net, and
    // on the V3F reading it means the other core's memory, with waits, which
    // at every turn of a 20 us wait halved the sampler's pass rate.
    for ( uint32_t turn = 0;; turn++ ) {
        uint16_t star1 = dev->STAR1;
        if ( star1 & flag ) {
            return true;
        }
        if ( star1 & MAGI2C_ERRORS ) {
            return false;
        }
        if ( mustBeMaster && !( dev->STAR2 & I2C_STAR2_MSL ) ) {
            return false; // a STOP the block did not send: the lines glitched
        }
        if ( ( turn & 31u ) == 31u && clockExpired( clock ) ) {
            return false;
        }
    }
}

// MAGI2C_OK, or where it broke; the block is left idle and ready for the
// next transaction. On the I2C block's registers alone, so it runs the same
// from either core (the sampler on the V3F calls it, MagSampler.cpp).
__itcm_func static int burstReadOn( I2C_TypeDef* dev, uint8_t address, int reg, uint8_t* out, size_t count, bool coarseClock ) {
    // The previous read's STOP may still be going out on the wires (the
    // request bit stays set until it has; the block is BUSY until the
    // lines are idle). A START asked for before that is lost and the read
    // fails: 2026-09-21, once the read path stopped calling into flash
    // between reads, sensors 1-7 (each read straight after the one before)
    // failed 11 % of their reads and sensor 0 (first in the pass, after
    // the pass's own overhead) none. So: wait for the block to be idle,
    // briefly - MAGI2C_IDLE_TURNS is well past a STOP's few microseconds
    // and short enough that a wedged block (BUSY stuck by a glitch, which
    // the reset logic below and in MagArray deals with) costs little.
    for ( uint32_t turn = 0; turn < MAGI2C_IDLE_TURNS && ( ( dev->CTLR1 & I2C_CTLR1_STOP ) || ( dev->STAR2 & I2C_STAR2_BUSY ) ); turn++ ) {
    }
    if ( ( dev->CTLR1 & I2C_CTLR1_STOP ) && !( dev->STAR2 & I2C_STAR2_MSL ) ) {
        dev->CTLR1 &= (uint16_t)~I2C_CTLR1_STOP; // left over from an aborted transaction
    }
    dev->STAR1 &= (uint16_t)~MAGI2C_ERRORS;
    ReadClock t0 = clockStart( coarseClock );
    dev->CTLR1 |= I2C_CTLR1_ACK;
    if ( reg >= 0 ) {
        // The register phase: START, the address for writing, the register
        // number, then the read below begins with a repeated START (no
        // STOP in between). Wire's endTransmission( false ) step for step:
        // SB, ADDR (STAR1 then STAR2 clears it), TXE after the byte, BTF.
        dev->CTLR1 |= I2C_CTLR1_START;
        if ( !waitFlag( dev, I2C_STAR1_SB, &t0, false ) ) {
            dev->CTLR1 |= I2C_CTLR1_STOP;
            return MAGI2C_FAIL_NO_START;
        }
        dev->DATAR = (uint16_t)( address << 1 ); // the address, write direction
        if ( !waitFlag( dev, I2C_STAR1_ADDR, &t0, true ) ) {
            dev->STAR1 &= (uint16_t)~I2C_STAR1_AF; // a NACK: nobody there
            dev->CTLR1 |= I2C_CTLR1_STOP;
            return MAGI2C_FAIL_ADDRESS;
        }
        (void)dev->STAR2;
        dev->DATAR = (uint16_t)( reg & 0xFF );
        if ( !waitFlag( dev, I2C_STAR1_TXE, &t0, true ) || !waitFlag( dev, I2C_STAR1_BTF, &t0, true ) ) {
            dev->STAR1 &= (uint16_t)~I2C_STAR1_AF;
            dev->CTLR1 |= I2C_CTLR1_STOP;
            return MAGI2C_FAIL_REGISTER;
        }
    }
    dev->CTLR1 |= I2C_CTLR1_START; // (a repeated one after the register phase)
    if ( !waitFlag( dev, I2C_STAR1_SB, &t0, false ) ) {
        dev->CTLR1 |= I2C_CTLR1_STOP;
        return MAGI2C_FAIL_NO_START;
    }
    dev->DATAR = (uint16_t)( ( address << 1 ) | 1u ); // the address, read direction
    if ( !waitFlag( dev, I2C_STAR1_ADDR, &t0, true ) ) {
        dev->STAR1 &= (uint16_t)~I2C_STAR1_AF; // a NACK: nobody there
        dev->CTLR1 |= I2C_CTLR1_STOP;
        return MAGI2C_FAIL_ADDRESS;
    }
    (void)dev->STAR2; // reading STAR1 then STAR2 clears ADDR
    for ( size_t i = 0; i < count; i++ ) {
        if ( i + 1 == count ) {
            // NACK the last byte, and the STOP after it, before it arrives.
            dev->CTLR1 &= (uint16_t)~I2C_CTLR1_ACK;
            dev->CTLR1 |= I2C_CTLR1_STOP;
        }
        if ( !waitFlag( dev, I2C_STAR1_RXNE, &t0, i + 1 < count ) ) {
            dev->CTLR1 |= I2C_CTLR1_STOP;
            return MAGI2C_FAIL_DATA;
        }
        out[ i ] = (uint8_t)dev->DATAR;
    }
    return MAGI2C_OK;
}

// The V3F's entry (the sampler): its clock is millis().
__itcm_func int magI2cBurstReadOn( void* registers, uint8_t address, int reg, uint8_t* out, size_t count ) {
    if ( registers == nullptr ) {
        return MAGI2C_FAIL_NO_START;
    }
    return burstReadOn( (I2C_TypeDef*)registers, address, reg, out, count, true );
}

// The V5F's entry (the drivers, the status line's timing read): micros().
bool magI2cBurstRead( int peripheral, uint8_t address, int reg, uint8_t* out, size_t count ) {
    I2C_TypeDef* dev = ch32h4_i2c_regs( (uint8_t)peripheral );
    if ( dev == nullptr ) {
        return false;
    }
    return burstReadOn( dev, address, reg, out, count, false ) == MAGI2C_OK;
}
#endif
