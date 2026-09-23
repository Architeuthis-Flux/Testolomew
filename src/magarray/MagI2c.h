// SPDX-License-Identifier: MIT
#ifndef MAGI2C_H
#define MAGI2C_H
// ---------------------------------------------------------------------------
// The I2C block driven by its registers, for the sensor reads. Wire waits a
// fixed 10 ms for every flag; a glitch on the lines (the LED strip's data
// wire coupling into SDA, 2026-09-18) drops the block out of master mode
// mid-read and every remaining wait then burns the full 10 ms. Here a wait
// ends the moment the block is no longer master or reports an error, and in
// a millisecond anyway. The hot path is in ITCM so the V3F, which has no
// instruction cache, runs it at speed (MagSampler.h).
//
// Two kinds of read, one per sensor family:
//   - a BARE read (MAGI2C_NO_REGISTER): START, address+R, the bytes. The
//     TMAG5273 in its 1-byte-read mode answers with its channels.
//   - a REGISTER read: START, address+W, the register number, a repeated
//     START, address+R, the bytes. Any ordinary register-addressed part
//     (the MMC56x3's nine data bytes from register 0x00).
// The flag sequence of the register phase is the one the core's Wire.cpp
// uses (endTransmission(false) then requestFrom): SB, ADDR, TXE per byte,
// BTF, then the second START.
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <stddef.h>

#define MAGI2C_NO_REGISTER -1

// Where a failed read broke, for the status line (magI2cFailureName()).
#define MAGI2C_OK 0
#define MAGI2C_FAIL_NO_START 1  // the block never raised SB: it is wedged (BUSY stuck), or the lines are held
#define MAGI2C_FAIL_ADDRESS 2   // nobody acknowledged the address (the part is absent, resetting or unpowered)
#define MAGI2C_FAIL_REGISTER 3  // the register number was not taken (a NACK, or the block dropped out of master mode)
#define MAGI2C_FAIL_DATA 4      // a data byte never came (a glitch mid-read, or the part stopped)
const char* magI2cFailureName( int code );

// The registers of I2C block `peripheral` (1-4) as an opaque pointer, for
// the sampler's shared block (the V3F reads through it without a lookup
// from flash). nullptr = no such block, or the host.
void* magI2cRegisters( int peripheral );

// One read of `count` bytes from `address` on block `peripheral`, bare or
// from register `reg` (see above). false = failed, within a millisecond;
// the block is left idle and ready for the next transaction. FROM THE V5F
// (its timeout clock is micros()).
bool magI2cBurstRead( int peripheral, uint8_t address, int reg, uint8_t* out, size_t count );
// The same on the block's registers (what magI2cRegisters() gave), FROM
// THE V3F (the sampler; its clock is millis(): it has no finer one, and the
// code in ITCM must not ask which core it is on - that is a call into flash).
// Returns MAGI2C_OK or the MAGI2C_FAIL_ code of where it broke.
int magI2cBurstReadOn( void* registers, uint8_t address, int reg, uint8_t* out, size_t count );

// The block reset and re-initialised as a master at `hz` (a wedge cleared),
// the pins left as they are. Registers only, so either core can do it.
void magI2cBusReset( int peripheral, uint32_t hz );

#endif // MAGI2C_H
