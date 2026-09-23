// SPDX-License-Identifier: MIT
#include "MagArray.h"

#include <Wire.h>
#include <math.h>
#if __has_include( <ch32h4_i2c.h> )
#include <ch32h4_i2c.h> // the block's registers, for the wedge check below
#define MAG_HAVE_I2C_REGS 1
#else
#define MAG_HAVE_I2C_REGS 0 // the host simulation
#endif

#include "Console.h"
#include "MagI2c.h"
#include "MagSampler.h"

#define STREAM_EVERY_N_FRAMES 5 // CSV at 20 Hz; 100 Hz of text outruns 115200 baud
#define IDENTIFY_PERIOD_MS 250
#define RECOVERY_PERIOD_MS 2000
#define BUS_STUCK_US 5000   // a read that takes this long failed by Wire's 10 ms timeout, not by a NACK
#define SAMPLER_LOST_MS 100 // no successful read by the other core's sampler for this long = the sensor is lost
#define MAG_MMC_ZERO_READS_TO_RESET 3 // an MMC answering all zeros this many reads running has reset: configure it again
// A bus nothing can be talked to on (no pull-ups: the second bus before it
// is wired) costs Wire's 10 ms timeout to ask. One that has failed this many
// recoveries running is asked again only every MAG_BUS_FAULT_RETRY_RUNS
// recoveries (16 s), and taken as dead in between: its sensors wait, the
// other buses' sensors are recovered as usual, and the sampler is not held
// 10 ms every 2 s for a bus that is not there.
#define MAG_BUS_FAULT_STREAK 3
#define MAG_BUS_FAULT_RETRY_RUNS 8

MagArray& magArray = MagArray::getInstance( );

MagArray& MagArray::getInstance( ) {
    static MagArray instance;
    return instance;
}

// The Wire objects behind the buses: Wire for the first pair in the table,
// these for any other pairs (each is told its pins before begin()).
static TwoWire extraWires[ MAG_MAX_BUSES - 1 ];

// ---- console commands ------------------------------------------------------

static void onStatus( Stream* out ) { magArray.printStatus( out ); }

static void onZero( Stream* out ) {
    out->println( "re-zeroing - keep the magnet away" );
    magArray.startBaseline( );
}

static void onPowerCycle( Stream* out ) {
    magArray.pauseSampler( );
    int found = magArray.begin( false );
    out->print( found );
    out->println( " sensors answered" );
    magArray.printStatus( out );
}

static void onStream( Stream* out ) {
    magArray.streaming = !magArray.streaming;
    if ( magArray.streaming ) {
        out->println( "mag,t_ms,bx0,by0,bz0,bx1,by1,bz1,... (mT, board frame, baseline removed)" );
    }
}

static void onBusCheck( Stream* out ) {
    magArray.pauseSampler( );
    magArray.printBusCheck( out ); // powers the sensors one at a time: they come back at their factory addresses...
    out->println( "re-addressing after the check" );
    magArray.begin( false ); // ...so they are walked through addressing again (the zero in use is kept)
}

static void onIdentify( Stream* out ) {
    magArray.identifying = !magArray.identifying;
    out->println( magArray.identifying ? "identify on - hold the magnet over one sensor at a time" : "identify off" );
}

static void onParkForFlash( Stream* out ) {
    bool parked = magSamplerParkForFlash( );
    out->println( parked ? "the V3F is parked in ITCM and the sampler stopped: flash now (pio run -t upload does this itself). No sensors are read until a reset; if the flash fails, wlink reset"
                         : "the V3F did not park (it is not running the sampler?): flash anyway, and hold RESET if the flash fails" );
    out->flush( );
}

static void onWeightCap( Stream* out ) {
    long tenths = consoleReadNumber( out, "how much more the fit trusts the quieter sensor types, in tenths (20 = 2.0x a TMAG5273; 0 = every sensor equal), then Enter: ", 8000 );
    if ( tenths < 0 || tenths > 1000 ) {
        out->println( "no number (0-1000) - nothing changed" );
        return;
    }
    magArray.setWeightCap( tenths / 10.0f );
    char line[ 160 ];
    snprintf( line, sizeof( line ), "weight cap %.1f (MAG_WEIGHT_CAP makes it permanent); weights now:", tenths / 10.0f );
    out->print( line );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        snprintf( line, sizeof( line ), " %d:%.1f", i, magArray.weight[ i ] );
        out->print( line );
    }
    out->println( );
}

static void onMmcVerb( int argc, char** argv, Stream* out ) {
    if ( argc < 2 ) {
        out->println( "usage: :mmc <sensor> [cfg <odrHz> <bw 0-3> <autoSR 0|1>]" );
        return;
    }
    int i = atoi( argv[ 1 ] );
    bool cfg = argc >= 6 && strcmp( argv[ 2 ], "cfg" ) == 0;
    magArray.probeMmc( out, i, cfg, cfg ? atoi( argv[ 3 ] ) : 0, cfg ? atoi( argv[ 4 ] ) : 0, cfg ? atoi( argv[ 5 ] ) != 0 : true );
}

static void onWatchVerb( int argc, char** argv, Stream* out ) {
    if ( argc < 2 ) {
        out->println( "usage: :watch <sensor>[,<sensor>...]|all|off" );
        return;
    }
    if ( strcmp( argv[ 1 ], "off" ) == 0 ) {
        magArray.watchMask = 0;
        out->println( "watch off" );
        return;
    }
    uint32_t mask = 0;
    if ( strcmp( argv[ 1 ], "all" ) == 0 ) {
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            if ( !magArray.sensor( i ).ok ) {
                mask |= 1u << i; // every sensor missing now
            }
        }
    } else {
        for ( char* tok = strtok( argv[ 1 ], "," ); tok != nullptr; tok = strtok( nullptr, "," ) ) {
            int i = atoi( tok );
            if ( i >= 0 && i < MAG_SENSOR_COUNT ) {
                mask |= 1u << i;
            }
        }
    }
    if ( mask == 0 ) {
        out->println( "nothing to watch" );
        return;
    }
    magArray.watchMask = mask;
    magArray.watchSinceMs = millis( );
    char line[ 160 ];
    int n = snprintf( line, sizeof( line ), "watching" );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( mask & ( 1u << i ) ) {
            n += snprintf( line + n, sizeof( line ) - n, " sensor %d (bus %d, 0x%02X)", i, magArray.sensor( i ).bus, magSensorPlaces[ i ].address );
        }
    }
    snprintf( line + n, sizeof( line ) - n, ": looked for every half second, the screen says when each answers" );
    out->println( line );
}

void MagArray::probeMmc( Stream* out, int i, bool reconfigure, int odrHz, int bw, bool autoSr ) {
    char line[ 200 ];
    if ( i < 0 || i >= MAG_SENSOR_COUNT || sensors[ i ].type != MAG_MMC56X3 ) {
        out->println( "not an MMC56x3 slot" );
        return;
    }
    MagSensorState& s = sensors[ i ];
    if ( !busUsable( s.bus ) ) {
        out->println( "its bus is not set up" );
        return;
    }
    pauseSampler( );
    TwoWire* wire = wireOf( i );
    if ( reconfigure ) {
        bool ok = mmc56x3Begin( wire, &s.mmc, s.address, (uint8_t)bw, (uint16_t)odrHz, autoSr );
        snprintf( line, sizeof( line ), "sensor %d reconfigured: ODR %d Hz, BW %d, auto SET/RESET %s: %s", i, odrHz, bw, autoSr ? "on" : "off", ok ? "ok" : "FAILED" );
        out->println( line );
        s.ok = ok;
        delay( 30 );
    }
    for ( int n = 0; n < 3; n++ ) {
        uint8_t status = 0, raw[ MMC56X3_DATA_BYTES ] = { 0 };
        bool st = mmc56x3ReadStatus( wire, s.address, &status );
        bool rd = mmc56x3ReadRaw( wire, s.address, raw );
        MMC56x3Reading r;
        mmc56x3Decode( raw, &r );
        snprintf( line, sizeof( line ), "  %d: status 0x%02X%s%s  data %02X%02X %02X%02X %02X%02X %02X %02X %02X -> x %+.4f y %+.4f z %+.4f mT%s", n,
                  status, ( status & 0x40 ) ? " meas-done" : "", ( status & 0x10 ) ? " otp-ok" : "", raw[ 0 ], raw[ 1 ], raw[ 2 ], raw[ 3 ], raw[ 4 ], raw[ 5 ], raw[ 6 ],
                  raw[ 7 ], raw[ 8 ], r.x, r.y, r.z, ( st && rd ) ? "" : "  (a read FAILED)" );
        out->println( line );
        delay( 50 );
    }
    resumeSampler( );
}

// ---- power-up addressing ---------------------------------------------------

void MagArray::setVcc( int i, int level ) {
    if ( magSensorPlaces[ i ].vccPin >= 0 ) {
        digitalWrite( magSensorPlaces[ i ].vccPin, level );
    }
}

// Each sensor's weight for the fit where it reads noise, from its type's
// noise and the cap (MagArrayConfig.h: magSensorFrameWeight at zero field).
// The fit itself takes frameWeights(), which also counts the reading.
void MagArray::setWeightCap( float cap ) {
    weightCap = cap;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        noiseMt[ i ] = magSensorTypeNoiseMt( magSensorPlaces[ i ].type );
        weight[ i ] = magSensorFrameWeight( magSensorPlaces[ i ].type, 0.0f, cap );
    }
}

void MagArray::frameWeights( const Vec3* fields, float* out ) const {
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        float size = sqrtf( fields[ i ].x * fields[ i ].x + fields[ i ].y * fields[ i ].y + fields[ i ].z * fields[ i ].z );
        out[ i ] = magSensorFrameWeight( magSensorPlaces[ i ].type, size, weightCap );
    }
}

// The buses the table names: one MagBus per distinct SCL/SDA pair, in the
// order the pairs first appear (the first is Wire). Each is started at
// MAG_I2C_HZ; a pair the silicon cannot serve (or -1: not wired) gets
// peripheral 0 and its sensors are reported absent.
void MagArray::setupBuses( ) {
    buses = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        const MagSensorPlace& p = magSensorPlaces[ i ];
        int b = -1;
        for ( int k = 0; k < buses; k++ ) {
            if ( busTable[ k ].sclPin == p.sclPin && busTable[ k ].sdaPin == p.sdaPin ) {
                b = k;
                break;
            }
        }
        if ( b < 0 ) {
            if ( buses >= MAG_MAX_BUSES ) {
                b = MAG_MAX_BUSES - 1; // more pairs than blocks: the last bus takes the overflow (and its sensors fail)
            } else {
                b = buses++;
                busTable[ b ].wire = b == 0 ? &Wire : &extraWires[ b - 1 ];
                busTable[ b ].sclPin = p.sclPin;
                busTable[ b ].sdaPin = p.sdaPin;
                busTable[ b ].peripheral = 0;
                busTable[ b ].probeAddress = 0;
                busTable[ b ].resets = 0;
            }
        }
        sensors[ i ].bus = b;
        // What should answer on this bus for a bus-fault check: a fixed-
        // address part if there is one, else the first TMAG factory address
        // (an address nobody is at answers with a clean NACK, which is fine).
        if ( busTable[ b ].probeAddress == 0 && p.type != MAG_TMAG5273 ) {
            busTable[ b ].probeAddress = p.address;
        }
    }
    for ( int b = 0; b < buses; b++ ) {
        MagBus& bus = busTable[ b ];
        if ( bus.probeAddress == 0 ) {
            bus.probeAddress = tmag5273FactoryAddresses[ 0 ];
        }
        bus.wire->end( );
        if ( bus.sclPin < 0 || bus.sdaPin < 0 ) {
            bus.peripheral = 0;
            continue;
        }
        bus.wire->setSCL( bus.sclPin );
        bus.wire->setSDA( bus.sdaPin );
        bus.wire->begin( );
        bus.wire->setClock( MAG_I2C_HZ );
        bus.peripheral = bus.wire->peripheral( );
    }
    busPeripheral = buses > 0 ? busTable[ 0 ].peripheral : 0;
}

int MagArray::begin( bool zero ) {
    static bool commandsAdded = false;
    if ( !commandsAdded ) {
        commandsAdded = true;
        consoleAddCommand( 'm', "magnetometer array status", onStatus );
        consoleAddCommand( 'z', "re-zero the ambient baseline (magnet away!)", onZero );
        consoleAddCommand( 'p', "power-cycle and re-address every sensor", onPowerCycle );
        consoleAddCommand( 'f', "stream field frames as CSV (toggle)", onStream );
        consoleAddCommand( 'i', "identify: show the strongest sensor (toggle)", onIdentify );
        consoleAddCommand( 'b', "bus check: pull-ups, and what answers with each sensor powered alone", onBusCheck );
        consoleAddCommand( 'F', "before a reflash: stop the sampler and park the other core (it fetches the flash being written); reset brings it back", onParkForFlash );
        consoleAddCommand( 'w', "weight cap: how much more the fit trusts the quieter sensor types (w400<Enter> = 40.0, the boot default; w20 = 2.0; 0 = equal)", onWeightCap );
        consoleAddVerb( "mmc", "<sensor> [cfg <odrHz> <bw 0-3> <autoSR 0|1>]", "an MMC56x3's status and raw data three times 50 ms apart, reconfigured first if asked", CONSOLE_CHANGES, onMmcVerb );
        consoleAddVerb( "watch", "<sensor>[,<sensor>...]|all|off", "look for sensors every half second while they are wired; the screen shows a line per sensor, green when it answers", CONSOLE_CHANGES, onWatchVerb );
    }

    // Grounds first, then every switched supply driven low, and long enough
    // for the bypass capacitors to empty so each sensor really does
    // power-on-reset. (A fixed-address part has no pins here: it is powered
    // for good.) The baseline is left alone: a run-time power-cycle keeps
    // the zero in use, since the probe may be on the board.
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( magSensorPlaces[ i ].gndPin >= 0 ) {
            pinMode( magSensorPlaces[ i ].gndPin, OUTPUT );
            digitalWrite( magSensorPlaces[ i ].gndPin, LOW );
        }
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( magSensorPlaces[ i ].vccPin >= 0 ) {
            pinMode( magSensorPlaces[ i ].vccPin, OUTPUT );
            digitalWrite( magSensorPlaces[ i ].vccPin, LOW );
        }
        sensors[ i ] = MagSensorState( );
        sensors[ i ].type = magSensorPlaces[ i ].type;
        sensors[ i ].address = magSensorPlaces[ i ].address;
        sensors[ i ].rangeMt = magSensorPlaces[ i ].type == MAG_MMC56X3 ? MMC56X3_RANGE_MT : 40.0f;
        position[ i ] = { magSensorPlaces[ i ].x, magSensorPlaces[ i ].y, magSensorPlaces[ i ].z };
        field[ i ] = raw[ i ] = { 0, 0, 0 };
        fresh[ i ] = false;
        sampleSum[ i ] = { 0, 0, 0 };
        sampleCount[ i ] = 0;
        sampleSaturated[ i ] = false;
    }
    setWeightCap( weightCap );
    delay( MAG_POWER_OFF_MS );

    setupBuses( );

    bool all[ MAG_SENSOR_COUNT ];
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        all[ i ] = true;
    }
    addressSensors( all ); // the TMAG5273s, one at a time
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( sensors[ i ].type == MAG_MMC56X3 && !disabled[ i ] ) {
            beginFixedSensor( i );
        }
    }

    if ( zero || !baselineReady( ) ) {
        startBaseline( ); // (a power-cycle at run time keeps the zero in use: the probe may be on the board)
    }
    startSampler( );
    return sensorsOk( );
}

// The other core takes over the buses: told which sensors to read, and run.
// samplerOn says whether it obeyed (loop1() is running there); if not, the
// buses are read from here as before.
void MagArray::setSamplerPassPeriodMs( uint32_t ms ) {
    samplerPassPeriodMs = ms;
#if MAG_SAMPLER_CORE1
    magSamplerSetPassPeriodMs( ms );
#endif
}

float MagArray::samplerPassesPerSecond( ) {
#if MAG_SAMPLER_CORE1
    uint32_t now = millis( );
    uint32_t passes = magSampler.passes;
    float rate = now > samplerPassesSeenMs ? ( passes - samplerPassesSeen ) * 1000.0f / ( now - samplerPassesSeenMs ) : 0.0f;
    samplerPassesSeen = passes;
    samplerPassesSeenMs = now;
    return rate;
#else
    return 0.0f;
#endif
}

// Sensor i as the sampler needs it: its bus's block, its address, the
// register a read starts at (none for a TMAG5273 in 1-byte-read mode) and
// the read's length.
void MagArray::describeSensorToSampler( int i ) {
#if MAG_SAMPLER_CORE1
    const MagSensorState& s = sensors[ i ];
    int reg = s.type == MAG_MMC56X3 ? MMC56X3_DATA_REGISTER : MAGI2C_NO_REGISTER;
    int bytes = s.type == MAG_MMC56X3 ? (int)mmc56x3ReadBytes( ) : (int)tmag5273ReadBytes( &s.dev );
    magSamplerSetSensor( i, s.bus, magI2cRegisters( busTable[ s.bus ].peripheral ), s.address, reg, bytes, s.ok );
#else
    (void)i;
#endif
}

void MagArray::startSampler( ) {
#if MAG_SAMPLER_CORE1
    magSamplerSetup( MAG_SENSOR_COUNT );
    magSamplerSetBusHz( MAG_I2C_HZ );
    magSamplerSetPassPeriodMs( samplerPassPeriodMs );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        describeSensorToSampler( i );
        samplerSeq[ i ] = 0;
        samplerSetCount[ i ] = -1;
        for ( int k = 0; k < MAGSAMPLER_MAX_BYTES; k++ ) {
            samplerLastRaw[ i ][ k ] = 0xFF;
        }
        samplerZeroReads[ i ] = 0;
        samplerFailsSeen[ i ] = 0;
        samplerReadsSeen[ i ] = 0;
        samplerLastReadMs[ i ] = millis( );
    }
    samplerPassesSeen = 0;
    samplerPassesSeenMs = millis( );
    samplerOn = buses > 0 && busUsable( 0 ) && magSamplerCommand( MAGSAMPLER_RUN );
#else
    samplerOn = false;
#endif
}

// Before this core uses a bus itself (recovery, a power-cycle, the bus
// check): the sampler stops between two reads and says so.
void MagArray::pauseSampler( ) {
    if ( samplerOn ) {
        magSamplerCommand( MAGSAMPLER_PAUSE );
    }
}

void MagArray::resumeSampler( ) {
    if ( samplerOn && !samplerHeld ) {
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            describeSensorToSampler( i );
        }
        magSamplerCommand( MAGSAMPLER_RUN );
    }
}

// Nothing on the bus can be talked to (a line with no pull-up, or held down).
// Every transaction then burns its full timeout, so callers check this first.
bool MagArray::busFault( int b ) {
    MagBus& bus = busTable[ b ];
    if ( !busUsable( b ) ) {
        return true;
    }
    bus.wire->beginTransmission( bus.probeAddress );
    uint8_t result = bus.wire->endTransmission( );
    return result != 0 && result != 2; // 0 = ACK, 2 = nobody at that address
}

// The I2C block's idea of the bus. Its BUSY flag is set by the lines
// themselves - SDA falling while SCL is high is a START - and only a STOP
// clears it, so a glitch on the wires (the LED strip's data line running
// next to them does it) leaves the block sure another master holds the bus,
// and every transaction after that waits Wire's full 10 ms for a START that
// never comes. Between reads nothing of ours is on the bus, so BUSY set
// then, and still set 100 us later, is that wedge: caught here it costs
// microseconds instead of the timeouts.
bool MagArray::busWedged( int b ) {
#if MAG_HAVE_I2C_REGS
    if ( busTable[ b ].peripheral == 0 ) {
        return false;
    }
    I2C_TypeDef* dev = ch32h4_i2c_regs( busTable[ b ].peripheral );
    // The wedge as it was actually found (2026-09-18, `m` after a timed-out
    // read: STAR1 0, STAR2 0, CTLR1 0x0601, both lines high): the STOP
    // request bit still set with the block idle and not master. A STOP
    // asked for once the bus had already gone idle (a glitch ended the
    // transaction early) is never carried out and never cleared, and the
    // block will not START while it is pending. Clearing it by hand is the
    // whole cure, and costs nothing.
    if ( ( dev->CTLR1 & I2C_CTLR1_STOP ) && !( dev->STAR2 & I2C_STAR2_MSL ) ) {
        dev->CTLR1 &= (uint16_t)~I2C_CTLR1_STOP;
        stopClears++;
        return false;
    }
    if ( !( dev->STAR2 & I2C_STAR2_BUSY ) ) {
        return false;
    }
    uint32_t t0 = micros( );
    while ( dev->STAR2 & I2C_STAR2_BUSY ) {
        if ( micros( ) - t0 > 100 ) {
            return true;
        }
    }
#else
    (void)b;
#endif
    return false;
}

// Re-initialise an I2C block: begin() resets the peripheral and, if a device
// is holding SDA, clocks it out.
void MagArray::resetBus( int b ) {
    MagBus& bus = busTable[ b ];
    if ( !busUsable( b ) ) {
        return;
    }
    bus.wire->end( );
    bus.wire->begin( );
    bus.wire->setClock( MAG_I2C_HZ );
    bus.resets++;
    busResets++;
    previousBusResetMs = lastBusResetMs;
    lastBusResetMs = millis( );
}

void MagArray::resetAllBuses( ) {
    for ( int b = 0; b < buses; b++ ) {
        resetBus( b );
    }
}

// The addressing walk, over the TMAG5273s marked in `which`. This is the ONLY
// place sensor power is switched: those sensors go off together, come up one
// at a time to be moved off the factory address, and then EVERY switched
// sensor is powered and stays powered - the ones that did not answer
// included. Supplies are never touched again unless a sensor turns up back
// at a factory address (see recoverLostSensors). Fixed-address parts take
// no part: they are powered for good and found by beginFixedSensor().
void MagArray::addressSensors( const bool* which ) {
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( which[ i ] && sensors[ i ].type == MAG_TMAG5273 ) {
            sensors[ i ].ok = false;
            setVcc( i, LOW );
        }
    }
    delay( MAG_POWER_OFF_MS );

    // Only the buses the walk touches are asked (a dead bus costs 10 ms to ask).
    bool dead[ MAG_MAX_BUSES ];
    for ( int b = 0; b < buses; b++ ) {
        bool needed = false;
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            needed |= which[ i ] && sensors[ i ].type == MAG_TMAG5273 && sensors[ i ].bus == b;
        }
        dead[ b ] = !busUsable( b );
        if ( needed && !dead[ b ] && busFault( b ) ) {
            resetBus( b );
            dead[ b ] = busFault( b );
        }
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( !which[ i ] || sensors[ i ].type != MAG_TMAG5273 ) {
            continue;
        }
        if ( dead[ busOf( i ) ] ) {
            sensors[ i ].trouble = "its bus cannot be talked to at all (pull-ups? a line held low?)";
            continue;
        }
        for ( int attempt = 0; attempt < MAG_ADDRESS_TRIES && !powerUpSensor( i ); attempt++ ) {
            // It failed and is powered down again. A failed transaction can
            // leave the I2C block wedged, so start the next try clean.
            resetBus( busOf( i ) );
            delay( MAG_POWER_OFF_MS );
        }
    }

    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        setVcc( i, HIGH );
    }
}

// One step of the walk: VCC up, find the sensor at its factory address, move
// it to its own address, configure it. On failure VCC goes back low FOR THE
// REST OF THE WALK ONLY - a sensor left powered at a factory address would
// answer for the next one in line. addressSensors() powers it again at the end.
bool MagArray::powerUpSensor( int i ) {
    MagSensorState& s = sensors[ i ];
    TwoWire* wire = wireOf( i );
    s.ok = false;
    s.fails = 0;

    setVcc( i, HIGH );
    delay( MAG_POWER_ON_MS );

    uint8_t assigned = MAG_BASE_ADDRESS + i;
    s.trouble = "nothing at any factory address";
    for ( unsigned a = 0; a < sizeof( tmag5273FactoryAddresses ); a++ ) {
        uint8_t factory = tmag5273FactoryAddresses[ a ];
        if ( !tmag5273Present( wire, factory ) ) {
            continue;
        }
        s.factoryAddress = factory;
        if ( !tmag5273SetAddress( wire, factory, assigned ) ) {
            s.trouble = "found, but it did not move to its new address";
        } else if ( !tmag5273Begin( wire, &s.dev, assigned, MAG_AVERAGING, MAG_HIGH_RANGE, MAG_READ_TEMPERATURE ) ) {
            s.trouble = "moved to its new address, but configuring it failed";
        } else {
            s.trouble = nullptr;
            s.address = assigned;
            s.rangeMt = s.dev.rangeMt;
            s.ok = true;
            return true;
        }
        break;
    }

    s.troubleRegister = tmag5273LastFailedRegister;
    s.troubleCode = tmag5273LastFailedCode;
    setVcc( i, LOW );
    return false;
}

// A fixed-address part (an MMC56x3): is it at its address on its bus, and
// does it take its configuration? No power to switch: it is either there
// or not.
bool MagArray::beginFixedSensor( int i ) {
    MagSensorState& s = sensors[ i ];
    s.ok = false;
    s.fails = 0;
    s.address = magSensorPlaces[ i ].address;
    s.rangeMt = MMC56X3_RANGE_MT;
    MagBus& bus = busTable[ s.bus ];
    if ( !busUsable( s.bus ) ) {
        s.trouble = bus.sclPin < 0 ? "its bus is not wired (BoardPins.h: -1)" : "its bus pins are not a pair this chip can use for I2C";
        return false;
    }
    if ( busFault( s.bus ) ) {
        s.trouble = "its bus cannot be talked to at all (pull-ups? a line held low?)";
        return false;
    }
    uint8_t id = 0;
    if ( !mmc56x3Present( bus.wire, s.address, &id ) ) {
        s.trouble = "nothing answers at its address on its bus";
    } else if ( !mmc56x3Begin( bus.wire, &s.mmc, s.address, MAG_MMC_BANDWIDTH, MAG_MMC_ODR_HZ ) ) {
        s.trouble = "answers, but configuring it failed";
    } else {
        s.trouble = nullptr;
        s.ok = true;
        for ( int k = 0; k < MAGSAMPLER_MAX_BYTES; k++ ) {
            samplerLastRaw[ i ][ k ] = 0xFF;
        }
        return true;
    }
    s.troubleRegister = mmc56x3LastFailedRegister;
    s.troubleCode = mmc56x3LastFailedCode;
    return false;
}

// Sensors that stopped answering, without disturbing anyone's power if that
// can be avoided:
//   1. a bus dead -> re-initialise its I2C block and try again later;
//   2. a fixed-address part -> look for it where it lives and configure it;
//   3. a TMAG5273 still at its own address (it was the bus that hiccupped,
//      the sensor never reset) -> configure it again and carry on;
//   4. something answers at a factory address -> a sensor really did reset,
//      so walk the lost ones (and only them) through addressing again;
//   5. otherwise nobody is home: leave them powered and look again later.
void MagArray::recoverLostSensors( ) {
    recoveryRuns++;
    pauseSampler( );
    // Buses first: one that cannot be talked to is reset and its sensors
    // left for later; one that has been dead MAG_BUS_FAULT_STREAK times
    // running is only asked again every MAG_BUS_FAULT_RETRY_RUNS recoveries
    // (the asking is what costs). The other buses' sensors are recovered
    // regardless: a second bus that is not wired must not stop a TMAG on
    // the first from coming back.
    bool dead[ MAG_MAX_BUSES ];
    int deadBuses = 0;
    for ( int b = 0; b < buses; b++ ) {
        if ( !busUsable( b ) ) {
            dead[ b ] = true;
            deadBuses++;
            continue;
        }
        bool ask = busFaultStreak[ b ] < MAG_BUS_FAULT_STREAK || ( recoveryRuns % MAG_BUS_FAULT_RETRY_RUNS ) == 0;
        if ( !ask ) {
            dead[ b ] = true; // taken as still dead, not asked
            deadBuses++;
            continue;
        }
        dead[ b ] = busFault( b );
        if ( dead[ b ] ) {
            resetBus( b );
            if ( busFaultStreak[ b ] < 255 ) {
                busFaultStreak[ b ]++;
            }
            deadBuses++;
        } else {
            busFaultStreak[ b ] = 0;
        }
    }
    if ( deadBuses == buses ) {
        recoveryNote = "bus fault on every bus: re-initialised the I2C block(s)";
        resumeSampler( );
        return;
    }

    int fixedBack = 0, fixedLost = 0;
    bool needZero[ MAG_SENSOR_COUNT ];
    int needZeroCount = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        needZero[ i ] = false;
        MagSensorState& s = sensors[ i ];
        if ( s.type != MAG_MMC56X3 || s.ok || disabled[ i ] || dead[ s.bus ] ) {
            continue;
        }
        if ( beginFixedSensor( i ) ) {
            s.recoveries++;
            fixedBack++;
            if ( !zeroKnown[ i ] ) {
                needZero[ i ] = true; // first seen since the others zeroed: a provisional zero of its own
                zeroProvisional[ i ] = true;
                needZeroCount++;
            }
        } else {
            fixedLost++;
        }
    }
    if ( needZeroCount > 0 && baselineLeft == 0 ) {
        startBaselineFor( needZero );
    }

    bool lost[ MAG_SENSOR_COUNT ];
    int lostCount = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        MagSensorState& s = sensors[ i ];
        lost[ i ] = false;
        if ( s.type != MAG_TMAG5273 || s.ok || disabled[ i ] || dead[ s.bus ] ) {
            continue;
        }
        uint8_t assigned = MAG_BASE_ADDRESS + i;
        if ( tmag5273Acknowledges( wireOf( i ), assigned ) &&
             tmag5273Begin( wireOf( i ), &s.dev, assigned, MAG_AVERAGING, MAG_HIGH_RANGE, MAG_READ_TEMPERATURE ) ) {
            s.ok = true;
            s.fails = 0;
            s.address = assigned;
            s.rangeMt = s.dev.rangeMt;
            s.recoveries++;
            continue;
        }
        lost[ i ] = true;
        lostCount++;
    }
    if ( lostCount == 0 ) {
        if ( deadBuses > 0 ) {
            recoveryNote = "a bus cannot be talked to (its pull-ups? wired at all?): its sensors wait; the rest are back";
        } else {
            recoveryNote = fixedLost > 0 ? "found the TMAGs still at their own addresses; a fixed-address part is still missing" : ( fixedBack > 0 ? "found them all where they live" : "found them all still at their own addresses" );
        }
        resumeSampler( );
        return;
    }

    bool someoneAtFactoryAddress = false;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( !lost[ i ] ) {
            continue;
        }
        for ( unsigned a = 0; a < sizeof( tmag5273FactoryAddresses ); a++ ) {
            someoneAtFactoryAddress |= tmag5273Present( wireOf( i ), tmag5273FactoryAddresses[ a ] );
        }
    }
    if ( !someoneAtFactoryAddress ) {
        recoveryNote = "lost sensors answer at neither their own nor a factory address";
        resumeSampler( );
        return;
    }

    addressSensors( lost );
    recoveryNote = "walked the lost sensors through addressing again";
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( lost[ i ] && sensors[ i ].ok ) {
            sensors[ i ].recoveries++;
        }
    }
    resumeSampler( );
}

int MagArray::sensorsOk( ) const {
    int n = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        n += sensors[ i ].ok ? 1 : 0;
    }
    return n;
}

// ---- sampling --------------------------------------------------------------

// Sensor readings -> board frame.
//
// The TMAG5273 counts a field as positive when a north pole approaches along
// the arrows of datasheet figure 6-1, i.e. when B points INTO the top of the
// package (+Z), from the pin 4-5-6 side toward the pin 1-2-3 side (+X), and
// from the pin 3 end toward the pin 1 end (+Y). That is a right-handed frame
// with z pointing down into the board. A half turn about its x axis gives the
// "package frame" used here: x toward the pin 1-2-3 side, y from pin 1 toward
// pin 3, z up out of the package top. The MMC56x3's own frame already has z
// up out of its top (datasheet page 18), so it needs no such turn. Then the
// mounting: a half turn about x if the part is on the underside (which
// cancels the first for a TMAG), and its rotation on the board.
Vec3 magSensorToBoard( Vec3 reading, float rotationDeg, bool underside, bool zIntoTop ) {
    bool flip = zIntoTop != underside; // one half turn about x, not two
    float x = reading.x;
    float y = flip ? -reading.y : reading.y;
    float z = flip ? -reading.z : reading.z;
    float a = rotationDeg * (float)M_PI / 180.0f;
    float c = cosf( a );
    float s = sinf( a );
    Vec3 b = { x * c - y * s, x * s + y * c, z };
    return b;
}

Vec3 magBoardToSensor( Vec3 field, float rotationDeg, bool underside, bool zIntoTop ) {
    bool flip = zIntoTop != underside;
    float a = rotationDeg * (float)M_PI / 180.0f;
    float c = cosf( a );
    float s = sinf( a );
    float x = field.x * c + field.y * s;
    float y = -field.x * s + field.y * c;
    Vec3 r = { x, flip ? -y : y, flip ? -field.z : field.z };
    return r;
}

Vec3 MagArray::sensorFrameField( int i ) const {
    float k = 1.0f / magSensorPlaces[ i ].gain;
    Vec3 ungained = { field[ i ].x * k, field[ i ].y * k, field[ i ].z * k };
    return magBoardToSensor( ungained, magSensorPlaces[ i ].rotationDeg, magSensorPlaces[ i ].underside, magSensorTypeZIntoTop( sensors[ i ].type ) );
}

Vec3 MagArray::zeroInSensorFrame( int i ) const {
    float k = 1.0f / magSensorPlaces[ i ].gain;
    Vec3 ungained = { zeroed[ i ].x * k, zeroed[ i ].y * k, zeroed[ i ].z * k };
    return magBoardToSensor( ungained, magSensorPlaces[ i ].rotationDeg, magSensorPlaces[ i ].underside, magSensorTypeZIntoTop( magSensorPlaces[ i ].type ) );
}

Vec3 MagArray::sensorFrameToBoard( int i, Vec3 s ) const {
    float g = magSensorPlaces[ i ].gain;
    return magSensorToBoard( { s.x * g, s.y * g, s.z * g }, magSensorPlaces[ i ].rotationDeg, magSensorPlaces[ i ].underside, magSensorTypeZIntoTop( magSensorPlaces[ i ].type ) );
}

void MagArray::startBaseline( ) {
    baselineRestored = false;
    bool all[ MAG_SENSOR_COUNT ];
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        all[ i ] = true;
        zeroProvisional[ i ] = false; // a zero asked for is taken as meant: the probe is away
    }
    startBaselineFor( all );
}

// A zero for the sensors marked in `which`, from the next MAG_BASELINE_FRAMES
// frames; the others keep the baseline they have.
void MagArray::startBaselineFor( const bool* which ) {
    if ( simulatedFrames ) {
        // No frames of the bus to average: the zero is zero, at once.
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            if ( which[ i ] ) {
                baseline[ i ] = zeroed[ i ] = { 0, 0, 0 };
            }
        }
        baselineLeft = 0;
        baselineCount++;
        zeroedAt++;
        return;
    }
    bool every = true;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        zeroing[ i ] = which[ i ];
        every &= which[ i ];
        baselineSum[ i ] = { 0, 0, 0 };
        baselineFrames[ i ] = 0;
    }
    partialZeroing = !every;
    baselineLeft = MAG_BASELINE_FRAMES;
}

void MagArray::restoreBaseline( const Vec3* list, int count, const char* origin ) {
    restoreBaseline( list, nullptr, count, origin );
}

// A saved zero for the sensors it covers; any sensor it does not cover (one
// added since it was saved, or missing from the record) zeroes itself from
// the live frames instead - the one time a zero is taken without being
// asked, so keep the probe away at boot after adding a sensor.
void MagArray::restoreBaseline( const Vec3* list, const bool* have, int count, const char* origin ) {
    if ( simulatedFrames ) {
        // The host simulation: the world's true zero is 0 (useSimulatedFrames).
        bool all[ MAG_SENSOR_COUNT ];
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ )
            all[ i ] = true;
        startBaselineFor( all );
        baselineRestored = true;
        return;
    }
    bool rest[ MAG_SENSOR_COUNT ];
    int restored = 0, missing = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        bool got = i < count && ( have == nullptr || have[ i ] );
        if ( got && list[ i ].x == 0.0f && list[ i ].y == 0.0f && list[ i ].z == 0.0f ) {
            got = false; // exactly nothing is no zero (a record written while the sensor was absent)
        }
        if ( got ) {
            baseline[ i ] = zeroed[ i ] = list[ i ];
            restored++;
            zeroProvisional[ i ] = false;
            zeroKnown[ i ] = true;
        } else {
            zeroProvisional[ i ] = !simulatedFrames; // a live zero nobody asked for: settled by the locator, or by z
            zeroKnown[ i ] = false;
        }
        rest[ i ] = !got;
        missing += got ? 0 : 1;
    }
    baselineLeft = 0; // whatever zeroing was under way is off: this one is used
    baselineCount++;
    zeroedAt++; // it counts as a zero taken (the settings keep serialising it)
    baselineRestored = true;
    if ( missing > 0 && !simulatedFrames ) {
        startBaselineFor( rest );
    }
    Stream* out = console.port( );
    if ( out != nullptr ) {
        char line[ 200 ];
        snprintf( line, sizeof( line ), "baseline: %s put back for %d sensors%s (z zeroes afresh - with the magnet away)", origin, restored,
                  missing > 0 ? "; the others take a provisional zero from live frames now (out of the fit until a good fix settles it)" : "" );
        out->println( line );
    }
}

bool MagArray::usedInFit( int i ) const {
    if ( !fresh[ i ] ) {
        return false;
    }
    if ( trustAllSensors ) {
        return true;
    }
    return magSensorPlaces[ i ].calibrated && zeroKnown[ i ] && !zeroProvisional[ i ];
}

int MagArray::provisionalCount( ) const {
    int n = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        n += zeroProvisional[ i ] ? 1 : 0;
    }
    return n;
}

// The locator, from a good fix of the trusted sensors: the magnet's field
// at this provisional sensor. If the sensor's reading is missing it (it was
// in the zero: the probe lay there at boot), the zero gives it back; if the
// reading shows it, the zero was clean. Either way the zero is settled and
// the sensor joins the fit.
void MagArray::settleProvisionalZero( int i, Vec3 magnetFieldAtSensor, bool takeOutOfZero ) {
    if ( i < 0 || i >= MAG_SENSOR_COUNT || !zeroProvisional[ i ] ) {
        return;
    }
    if ( takeOutOfZero ) {
        shiftBaseline( i, magnetFieldAtSensor );
    }
    zeroProvisional[ i ] = false;
    zeroedAt++; // the settings keep the settled zero
    Stream* out = console.port( );
    if ( out != nullptr ) {
        char line[ 160 ];
        snprintf( line, sizeof( line ), "sensor %d's provisional zero settled: %s (the magnet's %.3f mT there)", i,
                  takeOutOfZero ? "the magnet was in it and is taken out" : "it was clean",
                  sqrtf( magnetFieldAtSensor.x * magnetFieldAtSensor.x + magnetFieldAtSensor.y * magnetFieldAtSensor.y + magnetFieldAtSensor.z * magnetFieldAtSensor.z ) );
        out->println( line );
    }
}

bool MagArray::settleDriftedZeros( float thresholdMt ) {
    if ( baselineLeft > 0 ) {
        return false;
    }
    bool moved = false;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( !fresh[ i ] || !zeroKnown[ i ] || zeroProvisional[ i ] )
            continue;
        float dx = baseline[ i ].x - zeroed[ i ].x, dy = baseline[ i ].y - zeroed[ i ].y, dz = baseline[ i ].z - zeroed[ i ].z;
        if ( dx * dx + dy * dy + dz * dz > thresholdMt * thresholdMt ) {
            zeroed[ i ] = baseline[ i ];
            moved = true;
        }
    }
    if ( moved ) {
        zeroedAt++; // the settings serialise it
    }
    return moved;
}

void MagArray::shiftBaseline( int i, Vec3 by ) {
    if ( i < 0 || i >= MAG_SENSOR_COUNT ) {
        return;
    }
    baseline[ i ] = { baseline[ i ].x - by.x, baseline[ i ].y - by.y, baseline[ i ].z - by.z };
    zeroed[ i ] = baseline[ i ];
}

void MagArray::driftBaseline( float fraction, const bool* only ) {
    if ( baselineLeft > 0 ) {
        return;
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( !fresh[ i ] || ( only != nullptr && !only[ i ] ) )
            continue;
        baseline[ i ].x += fraction * ( raw[ i ].x - baseline[ i ].x );
        baseline[ i ].y += fraction * ( raw[ i ].y - baseline[ i ].y );
        baseline[ i ].z += fraction * ( raw[ i ].z - baseline[ i ].z );
    }
}

// A reading (the sensor's own frame, mT) into the running sums (the frame
// is their average).
void MagArray::takeReading( int i, Vec3 reading, float tempC ) {
    MagSensorState& s = sensors[ i ];
    float limit = MAG_SATURATED * s.rangeMt;
    if ( fabsf( reading.x ) > limit || fabsf( reading.y ) > limit || fabsf( reading.z ) > limit ) {
        sampleSaturated[ i ] = true;
    }
    sampleSum[ i ].x += reading.x;
    sampleSum[ i ].y += reading.y;
    sampleSum[ i ].z += reading.z;
    sampleCount[ i ]++;
    temperatureC[ i ] = tempC;
}

// What the other core has left in shared RAM since the last look: every
// sensor's newest sample, if it is a conversion not yet taken. A TMAG5273's
// own SET_COUNT tells a re-read of the same conversion from a new one; an
// MMC56x3 has no such counter, so the same nine bytes again is the same
// measurement (with 30 counts of noise a sample, a true repeat is rare).
// The sampler reads faster than either converts.
void MagArray::takeSamplerReadings( ) {
    uint32_t wanted = magSampler.resetWanted;
    if ( wanted ) {
        // Every sensor on a bus failed two passes running on the other
        // core: that bus's block is wedged. The sampler waits off that bus;
        // this core resets the block (Wire knows the clock) and lets it go on.
        for ( int b = 0; b < buses; b++ ) {
            if ( wanted & ( 1u << b ) ) {
                resetBus( b );
            }
        }
        magSampler.resetWanted &= ~wanted;
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        MagSensorState& s = sensors[ i ];
        if ( !s.ok ) {
            continue;
        }
        uint8_t raw[ MAGSAMPLER_MAX_BYTES ];
        uint32_t stampUs;
        if ( !magSamplerTake( i, &samplerSeq[ i ], raw, &stampUs ) ) {
            continue;
        }
        Vec3 r;
        float temperature = 0.0f;
        if ( s.type == MAG_MMC56X3 ) {
            size_t n = mmc56x3ReadBytes( );
            bool same = true, allZero = true;
            for ( size_t k = 0; k < n; k++ ) {
                same &= samplerLastRaw[ i ][ k ] == raw[ k ];
                allZero &= raw[ k ] == 0;
                samplerLastRaw[ i ][ k ] = raw[ k ];
            }
            if ( allZero ) {
                // Nine zero bytes is not a field (-3.2 mT on every axis): the
                // part has reset since it was configured - its VDD dropped
                // out, and a part with no VDD still answers, powered through
                // the bus lines - and sits idle with its registers cleared.
                // It is handed to the recovery, which configures it again.
                if ( ++samplerZeroReads[ i ] >= MAG_MMC_ZERO_READS_TO_RESET ) {
                    s.ok = false;
                    s.trouble = "answered with all zeros: reset (its VDD dropped?) - being configured again";
                    nextRecoveryMs = millis( );
                    samplerZeroReads[ i ] = 0;
                }
                continue;
            }
            samplerZeroReads[ i ] = 0;
            if ( same ) {
                samplerDuplicates++;
                continue; // the same measurement read again
            }
            MMC56x3Reading reading;
            mmc56x3Decode( raw, &reading );
            r = { reading.x, reading.y, reading.z };
        } else {
            TMAG5273Reading reading;
            tmag5273Decode( &s.dev, raw, &reading );
            int setCount = TMAG5273_SET_COUNT( reading.status );
            if ( setCount == samplerSetCount[ i ] ) {
                samplerDuplicates++;
                continue; // the same conversion read again
            }
            samplerSetCount[ i ] = setCount;
            r = { reading.x, reading.y, reading.z };
            temperature = reading.temperatureC;
        }
        s.fails = 0;
        takeReading( i, r, temperature );
    }
    // A sensor the sampler cannot read any more is lost: no successful read
    // at all for SAMPLER_LOST_MS while the sampler keeps trying (a glitched
    // read now and then is not that - the bus glitches when the LED strip
    // is busy, a failed read is simply the next one's turn). The first
    // version counted failures between two looks 2.5 ms apart and lost
    // every sensor 42 times in ten minutes, each loss a power-cycle and a
    // re-addressing, until the sensors were stranded at factory addresses.
    uint32_t now = millis( );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        MagSensorState& s = sensors[ i ];
        if ( !s.ok ) {
            continue;
        }
        uint32_t reads = magSampler.reads[ i ];
        if ( reads != samplerReadsSeen[ i ] ) {
            samplerReadsSeen[ i ] = reads;
            samplerLastReadMs[ i ] = now;
        } else if ( magSampler.fails[ i ] != samplerFailsSeen[ i ] && now - samplerLastReadMs[ i ] > SAMPLER_LOST_MS ) {
            s.ok = false; // stays powered; recoverLostSensors() looks for it
            nextRecoveryMs = now + RECOVERY_PERIOD_MS;
        }
        samplerFailsSeen[ i ] = magSampler.fails[ i ];
    }
}

// One reading of sensor i on this core, through its driver (the fallback
// when the sampler is not running, and the bench tools).
bool MagArray::readSensorHere( int i, Vec3* reading, float* temperatureC ) {
    MagSensorState& s = sensors[ i ];
    *temperatureC = 0.0f;
    if ( s.type == MAG_MMC56X3 ) {
        MMC56x3Reading m;
        if ( !mmc56x3Read( wireOf( i ), &s.mmc, &m ) ) {
            return false;
        }
        *reading = { m.x, m.y, m.z };
        return true;
    }
    TMAG5273Reading t;
    if ( !tmag5273Read( wireOf( i ), &s.dev, &t ) ) {
        return false;
    }
    *reading = { t.x, t.y, t.z };
    *temperatureC = t.temperatureC;
    return true;
}

// Read every sensor once and add the readings to the running sums.
void MagArray::sampleSensors( uint32_t now ) {
    if ( samplerOn ) {
        takeSamplerReadings( );
        return;
    }
    int tried = 0, failed = 0;
    for ( int b = 0; b < buses; b++ ) {
        if ( busWedged( b ) ) {
            resetBus( b );
            busWedges++;
        }
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        MagSensorState& s = sensors[ i ];
        if ( !s.ok ) {
            continue;
        }
        tried++;
        Vec3 reading;
        float temperature;
        uint32_t t0 = micros( );
        if ( !readSensorHere( i, &reading, &temperature ) ) {
            failed++;
            if ( micros( ) - t0 > BUS_STUCK_US ) {
                // No START, or no ACK, within Wire's 10 ms: the I2C block is
                // wedged (or a device holds a line), not this sensor - a
                // sensor that is merely absent answers with a NACK at once.
                // Reset the block now, not after the other sensors have each
                // burnt the same 10 ms (80 ms for eight: a hole in every
                // stream downstream of this one). First a note of what the
                // block and the lines look like, for `m`.
#if MAG_HAVE_I2C_REGS
                I2C_TypeDef* dev = ch32h4_i2c_regs( busTable[ s.bus ].peripheral );
                if ( dev != nullptr ) {
                    stuckStar1 = dev->STAR1;
                    stuckStar2 = dev->STAR2;
                    stuckCtlr1 = dev->CTLR1;
                }
                stuckScl = digitalRead( busTable[ s.bus ].sclPin );
                stuckSda = digitalRead( busTable[ s.bus ].sdaPin );
#endif
                resetBus( s.bus );
                for ( int k = 0; k < MAG_SENSOR_COUNT; k++ ) {
                    sensors[ k ].fails = 0;
                }
                return;
            }
            if ( ++s.fails >= MAG_MAX_READ_FAILS ) {
                s.ok = false; // stays powered; recoverLostSensors() looks for it
                nextRecoveryMs = now + RECOVERY_PERIOD_MS;
            }
            continue;
        }
        s.fails = 0;
        takeReading( i, reading, temperature );
    }

    // Every sensor failing at once (with NACKs, or the timeout above would
    // have caught it) is the buses, not the sensors: they are still powered
    // and still at their addresses, so reset the buses and carry on.
    if ( tried > 1 && failed == tried ) {
        resetAllBuses( );
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            sensors[ i ].fails = 0;
        }
    }
}

// Turn the sums into one frame: the average reading of each sensor, in the
// board frame, less the baseline.
void MagArray::publishFrame( ) {
    int samples = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        saturated[ i ] = sampleSaturated[ i ];
        fresh[ i ] = sampleCount[ i ] > 0 && !saturated[ i ];
        if ( sampleCount[ i ] > 0 ) {
            float k = magSensorPlaces[ i ].gain / sampleCount[ i ];
            raw[ i ] = magSensorToBoard( { sampleSum[ i ].x * k, sampleSum[ i ].y * k, sampleSum[ i ].z * k }, magSensorPlaces[ i ].rotationDeg,
                                         magSensorPlaces[ i ].underside, magSensorTypeZIntoTop( sensors[ i ].type ) );
            if ( baselineLeft > 0 && zeroing[ i ] ) {
                baselineSum[ i ].x += raw[ i ].x;
                baselineSum[ i ].y += raw[ i ].y;
                baselineSum[ i ].z += raw[ i ].z;
                baselineFrames[ i ]++;
            }
            field[ i ] = { raw[ i ].x - baseline[ i ].x, raw[ i ].y - baseline[ i ].y, raw[ i ].z - baseline[ i ].z };
            if ( sampleCount[ i ] > samples ) {
                samples = sampleCount[ i ];
            }
        }
        sampleSum[ i ] = { 0, 0, 0 };
        sampleCount[ i ] = 0;
        sampleSaturated[ i ] = false;
    }
    samplesPerFrame += 0.05f * ( samples - samplesPerFrame );

    if ( baselineLeft > 0 && --baselineLeft == 0 ) {
        bool again[ MAG_SENSOR_COUNT ];
        int unread = 0;
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            again[ i ] = false;
            if ( !zeroing[ i ] ) {
                continue; // keeps the baseline it had (a saved one)
            }
            // Each sensor's own count: one that missed frames must not get a
            // scaled-down baseline (a phantom field from then on); one that
            // was not read at all keeps zeroing until it is (or stays
            // without a zero: nothing is saved for it).
            int n = baselineFrames[ i ];
            if ( n > 0 ) {
                baseline[ i ] = { baselineSum[ i ].x / n, baselineSum[ i ].y / n, baselineSum[ i ].z / n };
                zeroed[ i ] = baseline[ i ];
                zeroKnown[ i ] = true;
            } else if ( sensors[ i ].ok ) {
                again[ i ] = true; // answering but not read this time round: once more
                unread++;
            }
            // (an absent sensor is left without a zero; it gets one when it turns up: recoverLostSensors)
        }
        baselineCount++;
        zeroedAt++;
        partialZeroing = false;
        if ( unread > 0 ) {
            startBaselineFor( again );
        }
    }
    frameCount++;
}

void MagArray::powerOff( ) {
    if ( simulatedFrames )
        return;
    pauseSampler( );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        setVcc( i, LOW ); // (a fixed-address part stays powered: it is simply not read)
        sensors[ i ].ok = false;
    }
    poweredOff = true;
}

void MagArray::powerOnly( uint32_t mask ) {
    if ( simulatedFrames )
        return;
    pauseSampler( );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        disabled[ i ] = !( ( mask >> i ) & 1u );
        if ( disabled[ i ] ) {
            setVcc( i, LOW );
            sensors[ i ].ok = false;
        }
    }
    resumeSampler( );
}

void MagArray::powerOffFrom( int n ) {
    powerOnly( n >= 32 ? 0xffffffffu : ( 1u << n ) - 1u );
}

void MagArray::powerOn( ) {
    bool anyOff = poweredOff;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        anyOff = anyOff || disabled[ i ];
        disabled[ i ] = false;
    }
    if ( !anyOff )
        return;
    poweredOff = false;
    pauseSampler( ); // the walk needs the bus to itself (as p does); begin() starts the sampler again
    begin( false );
}

void MagArray::holdSampler( bool hold ) {
    samplerHeld = hold;
    if ( hold ) {
        pauseSampler( );
    } else {
        resumeSampler( );
    }
}

void MagArray::setLowNoise( bool on ) {
    if ( simulatedFrames )
        return;
    lowNoise = on;
    pauseSampler( );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( sensors[ i ].ok && sensors[ i ].type == MAG_TMAG5273 ) {
            tmag5273SetLowNoise( wireOf( i ), sensors[ i ].dev.address, on );
        }
    }
    resumeSampler( );
}

int MagArray::enabledCount( ) const {
    int n = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        n += disabled[ i ] ? 0 : 1;
    }
    return n;
}

void MagArray::useSimulatedFrames( ) {
    simulatedFrames = true;
    trustAllSensors = true;
    samplerOn = false;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        zeroKnown[ i ] = true;
    }
    setWeightCap( weightCap );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        sensors[ i ].type = magSensorPlaces[ i ].type;
        sensors[ i ].rangeMt = magSensorPlaces[ i ].type == MAG_MMC56X3 ? MMC56X3_RANGE_MT : 40.0f;
        sensors[ i ].ok = true;
        sensors[ i ].trouble = nullptr;
        fresh[ i ] = true;
    }
    baselineLeft = 0;
    // The world's readings carry no offset and no Earth field: the true zero
    // is 0, whatever setup() restored (the compiled-in bench zero) before
    // the frames were handed over.
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        baseline[ i ] = zeroed[ i ] = { 0, 0, 0 };
        zeroProvisional[ i ] = false;
    }
    if ( baselineCount == 0 ) {
        baselineCount = 1;
    }
}

ServiceStatus MagArray::service( ) {
    uint32_t now = millis( );

    if ( simulatedFrames ) {
        lastStatus = ServiceStatus::BUSY; // the world publishes the frames
        return lastStatus;
    }

    if ( magSamplerParked( ) || poweredOff ) {
        return ServiceStatus::IDLE; // F: the other core is parked for a flash; or :load sensors off. Nothing to read, nothing to recover
    }
    if ( sensorsOk( ) < enabledCount( ) && now >= nextRecoveryMs ) {
        nextRecoveryMs = now + ( watchMask != 0 ? RECOVERY_PERIOD_MS / 4 : RECOVERY_PERIOD_MS );
        recoverLostSensors( );
    }

    sampleSensors( now );

    uint32_t nowUs = micros( );
    if ( nowUs - lastFrameUs >= MAG_FRAME_PERIOD_US ) {
        // The next frame is due a period after this one was, not after this
        // one was published: stamping "from now" made the frames 12-14 ms
        // apart (the service's own latency on top of every period) - 60-80
        // frames a second for a 100 Hz frame period. A loop that has fallen
        // more than two periods behind starts afresh rather than catching up.
        lastFrameUs = nowUs - lastFrameUs >= 2 * MAG_FRAME_PERIOD_US ? nowUs : lastFrameUs + MAG_FRAME_PERIOD_US;
        publishFrame( );

        Stream* out = console.port( );
        if ( out != nullptr ) {
            if ( streaming && frameCount % STREAM_EVERY_N_FRAMES == 0 ) {
                printFrameCsv( out );
            }
            if ( identifying && now - lastIdentifyMs >= IDENTIFY_PERIOD_MS ) {
                lastIdentifyMs = now;
                printStrongest( out );
            }
        }
    }

    lastStatus = sensorsOk( ) > 0 ? ServiceStatus::BUSY : ServiceStatus::ERROR;
    return lastStatus;
}

// ---- printing --------------------------------------------------------------

void MagArray::printFrameCsv( Stream* out ) const {
    out->print( "mag," );
    out->print( millis( ) );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        // Three decimals (a microtesla) is below a TMAG's noise; an MMC's
        // 0.2 uT needs five (2026-09-21: at three its stream read as frozen).
        int decimals = sensors[ i ].type == MAG_MMC56X3 ? 5 : 3;
        out->print( ',' );
        out->print( field[ i ].x, decimals );
        out->print( ',' );
        out->print( field[ i ].y, decimals );
        out->print( ',' );
        out->print( field[ i ].z, decimals );
    }
    out->println( );
}

void MagArray::printStrongest( Stream* out ) {
    int best = -1;
    float bestSq = 0.0f;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        float sq = field[ i ].x * field[ i ].x + field[ i ].y * field[ i ].y + field[ i ].z * field[ i ].z;
        if ( sensors[ i ].ok && sq > bestSq ) {
            bestSq = sq;
            best = i;
        }
    }
    if ( best < 0 ) {
        return;
    }
    char line[ 96 ];
    snprintf( line, sizeof( line ), "strongest: sensor %d  |B| %.2f mT  (x %+.2f  y %+.2f  z %+.2f)",
              best, sqrtf( bestSq ), field[ best ].x, field[ best ].y, field[ best ].z );
    out->println( line );
}

void MagArray::printStatus( Stream* out ) const {
    char line[ 240 ];
    snprintf( line, sizeof( line ), "MagArray: %d of %d sensors on %d bus%s, frame %lu (%.1f reads averaged per frame), baseline %s, VIO rail %s",
              sensorsOk( ), MAG_SENSOR_COUNT, buses, buses == 1 ? "" : "es", (unsigned long)frameCount, samplesPerFrame, baselineReady( ) ? "set" : "averaging",
              boardVioIs3V3( ) ? "3.3 V" : "NOT 3.3 V - sensors cannot run" );
    out->println( line );
    for ( int b = 0; b < buses; b++ ) {
        const MagBus& bus = busTable[ b ];
        if ( bus.peripheral == 0 ) {
            snprintf( line, sizeof( line ), "bus %d: pins %d/%d are not a pair this chip can use for I2C (or not wired)", b, bus.sclPin, bus.sdaPin );
        } else {
            snprintf( line, sizeof( line ), "bus %d: I2C%u at %u kHz, re-initialised %lu times", b, bus.peripheral, (unsigned)( MAG_I2C_HZ / 1000 ), (unsigned long)bus.resets );
        }
        out->println( line );
    }
    if ( samplerOn ) {
        uint32_t passes = magSampler.passes;
        uint32_t dtMs = millis( ) - samplerPassesSeenMs;
        float perS = dtMs > 0 ? ( passes - samplerPassesSeen ) * 1000.0f / dtMs : 0.0f;
        samplerPassesSeen = passes;
        samplerPassesSeenMs = millis( );
        uint32_t enabledBits = 0;
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            enabledBits |= magSampler.enabled[ i ] ? ( 1u << i ) : 0;
        }
        snprintf( line, sizeof( line ), "sampler on the V3F: command %lu state %lu count %lu enabled 0x%03lx, %lu passes (%.0f a second since the last look); reads/fails per sensor",
                  (unsigned long)magSampler.command, (unsigned long)magSampler.state, (unsigned long)magSampler.count, (unsigned long)enabledBits, (unsigned long)passes, perS );
        out->print( line );
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            snprintf( line, sizeof( line ), " %lu/%lu", (unsigned long)magSampler.reads[ i ], (unsigned long)magSampler.fails[ i ] );
            out->print( line );
        }
        snprintf( line, sizeof( line ), "; %lu re-reads of one conversion skipped; the sampler asked for the blocks reset %lu times", (unsigned long)samplerDuplicates,
                  (unsigned long)magSampler.resets );
        out->println( line );
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            if ( magSampler.fails[ i ] > 0 ) {
                snprintf( line, sizeof( line ), "  sensor %d's last failed read broke at: %s", i, magI2cFailureName( magSampler.lastFail[ i ] ) );
                out->println( line );
            }
        }
        // How long one read takes from this core, the sampler held off: the
        // bus clock as it really is (a 7-byte read is ~200 us at 400 kHz).
        if ( sensors[ 0 ].ok && sensors[ 0 ].type == MAG_TMAG5273 ) {
            MagArray* self = const_cast<MagArray*>( this );
            self->pauseSampler( );
            uint32_t t0 = micros( );
            int good = 0;
            for ( int n = 0; n < 10; n++ ) {
                uint8_t raw[ 10 ];
                good += tmag5273BurstRead( busTable[ sensors[ 0 ].bus ].peripheral, sensors[ 0 ].dev.address, raw, tmag5273ReadBytes( &sensors[ 0 ].dev ) ) ? 1 : 0;
            }
            uint32_t took = micros( ) - t0;
            self->resumeSampler( );
            snprintf( line, sizeof( line ), "one read of sensor 0 from this core: %lu us (%d of 10 good; 200 us is 400 kHz)", (unsigned long)( took / 10 ), good );
            out->println( line );
        }
    } else {
        out->println( "sampler: NOT running on the V3F - the buses are read on this core" );
    }
    if ( recoveryRuns > 0 || busResets > 0 || stopClears > 0 ) {
        snprintf( line, sizeof( line ), "I2C blocks re-initialised %lu times, %lu of them caught as a wedge before a read (last %.1f s ago, %.1f s after the one before); a pending STOP cleared %lu times; recovery has run %lu times, last: %s",
                  (unsigned long)busResets, (unsigned long)busWedges, ( millis( ) - lastBusResetMs ) * 1e-3f, ( lastBusResetMs - previousBusResetMs ) * 1e-3f,
                  (unsigned long)stopClears, (unsigned long)recoveryRuns, recoveryNote );
        out->println( line );
        if ( stuckStar1 != 0 || stuckStar2 != 0 || stuckCtlr1 != 0 ) {
            snprintf( line, sizeof( line ), "at the last timed-out read: STAR1 0x%04x STAR2 0x%04x CTLR1 0x%04x, SCL %d SDA %d", stuckStar1, stuckStar2, stuckCtlr1, stuckScl, stuckSda );
            out->println( line );
        }
    }
    snprintf( line, sizeof( line ), "fit weights (wt): a TMAG5273 is 1, a quieter type up to the cap %.1f where it reads noise, 1 by a strong reading (MAG_MODEL_ERROR; w changes the cap); fit column: y = in the fit, uncal = not calibrated (MagArrayConfig.h), prov = zero provisional", weightCap );
    out->println( line );
    out->println( " #  bus  addr  part          range   x_mm  y_mm   wt  fit    fails  recov      Bx      By      Bz  (mT)   degC" );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        const MagSensorState& s = sensors[ i ];
        if ( !s.ok ) {
            snprintf( line, sizeof( line ), "%2d  %3d  ----  no answer            %6.1f %5.1f  %3.1f  %s (register 0x%02X, Wire code %u)", i, s.bus, position[ i ].x, position[ i ].y,
                      weight[ i ], s.trouble ? s.trouble : "stopped answering", s.troubleRegister, s.troubleCode );
            out->println( line );
            continue;
        }
        char part[ 16 ];
        if ( s.type == MAG_MMC56X3 ) {
            snprintf( part, sizeof( part ), "MMC56x3 id%02X", s.mmc.productId );
        } else {
            char variant = '?';
            switch ( s.factoryAddress ) {
            case 0x35:
                variant = 'A';
                break;
            case 0x22:
                variant = 'B';
                break;
            case 0x78:
                variant = 'C';
                break;
            case 0x44:
                variant = 'D';
                break;
            }
            snprintf( part, sizeof( part ), "TMAG5273%c%d", variant, s.dev.version );
        }
        const char* fitUse = !magSensorPlaces[ i ].calibrated ? "uncal" : ( !zeroKnown[ i ] ? "nozero" : ( zeroProvisional[ i ] ? "prov" : "y" ) );
        snprintf( line, sizeof( line ), "%2d  %3d  0x%02X  %-12s  %5.1f  %6.1f %5.1f  %3.1f  %-5s  %5u  %5lu  %+6.3f  %+6.3f  %+6.3f        %5.1f",
                  i, s.bus, s.address, part, s.rangeMt, position[ i ].x, position[ i ].y, weight[ i ], fitUse,
                  s.fails, (unsigned long)s.recoveries, field[ i ].x, field[ i ].y, field[ i ].z, temperatureC[ i ] );
        out->print( line );
        out->println( saturated[ i ] ? "  SATURATED" : "" );
    }
}

// ---- bus check --------------------------------------------------------------

// What one bus line looks like from the pin. Three reads: against the chip's
// own weak pull-down (only a real pull-up resistor wins that), and with the
// chip's weak pull-up (only something actively holding the line loses that).
// Then how long the line takes to rise after being driven low and let go,
// which is pull-up strength x bus capacitance: about a microsecond with
// 2.2k-4.7k, tens of microseconds with a resistor that is too weak.
enum LineState {
    LINE_OK,         // a real pull-up is there
    LINE_NO_PULL_UP, // floating: resistor missing, on the wrong pin, or a loose contact
    LINE_HELD_LOW    // something on the bus is holding it down
};

static LineState checkLine( int pin, uint32_t* riseUs ) {
    pinMode( pin, INPUT_PULLDOWN );
    delay( 2 );
    bool beatsPullDown = digitalRead( pin ) == HIGH;
    pinMode( pin, INPUT_PULLUP );
    delay( 2 );
    bool risesWithPullUp = digitalRead( pin ) == HIGH;

    pinMode( pin, OUTPUT );
    digitalWrite( pin, LOW );
    delay( 2 );
    uint32_t start = micros( );
    pinMode( pin, INPUT );
    *riseUs = 0;
    while ( digitalRead( pin ) != HIGH && ( *riseUs = micros( ) - start ) < 20000 ) {
    }

    if ( beatsPullDown ) {
        return LINE_OK;
    }
    return risesWithPullUp ? LINE_NO_PULL_UP : LINE_HELD_LOW;
}

static void printLine( Stream* out, const char* name, LineState state, uint32_t riseUs ) {
    char line[ 140 ];
    switch ( state ) {
    case LINE_OK:
        snprintf( line, sizeof( line ), "  %s ok: pulled up, rises in %lu us after release", name, (unsigned long)riseUs );
        break;
    case LINE_NO_PULL_UP:
        snprintf( line, sizeof( line ), "  %s NOT PULLED UP: the line is floating. Resistor missing, on the wrong pin, or a loose contact.", name );
        break;
    default:
        snprintf( line, sizeof( line ), "  %s HELD LOW: something on the bus is holding it down (a stuck sensor, or a short).", name );
        break;
    }
    out->println( line );
}

void MagArray::printBusCheck( Stream* out ) {
    char line[ 120 ];

    // Everything switched off, the buses released, then look at the bare
    // lines of each bus.
    for ( int b = 0; b < buses; b++ ) {
        busTable[ b ].wire->end( );
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        sensors[ i ].ok = false;
        setVcc( i, LOW );
    }
    delay( MAG_POWER_OFF_MS );
    uint32_t sclRise, sdaRise;
    LineState sclOff = LINE_OK, sdaOff = LINE_OK;
    for ( int b = 0; b < buses; b++ ) {
        const MagBus& bus = busTable[ b ];
        if ( bus.sclPin < 0 || bus.sdaPin < 0 ) {
            snprintf( line, sizeof( line ), "bus %d: not wired (-1 in BoardPins.h)", b );
            out->println( line );
            continue;
        }
        LineState scl = checkLine( bus.sclPin, &sclRise );
        LineState sda = checkLine( bus.sdaPin, &sdaRise );
        snprintf( line, sizeof( line ), "bus %d lines with every switched sensor's VCC driven low%s:", b, b == 0 ? "" : " (its fixed-address part stays powered)" );
        out->println( line );
        printLine( out, "SCL", scl, sclRise );
        printLine( out, "SDA", sda, sdaRise );
        if ( b == 0 ) {
            sclOff = scl;
            sdaOff = sda;
        }
    }

    // One switched sensor at a time, then all, on the first bus's lines.
    // Only differences are printed: a line that is fine until one
    // particular sensor is powered points at that sensor or its wiring.
    const MagBus& first = busTable[ 0 ];
    bool changed = false;
    for ( int i = 0; i <= MAG_SENSOR_COUNT; i++ ) {
        bool all = ( i == MAG_SENSOR_COUNT );
        if ( !all && magSensorPlaces[ i ].vccPin < 0 ) {
            continue;
        }
        for ( int k = 0; k < MAG_SENSOR_COUNT; k++ ) {
            setVcc( k, ( all || k == i ) ? HIGH : LOW );
        }
        delay( MAG_POWER_ON_MS );
        LineState scl = checkLine( first.sclPin, &sclRise );
        LineState sda = checkLine( first.sdaPin, &sdaRise );
        if ( scl != sclOff || sda != sdaOff ) {
            changed = true;
            if ( all ) {
                out->println( "with every sensor powered:" );
            } else {
                snprintf( line, sizeof( line ), "with only sensor %d powered:", i );
                out->println( line );
            }
            printLine( out, "SCL", scl, sclRise );
            printLine( out, "SDA", sda, sdaRise );
        }
    }
    if ( !changed ) {
        out->println( "  (the same with any one sensor powered, and with all of them)" );
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        setVcc( i, LOW );
    }
    if ( sclOff != LINE_OK || sdaOff != LINE_OK ) {
        out->println( "I2C cannot work until both lines read ok. This chip has no internal pull-ups for I2C:" );
        out->println( "2.2k-4.7k from SCL (PA8) to 3.3 V and from SDA (PC9) to 3.3 V (and the same on the second bus, PF12/PF13). Wiggle, fix, run b again." );
    }
    delay( MAG_POWER_OFF_MS );

    for ( int b = 0; b < buses; b++ ) {
        MagBus& bus = busTable[ b ];
        if ( bus.sclPin < 0 || bus.sdaPin < 0 ) {
            continue;
        }
        bus.wire->setSCL( bus.sclPin );
        bus.wire->setSDA( bus.sdaPin );
        bus.wire->begin( );
        bus.wire->setClock( 100000 ); // slow, to take bus speed out of the question
    }

    // Each switched sensor powered alone: which addresses acknowledge on
    // its bus? (0x30 on a bus with an MMC56x3 is that part, always there.)
    // endTransmission(): 0 = ACK, 2 = nothing at that address, 4 = bus fault.
    int silent = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( magSensorPlaces[ i ].vccPin < 0 ) {
            continue;
        }
        setVcc( i, HIGH );
        delay( MAG_POWER_ON_MS );

        snprintf( line, sizeof( line ), "sensor %d powered (bus %d):", i, sensors[ i ].bus );
        out->print( line );
        int found = 0;
        TwoWire* wire = wireOf( i );
        for ( uint8_t address = 0x08; address < 0x78; address++ ) {
            wire->beginTransmission( address );
            uint8_t result = wire->endTransmission( );
            if ( result == 0 ) {
                snprintf( line, sizeof( line ), " 0x%02X", address );
                out->print( line );
                found++;
            } else if ( result != 2 ) {
                snprintf( line, sizeof( line ), " bus fault (code %u) - giving up on the scan", result );
                out->print( line );
                found = -1;
                break;
            }
        }
        if ( found == 0 ) {
            out->print( " nothing acknowledges" );
            silent++;
        }
        out->println( );
        setVcc( i, LOW );
        if ( found < 0 ) {
            break;
        }
        delay( MAG_POWER_OFF_MS );
    }

    // The fixed-address parts: there, and what do they call themselves?
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( magSensorPlaces[ i ].type != MAG_MMC56X3 ) {
            continue;
        }
        const MagBus& bus = busTable[ sensors[ i ].bus ];
        if ( bus.sclPin < 0 || bus.sdaPin < 0 ) {
            snprintf( line, sizeof( line ), "sensor %d (MMC56x3, bus %d): its bus is not wired", i, sensors[ i ].bus );
        } else {
            uint8_t id = 0;
            bool there = mmc56x3Present( bus.wire, magSensorPlaces[ i ].address, &id );
            snprintf( line, sizeof( line ), "sensor %d (MMC56x3 at 0x%02X on bus %d): %s%s", i, magSensorPlaces[ i ].address, sensors[ i ].bus,
                      there ? "answers, product ID 0x" : "nothing acknowledges", there ? "" : "" );
            out->print( line );
            if ( there ) {
                snprintf( line, sizeof( line ), "%02X%s", id, id == MMC56X3_PRODUCT_ID ? " (the MMC5603NJ's)" : " (not the 0x10 of the MMC5603NJ - an MMC5633NJL may differ; the readings tell)" );
                out->print( line );
            }
            line[ 0 ] = '\0';
        }
        out->println( line );
    }

    // Sensors that stayed silent: is one of them wired to a neighbouring pin?
    if ( silent > 0 ) {
        static const int candidates[] = PIN_MAG_VCC_CANDIDATES;
        static const char* const candidateNames[] = PIN_MAG_VCC_CANDIDATE_NAMES;
        out->println( "looking for the silent sensors on the spare header pins:" );
        for ( unsigned c = 0; c < sizeof( candidates ) / sizeof( candidates[ 0 ] ); c++ ) {
            pinMode( candidates[ c ], OUTPUT );
            digitalWrite( candidates[ c ], HIGH );
            delay( MAG_POWER_ON_MS );
            bool answered = false;
            for ( unsigned a = 0; a < sizeof( tmag5273FactoryAddresses ); a++ ) {
                answered |= tmag5273Present( first.wire, tmag5273FactoryAddresses[ a ] );
            }
            digitalWrite( candidates[ c ], LOW );
            delay( 20 );
            pinMode( candidates[ c ], INPUT );
            snprintf( line, sizeof( line ), "  %s: %s", candidateNames[ c ], answered ? "A SENSOR ANSWERS - this pin powers one" : "nothing" );
            out->println( line );
        }
    }

    out->println( "re-addressing the array..." );
    begin( false ); // the zero in use is kept: a bus check is not a reason to zero (the probe may be on the board)
    printStatus( out );
}
