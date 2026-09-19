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
#include "MagSampler.h"

#define MAG_BUS Wire

#define STREAM_EVERY_N_FRAMES 5 // CSV at 20 Hz; 100 Hz of text outruns 115200 baud
#define IDENTIFY_PERIOD_MS 250
#define RECOVERY_PERIOD_MS 2000
#define BUS_STUCK_US 5000   // a read that takes this long failed by Wire's 10 ms timeout, not by a NACK
#define SAMPLER_LOST_MS 100 // no successful read by the other core's sampler for this long = the sensor is lost

MagArray& magArray = MagArray::getInstance( );

MagArray& MagArray::getInstance( ) {
    static MagArray instance;
    return instance;
}

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

// ---- power-up addressing ---------------------------------------------------

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
    }

    // Grounds first, then every supply driven low, and long enough for the
    // bypass capacitors to empty so each sensor really does power-on-reset.
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        pinMode( magSensorPlaces[ i ].gndPin, OUTPUT );
        digitalWrite( magSensorPlaces[ i ].gndPin, LOW );
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        pinMode( magSensorPlaces[ i ].vccPin, OUTPUT );
        digitalWrite( magSensorPlaces[ i ].vccPin, LOW );
        sensors[ i ] = MagSensorState( );
        position[ i ] = { magSensorPlaces[ i ].x, magSensorPlaces[ i ].y, magSensorPlaces[ i ].z };
        field[ i ] = raw[ i ] = baseline[ i ] = { 0, 0, 0 };
        fresh[ i ] = false;
        sampleSum[ i ] = { 0, 0, 0 };
        sampleCount[ i ] = 0;
        sampleSaturated[ i ] = false;
    }
    delay( MAG_POWER_OFF_MS );

    MAG_BUS.end( );
    MAG_BUS.setSCL( PIN_MAG_SCL );
    MAG_BUS.setSDA( PIN_MAG_SDA );
    MAG_BUS.begin( );
    MAG_BUS.setClock( MAG_I2C_HZ );
    busPeripheral = MAG_BUS.peripheral( );

    bool all[ MAG_SENSOR_COUNT ];
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        all[ i ] = true;
    }
    addressSensors( all );

    if ( zero || !baselineReady( ) ) {
        startBaseline( ); // (a power-cycle at run time keeps the zero in use: the probe may be on the board)
    }
    startSampler( );
    return sensorsOk( );
}

// The other core takes over the bus: told which sensors to read, and run.
// samplerOn says whether it obeyed (loop1() is running there); if not, the
// bus is read from here as before.
void MagArray::startSampler( ) {
#if MAG_SAMPLER_CORE1
    magSamplerSetup( busPeripheral, MAG_SENSOR_COUNT );
    magSamplerSetBusHz( MAG_I2C_HZ );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        magSamplerSetSensor( i, sensors[ i ].dev.address, (int)tmag5273ReadBytes( &sensors[ i ].dev ), sensors[ i ].ok );
        samplerSeq[ i ] = 0;
        samplerSetCount[ i ] = -1;
        samplerFailsSeen[ i ] = 0;
        samplerReadsSeen[ i ] = 0;
        samplerLastReadMs[ i ] = millis( );
    }
    samplerPassesSeen = 0;
    samplerPassesSeenMs = millis( );
    samplerOn = busPeripheral != 0 && magSamplerCommand( MAGSAMPLER_RUN );
#else
    samplerOn = false;
#endif
}

// Before this core uses the bus itself (recovery, a power-cycle, the bus
// check): the sampler stops between two reads and says so.
void MagArray::pauseSampler( ) {
    if ( samplerOn ) {
        magSamplerCommand( MAGSAMPLER_PAUSE );
    }
}

void MagArray::resumeSampler( ) {
    if ( samplerOn ) {
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            magSamplerSetSensor( i, sensors[ i ].dev.address, (int)tmag5273ReadBytes( &sensors[ i ].dev ), sensors[ i ].ok );
        }
        magSamplerCommand( MAGSAMPLER_RUN );
    }
}

// Nothing on the bus can be talked to (a line with no pull-up, or held down).
// Every transaction then burns its full timeout, so callers check this first.
static bool busFault( ) {
    MAG_BUS.beginTransmission( tmag5273FactoryAddresses[ 0 ] );
    uint8_t result = MAG_BUS.endTransmission( );
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
bool MagArray::busWedged( ) {
#if MAG_HAVE_I2C_REGS
    if ( busPeripheral == 0 ) {
        return false;
    }
    I2C_TypeDef* dev = ch32h4_i2c_regs( busPeripheral );
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
#endif
    return false;
}

// Re-initialise the I2C block: begin() resets the peripheral and, if a device
// is holding SDA, clocks it out.
void MagArray::resetBus( ) {
    MAG_BUS.end( );
    MAG_BUS.begin( );
    MAG_BUS.setClock( MAG_I2C_HZ );
    busResets++;
    previousBusResetMs = lastBusResetMs;
    lastBusResetMs = millis( );
}

// The addressing walk, over the sensors marked in `which`. This is the ONLY
// place sensor power is switched: those sensors go off together, come up one
// at a time to be moved off the factory address, and then EVERY sensor is
// powered and stays powered - the ones that did not answer included. Supplies
// are never touched again unless a sensor turns up back at a factory address
// (see recoverLostSensors).
void MagArray::addressSensors( const bool* which ) {
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( which[ i ] ) {
            sensors[ i ].ok = false;
            digitalWrite( magSensorPlaces[ i ].vccPin, LOW );
        }
    }
    delay( MAG_POWER_OFF_MS );

    if ( busFault( ) ) {
        resetBus( );
    }
    if ( !busFault( ) ) {
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            if ( !which[ i ] ) {
                continue;
            }
            for ( int attempt = 0; attempt < MAG_ADDRESS_TRIES && !powerUpSensor( i ); attempt++ ) {
                // It failed and is powered down again. A failed transaction can
                // leave the I2C block wedged, so start the next try clean.
                resetBus( );
                delay( MAG_POWER_OFF_MS );
            }
        }
    }

    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        digitalWrite( magSensorPlaces[ i ].vccPin, HIGH );
    }
}

// One step of the walk: VCC up, find the sensor at its factory address, move
// it to its own address, configure it. On failure VCC goes back low FOR THE
// REST OF THE WALK ONLY - a sensor left powered at a factory address would
// answer for the next one in line. addressSensors() powers it again at the end.
bool MagArray::powerUpSensor( int i ) {
    MagSensorState& s = sensors[ i ];
    s.ok = false;
    s.fails = 0;

    digitalWrite( magSensorPlaces[ i ].vccPin, HIGH );
    delay( MAG_POWER_ON_MS );

    uint8_t assigned = MAG_BASE_ADDRESS + i;
    s.trouble = "nothing at any factory address";
    for ( unsigned a = 0; a < sizeof( tmag5273FactoryAddresses ); a++ ) {
        uint8_t factory = tmag5273FactoryAddresses[ a ];
        if ( !tmag5273Present( &MAG_BUS, factory ) ) {
            continue;
        }
        s.factoryAddress = factory;
        if ( !tmag5273SetAddress( &MAG_BUS, factory, assigned ) ) {
            s.trouble = "found, but it did not move to its new address";
        } else if ( !tmag5273Begin( &MAG_BUS, &s.dev, assigned, MAG_AVERAGING, MAG_HIGH_RANGE, MAG_READ_TEMPERATURE ) ) {
            s.trouble = "moved to its new address, but configuring it failed";
        } else {
            s.trouble = nullptr;
            s.ok = true;
            return true;
        }
        break;
    }

    s.troubleRegister = tmag5273LastFailedRegister;
    s.troubleCode = tmag5273LastFailedCode;
    digitalWrite( magSensorPlaces[ i ].vccPin, LOW );
    return false;
}

// Sensors that stopped answering, without disturbing anyone's power if that
// can be avoided:
//   1. bus dead -> re-initialise the I2C block and try again later;
//   2. still at its own address (it was the bus that hiccupped, the sensor
//      never reset) -> configure it again and carry on;
//   3. something answers at a factory address -> a sensor really did reset, so
//      walk the lost ones (and only them) through addressing again;
//   4. otherwise nobody is home: leave them powered and look again later.
void MagArray::recoverLostSensors( ) {
    recoveryRuns++;
    pauseSampler( );
    if ( busFault( ) ) {
        resetBus( );
        recoveryNote = "bus fault: re-initialised the I2C block";
        resumeSampler( );
        return;
    }

    bool lost[ MAG_SENSOR_COUNT ];
    int lostCount = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        MagSensorState& s = sensors[ i ];
        lost[ i ] = false;
        if ( s.ok ) {
            continue;
        }
        uint8_t assigned = MAG_BASE_ADDRESS + i;
        if ( tmag5273Acknowledges( &MAG_BUS, assigned ) &&
             tmag5273Begin( &MAG_BUS, &s.dev, assigned, MAG_AVERAGING, MAG_HIGH_RANGE, MAG_READ_TEMPERATURE ) ) {
            s.ok = true;
            s.fails = 0;
            s.recoveries++;
            continue;
        }
        lost[ i ] = true;
        lostCount++;
    }
    if ( lostCount == 0 ) {
        recoveryNote = "found them all still at their own addresses";
        resumeSampler( );
        return;
    }

    bool someoneAtFactoryAddress = false;
    for ( unsigned a = 0; a < sizeof( tmag5273FactoryAddresses ); a++ ) {
        someoneAtFactoryAddress |= tmag5273Present( &MAG_BUS, tmag5273FactoryAddresses[ a ] );
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
// pin 3, z up out of the package top. Then the mounting: a second half turn
// about x if the part is on the underside, and its rotation on the board.
Vec3 magSensorToBoard( Vec3 reading, float rotationDeg, bool underside ) {
    float x = reading.x;
    float y = underside ? reading.y : -reading.y; // two half turns cancel
    float z = underside ? reading.z : -reading.z;
    float a = rotationDeg * (float)M_PI / 180.0f;
    float c = cosf( a );
    float s = sinf( a );
    Vec3 b = { x * c - y * s, x * s + y * c, z };
    return b;
}

Vec3 magBoardToSensor( Vec3 field, float rotationDeg, bool underside ) {
    float a = rotationDeg * (float)M_PI / 180.0f;
    float c = cosf( a );
    float s = sinf( a );
    float x = field.x * c + field.y * s;
    float y = -field.x * s + field.y * c;
    Vec3 r = { x, underside ? y : -y, underside ? field.z : -field.z };
    return r;
}

Vec3 MagArray::sensorFrameField( int i ) const {
    float k = 1.0f / magSensorPlaces[ i ].gain;
    Vec3 ungained = { field[ i ].x * k, field[ i ].y * k, field[ i ].z * k };
    return magBoardToSensor( ungained, magSensorPlaces[ i ].rotationDeg, magSensorPlaces[ i ].underside );
}

void MagArray::startBaseline( ) {
    baselineRestored = false;
    if ( simulatedFrames ) {
        // No frames of the bus to average: the zero is zero, at once.
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            baseline[ i ] = zeroed[ i ] = { 0, 0, 0 };
        }
        baselineLeft = 0;
        baselineCount++;
        zeroedAt++;
        return;
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        baselineSum[ i ] = { 0, 0, 0 };
        baselineFrames[ i ] = 0;
    }
    baselineLeft = MAG_BASELINE_FRAMES;
}

void MagArray::restoreBaseline( const Vec3* list, int count, const char* origin ) {
    for ( int i = 0; i < MAG_SENSOR_COUNT && i < count; i++ ) {
        baseline[ i ] = zeroed[ i ] = list[ i ];
    }
    baselineLeft = 0; // whatever zeroing was under way is off: this one is used
    baselineCount++;
    zeroedAt++; // it counts as a zero taken (the settings keep serialising it)
    baselineRestored = true;
    Stream* out = console.port( );
    if ( out != nullptr ) {
        out->print( "baseline: " );
        out->print( origin );
        out->println( " put back (z zeroes afresh - with the magnet away)" );
    }
}

void MagArray::shiftBaseline( int i, Vec3 by ) {
    if ( i < 0 || i >= MAG_SENSOR_COUNT ) {
        return;
    }
    baseline[ i ] = { baseline[ i ].x - by.x, baseline[ i ].y - by.y, baseline[ i ].z - by.z };
    zeroed[ i ] = baseline[ i ];
}

void MagArray::driftBaseline( float fraction ) {
    if ( baselineLeft > 0 ) {
        return;
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( !fresh[ i ] )
            continue;
        baseline[ i ].x += fraction * ( raw[ i ].x - baseline[ i ].x );
        baseline[ i ].y += fraction * ( raw[ i ].y - baseline[ i ].y );
        baseline[ i ].z += fraction * ( raw[ i ].z - baseline[ i ].z );
    }
}

// A reading into the running sums (the frame is their average).
void MagArray::takeReading( int i, const TMAG5273Reading& reading ) {
    MagSensorState& s = sensors[ i ];
    float limit = MAG_SATURATED * s.dev.rangeMt;
    if ( fabsf( reading.x ) > limit || fabsf( reading.y ) > limit || fabsf( reading.z ) > limit ) {
        sampleSaturated[ i ] = true;
    }
    sampleSum[ i ].x += reading.x;
    sampleSum[ i ].y += reading.y;
    sampleSum[ i ].z += reading.z;
    sampleCount[ i ]++;
    temperatureC[ i ] = reading.temperatureC;
}

// What the other core has left in shared RAM since the last look: every
// sensor's newest sample, if it is a conversion not yet taken (the
// sensor's own SET_COUNT tells a re-read of the same conversion from a new
// one: the sampler reads faster than the sensors convert).
void MagArray::takeSamplerReadings( ) {
    if ( magSampler.resetWanted ) {
        // Every sensor failed two passes running on the other core: the
        // block is wedged. It waits off the bus; this core resets the block
        // (Wire knows the clock) and lets it go on.
        resetBus( );
        magSampler.resetWanted = 0;
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
        TMAG5273Reading reading;
        tmag5273Decode( &s.dev, raw, &reading );
        int setCount = TMAG5273_SET_COUNT( reading.status );
        if ( setCount == samplerSetCount[ i ] ) {
            samplerDuplicates++;
            continue; // the same conversion read again
        }
        samplerSetCount[ i ] = setCount;
        s.fails = 0;
        takeReading( i, reading );
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

// Read every sensor once and add the readings to the running sums.
void MagArray::sampleSensors( uint32_t now ) {
    if ( samplerOn ) {
        takeSamplerReadings( );
        return;
    }
    int tried = 0, failed = 0;
    if ( busWedged( ) ) {
        resetBus( );
        busWedges++;
    }
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        MagSensorState& s = sensors[ i ];
        if ( !s.ok ) {
            continue;
        }
        tried++;
        TMAG5273Reading reading;
        uint32_t t0 = micros( );
        if ( !tmag5273Read( &MAG_BUS, &s.dev, &reading ) ) {
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
                I2C_TypeDef* dev = ch32h4_i2c_regs( busPeripheral );
                stuckStar1 = dev->STAR1;
                stuckStar2 = dev->STAR2;
                stuckCtlr1 = dev->CTLR1;
                stuckScl = digitalRead( PIN_MAG_SCL );
                stuckSda = digitalRead( PIN_MAG_SDA );
#endif
                resetBus( );
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
        takeReading( i, reading );
    }

    // Every sensor failing at once (with NACKs, or the timeout above would
    // have caught it) is the bus, not the sensors: they are still powered
    // and still at their addresses, so reset the bus and carry on.
    if ( tried > 1 && failed == tried ) {
        resetBus( );
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
                                         magSensorPlaces[ i ].underside );
            if ( baselineLeft > 0 ) {
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
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            // Each sensor's own count: one that missed frames must not get a
            // scaled-down baseline (a phantom field from then on).
            int n = baselineFrames[ i ];
            if ( n > 0 ) {
                baseline[ i ] = { baselineSum[ i ].x / n, baselineSum[ i ].y / n, baselineSum[ i ].z / n };
            }
            zeroed[ i ] = baseline[ i ];
        }
        baselineCount++;
        zeroedAt++;
    }
    frameCount++;
}

void MagArray::useSimulatedFrames( ) {
    simulatedFrames = true;
    samplerOn = false;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        sensors[ i ].ok = true;
        sensors[ i ].trouble = nullptr;
        fresh[ i ] = true;
    }
    baselineLeft = 0;
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

    if ( magSamplerParked( ) ) {
        return ServiceStatus::IDLE; // F: the other core is parked for a flash; nothing to read, nothing to recover
    }
    if ( sensorsOk( ) < MAG_SENSOR_COUNT && now >= nextRecoveryMs ) {
        nextRecoveryMs = now + RECOVERY_PERIOD_MS;
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
        out->print( ',' );
        out->print( field[ i ].x, 3 );
        out->print( ',' );
        out->print( field[ i ].y, 3 );
        out->print( ',' );
        out->print( field[ i ].z, 3 );
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
    snprintf( line, sizeof( line ), "MagArray: %d of %d sensors, frame %lu (%.1f reads averaged per frame), baseline %s, VIO rail %s",
              sensorsOk( ), MAG_SENSOR_COUNT, (unsigned long)frameCount, samplesPerFrame, baselineReady( ) ? "set" : "averaging",
              boardVioIs3V3( ) ? "3.3 V" : "NOT 3.3 V - sensors cannot run" );
    out->println( line );
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
        snprintf( line, sizeof( line ), "sampler on the V3F: command %lu state %lu count %lu enabled 0x%02lx, %lu passes (%.0f a second since the last look); reads/fails per sensor",
                  (unsigned long)magSampler.command, (unsigned long)magSampler.state, (unsigned long)magSampler.count, (unsigned long)enabledBits, (unsigned long)passes, perS );
        out->print( line );
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            snprintf( line, sizeof( line ), " %lu/%lu", (unsigned long)magSampler.reads[ i ], (unsigned long)magSampler.fails[ i ] );
            out->print( line );
        }
        snprintf( line, sizeof( line ), "; %lu re-reads of one conversion skipped; the sampler reset the block %lu times", (unsigned long)samplerDuplicates,
                  (unsigned long)magSampler.resets );
        out->println( line );
        // How long one read takes from this core, the sampler held off: the
        // bus clock as it really is (a 7-byte read is ~200 us at 400 kHz).
        MagArray* self = const_cast<MagArray*>( this );
        self->pauseSampler( );
        uint32_t t0 = micros( );
        int good = 0;
        for ( int n = 0; n < 10; n++ ) {
            uint8_t raw[ 10 ];
            good += tmag5273BurstRead( busPeripheral, sensors[ 0 ].dev.address, raw, tmag5273ReadBytes( &sensors[ 0 ].dev ) ) ? 1 : 0;
        }
        uint32_t took = micros( ) - t0;
        self->resumeSampler( );
        snprintf( line, sizeof( line ), "one read of sensor 0 from this core: %lu us (%d of 10 good; 200 us is 400 kHz)", (unsigned long)( took / 10 ), good );
        out->println( line );
    } else {
        out->println( "sampler: NOT running on the V3F - the bus is read on this core" );
    }
    if ( recoveryRuns > 0 || busResets > 0 || stopClears > 0 ) {
        snprintf( line, sizeof( line ), "I2C block re-initialised %lu times, %lu of them caught as a wedge before a read (last %.1f s ago, %.1f s after the one before); a pending STOP cleared %lu times; recovery has run %lu times, last: %s",
                  (unsigned long)busResets, (unsigned long)busWedges, ( millis( ) - lastBusResetMs ) * 1e-3f, ( lastBusResetMs - previousBusResetMs ) * 1e-3f,
                  (unsigned long)stopClears, (unsigned long)recoveryRuns, recoveryNote );
        out->println( line );
        if ( stuckStar1 != 0 || stuckStar2 != 0 || stuckCtlr1 != 0 ) {
            snprintf( line, sizeof( line ), "at the last timed-out read: STAR1 0x%04x STAR2 0x%04x CTLR1 0x%04x, SCL %d SDA %d", stuckStar1, stuckStar2, stuckCtlr1, stuckScl, stuckSda );
            out->println( line );
        }
    }
    if ( busPeripheral == 0 ) {
        out->println( "the SCL/SDA pins in BoardPins.h are not a pair this chip can use for I2C" );
    }
    out->println( " #  addr  part        range   x_mm  y_mm  fails  recov      Bx      By      Bz  (mT)   degC" );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        const MagSensorState& s = sensors[ i ];
        if ( !s.ok ) {
            snprintf( line, sizeof( line ), "%2d  ----  no answer          %6.1f %5.1f   %s (register 0x%02X, Wire code %u)", i, position[ i ].x, position[ i ].y,
                      s.trouble ? s.trouble : "stopped answering", s.troubleRegister, s.troubleCode );
            out->println( line );
            continue;
        }
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
        snprintf( line, sizeof( line ), "%2d  0x%02X  TMAG5273%c%d  %5.0f  %6.1f %5.1f  %5u  %5lu  %+6.2f  %+6.2f  %+6.2f        %5.1f",
                  i, s.dev.address, variant, s.dev.version, s.dev.rangeMt, position[ i ].x, position[ i ].y,
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

    // Everything off, bus released, then look at the bare lines.
    MAG_BUS.end( );
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        sensors[ i ].ok = false;
        digitalWrite( magSensorPlaces[ i ].vccPin, LOW );
    }
    delay( MAG_POWER_OFF_MS );
    uint32_t sclRise, sdaRise;
    LineState sclOff = checkLine( PIN_MAG_SCL, &sclRise );
    LineState sdaOff = checkLine( PIN_MAG_SDA, &sdaRise );
    out->println( "bus lines with every sensor's VCC driven low:" );
    printLine( out, "SCL", sclOff, sclRise );
    printLine( out, "SDA", sdaOff, sdaRise );

    // One sensor at a time, then all. Only differences are printed: a line
    // that is fine until one particular sensor is powered points at that
    // sensor or its wiring.
    bool changed = false;
    for ( int i = 0; i <= MAG_SENSOR_COUNT; i++ ) {
        bool all = ( i == MAG_SENSOR_COUNT );
        for ( int k = 0; k < MAG_SENSOR_COUNT; k++ ) {
            digitalWrite( magSensorPlaces[ k ].vccPin, ( all || k == i ) ? HIGH : LOW );
        }
        delay( MAG_POWER_ON_MS );
        LineState scl = checkLine( PIN_MAG_SCL, &sclRise );
        LineState sda = checkLine( PIN_MAG_SDA, &sdaRise );
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
        digitalWrite( magSensorPlaces[ i ].vccPin, LOW );
    }
    if ( sclOff != LINE_OK || sdaOff != LINE_OK ) {
        out->println( "I2C cannot work until both lines read ok. This chip has no internal pull-ups for I2C:" );
        out->println( "2.2k-4.7k from SCL (PA8) to 3.3 V and from SDA (PC9) to 3.3 V. Wiggle, fix, run b again." );
    }
    delay( MAG_POWER_OFF_MS );

    MAG_BUS.setSCL( PIN_MAG_SCL );
    MAG_BUS.setSDA( PIN_MAG_SDA );
    MAG_BUS.begin( );
    MAG_BUS.setClock( 100000 ); // slow, to take bus speed out of the question

    // Each sensor powered alone: which addresses acknowledge?
    // endTransmission(): 0 = ACK, 2 = nothing at that address, 4 = bus fault.
    int silent = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        digitalWrite( magSensorPlaces[ i ].vccPin, HIGH );
        delay( MAG_POWER_ON_MS );

        snprintf( line, sizeof( line ), "sensor %d powered:", i );
        out->print( line );
        int found = 0;
        for ( uint8_t address = 0x08; address < 0x78; address++ ) {
            MAG_BUS.beginTransmission( address );
            uint8_t result = MAG_BUS.endTransmission( );
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
        digitalWrite( magSensorPlaces[ i ].vccPin, LOW );
        if ( found < 0 ) {
            break;
        }
        delay( MAG_POWER_OFF_MS );
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
                answered |= tmag5273Present( &MAG_BUS, tmag5273FactoryAddresses[ a ] );
            }
            digitalWrite( candidates[ c ], LOW );
            delay( 20 );
            pinMode( candidates[ c ], INPUT );
            snprintf( line, sizeof( line ), "  %s: %s", candidateNames[ c ], answered ? "A SENSOR ANSWERS - this pin powers one" : "nothing" );
            out->println( line );
        }
    }

    out->println( "re-addressing the array..." );
    begin( );
    printStatus( out );
}
