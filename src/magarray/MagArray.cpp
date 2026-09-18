// SPDX-License-Identifier: MIT
#include "MagArray.h"

#include <Wire.h>
#include <math.h>

#include "Console.h"

#define MAG_BUS Wire

#define STREAM_EVERY_N_FRAMES 5 // CSV at 20 Hz; 100 Hz of text outruns 115200 baud
#define IDENTIFY_PERIOD_MS 250
#define RECOVERY_PERIOD_MS 2000

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
    int found = magArray.begin( );
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

static void onBusCheck( Stream* out ) { magArray.printBusCheck( out ); }

static void onIdentify( Stream* out ) {
    magArray.identifying = !magArray.identifying;
    out->println( magArray.identifying ? "identify on - hold the magnet over one sensor at a time" : "identify off" );
}

// ---- power-up addressing ---------------------------------------------------

int MagArray::begin( ) {
    static bool commandsAdded = false;
    if ( !commandsAdded ) {
        commandsAdded = true;
        consoleAddCommand( 'm', "magnetometer array status", onStatus );
        consoleAddCommand( 'z', "re-zero the ambient baseline (magnet away!)", onZero );
        consoleAddCommand( 'p', "power-cycle and re-address every sensor", onPowerCycle );
        consoleAddCommand( 'f', "stream field frames as CSV (toggle)", onStream );
        consoleAddCommand( 'i', "identify: show the strongest sensor (toggle)", onIdentify );
        consoleAddCommand( 'b', "bus check: pull-ups, and what answers with each sensor powered alone", onBusCheck );
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

    startBaseline( );
    return sensorsOk( );
}

// Nothing on the bus can be talked to (a line with no pull-up, or held down).
// Every transaction then burns its full timeout, so callers check this first.
static bool busFault( ) {
    MAG_BUS.beginTransmission( tmag5273FactoryAddresses[ 0 ] );
    uint8_t result = MAG_BUS.endTransmission( );
    return result != 0 && result != 2; // 0 = ACK, 2 = nobody at that address
}

// Re-initialise the I2C block: begin() resets the peripheral and, if a device
// is holding SDA, clocks it out.
void MagArray::resetBus( ) {
    MAG_BUS.end( );
    MAG_BUS.begin( );
    MAG_BUS.setClock( MAG_I2C_HZ );
    busResets++;
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
    if ( busFault( ) ) {
        resetBus( );
        recoveryNote = "bus fault: re-initialised the I2C block";
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
        return;
    }

    bool someoneAtFactoryAddress = false;
    for ( unsigned a = 0; a < sizeof( tmag5273FactoryAddresses ); a++ ) {
        someoneAtFactoryAddress |= tmag5273Present( &MAG_BUS, tmag5273FactoryAddresses[ a ] );
    }
    if ( !someoneAtFactoryAddress ) {
        recoveryNote = "lost sensors answer at neither their own nor a factory address";
        return;
    }

    addressSensors( lost );
    recoveryNote = "walked the lost sensors through addressing again";
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        if ( lost[ i ] && sensors[ i ].ok ) {
            sensors[ i ].recoveries++;
        }
    }
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
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        baselineSum[ i ] = { 0, 0, 0 };
    }
    baselineLeft = MAG_BASELINE_FRAMES;
}

// Read every sensor once and add the readings to the running sums.
void MagArray::sampleSensors( uint32_t now ) {
    int tried = 0, failed = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        MagSensorState& s = sensors[ i ];
        if ( !s.ok ) {
            continue;
        }
        tried++;
        TMAG5273Reading reading;
        if ( !tmag5273Read( &MAG_BUS, &s.dev, &reading ) ) {
            failed++;
            if ( ++s.fails >= MAG_MAX_READ_FAILS ) {
                s.ok = false; // stays powered; recoverLostSensors() looks for it
                nextRecoveryMs = now + RECOVERY_PERIOD_MS;
            }
            continue;
        }
        s.fails = 0;
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

    // Every sensor failing at once is the bus, not the sensors: they are still
    // powered and still at their addresses, so reset the bus and carry on.
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
            baseline[ i ] = { baselineSum[ i ].x / MAG_BASELINE_FRAMES, baselineSum[ i ].y / MAG_BASELINE_FRAMES, baselineSum[ i ].z / MAG_BASELINE_FRAMES };
        }
    }
    frameCount++;
}

ServiceStatus MagArray::service( ) {
    uint32_t now = millis( );

    if ( sensorsOk( ) < MAG_SENSOR_COUNT && now >= nextRecoveryMs ) {
        nextRecoveryMs = now + RECOVERY_PERIOD_MS;
        recoverLostSensors( );
    }

    sampleSensors( now );

    uint32_t nowUs = micros( );
    if ( nowUs - lastFrameUs >= MAG_FRAME_PERIOD_US ) {
        lastFrameUs = nowUs;
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
    char line[ 140 ];
    snprintf( line, sizeof( line ), "MagArray: %d of %d sensors, frame %lu (%.1f reads averaged per frame), baseline %s, VIO rail %s",
              sensorsOk( ), MAG_SENSOR_COUNT, (unsigned long)frameCount, samplesPerFrame, baselineReady( ) ? "set" : "averaging",
              boardVioIs3V3( ) ? "3.3 V" : "NOT 3.3 V - sensors cannot run" );
    out->println( line );
    if ( recoveryRuns > 0 || busResets > 0 ) {
        snprintf( line, sizeof( line ), "I2C block re-initialised %lu times; recovery has run %lu times, last: %s", (unsigned long)busResets,
                  (unsigned long)recoveryRuns, recoveryNote );
        out->println( line );
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
