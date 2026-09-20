// SPDX-License-Identifier: MIT
#ifndef MAGARRAY_H
#define MAGARRAY_H
// ---------------------------------------------------------------------------
// The magnetometer array: N TMAG5273s on one I2C bus, read as one frame of
// field vectors in the board frame with the ambient field subtracted.
//
// Addressing. Every sensor of one variant leaves the factory at the same
// address and forgets any other address when it loses power. So each sensor's
// VCC is a GPIO, and begin() walks them at EVERY boot: all off (VCC driven
// LOW, not floating - an unpowered sensor with a floating supply can be
// half-powered through the bus pull-ups and answer at the factory address),
// then one at a time: power up, find it at whichever factory address it
// answers on, move it to MAG_BASE_ADDRESS + i, configure it.
//
// The walk is the only time power is switched. When it ends every sensor is
// powered and stays powered, whether it answered or not. The VCC pins do have
// to stay VCC pins, though: the walk is needed again after any power loss.
//
// A sensor that stops answering is first looked for at its own address, with
// nobody's power touched - a bus hiccup does not reset a sensor, so it is
// usually still there. Only if something turns up back at a factory address
// (a sensor really did reset) are the lost sensors, and only those, walked
// again; the rest keep running at their addresses.
//
// Sampling. The sensors are read as often as the loop allows and the reads
// are averaged into frames at MAG_FRAME_PERIOD_US; consumers only see frames.
//
// Baseline. The first MAG_BASELINE_FRAMES frames after begin() (and after the
// `z` command) are averaged and subtracted from then on. Keep the magnet away
// while that runs.
//
// Console: m = status, z = re-zero, p = power-cycle and re-address everything,
// f = stream frames as CSV, i = identify (which sensor reads strongest),
// b = bus check (pull-ups, and what acknowledges with each sensor powered).
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "MagArrayConfig.h"
#include "Vec3.h"

struct MagSensorState {
    TMAG5273 dev;
    bool ok;                 // addressed, configured and answering
    uint8_t factoryAddress;  // where it was found at power-up (tells the variant)
    uint8_t fails;           // consecutive failed reads
    const char* trouble;     // why the last addressing attempt failed (nullptr = it did not)
    uint8_t troubleRegister; // ...and the register and Wire code of the transaction that failed
    uint8_t troubleCode;
    uint32_t recoveries; // times it has been power-cycled back to life
};

// A TMAG5273 reading (its own axes and signs) to the board frame, for a
// package turned rotationDeg and on the top or the underside - and back again.
// See the definition for the conventions. MagArray uses the table in
// MagArrayConfig.h; the orientation check tries other values.
Vec3 magSensorToBoard( Vec3 reading, float rotationDeg, bool underside );
Vec3 magBoardToSensor( Vec3 field, float rotationDeg, bool underside );

class MagArray : public Service {
  public:
    static MagArray& getInstance( );

    MagArray( const MagArray& ) = delete;
    MagArray& operator=( const MagArray& ) = delete;

    // Power-up addressing of every sensor. Returns how many answered.
    int begin( bool zero = true ); // zero = take a new baseline (a run-time power-cycle keeps the one in use)

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "MagArray"; }
    ServicePriority getPriority( ) const override { return ServicePriority::HIGH; }
    uint32_t periodUs( ) const override { return MAG_SAMPLE_PERIOD_US; }

    // ---- what consumers read ----
    int sensorCount( ) const { return MAG_SENSOR_COUNT; }
    int sensorsOk( ) const;
    uint32_t busResetCount( ) const { return busResets; } // times the I2C block had to be re-initialised
    const MagSensorState& sensor( int i ) const { return sensors[ i ]; }

    // The latest baseline-removed reading of sensor i back in the sensor's
    // own frame (the configured mounting undone).
    Vec3 sensorFrameField( int i ) const;

    Vec3 position[ MAG_SENSOR_COUNT ];              // where each sensor sits, mm
    Vec3 field[ MAG_SENSOR_COUNT ];                 // latest reading minus baseline, mT, board frame
    Vec3 raw[ MAG_SENSOR_COUNT ];                   // latest reading as measured, mT, board frame
    bool fresh[ MAG_SENSOR_COUNT ];                 // this sensor was read in the latest frame, and was not saturated
    bool saturated[ MAG_SENSOR_COUNT ] = { false }; // it read full scale in the latest frame
    float temperatureC[ MAG_SENSOR_COUNT ] = { 0 }; // die temperature of each sensor
    uint32_t frameCount = 0;                        // bumps once per completed frame
    uint8_t busPeripheral = 0;                      // which I2C block the pins resolved to (0 = not a valid pair)
    bool baselineReady( ) const { return baselineLeft == 0; }
    Vec3 baselineOf( int i ) const { return baseline[ i ]; } // what is being subtracted, mT, board frame
    uint32_t baselineCount = 0;                              // bumps each time a baseline is finished
    // The baseline as last zeroed (z, or boot), before the slow drift
    // tracking moved it: what the settings keep, so a reboot with the probe
    // lying on the board does not zero the magnet into the baseline. zeroedAt
    // bumps with each new one. restoreBaseline() installs a saved one (and
    // ends any zeroing under way); shiftBaseline() subtracts a field from a
    // sensor's baseline (a magnet that was there while it zeroed, MagLocator's Y).
    Vec3 zeroed[ MAG_SENSOR_COUNT ];
    uint32_t zeroedAt = 0;
    bool baselineRestored = false; // the baseline in use came from the settings (not to be retaken as "polluted": it was checked when taken)
    void restoreBaseline( const Vec3* list, int count, const char* origin = "the saved zero" );
    void shiftBaseline( int i, Vec3 by );

    // For a power bisect (:load). powerOff(): every sensor's supply pin low
    // and the sampler paused. powerOffFrom( n ): only the first n stay
    // powered (the rest are left alone by the recovery). powerOn() is
    // begin( false ): the walk again, the zero in use kept. holdSampler():
    // the other core stops reading, the sensors stay powered and
    // converting. setLowNoise( false ): the low-power conversion mode
    // (2.3 mA a sensor instead of 3.0; a recovery puts low-noise back).
    void powerOff( );
    void powerOffFrom( int n );
    void powerOnly( uint32_t mask ); // bit i set = sensor i stays powered
    void powerOn( );
    void holdSampler( bool hold );
    void setLowNoise( bool on );
    bool poweredOff = false;
    bool samplerHeld = false;
    bool lowNoise = true;
    uint32_t samplerPassPeriodMs = 0; // :load sampler <ms>: the V3F's passes paced (0 = free-running); applied at every start
    void setSamplerPassPeriodMs( uint32_t ms );
    float samplerPassesPerSecond( ); // measured since the last call
    int enabledCount( ) const; // sensors not disabled by powerOffFrom()

    void startBaseline( );
    // Move the baseline a fraction of the way to the latest raw readings (the
    // locator calls this while it sees nothing, to track slow drift).
    void driftBaseline( float fraction );
    void printStatus( Stream* out ) const;

    // Bench diagnostic for "no sensor answers": are there pull-ups on the bus,
    // and with each sensor powered alone, what acknowledges? Stops sampling
    // while it runs and re-addresses everything afterwards.
    void printBusCheck( Stream* out );
    // The other core's sampler off the bus while this core uses it, and back.
    void pauseSampler( );
    void resumeSampler( );

    bool streaming = false;   // CSV frames to the console
    bool identifying = false; // print the strongest sensor

    // The host simulator: no bus, no sampler; whoever runs the world writes
    // field[] and fresh[] and bumps frameCount itself. Every sensor is
    // called ok and the baseline ready.
    void useSimulatedFrames( );
    bool simulatedFrames = false;

  private:
    MagArray( ) = default;

    MagSensorState sensors[ MAG_SENSOR_COUNT ];
    bool disabled[ MAG_SENSOR_COUNT ] = { false }; // left unpowered on purpose (powerOffFrom)
    Vec3 baseline[ MAG_SENSOR_COUNT ];
    Vec3 baselineSum[ MAG_SENSOR_COUNT ];
    int baselineFrames[ MAG_SENSOR_COUNT ] = { 0 }; // frames each sensor contributed to the baseline being taken
    int baselineLeft = 0;
    uint32_t nextRecoveryMs = 0;
    const char* recoveryNote = "not needed yet"; // what recoverLostSensors() last did
    uint32_t recoveryRuns = 0;
    // The other core's sampler (MagSampler.h): on, and per sensor the
    // sequence and conversion count last taken, and the counters last seen.
    bool samplerOn = false;
    uint32_t samplerSeq[ MAG_SENSOR_COUNT ];
    int samplerSetCount[ MAG_SENSOR_COUNT ];
    uint32_t samplerFailsSeen[ MAG_SENSOR_COUNT ];
    uint32_t samplerReadsSeen[ MAG_SENSOR_COUNT ];
    uint32_t samplerLastReadMs[ MAG_SENSOR_COUNT ];
    uint32_t samplerDuplicates = 0;
    mutable uint32_t samplerPassesSeen = 0, samplerPassesSeenMs = 0; // the status line reports the pass rate since it last looked
    void startSampler( );
    void takeReading( int i, const TMAG5273Reading& reading );
    void takeSamplerReadings( );
    uint32_t busResets = 0; // times the I2C block had to be re-initialised
    uint32_t busWedges = 0; // of those, caught by busWedged() before a read (cheap) rather than by a timeout (10 ms)
    uint32_t stopClears = 0; // a STOP left pending on an idle block, cleared before a read (busWedged())
    uint32_t lastBusResetMs = 0, previousBusResetMs = 0;
    uint16_t stuckStar1 = 0, stuckStar2 = 0, stuckCtlr1 = 0; // the block's registers at the last timed-out read
    int stuckScl = -1, stuckSda = -1;                       // and the lines
    void resetBus( );
    bool busWedged( );
    uint32_t lastIdentifyMs = 0;

    void addressSensors( const bool* which );
    bool powerUpSensor( int i );
    void recoverLostSensors( );
    void sampleSensors( uint32_t now );
    void publishFrame( );

    // Reads since the last published frame, in each sensor's own frame.
    Vec3 sampleSum[ MAG_SENSOR_COUNT ];
    uint16_t sampleCount[ MAG_SENSOR_COUNT ] = { 0 };
    bool sampleSaturated[ MAG_SENSOR_COUNT ] = { false };
    uint32_t lastFrameUs = 0;
    float samplesPerFrame = 0.0f; // running average, for the status line
    void printFrameCsv( Stream* out ) const;
    void printStrongest( Stream* out );
};

extern MagArray& magArray;

#endif // MAGARRAY_H
