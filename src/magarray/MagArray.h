// SPDX-License-Identifier: MIT
#ifndef MAGARRAY_H
#define MAGARRAY_H
// ---------------------------------------------------------------------------
// The magnetometer array: N sensors of one or more kinds (MagArrayConfig.h:
// TMAG5273 Hall sensors, MMC56x3 AMR compasses) on up to four I2C buses,
// read as one frame of field vectors in the board frame with the ambient
// field subtracted. Every sensor comes out in mT in the same frame whatever
// its kind, and each carries a weight for the fit from its kind's noise
// (weight[]), so a mixed array gives one reading.
//
// Addressing. Every TMAG5273 of one variant leaves the factory at the same
// address and forgets any other address when it loses power. So each one's
// VCC is a GPIO, and begin() walks them at EVERY boot: all off (VCC driven
// LOW, not floating - an unpowered sensor with a floating supply can be
// half-powered through the bus pull-ups and answer at the factory address),
// then one at a time: power up, find it at whichever factory address it
// answers on, move it to MAG_BASE_ADDRESS + i, configure it. A fixed-address
// part (the MMC56x3 at 0x30) is powered for good and simply looked for at
// its address on its bus once the walk is done; two of them need two buses.
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
#include "MagSampler.h"
#include "Vec3.h"

struct MagSensorState {
    MagSensorType type;
    TMAG5273 dev; // type == MAG_TMAG5273
    MMC56x3 mmc;  // type == MAG_MMC56X3
    uint8_t address;         // where it answers now (assigned, or its fixed one)
    float rangeMt;           // full scale of each axis; a reading at MAG_SATURATED of it is clipped
    int bus;                 // index into the array's buses
    bool ok;                 // addressed, configured and answering
    uint8_t factoryAddress;  // TMAG5273: where it was found at power-up (tells the variant)
    uint8_t fails;           // consecutive failed reads
    const char* trouble;     // why the last addressing attempt failed (nullptr = it did not)
    uint8_t troubleRegister; // ...and the register and Wire code of the transaction that failed
    uint8_t troubleCode;
    uint32_t recoveries; // times it has been brought back to life
};

// One I2C bus the array uses: the pins, the Wire object on them and the
// block they resolved to. The first is Wire on PIN_MAG_SCL/SDA.
struct MagBus {
    TwoWire* wire;
    int sclPin, sdaPin;
    uint8_t peripheral; // 1-4, or 0 = the pins are not a pair this chip can use
    uint8_t probeAddress; // an address that should ACK on this bus (a bus-fault check)
    uint32_t resets;      // times its block was re-initialised
};

// A sensor's reading (its own axes and signs) to the board frame, for a
// package turned rotationDeg, on the top or the underside, of a type whose
// own +z points into its top (zIntoTop: the TMAG5273) or out of it (the
// MMC56x3) - and back again. See the definition for the conventions.
// MagArray uses the table in MagArrayConfig.h; the orientation check tries
// other values.
Vec3 magSensorToBoard( Vec3 reading, float rotationDeg, bool underside, bool zIntoTop = true );
Vec3 magBoardToSensor( Vec3 field, float rotationDeg, bool underside, bool zIntoTop = true );

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
    uint32_t busResetCount( ) const { return busResets; } // times an I2C block had to be re-initialised
    const MagSensorState& sensor( int i ) const { return sensors[ i ]; }
    int busCount( ) const { return buses; }
    const MagBus& bus( int b ) const { return busTable[ b ]; }

    // The latest baseline-removed reading of sensor i back in the sensor's
    // own frame (the configured mounting undone).
    Vec3 sensorFrameField( int i ) const;
    // A sensor's zero in its OWN frame - the reading as the part gives it,
    // before the table's gain, rotation and flip - which is what the
    // settings keep (Settings.cpp, `sensorzero=`): a zero saved in the
    // board frame is only right under the table row it was taken with, and
    // a row calibrated afterwards (a new rotation, flip or gain: magcal)
    // turns it into a phantom (2026-09-21: the MMC's zero, saved under its
    // first row and put back under the measured one, read 0.09 mT with
    // nothing there, and every fix leaned on it). And a saved one back into
    // the board frame under the row compiled now.
    Vec3 zeroInSensorFrame( int i ) const;
    Vec3 sensorFrameToBoard( int i, Vec3 sensorFrame ) const;

    Vec3 position[ MAG_SENSOR_COUNT ];              // where each sensor sits, mm
    Vec3 field[ MAG_SENSOR_COUNT ];                 // latest reading minus baseline, mT, board frame
    Vec3 raw[ MAG_SENSOR_COUNT ];                   // latest reading as measured, mT, board frame
    bool fresh[ MAG_SENSOR_COUNT ];                 // this sensor was read in the latest frame, and was not saturated
    // ...and whether the fit should use it: fresh, calibrated (the table)
    // and its zero not provisional (below). The locator passes this, not
    // fresh[], to the fit; everything else (the scene, `m`, `f`) sees it.
    bool usedInFit( int i ) const;
    bool trustAllSensors = false; // the host simulation: every simulated sensor is calibrated and zeroed
    // The MMC56x3 in or out (MAG_USE_MMC_AT_BOOT; the menu's "sensors / use
    // MMC"): out, it is read and shown but ignored by the locator entirely.
    bool useMmc = MAG_USE_MMC_AT_BOOT;
    bool ignored( int i ) const { return i >= 0 && i < MAG_SENSOR_COUNT && magSensorPlaces[ i ].type == MAG_MMC56X3 && !useMmc; }
    bool saturated[ MAG_SENSOR_COUNT ] = { false }; // it read full scale in the latest frame
    float temperatureC[ MAG_SENSOR_COUNT ] = { 0 }; // die temperature of each sensor (TMAG5273 with the channel on)
    // How much the fit should trust each sensor, from its type's noise
    // (MAG_WEIGHT_* in MagArrayConfig.h): 1 for a TMAG5273, up to the cap
    // for a quieter type. noiseMt[] is the type's noise a frame (the host
    // simulation draws each sensor's noise from it).
    float weight[ MAG_SENSOR_COUNT ];
    float noiseMt[ MAG_SENSOR_COUNT ];
    float weightCap = MAG_WEIGHT_CAP; // the console's w changes it at run time (0 = every sensor equal)
    // The weights for one frame's fit: each sensor's by the size of its
    // reading (MagArrayConfig.h: magSensorFrameWeight - a quiet type counts
    // for up to the cap where it reads noise and for 1 by a strong reading,
    // where the model's error outweighs anyone's noise). out[] has
    // MAG_SENSOR_COUNT entries.
    void frameWeights( const Vec3* fields, float* out ) const;
    void setWeightCap( float cap );
    uint32_t frameCount = 0;   // bumps once per completed frame
    uint8_t busPeripheral = 0; // which I2C block the first bus's pins resolved to (0 = not a valid pair)
    // Ready = every sensor the fit may use has its zero. A zero under way
    // for a few provisional sensors only (none of them in the fit until it
    // is done) does not hold the fit of the others (2026-09-21: an absent
    // sensor kept re-zeroing and the locator waited on it all night).
    bool baselineReady( ) const { return baselineLeft == 0 || partialZeroing; }
    Vec3 baselineOf( int i ) const { return baseline[ i ]; } // what is being subtracted, mT, board frame
    uint32_t baselineCount = 0;                              // bumps each time a baseline is finished
    // The baseline as last zeroed (z, or boot), before the slow drift
    // tracking moved it: what the settings keep, so a reboot with the probe
    // lying on the board does not zero the magnet into the baseline. zeroedAt
    // bumps with each new one. restoreBaseline() installs a saved one (and
    // ends any zeroing under way) for the sensors it covers - `count` of
    // them, or those marked in `have` - and the rest zero themselves from
    // the next MAG_BASELINE_FRAMES frames (a sensor added since the zero
    // was saved); shiftBaseline() subtracts a field from a sensor's
    // baseline (a magnet that was there while it zeroed, MagLocator's Y).
    Vec3 zeroed[ MAG_SENSOR_COUNT ];
    uint32_t zeroedAt = 0;
    bool baselineRestored = false; // the baseline in use came from the settings (not to be retaken as "polluted": it was checked when taken)
    // A sensor's zero is PROVISIONAL when it was taken from live frames
    // while the others' came from a saved or compiled zero - a sensor added
    // since that zero was taken - because nothing says the probe was away
    // at the time. Such a sensor is read but kept out of the fit until the
    // locator has settled its zero from a good fix of the others
    // (settleProvisionalZero: the magnet's field at the sensor taken out of
    // the zero if it is in it, or the zero left as it is if the sensor
    // plainly sees the magnet), or `z` zeroes everything deliberately.
    bool zeroProvisional[ MAG_SENSOR_COUNT ] = { false };
    // ...and whether a sensor has a zero at all: one restored from a record
    // or taken from frames it was actually read in. A sensor that was absent
    // while the others zeroed has none, and the settings write no zero=
    // line for it (2026-09-21: an absent MMC's "zero" of 0,0,0 was saved,
    // then restored at every boot as if it were real, so the sensor never
    // zeroed again once it was wired).
    bool zeroKnown[ MAG_SENSOR_COUNT ] = { false };
    int provisionalCount( ) const;
    void settleProvisionalZero( int i, Vec3 magnetFieldAtSensor, bool takeOutOfZero );
    // The saved zero follows the drifted baseline for every known,
    // settled sensor read this frame whose two differ by more than
    // thresholdMt (the locator calls this while nothing is present).
    bool settleDriftedZeros( float thresholdMt ); // true if any moved
    void restoreBaseline( const Vec3* list, int count, const char* origin = "the saved zero" );
    void restoreBaseline( const Vec3* list, const bool* have, int count, const char* origin );
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

    void startBaseline( );                        // every sensor zeroes afresh
    void startBaselineFor( const bool* which ); // ...or only these; the others keep theirs
    // The live baseline set outright (the locator's offset filter, every
    // frame: MagLocator::keepZeros). The saved zero (zeroed[]) is not touched:
    // that is z's and the audit's. Refused while a zeroing is under way.
    void setBaseline( int i, Vec3 zero );
    void printStatus( Stream* out ) const;
    // An MMC56x3 looked at from this core: its Status1 and raw data bytes
    // three times, 50 ms apart (do they change? is a measurement done?),
    // after reconfiguring it if asked (:mmc <i> cfg <odrHz> <bw> <autoSR>).
    void probeMmc( Stream* out, int i, bool reconfigure, int odrHz, int bw, bool autoSr );
    // :watch <i>[,<j>...]|all|off - while sensors are being wired: the
    // missing ones are looked for every half second instead of every two,
    // and the screen shows a banner (main.cpp's overlay) with a line per
    // watched sensor that turns green when it answers. Bit i = sensor i.
    uint32_t watchMask = 0;
    uint32_t watchSinceMs = 0;

    // Bench diagnostic for "no sensor answers": are there pull-ups on each
    // bus, and with each TMAG5273 powered alone, what acknowledges? Stops
    // sampling while it runs and re-addresses everything afterwards.
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
    bool disabled[ MAG_SENSOR_COUNT ] = { false }; // left unpowered (or unread) on purpose (powerOffFrom)
    MagBus busTable[ MAG_MAX_BUSES ];
    int buses = 0;
    uint8_t busFaultStreak[ MAG_MAX_BUSES ] = { 0 }; // recoveries running in which the bus could not be talked to
    int busOf( int sensor ) const { return sensors[ sensor ].bus; }
    TwoWire* wireOf( int sensor ) const { return busTable[ sensors[ sensor ].bus ].wire; }
    bool busUsable( int b ) const { return busTable[ b ].peripheral != 0; } // its pins resolved to an I2C block
    void setupBuses( );
    void setVcc( int i, int level ); // digitalWrite on the sensor's VCC pin, if it has one
    Vec3 baseline[ MAG_SENSOR_COUNT ];
    Vec3 baselineSum[ MAG_SENSOR_COUNT ];
    int baselineFrames[ MAG_SENSOR_COUNT ] = { 0 }; // frames each sensor contributed to the baseline being taken
    bool zeroing[ MAG_SENSOR_COUNT ] = { false };   // the sensors the baseline being taken is for
    bool partialZeroing = false;                    // ...and it is not every sensor (the fit goes on meanwhile)
    int baselineLeft = 0;
    uint32_t nextRecoveryMs = 0;
    const char* recoveryNote = "not needed yet"; // what recoverLostSensors() last did
    uint32_t recoveryRuns = 0;
    // The other core's sampler (MagSampler.h): on, and per sensor the
    // sequence and conversion count last taken, and the counters last seen.
    bool samplerOn = false;
    uint32_t samplerSeq[ MAG_SENSOR_COUNT ];
    int samplerSetCount[ MAG_SENSOR_COUNT ];
    uint8_t samplerLastRaw[ MAG_SENSOR_COUNT ][ MAGSAMPLER_MAX_BYTES ]; // a fixed-address part has no conversion counter: the same bytes again is the same measurement
    uint8_t samplerZeroReads[ MAG_SENSOR_COUNT ] = { 0 };               // MMC reads of all zeros running (a part that has reset)
    uint32_t samplerFailsSeen[ MAG_SENSOR_COUNT ];
    uint32_t samplerReadsSeen[ MAG_SENSOR_COUNT ];
    uint32_t samplerLastReadMs[ MAG_SENSOR_COUNT ];
    uint32_t samplerDuplicates = 0;
    mutable uint32_t samplerPassesSeen = 0, samplerPassesSeenMs = 0; // the status line reports the pass rate since it last looked
    void startSampler( );
    void describeSensorToSampler( int i );
    void takeReading( int i, Vec3 reading, float tempC ); // one reading in the sensor's own frame, mT
    void takeSamplerReadings( );
    bool readSensorHere( int i, Vec3* reading, float* temperatureC ); // on this core, through the sensor's driver
    uint32_t busResets = 0; // times an I2C block had to be re-initialised
    uint32_t busWedges = 0; // of those, caught by busWedged() before a read (cheap) rather than by a timeout (10 ms)
    uint32_t stopClears = 0; // a STOP left pending on an idle block, cleared before a read (busWedged())
    uint32_t lastBusResetMs = 0, previousBusResetMs = 0;
    uint16_t stuckStar1 = 0, stuckStar2 = 0, stuckCtlr1 = 0; // the block's registers at the last timed-out read
    int stuckScl = -1, stuckSda = -1;                       // and the lines
    void resetBus( int b = 0 );
    void resetAllBuses( );
    bool busFault( int b );
    bool busWedged( int b );
    uint32_t lastIdentifyMs = 0;

    void addressSensors( const bool* which );
    bool powerUpSensor( int i );
    bool beginFixedSensor( int i ); // an MMC56x3: look for it at its address on its bus and configure it
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
