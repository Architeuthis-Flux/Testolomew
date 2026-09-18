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
    int begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "MagArray"; }
    ServicePriority getPriority( ) const override { return ServicePriority::HIGH; }
    uint32_t periodUs( ) const override { return MAG_SAMPLE_PERIOD_US; }

    // ---- what consumers read ----
    int sensorCount( ) const { return MAG_SENSOR_COUNT; }
    int sensorsOk( ) const;
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

    void startBaseline( );
    void printStatus( Stream* out ) const;

    // Bench diagnostic for "no sensor answers": are there pull-ups on the bus,
    // and with each sensor powered alone, what acknowledges? Stops sampling
    // while it runs and re-addresses everything afterwards.
    void printBusCheck( Stream* out );

    bool streaming = false;   // CSV frames to the console
    bool identifying = false; // print the strongest sensor

  private:
    MagArray( ) = default;

    MagSensorState sensors[ MAG_SENSOR_COUNT ];
    Vec3 baseline[ MAG_SENSOR_COUNT ];
    Vec3 baselineSum[ MAG_SENSOR_COUNT ];
    int baselineLeft = 0;
    uint32_t nextRecoveryMs = 0;
    const char* recoveryNote = "not needed yet"; // what recoverLostSensors() last did
    uint32_t recoveryRuns = 0;
    uint32_t busResets = 0; // times the I2C block had to be re-initialised
    void resetBus( );
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
