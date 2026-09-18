// SPDX-License-Identifier: MIT
#ifndef MAGARRAYCONFIG_H
#define MAGARRAYCONFIG_H
// ---------------------------------------------------------------------------
// The magnetometer array as built: how many sensors, which pins power them,
// where each one sits and which way it faces. Pins come from BoardPins.h;
// everything about the array's geometry is here.
//
// BOARD FRAME (what every position and field downstream is in): millimetres,
// origin at sensor 0, +x along the row of four, +y toward the second row, +z
// up out of the board toward the probe.
// ---------------------------------------------------------------------------
#include "BoardPins.h"
#include "TMAG5273.h"

#define MAG_SENSOR_COUNT 8

// Sensor i is given address MAG_BASE_ADDRESS + i. Anything clear of the four
// factory addresses (0x22, 0x35, 0x44, 0x78) works.
#define MAG_BASE_ADDRESS 0x10

#define MAG_I2C_HZ 400000
#define MAG_AVERAGING TMAG5273_AVG_32X // 400 results/s per sensor, the quietest: 22 uT rms on X/Y, 9 uT on Z
// The narrow range (+/-40 mT on an x1 part, +/-133 on an x2): the probe's magnet
// is meant to be weak. A sensor that reaches MAG_SATURATED of full scale on any
// axis is left out of that frame - a clipped reading is a wrong reading, and
// one wrong sensor spoils a fit the other seven could have made.
#define MAG_HIGH_RANGE false
#define MAG_SATURATED 0.95f
// Sensitivity: at 32x a sensor finishes a result every 2.4 ms, and a result
// nobody reads is noise reduction thrown away. So the sensors are read as often
// as the loop allows, and what consumers see is the AVERAGE of the reads in
// each frame period - the "averaging in the microcontroller" TI's datasheet
// points to. Two or three reads per frame is 1.5x less noise for nothing.
#define MAG_SAMPLE_PERIOD_US 2500  // read every sensor this often (if the loop keeps up)
#define MAG_FRAME_PERIOD_US 10000  // publish the average of those reads at 100 Hz
#define MAG_READ_TEMPERATURE false // die temperature: costs 1/4 of the conversion rate, see TMAG5273.h

// How long VCC is held low before the power-up sequence (the sensors' bypass
// capacitors have to discharge through the GPIO), and how long after VCC
// rises before the sensor is spoken to (270 us needed; the rest is the
// capacitor charging through the GPIO).
#define MAG_POWER_OFF_MS 100
#define MAG_POWER_ON_MS 5

// Frames averaged into the ambient baseline (Earth's field, sensor offsets and
// whatever steel is nearby) that is subtracted from every reading.
#define MAG_BASELINE_FRAMES 64

// The bench wiring is marginal at 400 kHz: now and then a transaction fails and
// leaves the I2C block wedged, after which nothing answers. So a sensor gets
// this many tries in the addressing walk, with the bus re-initialised between
// them, and a frame in which EVERY sensor fails re-initialises the bus at once.
#define MAG_ADDRESS_TRIES 3

// A sensor that fails this many reads in a row is treated as lost.
#define MAG_MAX_READ_FAILS 5

struct MagSensorPlace {
    int vccPin;        // GPIO that powers this sensor
    int gndPin;        // GPIO that is this sensor's ground
    float x, y, z;     // where it sits, mm, board frame
    float rotationDeg; // how the package is turned, see below
    bool underside;    // true if it is soldered to the bottom of the PCB
    float gain;        // its readings are multiplied by this (1 = as the datasheet says)
};

// rotationDeg: the direction the package's pin 1-2-3 side faces, as an angle
// counter-clockwise from board +x, seen from above the board. The same rule
// holds for an underside part (looking down through the board). At 0 on the
// top side, pin 1 -> pin 3 runs toward board +y.

// The bench array is hand wired on two SMD proto boards: four sensors along
// the far side, four along the near side about 44 mm away, all on the top side.
//
// The board frame: the array seen from above with the MCU board at the far
// side. Origin at sensor 4, +x toward sensor 7 (to the right), +y away from you
// (toward sensors 0-3), +z up.
//
// Positions, rotations and gains below are MEASURED, by tools/magcal from a
// recording of a magnet waved over the array (tools/recordings/2026-09-17-...).
// The one real length it needs is sensor 4 to sensor 7: 53.4 mm by caliper. On
// frames kept out of the calibration the dipole misfit is 2.4 %; with the first
// hand-entered table ("about 15 mm apart, 45 mm between rows") it was 10.7 %.
// Re-run magcal if a sensor is moved.
//
// How the rest was found, before there was a calibration:
//  - Order: sweeping a magnet counter-clockwise, `i` reported 3,2,1,0 then
//    4,5,6,7, so both rows run left to right.
//  - Rotation: far row 90, near row 270. `o` said so twice, and the sweep says
//    so without any fit: Bx flips the same way on both rows although the magnet
//    crosses them in opposite directions, which only oppositely turned packages
//    do. (The parts look alike because the near proto board is the far one
//    turned half way round.)
static const MagSensorPlace magSensorPlaces[ MAG_SENSOR_COUNT ] = {
    //  vcc            gnd             x      y      z     rotation  underside  gain
    { PIN_MAG_VCC_0, PIN_MAG_GND_A, 0.10f, 44.29f, 0.0f, 85.0f, false, 0.987f },
    { PIN_MAG_VCC_1, PIN_MAG_GND_A, 16.75f, 44.04f, 0.0f, 89.3f, false, 0.985f },
    { PIN_MAG_VCC_2, PIN_MAG_GND_A, 38.78f, 43.95f, 0.0f, 89.7f, false, 0.977f },
    { PIN_MAG_VCC_3, PIN_MAG_GND_A, 54.18f, 44.72f, 0.0f, 91.7f, false, 0.968f },
    { PIN_MAG_VCC_4, PIN_MAG_GND_B, 0.00f, 0.00f, 0.0f, 271.5f, false, 1.019f },
    { PIN_MAG_VCC_5, PIN_MAG_GND_B, 15.60f, 0.19f, 0.0f, 271.9f, false, 1.015f },
    { PIN_MAG_VCC_6, PIN_MAG_GND_B, 37.37f, 0.42f, 0.0f, 271.6f, false, 1.022f },
    { PIN_MAG_VCC_7, PIN_MAG_GND_B, 53.40f, 0.00f, 0.0f, 271.8f, false, 1.028f },
};

#endif // MAGARRAYCONFIG_H
