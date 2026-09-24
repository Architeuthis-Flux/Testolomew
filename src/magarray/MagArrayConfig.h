// SPDX-License-Identifier: MIT
#ifndef MAGARRAYCONFIG_H
#define MAGARRAYCONFIG_H
// ---------------------------------------------------------------------------
// The magnetometer array as built: how many sensors, of which kinds, on
// which bus, which pins power them, where each one sits and which way it
// faces. Pins come from BoardPins.h; everything about the array's geometry
// is here.
//
// BOARD FRAME (what every position and field downstream is in): millimetres,
// origin at sensor 0, +x along the row of four, +y toward the second row, +z
// up out of the board toward the probe.
// ---------------------------------------------------------------------------
#include <math.h>

#include "BoardPins.h"
#include "MMC56x3.h"
#include "TMAG5273.h"

// The sensor types the array can carry, mixed as the table below says. Each
// slot has its own driver, bus, address, range and noise; the array turns
// every one into a field in mT in the board frame, and the fit uses them
// all together, each weighted by how quiet its type is (MAG_WEIGHT_*).
enum MagSensorType {
    MAG_TMAG5273 = 0, // TI Hall sensor: +/-40 mT, ~0.011 mT rms a frame, a VOLATILE address (its supply switched at boot)
    MAG_MMC56X3 = 1   // Memsic AMR compass (MMC5603NJ / MMC5633NJL): +/-3 mT, ~0.0003 mT a frame, FIXED address 0x30 (one per bus)
};

#define MAG_SENSOR_COUNT 9 // 8 TMAG5273 (the 4x2 array) + 1 MMC5633NJL (2026-09-21: one, at the centre, on the back)
#define MAG_MAX_BUSES 4     // the chip has four I2C blocks

// TMAG5273 sensor i is given address MAG_BASE_ADDRESS + i. Anything clear of
// the four factory addresses (0x22, 0x35, 0x44, 0x78) and of the fixed
// parts (0x30) works.
#define MAG_BASE_ADDRESS 0x10

#define MAG_I2C_HZ 400000
// The sensor round-robin runs on the other core (MagSampler.h) and this
// core only decodes what it left in shared RAM. 0 = read the bus here, as
// before (the fallback if loop1() gives trouble).
#define MAG_SAMPLER_CORE1 1
#define MAG_AVERAGING TMAG5273_AVG_32X // 400 results/s per sensor, the quietest: 22 uT rms on X/Y, 9 uT on Z
// The narrow range (+/-40 mT on an x1 part, +/-133 on an x2): the probe's magnet
// is meant to be weak. A sensor that reaches MAG_SATURATED of full scale on any
// axis is left out of that frame - a clipped reading is a wrong reading, and
// one wrong sensor spoils a fit the others could have made. The MMC56x3's
// +/-3 mT is reached by the present magnet (4232 mT*mm^3) about 14 mm away
// on its axis, 11 mm beside it: an MMC under the probe drops out while the
// magnet is that close and is back the frame after (its SET/RESET heals it).
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

// The MMC56x3s: continuous measurement at this rate with this decimation
// filter (MMC56x3.h). 3.5 ms a measurement is 2 mG rms and reaches 150 Hz:
// one or two results per 10 ms frame. (2.0 ms / 255 Hz is 3 mG and no
// quieter per frame; 6.6 ms / 75 Hz leaves some frames without a result.)
#define MAG_MMC_BANDWIDTH MMC56X3_BW_3_5MS
#define MAG_MMC_ODR_HZ 150
// Whether the MMC56x3 takes part at boot (the menu's "sensors / use MMC",
// saved; the console's `:mmc on|off`). Off: it is read and shown (`m`) but
// is out of the fit, out of presence and the counts, and the far regime
// with it - the array is the eight TMAG5273s as it was before 2026-09-21.
// Kevin, 2026-09-22 late: "the readings have been 100x sketchier since we
// added that one" - off by default until its calibration close in (the
// lumped gain 12 % off) and its bus (a missed frame or two every second)
// are sorted.
#define MAG_USE_MMC_AT_BOOT false

// How much the fit trusts each sensor: weight = MAG_WEIGHT_REFERENCE_MT /
// (the type's noise a frame), capped at MAG_WEIGHT_CAP - so a TMAG5273 is
// 1 and an MMC56x3, 30-50x quieter, would be 30-50 uncapped. It is capped
// low because noise is not the error that matters: what the dipole model
// fails to explain is 0.03-0.1 mT per axis on any sensor (soft iron near
// the array; docs/wireless-probe-sensing.md), and an MMC whose position,
// rotation and gain have not been through tools/magcal carries more
// systematic error than that. At the cap two MMCs count like eight of the
// Hall sensors; raise it once magcal has placed them (the console's `w`
// tries a value without a rebuild). 0 = every sensor equal.
#define MAG_WEIGHT_REFERENCE_MT 0.011f
// ...and its Z axis, twice as quiet (the datasheet: 11 against 22 uT at 32x;
// the bench 0.006 against 0.012 mT a frame, 2026-09-21): the fit weighs
// every sensor's Z rows by the ratio (magFitSetAxisWeights, MagLocator::begin).
#define MAG_NOISE_Z_MT 0.006f
#define MAG_WEIGHT_CAP 40.0f // the most a quiet type may count for (an MMC56x3 where it reads noise: ~37 by its noise; the console's w changes it)
// The share of a reading the dipole model does not explain, whatever the
// sensor: the dipole approximation, the sensor's place and gain in the
// table, soft iron near the board. The calibration recording's misfit
// (tools/magcal, 2026-09-21: 5.7 % with all nine sensors trusted). It is
// what stops a quiet sensor outvoting everyone close in, where its reading
// is big and this error with it (magSensorFrameWeight).
#define MAG_MODEL_ERROR 0.06f

// How long VCC is held low before the power-up sequence (the sensors' bypass
// capacitors have to discharge through the GPIO), and how long after VCC
// rises before the sensor is spoken to (270 us needed; the rest is the
// capacitor charging through the GPIO).
#define MAG_POWER_OFF_MS 100
#define MAG_POWER_ON_MS 5

// Frames averaged into the ambient baseline (Earth's field, sensor offsets and
// whatever steel is nearby) that is subtracted from every reading.
#define MAG_BASELINE_FRAMES 64

// The zero the board boots with when none is saved (a first boot, or the
// settings wiped by a chip erase), instead of zeroing blind: a zero taken with
// the probe lying anywhere near the board eats its field, and every fit after
// that is wrong (2026-09-19: 0.15 mT off on four sensors, misfit 20-50 %,
// height 5 mm out, the LEDs white). This one was taken with the probe held
// well away, and matches the previous night's within 0.02 mT eight hours
// apart; the slow drift tracking follows the rest. Board frame, mT, per
// sensor, as `s` prints the zero= lines. `z` zeroes afresh. It covers the
// first MAG_ZERO_AT_BOOT_COUNT sensors; the rest (the MMCs, added later)
// zero themselves from the first frames after boot - with the probe away.
#define MAG_ZERO_AT_BOOT_COUNT 8
#define MAG_ZERO_AT_BOOT \
    {                    \
        { -0.1597f, -0.0650f, -0.0341f }, { 0.0287f, 0.0655f, -0.1608f }, { -0.0366f, -0.0862f, -0.0319f }, { -0.1054f, -0.1093f, -0.0132f }, { -0.0408f, 0.0719f, -0.0923f }, { -0.0034f, -0.0055f, -0.0447f }, { -0.0827f, -0.0397f, -0.0481f }, { -0.0121f, 0.0653f, -0.1025f } }

// The bench wiring is marginal at 400 kHz: now and then a transaction fails and
// leaves the I2C block wedged, after which nothing answers. So a sensor gets
// this many tries in the addressing walk, with the bus re-initialised between
// them, and a frame in which EVERY sensor fails re-initialises the bus at once.
#define MAG_ADDRESS_TRIES 3

// A sensor that fails this many reads in a row is treated as lost.
#define MAG_MAX_READ_FAILS 5

struct MagSensorPlace {
    MagSensorType type;
    int vccPin;         // GPIO that powers this sensor; -1 = powered permanently (a fixed-address part needs no switching)
    int gndPin;         // GPIO that is this sensor's ground; -1 = wired to ground
    int sclPin, sdaPin; // the bus it is on (a pair the silicon offers; BoardPins.h). Sensors sharing a pair share the bus
    uint8_t address;    // fixed-address parts: where it answers. TMAG5273: 0 (the walk assigns MAG_BASE_ADDRESS + i)
    float x, y, z;      // where it sits, mm, board frame
    float rotationDeg;  // how the package is turned, see below
    bool underside;     // true if its top faces down (soldered to the bottom of the PCB): what magcal finds, see the MMC below
    float gain;         // its readings are multiplied by this (1 = as the datasheet says)
    bool calibrated;    // its place, rotation and gain have been measured (tools/magcal): false = read and shown, but kept OUT OF THE FIT
};

// A sensor whose rotation is a guess is worse than none in the fit: a
// reading turned by an unknown angle contradicts the others with all the
// weight of a quiet part behind it (2026-09-21: the MMC past row 30, at
// rotation 0 unmeasured and weight 2, made fixes worse at its end of the
// board). So `calibrated` false leaves a sensor out of the fit until magcal
// has been through it; it is still read, zeroed, shown by `m` and the
// scene, and streamed by `f` for that recording.

// rotationDeg: the direction the package's +x axis faces (the TMAG5273's
// pin 1-2-3 side; the MMC56x3's pin-1 column, MMC56x3.h), as an angle
// counter-clockwise from board +x, seen from above the board. The same rule
// holds for an underside part (looking down through the board). At 0 on the
// top side, a TMAG5273's pin 1 -> pin 3 runs toward board +y.

// The bench array is hand wired on two SMD proto boards: four sensors along
// the far side, four along the near side about 44 mm away, all on the top side.
//
// The board frame: the array seen from above with the MCU board at the far
// side. Origin at sensor 4, +x toward sensor 7 (to the right), +y away from you
// (toward sensors 0-3), +z up.
//
// Positions, rotations and gains of the TMAGs below are MEASURED, by
// tools/magcal from a recording of a magnet waved over the array
// (tools/recordings/2026-09-17-...). The one real length it needs is sensor
// 4 to sensor 7: 53.4 mm by caliper. On frames kept out of the calibration
// the dipole misfit is 2.4 %; with the first hand-entered table ("about 15
// mm apart, 45 mm between rows") it was 10.7 %. Re-run magcal if a sensor
// is moved.
//
// How the rest was found, before there was a calibration:
//  - Order: sweeping a magnet counter-clockwise, `i` reported 3,2,1,0 then
//    4,5,6,7, so both rows run left to right.
//  - Rotation: far row 90, near row 270. `o` said so twice, and the sweep says
//    so without any fit: Bx flips the same way on both rows although the magnet
//    crosses them in opposite directions, which only oppositely turned packages
//    do. (The parts look alike because the near proto board is the far one
//    turned half way round.)
//
// The MMC5633NJL (2026-09-21): ONE, on the second bus (BoardPins.h: the
// one that has worked; the TMAG-bus hookup never answered), at its fixed
// 0x30, powered from 3.3 V for good, at the centre of the board and ON THE
// BACK, 'about 2 mm' below the plane of the TMAG dies (Kevin) - MEASURED
// at 3.77 below by magcal on 2026-09-23 (its height searched with the
// rest: z = -3.77, gain 0.936; with z held at -2 the gain came out 1.129,
// a height error in a gain's clothing, and its readings cost 55 % more).
// Its place, rotation, gain and face are MEASURED, by tools/magcal
// (`only=8`) from tools/recordings/2026-09-21-mmc-centre-calibration.txt,
// the probe's magnet run over the board. underside = false because the
// recording says its +z reads UP in the board frame (the other face costs
// 40x more): under the datasheet's z-out-of-the-top frame that is a part
// whose top faces up although it hangs under the board - the module sits
// face up under the board, or Memsic's z arrow is the other way round; the
// fit does not care which. The gain of 1.13 is not a real gain (its scale
// is a datasheet constant): with z held at -2 it soaks up a height error,
// harmless as a lumped constant at the ranges recorded. The morning's
// two-sensor attempt (one past row 1, one past row 30) is in the docs.
static const MagSensorPlace magSensorPlaces[ MAG_SENSOR_COUNT ] = {
    //  type          vcc            gnd            scl           sda           addr  x       y       z     rotation  underside  gain    calibrated
    { MAG_TMAG5273, PIN_MAG_VCC_0, PIN_MAG_GND_A, PIN_MAG_SCL, PIN_MAG_SDA, 0x00, 0.10f, 44.29f, 0.0f, 85.0f, false, 0.987f, true },
    { MAG_TMAG5273, PIN_MAG_VCC_1, PIN_MAG_GND_A, PIN_MAG_SCL, PIN_MAG_SDA, 0x00, 16.75f, 44.04f, 0.0f, 89.3f, false, 0.985f, true },
    { MAG_TMAG5273, PIN_MAG_VCC_2, PIN_MAG_GND_A, PIN_MAG_SCL, PIN_MAG_SDA, 0x00, 38.78f, 43.95f, 0.0f, 89.7f, false, 0.977f, true },
    { MAG_TMAG5273, PIN_MAG_VCC_3, PIN_MAG_GND_A, PIN_MAG_SCL, PIN_MAG_SDA, 0x00, 54.18f, 44.72f, 0.0f, 91.7f, false, 0.968f, true },
    { MAG_TMAG5273, PIN_MAG_VCC_4, PIN_MAG_GND_B, PIN_MAG_SCL, PIN_MAG_SDA, 0x00, 0.00f, 0.00f, 0.0f, 271.5f, false, 1.019f, true },
    { MAG_TMAG5273, PIN_MAG_VCC_5, PIN_MAG_GND_B, PIN_MAG_SCL, PIN_MAG_SDA, 0x00, 15.60f, 0.19f, 0.0f, 271.9f, false, 1.015f, true },
    { MAG_TMAG5273, PIN_MAG_VCC_6, PIN_MAG_GND_B, PIN_MAG_SCL, PIN_MAG_SDA, 0x00, 37.37f, 0.42f, 0.0f, 271.6f, false, 1.022f, true },
    { MAG_TMAG5273, PIN_MAG_VCC_7, PIN_MAG_GND_B, PIN_MAG_SCL, PIN_MAG_SDA, 0x00, 53.40f, 0.00f, 0.0f, 271.8f, false, 1.028f, true },
    { MAG_MMC56X3, -1, -1, PIN_MAG2_SCL, PIN_MAG2_SDA, MMC56X3_ADDRESS, 27.52f, 25.19f, -3.77f, 260.6f, false, 0.936f, true }, // the centre, hanging under the board: magcal 2026-09-23 with its HEIGHT searched on the 09-21 recording (cost 7.75 against 12.05 with z held at -2 and the gain 1.129 soaking up the height error)
};

// What each type is, for the array and its status line.
static inline const char* magSensorTypeName( MagSensorType type ) {
    return type == MAG_MMC56X3 ? "MMC56x3" : "TMAG5273";
}
// Its noise per frame (mT rms per axis, the measured TMAG figure and the
// MMC's datasheet figure at MAG_MMC_BANDWIDTH with 1-2 results a frame).
static inline float magSensorTypeNoiseMt( MagSensorType type ) {
    return type == MAG_MMC56X3 ? 0.0003f : MAG_WEIGHT_REFERENCE_MT; // the MMC's is the bench's (2026-09-21: 0.3 uT rms a frame per axis; the datasheet's 0.2 until 2026-09-23)
}
// How far a sensor's zero may be off, as the fit's acceptance (magFitChi)
// doubts it: at boot a saved or compiled zero is doubted by the first pair
// (the bench's post-boot zeros are 0.02-0.045 mT stale); the doubt decays
// with the follow's time constant while the offset filter follows a reading
// of nothing, never under the floor (the second pair); and while a zero is
// HELD under a present magnet it grows at the drift rate (the third: a
// parked probe's zeros drift, and a fit the chi rejected for that could
// never be followed back - the zeros follow only with nothing present).
// ASSUMPTION (2026-09-24): the drift is knobs.md's bench note (0.0002 mT/s,
// right after power-up) halved; section 8's absent and rest captures set it.
#define MAG_ZERO_UNCERTAINTY_TMAG_MT 0.015f
#define MAG_ZERO_UNCERTAINTY_MMC_MT 0.003f
#define MAG_ZERO_DOUBT_FLOOR_TMAG_MT 0.001f
#define MAG_ZERO_DOUBT_FLOOR_MMC_MT 0.0005f
#define MAG_ZERO_DRIFT_MT_PER_S 0.0001f
// ...and never past this by drift: doubted more, a stale-zero pattern the
// size of a far probe's passes the fit as a phantom (a probe parked for an
// hour, then lifted: the zeros it left are not a magnet).
#define MAG_ZERO_DOUBT_MAX_MT 0.005f
static inline float magSensorTypeZeroMt( MagSensorType type ) {
    return type == MAG_MMC56X3 ? MAG_ZERO_UNCERTAINTY_MMC_MT : MAG_ZERO_UNCERTAINTY_TMAG_MT;
}
static inline float magSensorTypeZeroFloorMt( MagSensorType type ) {
    return type == MAG_MMC56X3 ? MAG_ZERO_DOUBT_FLOOR_MMC_MT : MAG_ZERO_DOUBT_FLOOR_TMAG_MT;
}
// Each sensor's zero is a state with a doubt (MagOffsetFilter.h; the locator
// sets the live baseline from it every frame, MagLocator::keepZeros). It
// FOLLOWS the reading, with this time constant, whenever the reading is not
// the magnet's: nothing present (and none lately), or this sensor quiet
// (MAGLOC_QUIET_FRACTION) - the drift (20 s, as the slow drift rule was). A reading no dipole
// explains is HELD for MAG_OFFSET_HOLD_S first, then followed at the same
// rate: a probe in a hand does not hold still for two minutes, a zero
// error or a screwdriver does - and time is the only evidence the array has
// against a steady phantom until the fit's acceptance over every sensor's
// own doubt (Phase 3 step 4) can disown it. Under an accepted fix that at
// least two sensors make,
// the zero holds: a hover is a magnet, not drift (a one-sensor far fix
// explains nothing - three numbers, five unknowns - so it runs the clock,
// and the far glow of a steady probe at the edge of reach goes after the
// hold, as it did). ASSUMPTION (2026-09-23): the simulator's scenes; the
// bench protocol (docs/knobs.md section 8) re-tunes both.
// ...and for MAG_OFFSET_HOLDOFF_S after a magnet was last present, a reading
// that is not quiet is not followed even with nothing present: a probe
// at the edge of reach flickers in and out of presence (one sensor reads it
// plainly), and followed on every absent frame its field was eaten in a
// cascade - less field, less presence, more following (2026-09-23, the sim:
// 0.027 mT into the end sensors in a minute, the next row-15 fix a row off).
// ...and the zero audit's gain trim on a sensor never goes further from 1
// than this, either way (MagArray::applyGainTrim): a row that wants magcal.
#define MAG_GAIN_TRIM_MAX 2.0f
#define MAG_OFFSET_FOLLOW_S 20.0f
#define MAG_OFFSET_HOLD_S 30.0f // (120 until 2026-09-24: Kevin's bench read 0.05 mT with the probe away and waited on it; a pattern few sensors see plainly that no dipole explains is drift after half a minute - what the fit refuses only by the footprint, or three TMAGs read plainly, or was a fix within the far hold, is held the far hold instead)
// A fix that ONE sensor makes (the MMC's far regime, three numbers for five
// unknowns) is real or the tail of a zero error at that sensor, and at the
// array's reach the others cannot say (their share of a probe 90 mm up is
// at their noise): its zero is held this long, then follows. A fix two or
// more sensors make holds for as long as it lasts.
#define MAG_OFFSET_FAR_HOLD_S 120.0f
#define MAG_OFFSET_HOLDOFF_S 10.0f
static inline float magSensorFrameWeight( MagSensorType type, float fieldMt, float cap ) {
    if ( cap <= 0.0f ) {
        return 1.0f; // the console's w0: every sensor equal
    }
    float model = MAG_MODEL_ERROR * fieldMt;
    float noise = magSensorTypeNoiseMt( type );
    float w = sqrtf( MAG_WEIGHT_REFERENCE_MT * MAG_WEIGHT_REFERENCE_MT + model * model ) / sqrtf( noise * noise + model * model );
    if ( w > cap ) {
        w = cap;
    }
    return w < 1.0f ? 1.0f : w; // nothing is trusted less than the reference type
}
// Whether the part's own +z points INTO its top (the TMAG5273: a north
// pole approaching the top reads positive) or out of it (the MMC56x3).
static inline bool magSensorTypeZIntoTop( MagSensorType type ) {
    return type != MAG_MMC56X3;
}

#endif // MAGARRAYCONFIG_H
