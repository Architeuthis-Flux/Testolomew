// SPDX-License-Identifier: MIT
#ifndef MAGLOCATOR_H
#define MAGLOCATOR_H
// ---------------------------------------------------------------------------
// Turns MagArray frames into "where is the probe": presence, magnet position,
// which way it points, and where the probe TIP is (the magnet sits some way up
// the probe's axis from the tip, and the fit gives that axis for free).
//
// Runs one dipole fit (MagFit) per new frame, warm-started from the last fix.
//
// Console: d = stream fixes as CSV, l = one-line summary of the latest fix,
// o = orientation check (which mounting of the sensors explains a held magnet),
// k = learn this magnet's strength and hold the fit to it, K = forget it,
// t = the magnet's centre is this far up the probe from its point (t12<Enter>),
// T = the magnet's angle to the shaft (T90<Enter> = a disc lying flat on it).
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "MagFit.h"
#include "Vec3.h"

// Weak magnets. One frame carries about 0.011 mT of noise per axis (measured), and a small
// magnet two sensor pitches away is not much more than that, so the fields
// are smoothed before anything looks at them - heavily when the strongest
// reading is weak, not at all once it passes MAGLOC_FAST_MT, so a strong magnet
// still tracks without lag. Smoothed noise is about 0.005 mT.
#define MAGLOC_FAST_MT 2.0f        // strongest reading at which smoothing is off
#define MAGLOC_SLOWEST_ALPHA 0.08f // smoothing at the weak end (~0.12 s time constant at 100 Hz)

// A magnet is "present" when the strongest smoothed reading passes this. A fix
// is then attempted once three sensors notice it at all - and noticing means
// MAGLOC_FAINT_MT, three times the smoothed noise, not the much higher
// MAGLOC_SEEN_MT of a sensor that sees it plainly. (Both counts are only a
// gate on spending the time. The fit itself always uses EVERY working sensor,
// however little it reads - "almost nothing here" pins a magnet down too - and
// what decides whether the answer is any good is its error bar.) So a weak
// magnet close to two sensors, with its neighbours down at 0.02 mT, locks.
#define MAGLOC_PRESENT_MT 0.06f
#define MAGLOC_SEEN_MT 0.04f
#define MAGLOC_FAINT_MT 0.015f
#define MAGLOC_MIN_SENSORS 3

// A fit from a cold start (no previous fix to start from) tries seven starting
// points and measures about 50 ms on the CH32H417; a tracking fit is two or
// three iterations. So while there is no fix, a cold start is only attempted
// this often, and the loop stays free for everything else.
#define MAGLOC_COLD_START_PERIOD_MS 200

// While tracking, this many frames in a row may fail to fit before the magnet
// is called lost and the (slow, paced) cold start takes over. Each failed frame
// has already cost a cold start inside the solver, so keep it small.
#define MAGLOC_MAX_MISSES 2

// Worst residual/signal ratio still called a fix (see magFitSolve). Far from
// the array a real magnet fits to a few percent. Close in it does not: the
// misfit is about 3 x (sensor position error / distance), so a magnet 10 mm
// from two sensors at once turns half a millimetre of table error into 15 %.
// A rough fix that says how rough it is (fix.misfit) beats no fix.
#define MAGLOC_MAX_MISFIT 0.40f

// A fix whose own error bar (all three axes, 1 sigma) is wider than this is not
// offered as a fix at all.
#define MAGLOC_MAX_ERROR_MM 15.0f

// The probe's magnet strength, mT*mm^3, if known: the fit then holds it fixed,
// which about halves the height noise (see magFitSolveKnownStrength). 0 = not
// known, fit it freely. The `k` console command measures it from live fixes
// and locks it until reset; put the number it prints here to make it stick.
#define MAGLOC_MAGNET_STRENGTH 0.0f

// Learning the strength: this many good free fixes, and what counts as good.
#define MAGLOC_LEARN_FIXES 150
#define MAGLOC_LEARN_MAX_MISFIT 0.10f
#define MAGLOC_LEARN_MIN_SENSORS 5

// Where the probe's POINT is, and how the magnet sits on the probe.
//
// The fit gives the magnet's pole direction. The magnet is mounted at some
// ANGLE to the shaft: 0 = magnetised along it (a coin on the end of the rod, a
// ring slid over it), 90 = across it (a disc lying flat on its side). A single
// dipole's field is symmetric about its pole, so what the fit can see of the
// probe's direction is only what the pole shows: with the magnet along the
// shaft that is everything; with it across, the shaft can be anything
// perpendicular to the pole, and the one taken is the most nearly vertical -
// right when the probe is held upright, off by (offset) x sin(lean) for a
// lean sideways to the pole, which it cannot see. In between, in between.
// The shaft is turned to point up (a probe is not held upside down), and the
// point is MAGLOC_TIP_OFFSET_MM down it from the magnet's centre. Both can be
// typed: t<mm> and T<degrees>.
#define MAGLOC_TIP_OFFSET_MM 0.0f
#define MAGLOC_MAGNET_ANGLE_DEG 0.0f

// Where the probe POINTS: carry the point on down the shaft to the breadboard's
// surface, this high above the sensors. With the point resting on the board
// that is the point itself; held above it, it is the hole the probe is aimed
// at rather than the one under it. 0 = not known: the sensor plane is used.
// The row calibration (`c`) sets it from the height the point rests at.
#define MAGLOC_BOARD_Z_MM 17.5f

// Display smoothing: fraction of each new fix blended in (1 = none).
#define MAGLOC_SMOOTHING 0.5f

struct MagProbeFix {
    bool present;    // something magnetic is near the array
    bool valid;      // ...and a dipole explains it; the fields below are good
    Vec3 magnet;     // magnet centre, mm, board frame (smoothed for display, MAGLOC_SMOOTHING)
    Vec3 tip;        // the probe's point, mm (smoothed the same way)
    Vec3 rawMagnet;  // magnet centre exactly as this frame's fit gave it - the one `sigma` is the error bar of
    Vec3 rawTip;     // ...and the point from that
    Vec3 pointer;    // where the shaft, carried on from the point, meets the board's surface (smoothed)
    Vec3 rawPointer; // ...from this frame's fit
    Vec3 axis;       // unit vector along the magnet, S -> N
    Vec3 shaft;      // unit vector up the probe's shaft, as far as the pole shows it (see MAGLOC_MAGNET_ANGLE_DEG)
    float tiltDeg;   // angle between the shaft and the board normal, 0..90
    float strength;  // |moment|, mT*mm^3 (constant for one magnet - a sanity check)
    float residual;  // RMS misfit, mT
    float misfit;    // residual / signal: 0.03 is a clean fix, 0.3 a rough one
    Vec3 sigma;      // 1-sigma error bar on the magnet position, mm, per axis (from the fit itself)
    float errorXyMm; // ...across the board: sqrt(sx^2 + sy^2)
    float errorMm;   // ...and in all three axes together
    float peakMt;    // strongest (smoothed) reading
    int seenBy;      // sensors reading above MAGLOC_SEEN_MT
    int faintBy;     // sensors reading between MAGLOC_FAINT_MT and MAGLOC_SEEN_MT
    uint32_t fitUs;  // how long the fit took
    uint32_t count;  // fixes since boot
};

class MagLocator : public Service {
  public:
    static MagLocator& getInstance( );

    MagLocator( const MagLocator& ) = delete;
    MagLocator& operator=( const MagLocator& ) = delete;

    void begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "MagLocator"; }
    ServicePriority getPriority( ) const override { return ServicePriority::HIGH; }
    uint32_t periodUs( ) const override { return 2000; }

    MagProbeFix fix = { };
    bool streaming = false;

    void printFix( Stream* out ) const;

    // The magnet strength the fit is held to (0 = free), and learning it.
    float knownStrength = MAGLOC_MAGNET_STRENGTH;
    void startLearningStrength( );
    void forgetStrength( );
    bool learning( ) const { return learnCount >= 0; }

    // How far the magnet's centre is up the shaft from the probe's point, and
    // the magnet's angle to the shaft.
    float tipOffsetMm = MAGLOC_TIP_OFFSET_MM;
    float magnetAngleDeg = MAGLOC_MAGNET_ANGLE_DEG;
    float boardZ = MAGLOC_BOARD_Z_MM; // the breadboard's surface, mm above the sensors

    // Bench tool for getting MagArrayConfig.h right. With a magnet held a
    // centimetre or two over the array (off-centre is best), fit it under every
    // combination of: the two rows swapped or not, each row's order along x
    // turned round or not, each row's package rotation (0/90/180/270), and top
    // side or underside. A wrong combination cannot be
    // explained by any dipole, so the misfit singles out the right one. Try it
    // at two or three magnet positions; the true combination wins every time.
    //
    // What it cannot decide: every answer appears four times - itself, its
    // mirror images (which put the parts on the other side of the board; the
    // mirror image of a magnet and sensors is magnetically just as valid), and
    // the same array with the frame turned half way round. Knowing which side
    // the parts are on and how one package is turned picks the line.
    void printOrientationCheck( Stream* out ) const;

  private:
    MagLocator( ) = default;

    MagFitResult result = { };
    Vec3 smooth[ MAGFIT_MAX_SENSORS ] = { };      // the fields the fit sees
    Vec3 recent[ MAGFIT_MAX_SENSORS ][ 2 ] = { }; // each sensor's last two frames, for the glitch filter
    uint32_t lastFrame = 0;
    bool haveSmoothed = false;
    uint32_t nextColdStartMs = 0;
    int misses = 0; // frames in a row that would not fit, while tracking
    uint32_t streamTick = 0;
    int learnCount = -1; // -1 = not learning, else good fixes collected so far
    float learned[ MAGLOC_LEARN_FIXES ];
    void learnFrom( float strength );

    Vec3 pointerOf( Vec3 tip, Vec3 shaft ) const;
    void printFixCsv( Stream* out ) const;
};

extern MagLocator& magLocator;

#endif // MAGLOCATOR_H
