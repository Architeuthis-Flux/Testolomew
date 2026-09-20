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
// T = the magnet's angle to the shaft (T90<Enter> = a disc lying flat on it),
// u = cursor under the tip / where it points, S = surface height, g = tracker on/off.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "MagFit.h"
#include "MagTracker.h"
#include "Vec3.h"

// Weak magnets. One frame carries about 0.011 mT of noise per axis (measured), and a small
// magnet two sensor pitches away is not much more than that, so the fields
// are smoothed before anything looks at them - heavily when the strongest
// reading is weak, not at all once it passes MAGLOC_FAST_MT, so a strong magnet
// still tracks without lag. Smoothed noise is about 0.005 mT.
#define MAGLOC_FAST_MT 2.0f        // strongest reading at which smoothing is off
#define MAGLOC_SLOWEST_ALPHA 0.08f // smoothing at the weak end (~0.12 s time constant at 100 Hz)
// ...and off again while the probe MOVES: smoothed fields lag, and a fix
// made of lagging fields is precise about where the probe was 100 ms ago
// (8 mm behind a hand at 80 mm/s, in simulation, with a 0.7 mm bar). The
// tracker knows the speed, so the smoothing backs off with it: alpha is at
// least speed x this (1 = none at 50 mm/s); at rest, where noise matters
// and lag does not, the weak-signal smoothing is back.
#define MAGLOC_ALPHA_PER_MM_S 0.02f
// The speed for that rule is the larger of two, each less what jitter alone
// would make of it: the distance the track moved over the last
// MAGLOC_SPEED_WINDOW frames, over that time, and the filter's own velocity.
// That velocity is also the fix jitter over one 10 ms frame - 10-20 mm/s on
// a probe at rest near the board (measured 2026-09-19), 50 on one far up -
// and taken as the speed with a fixed 5 mm/s off it opened the smoothing on
// any resting probe (a far one to alpha 0.6 in pencil.cpp: the smoothing
// defeated by its own noise, 0.9 mm of jitter at rest). Jitter's share of
// each speed is MAGLOC_SPEED_JITTER_K x the fix's own error bar over the
// time; 0.4 keeps a far probe at rest to 0.13 mm (from 0.88) for 10-20 ms
// more lag while it moves far up, and costs nothing near the board (track
// lag 10 ms as before; the field rule has the smoothing off there anyway).
// Higher K buys a little more rest and more lag (0.7: 0.105 mm, +10 ms). A
// moving/at-rest latch with hysteresis was tried and fed on itself the
// same way the old rule did.
#define MAGLOC_SPEED_WINDOW 10
#define MAGLOC_SPEED_JITTER_K 0.4f

// A magnet is "present" when the strongest smoothed reading passes this. A fix
// is then attempted once three sensors notice it at all - and noticing means
// MAGLOC_FAINT_MT, three times the smoothed noise, not the much higher
// MAGLOC_SEEN_MT of a sensor that sees it plainly. (Both counts are only a
// gate on spending the time. The fit itself always uses EVERY working sensor,
// however little it reads - "almost nothing here" pins a magnet down too - and
// what decides whether the answer is any good is its error bar.) So a weak
// magnet close to two sensors, with its neighbours down at 0.02 mT, locks.
#define MAGLOC_PRESENT_MT 0.04f // the menu's "presence" lever starts here
#define MAGLOC_PRESENT_LOW 0.5f // ...and once present it stays so down to this fraction of it (hysteresis)
#define MAGLOC_SEEN_MT 0.04f
#define MAGLOC_FAINT_MT 0.015f
#define MAGLOC_MIN_SENSORS 3

// A fit from a cold start (no previous fix to start from) tries seven starting
// points and measures about 50 ms on the CH32H417; a tracking fit is two or
// three iterations. So while there is no fix, a cold start is only attempted
// this often, and the loop stays free for everything else.
#define MAGLOC_COLD_START_PERIOD_MS 200
// ...and since a cold start runs in slices (magFitSolveStep: the lattice,
// its refinement, then up to seven seeds), the slices are spread evenly,
// one every MAGLOC_COLD_SLICE_MS, with no pause between one cold start and
// the next: the same work as a 25-38 ms burst every 200 ms, but as a steady
// 40 Hz pattern the supply (and the LCD's backlight, 2026-09-20) does not
// show, and never more than one slice's hold of the loop at a time.
#define MAGLOC_COLD_SLICE_MS 25
// The steady fit load (the tracker page's "fit load": steady, saved): every
// frame with a magnet present runs exactly this many refinement iterations
// - converged or not while tracking, one resumable slice of the cold start
// while hunting - so the V5F's work, and its supply current, is the same
// every 10 ms rather than a burst now and then. Kevin's idea, 2026-09-20.
#define MAGLOC_STEADY_ITERATIONS 2 // the default of "fit iters" (~0.85 ms each on the CH32H417). tools/hostsim/pencil.cpp: 2 a frame tracks exactly as the natural caps do (error, lag and jitter to the last digit); at 1 it loses 0.1 % of frames. More only makes the pulse the supply shows bigger (2026-09-20).

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
// surface, this high above the sensors, and no further (the tracker's cursor,
// MagTracker.h). With the point resting on the board that is the point itself;
// held above it, it is the hole the probe is aimed at rather than the one
// under it. 0 = not known: the sensor plane is used. The row calibration
// (`c`) sets it from the height the point rests at; `S` types it. On the
// bench the breadboard sits 17.5 mm over the sensors; on V6 the surface is
// 7.1 mm above the base PCB (plus wherever the sensors sit below that).
#define MAGLOC_BOARD_Z_MM 17.5f
// The surface is also LEARNED from where the point bottoms out: each second
// the lowest point of the tracked fixes is noted, and when the last
// MAGLOC_FLOOR_WINDOWS of them agree within MAGLOC_FLOOR_AGREE_MM and lie
// MAGLOC_FLOOR_STEP_MM or more below the surface as set, the surface comes
// down to them (2026-09-19: the setting was 17.5 mm and the point touched
// at 10.8, so the point was "6.7 mm under the board" - always touching,
// never pointing, and a tilt moved it a row). It only ever comes down by
// itself: a hover is never taken for the board. S sets it by hand.
#define MAGLOC_FLOOR_WINDOW_MS 1000
#define MAGLOC_FLOOR_WINDOWS 3
#define MAGLOC_FLOOR_AGREE_MM 0.7f
#define MAGLOC_FLOOR_STEP_MM 0.5f
#define MAGLOC_FLOOR_MISFIT 0.10f // a fix worse than this says nothing about the floor

// A fit is offered to the tracker as a ROUGH fix ("somewhere about here",
// the wide glow on the LEDs, no row counted) rather than a proper one when
// its error bar is wider than MAGLOC_ROUGH_ABOVE_MM - a probe far up or off
// the edge - and a fit that is not good enough to be a fix at all is still
// offered as rough if its misfit and bar are under the MAX limits, with the
// bar never claimed narrower than the lattice search resolves.
#define MAGLOC_ROUGH_ABOVE_MM 5.0f
#define MAGLOC_ROUGH_MAX_MISFIT 0.7f
#define MAGLOC_ROUGH_MAX_ERROR_MM 60.0f
#define MAGLOC_ROUGH_MIN_SIGMA_MM 3.0f

// The baseline check. A baseline taken with the probe lying on the board
// subtracts the magnet's own field from every reading after it, and nothing
// fits until `z` is pressed with the probe away - which after a reflash is
// easy to forget. Sensor offsets (up to a mT, but random from sensor to
// sensor) and the earth's field (the same at every sensor) do not look like
// a dipole; a magnet does. So each new baseline, less its mean across the
// sensors, is fitted as a dipole, and if one explains it (misfit under the
// limit) with at least this much strength, the baseline is called polluted:
// a warning is printed and shown, and it is retaken a few times.
#define MAGLOC_BASELINE_MAGNET_MISFIT 0.3f
#define MAGLOC_BASELINE_MAGNET_STRENGTH 300.0f // mT*mm^3 (a probe magnet is thousands)
#define MAGLOC_BASELINE_RETAKES 3
#define MAGLOC_BASELINE_RETAKE_MS 2000

// While nothing at all is read (the strongest smoothed reading under
// MAGLOC_FAINT_MT), the baseline follows the readings with this fraction per
// frame (0.0005 = a 20 s time constant at 100 Hz), so the slow drift of the
// ambient field and the sensors' offsets (0.1 mT in ten minutes was seen on
// the bench, 0.0002 mT/s, which a 20 s constant follows with 0.003 mT of
// lag) does not become a phantom magnet - and not at all within
// MAGLOC_DRIFT_HOLDOFF_MS of a magnet having been present: a probe at the
// edge of reach dips under the faint level as it wavers, and a 5 s constant
// was eating its field before it came back (2026-09-18: "fades out at the
// outer range"). A magnet that reads more than the faint level is never
// absorbed by this rule.
#define MAGLOC_BASELINE_DRIFT 0.0005f
#define MAGLOC_DRIFT_HOLDOFF_MS 10000
// ...and also while something has been "present" for this long without ever
// fitting as a magnet (rough fixes only): that is drift or a stuck reading,
// not a probe - a probe fits. (Overnight on the bench, drift crossed the
// 0.04 mT presence line now and then and made 2,293 rough fixes in six
// hours with no magnet in the room.) Two minutes: long enough to hover at
// the edge, short enough that a phantom is gone before anyone looks.
#define MAGLOC_ROUGH_ONLY_ABSORB_MS 120000

// A reference fix: the probe resting in row 35, hole 3, at 41 degrees, as
// fitted on 2026-09-18 with a zero taken while it was away (medians of two
// minutes of fixes, jitter 0.1 mm). Y1 takes this magnet out of a fresh
// zero taken with the probe still there.
#define MAGLOC_REFERENCE_X 1.14f
#define MAGLOC_REFERENCE_Y 9.35f
#define MAGLOC_REFERENCE_Z 10.82f
#define MAGLOC_REFERENCE_AX -0.257f
#define MAGLOC_REFERENCE_AY 0.606f
#define MAGLOC_REFERENCE_AZ -0.753f
#define MAGLOC_REFERENCE_STRENGTH 4232.0f

// Display smoothing: fraction of each new fix blended in (1 = none).
#define MAGLOC_SMOOTHING 0.5f

struct MagProbeFix {
    bool present;    // something magnetic is near the array
    bool valid;      // ...and a dipole explains it; the fields below are good
    bool rough;      // ...but only roughly: the error bar is wider than MAGLOC_ROUGH_ABOVE_MM (the tracker treats it as a rough fix)
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

    // The probe over time: the fixes above filtered, gated and carried
    // through gaps, and the cursor on the board's surface (see MagTracker.h).
    // Console: u = cursor under the tip / where it points, S = the surface
    // height (S17<Enter>), g = tracker on/off (off = raw fixes, for comparing).
    MagTrack track = { };

    // The baseline check (see MAGLOC_BASELINE_*): true while the baseline in
    // use looks like it has a magnet in it.
    bool baselinePolluted = false;
    float baselineMagnetMisfit = 0.0f, baselineMagnetStrength = 0.0f;

    void printFix( Stream* out ) const;

    // A simulated probe (the :probe verb, the host simulator): the tracker
    // is fed this magnet position and shaft instead of a fit - the array and
    // its sampler keep running, the fit is skipped - and `fix` is filled as
    // a fit would fill it, so everything downstream (rows, LEDs, paint, the
    // scene with its SIM mark) sees a probe that is exactly where it is said
    // to be. ms 0 = until simProbeOff().
    struct ProbeSim {
        bool on;
        uint32_t untilMs; // 0 = no end
        Vec3 position;    // the magnet's centre
        Vec3 shaft;       // unit, up the probe
        float sigmaMm;
        bool rough;
    };
    ProbeSim sim = { };
    bool fitHeld = false;   // :load fit off - the frames flow, nothing is fitted (the fit is the V5F's heaviest work)
    bool steadyFit = false;                          // the fit's work the same every frame instead of bursts (the tracker page's "fit load")
    float steadyIterations = MAGLOC_STEADY_ITERATIONS; // ...and how much: refinement iterations a frame ("fit iters", saved)
    uint32_t coldStarts = 0, coldStartUs = 0, coldSliceMaxUs = 0; // cold starts since boot; the last one's cost in all, and its longest slice (one per frame)
    void simProbeSet( Vec3 position, Vec3 shaft, float sigmaMm, bool rough, uint32_t ms );
    void simProbeOff( );
    bool simProbeActive( ) const { return sim.on; }

    // The magnet strength the fit is held to (0 = free), and learning it.
    float knownStrength = MAGLOC_MAGNET_STRENGTH;
    void startLearningStrength( );
    void forgetStrength( );
    bool learning( ) const { return learnCount >= 0; }

    // How far the magnet's centre is up the shaft from the probe's point, and
    // the magnet's angle to the shaft.
    float tipOffsetMm = MAGLOC_TIP_OFFSET_MM;
    float magnetAngleDeg = MAGLOC_MAGNET_ANGLE_DEG;
    float boardZ = MAGLOC_BOARD_Z_MM; // the breadboard's surface, mm above the sensors (the menu's "surface", saved; learned too, see MAGLOC_FLOOR_*)
    float floorWindowMinZ = 1e9f;     // the lowest point this second
    uint32_t floorWindowStartMs = 0;
    float floorMins[ MAGLOC_FLOOR_WINDOWS ]; // the last seconds' lowest points
    int floorMinCount = 0;
    uint32_t surfaceLearned = 0; // times the surface came down to the floor
    void learnFloor( uint32_t nowMs );
    float presentMt = MAGLOC_PRESENT_MT; // the strongest smoothed reading that counts as a magnet (menu: presence)
    // The last good fix (a sharp one, misfit under MAGLOC_LEARN_MAX_MISFIT):
    // kept in the settings, so that after a reboot with the probe lying where
    // it was, `Y` can take that magnet back out of the baseline the boot
    // zeroed it into (unpolluteBaseline). Starts as the fix measured on
    // 2026-09-18 with the probe resting in row 35, hole 3, at 41 degrees.
    Vec3 lastGoodPosition = { MAGLOC_REFERENCE_X, MAGLOC_REFERENCE_Y, MAGLOC_REFERENCE_Z };
    Vec3 lastGoodAxis = { MAGLOC_REFERENCE_AX, MAGLOC_REFERENCE_AY, MAGLOC_REFERENCE_AZ };
    float lastGoodStrength = MAGLOC_REFERENCE_STRENGTH;
    bool haveLastGood = true;
    bool baselineCorrected = false; // Y has been applied to the zero in use (a new zero clears it)
    void unpolluteBaseline( Stream* out );

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

    // The field smoothing as it stands, for the reports and the pencil bench
    // (tools/hostsim/pencil.cpp): the track's speed over the last
    // MAGLOC_SPEED_WINDOW frames, the alpha last applied (1 = none), and the
    // jitter lever.
    float smoothSpeed = 0.0f;
    float smoothAlpha = 1.0f;
    float speedJitterK = MAGLOC_SPEED_JITTER_K;

  private:
    MagLocator( ) = default;

    MagFitResult result = { };
    Vec3 smooth[ MAGFIT_MAX_SENSORS ] = { };      // the fields the fit sees
    Vec3 recent[ MAGFIT_MAX_SENSORS ][ 2 ] = { }; // each sensor's last two frames, for the glitch filter
    int missedFrames[ MAGFIT_MAX_SENSORS ] = { }; // frames in a row each sensor was not read
    uint32_t lastFrame = 0;
    bool haveSmoothed = false;
    uint32_t nextColdStartMs = 0;
    int misses = 0; // frames in a row that would not fit, while tracking
    uint32_t streamTick = 0;
    int learnCount = -1; // -1 = not learning, else good fixes collected so far
    float learned[ MAGLOC_LEARN_FIXES ];
    void learnFrom( float strength );

    uint32_t lastTrackUs = 0;
    uint32_t checkedBaseline = 0;          // magArray.baselineCount last checked
    uint32_t roughOnlySinceMs = 0;         // when "present without a fit" began (0 = not now)
    uint32_t lastPresentMs = 0;            // when a magnet was last present (the drift hold-off)
    Vec3 speedRing[ MAGLOC_SPEED_WINDOW ]; // the track's position, the last MAGLOC_SPEED_WINDOW frames
    int speedRingCount = 0;
    int baselineRetakes = 0;
    uint32_t retakeBaselineAtMs = 0;
    void checkBaseline( );
    ServiceStatus fitFrame( MagTrackInput* in );
    ServiceStatus simFrame( MagTrackInput* in );
    void offerRough( MagTrackInput* in, const MagFitResult* r ) const;

    Vec3 pointerOf( Vec3 tip, Vec3 shaft ) const;
    void printFixCsv( Stream* out ) const;
};

extern MagLocator& magLocator;

#endif // MAGLOCATOR_H
