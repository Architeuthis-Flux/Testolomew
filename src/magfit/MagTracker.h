// SPDX-License-Identifier: MIT
#ifndef MAGTRACKER_H
#define MAGTRACKER_H
// ---------------------------------------------------------------------------
// The probe over time. The fit answers "where is the magnet in THIS frame";
// this takes those answers, one per frame, and keeps a track: where the probe
// is and how fast it is moving, carried on through frames that gave no fix,
// with fixes that cannot be right left out, and turned into the one point the
// user cares about - the cursor on the breadboard's surface, either straight
// under the tip or where the tip points.
//
// The filter is a Kalman filter with a constant-velocity model, one per axis
// (three independent 2-state filters: position and velocity). Each fix comes
// with its own error bar from the fit, and that is its weight - a sharp fix
// over the middle of the array pulls hard, a rough one from the far edge
// barely nudges. Between fixes the track moves at its velocity and its
// uncertainty grows as a hand's acceleration allows (MAGTRACK_ACCEL_SIGMA).
//
// Gating: a fix that lands where the track says it cannot be - further than
// MAGTRACK_GATE in units of the combined uncertainty, or implying a speed
// past MAGTRACK_MAX_SPEED - is dropped. Because the fit's bar leaves out the
// array's own systematic error, the bar is floored (MAGTRACK_SIGMA_FLOOR_MM)
// before it gates anything, else real motion over a "perfect" fix would be
// thrown away. And a track can be wrong: after MAGTRACK_REINIT_AFTER dropped
// fixes in a row that agree with EACH OTHER, the track jumps to them.
//
// Coasting: with no fix, the track carries on for MAGTRACK_COAST_MS with its
// velocity dying away (a hand that vanished from the array is not still
// moving at the same speed), then gives up. A rough fix (the lattice search's
// answer for a far probe, tens of mm of bar) is a fix like any other; it only
// weighs a little, and keeps a far probe on the map at all.
//
// The shaft (the probe's direction) is smoothed separately and more heavily
// than the position, because the direction is the noisiest thing the fit
// gives and the pointer multiplies its error by the drop to the board.
//
// The cursor: the tip is MAGLOC_TIP_OFFSET_MM down the shaft; the cursor is
// on the surface plane (surfaceZ above the sensors), either straight under
// the tip (MAGCURSOR_UNDER) or where the shaft, carried on from the tip,
// meets the plane (MAGCURSOR_POINTED) - never past it, and never further
// from the tip than MAGTRACK_MAX_REACH_MM, so a probe held high and nearly
// level cannot throw the cursor across the board. A tip at or below the plane
// is its own cursor. The cursor then goes through a 1-Euro filter (Casiez,
// Roussel, Vogel, CHI 2012): a low-pass whose cutoff rises with speed, so it
// sits still when the hand is still and follows without lag when it moves.
//
// No Arduino in here; host-tested (test/test_magtracker).
// ---------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>

#include "TipModel.h"
#include "Vec3.h"

#define MAGTRACK_ACCEL_SIGMA 20000.0f     // mm/s^2: how jerky a hand's motion may be (process noise). 3000 lagged 20 ms; this lags 10 with the same rest jitter after the 1-Euro (tools/hostsim/pencil.cpp) (the bench's setting on 2026-09-27, taken as the default: 10000 until then)
#define MAGTRACK_ROUGH_ACCEL_SIGMA 2000.0f // mm/s^2: the process noise while ROUGH (far, 12 mm fixes: averaging beats following)
#define MAGTRACK_SIGMA_FLOOR_MM 0.2f      // the array's systematic error, not in the fit's bar: sigma_R, the std of the fix's second differences over sqrt 6 (tools/fixstats.py), measured 0.16-0.19 mm at rest with the 4232 magnet (2026-09-19 capture) and scaled to the 1839 one; the bench protocol (knobs.md section 8) replaces it (0.3 until 2026-09-23) (the bench's setting on 2026-09-27, taken as the default: 0.4 until then)
#define MAGTRACK_GATE 2.0f                // fixes further than this many sigmas from the track count for LESS (Huber: their variance grown by the square of the excess)... (the bench's setting on 2026-09-27, taken as the default: 4 until then)
#define MAGTRACK_GATE_DROP 11.3f          // ...and only past this many are dropped (sqrt 8 x the gate: a fix weighed to an eighth is worth keeping, beyond that it is another place)
#define MAGTRACK_MAX_SPEED_MM_S 3000.0f   // faster than a hand moves over a desk
#define MAGTRACK_REINIT_AFTER 3           // dropped fixes in a row that agree = the track was wrong
#define MAGTRACK_REINIT_AGREE_MM 6.0f     // ...agree = within this of each other
#define MAGTRACK_COAST_MS 400             // how long a track outlives its last fix
#define MAGTRACK_ROUGH_HOLD_S 3.0f        // ...and how long a ROUGH one (the far glow) outlives its last rough fix (menu: far hold; 2.5 until 2026-09-23: a stale glow for two seconds after the probe had gone) (the bench's setting on 2026-09-27, taken as the default: 1.0 until then)
#define MAGTRACK_ROUGH_SPREAD_MM_S 8.0f   // while it is held its bar widens this fast (the glow spreads and dims, honestly)
#define MAGTRACK_COAST_TAU_S 0.15f        // velocity dies away with this time constant while coasting
#define MAGTRACK_SHAFT_MIN_CUTOFF 3.6f    // Hz: the shaft direction's 1-Euro filter at rest (menu: shaft Hz) (the bench's setting on 2026-09-27, taken as the default: 1.0 until then)
#define MAGTRACK_SHAFT_BETA 1.0f         // per unit/s of a component's change: how fast the cutoff opens as the pencil turns (menu: shaft beta) (the bench's setting on 2026-09-27, taken as the default: 10 until then)
#define MAGTRACK_VIEW_MIN_CUTOFF 2.7f     // Hz: the 1-Euro filter on what the SCENE draws of the magnet (menu: view) (the bench's setting on 2026-09-27, taken as the default: 1.5 until then)
#define MAGTRACK_VIEW_BETA 0.04f           // per mm/s (0.05 lagged 40 ms at writing speed; this 20, jitter at rest 0.025 mm - pencil.cpp) (the bench's setting on 2026-09-27, taken as the default: 0.5 until then)
#define MAGTRACK_SHAFT_FLIP_DEG 45.0f     // a shaft swing bigger than this in one frame waits for confirmation
#define MAGTRACK_MAX_REACH_MM 60.0f       // the pointer never reaches further from the tip than this (the bench's setting on 2026-09-27, taken as the default: 40 until then)
// ...and never as if the probe were flatter than this: the reach is drop x
// tan(tilt), and past 70 degrees the tangent runs away - at 84 degrees a
// millimetre of drop and the shaft's 0.8 degree of jitter at rest put the
// cursor 1.5 mm from where it was a frame ago (the bench, 2026-09-21, the
// probe laid flat on the board). A probe flatter than this points as if it
// were at 70 degrees: the same direction, a bounded reach, and the jitter of
// the tilt no longer in it.
#define MAGTRACK_MAX_POINT_TILT_DEG 70.0f
// A point this close to the surface (or under it: in a hole) is TOUCHING,
// and a touching point is where the user means, whatever the tilt: the
// cursor is the point itself, not a projection. Above it the probe hovers
// and points. (2026-09-21: with the surface 1 mm under a resting magnet the
// projection turned 0.8 deg of tilt jitter into 1.5 mm of cursor, and a tap
// at an angle lit the row the shaft pointed at, not the one tapped.)
#define MAGTRACK_CONTACT_MM 1.5f
#define MAGTRACK_ONE_EURO_MIN_CUTOFF 3.2f // Hz: how much the cursor may jitter at rest (the bench's setting on 2026-09-27, taken as the default: 1.0 until then)
#define MAGTRACK_ONE_EURO_BETA 0.12f       // per mm/s: how fast the cutoff opens with speed (pencil.cpp: 20 ms behind at writing speed, 0.02 mm jitter at rest) (the bench's setting on 2026-09-27, taken as the default: 0.5 until then)
#define MAGTRACK_ONE_EURO_D_CUTOFF 1.0f   // Hz: smoothing of the speed estimate itself
// The cursor's and the view's filters slow with the point's height above the
// surface: their cutoffs (Hz) are divided by (1 + height / HZ_HALF) and their
// betas by (1 + height / BETA_HALF), so at those heights each runs at half
// the pace, at twice them a third (menu: Hz height, beta height; 0 = the
// same at any height). A far fix is a noisy one - its bar grows with the
// fourth power of the distance - and one lever at rest cannot serve both the
// board and a hover; the beta's own lever is what makes it "responsive with
// the probe close but very smooth at a distance" (Kevin, 2026-09-27): the
// speed estimate's noise is what opens the filter at rest, and far out that
// noise is all there is. The rough fix's own calmer filter (0.5 Hz, no beta)
// went with these: continuous where that was a step at the 5 mm bar.
// The point's HEIGHT above the surface has a 1-Euro of its own (menu: height
// Hz / height beta; 2026-09-28, Kevin: "we should have the height on its own
// smoothing setting"): the cursor's filter is x and y, and the height - what
// the LEDs colour and size by, and the View shows - came straight from the
// position's z, unfiltered. It starts at the cursor's levers.
#define MAGTRACK_HEIGHT_MIN_CUTOFF 3.2f
#define MAGTRACK_HEIGHT_BETA 0.12f
#define MAGTRACK_HZ_HALF_MM 25.0f // (the bench's setting on 2026-09-27, taken as the default: 40 until then)
#define MAGTRACK_BETA_HALF_MM 20.0f

enum MagTrackState {
    MAGTRACK_NONE,     // nothing tracked
    MAGTRACK_ROUGH,    // only rough fixes lately: the probe is far, somewhere about here
    MAGTRACK_COASTING, // no fix for a moment; carried on from the last ones
    MAGTRACK_TRACKING  // fixes coming in and agreeing
};

enum MagCursorMode {
    MAGCURSOR_UNDER,  // straight under the tip
    MAGCURSOR_POINTED // where the shaft points, on the surface plane
};

// One frame's input. A frame either has a fix (valid), a rough fix (rough:
// position with a large sigma, from the lattice search), or nothing.
struct MagTrackInput {
    bool valid;    // a proper fix
    bool rough;    // a rough one (valid is false)
    Vec3 position; // the magnet, mm, as this frame's fit gave it
    Vec3 sigma;    // its error bar, mm, per axis
    float alpha;   // the field smoothing that made it (1 = none): fixes are correlated by 1/alpha frames
    Vec3 shaft;    // unit vector up the shaft (only with a proper fix)
    bool haveShaft;
};

// The 1-Euro filter, one axis.
struct OneEuroAxis {
    bool started;
    float x;  // filtered value
    float dx; // filtered speed
};

struct MagTrackAxis {
    float p, v;          // position and velocity
    float p00, p01, p11; // covariance: position, cross, velocity
};

struct MagTrack {
    MagTrackState state;
    Vec3 position;       // the magnet, filtered, mm
    Vec3 velocity;       // mm/s
    Vec3 sigma;          // the track's own error bar, mm, per axis (floored)
    Vec3 shaft;          // unit vector up the shaft, smoothed
    float tiltDeg;       // its angle from vertical
    Vec3 tip;            // the probe's point
    Vec3 cursor;         // on the surface plane, 1-Euro filtered
    Vec3 rawCursor;      // before the 1-Euro filter
    Vec3 viewPosition;   // the magnet for the scene: the position through its own 1-Euro filter (viewMinCutoff, viewBeta)
    Vec3 viewTip;        // ...and the point from it
    float heightMm;      // the point's height above the surface, through its own 1-Euro (heightMinCutoff, heightBeta); tip.z - surfaceZ is the raw one; NEGATIVE below the believed surface (the surface setting is too high)
    float cursorSigmaMm; // about how far the cursor may be off, across the board
    float reachMm;       // how far the cursor sits from under the tip (0 in UNDER mode)
    uint32_t ageMs;      // since the last accepted proper fix
    uint32_t sinceAnyMs; // since the last accepted fix of any kind
    float lastGate;      // the last fix's distance from the track, in sigmas
    bool lastDropped;
    uint32_t accepted, dropped, reinits, coasted; // counters since reset

    // settings
    bool enabled; // off = fixes pass straight through the Kalman, gate and coast (for comparing); what is shown reads the track either way (2026-09-27)
    bool smooth;  // the cursor's, view's and shaft's 1-Euro filters; off = what is shown is the track's own output, and with the tracker off too that is the bare fix - RAW, for comparing (2026-09-27 evening: with the filters in both modes the tracker toggle changed nothing to see)
    MagCursorMode cursorMode;
    float surfaceZ;    // the breadboard's surface, mm above the sensors
    float tipOffsetMm; // the point is this far down the shaft from the magnet...
    TipModel tipModel; // ...or, learned by the lean calibration, where it is for a leaning shaft (TipModel.h; the locator copies it in each frame)
    float accelSigma;
    float sigmaFloorMm;
    float gate;
    float maxReachMm;
    float oneEuroMinCutoff, oneEuroBeta;
    float heightMinCutoff, heightBeta; // the height's own smoothing
    float viewMinCutoff, viewBeta;     // the scene's smoothing of the magnet
    float shaftMinCutoff, shaftBeta; // the shaft direction's own 1-Euro (a turn is followed at once, a resting shaft stays put)
    float hzHalfMm;                // the height at which the cursor's and the view's Hz are halved (0 = the same at any height)...
    float betaHalfMm;              // ...and their betas
    float roughHoldS;              // how long the far glow is held after its last rough fix

    // private-ish
    MagTrackAxis axis[ 3 ];
    OneEuroAxis euro[ 3 ];
    OneEuroAxis viewEuro[ 3 ];
    OneEuroAxis heightEuro;
    OneEuroAxis shaftEuro[ 3 ];
    Vec3 roughSigma; // the bar the far glow is held at (the last rough fix's, spreading)
    int shaftSwings; // frames in a row the shaft wanted to swing far
    int droppedRun;  // dropped fixes in a row
    Vec3 droppedAt[ MAGTRACK_REINIT_AFTER ];
    bool haveShaft;
    Vec3 lastFixSigma; // the last accepted proper fix's bar (floored)
    bool hadProperFix; // this track has had a proper fix (else it is rough from birth)
};

// Defaults into every setting and an empty track.
void magTrackInit( MagTrack* t, float surfaceZ, float tipOffsetMm );

// Forget the track (settings stay).
void magTrackReset( MagTrack* t );

// One frame: dtS since the last call, and what the fit gave. Updates everything.
void magTrackUpdate( MagTrack* t, float dtS, const MagTrackInput* in );

// The cursor for a given tip and shaft, in the track's mode, plus how far it
// reached. Exposed for the display.
Vec3 magTrackCursorOf( const MagTrack* t, Vec3 tip, Vec3 shaft, float* reachMm );
// The pointer alone: where the shaft, carried on from the tip, meets the
// plane at surfaceZ - never further from the tip than maxReachMm, the tilt
// taken no flatter than MAGTRACK_MAX_POINT_TILT_DEG, the tip itself when it
// is at or under the plane, or within MAGTRACK_CONTACT_MM of it. Pure geometry; the cursor in POINTED mode and
// the locator's fix.pointer are both this.
Vec3 magTrackPointer( float surfaceZ, float maxReachMm, Vec3 tip, Vec3 shaft, float* reachMm );

#endif // MAGTRACKER_H
