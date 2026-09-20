// SPDX-License-Identifier: MIT
#ifndef MAGFIT_H
#define MAGFIT_H
// ---------------------------------------------------------------------------
// Where is the magnet? A point-dipole fit to an array of 3-axis field readings.
//
// Model: a magnet far enough away to be a point dipole with moment m at p0
// makes, at a sensor at p,
//
//     r = p - p0,   B(p) = 3 r (m.r) / |r|^5  -  m / |r|^3
//
// Units are chosen so there are no constants: positions in mm, fields in mT,
// and the moment in mT*mm^3 (that is mu0*m/4pi; a 6x3 mm N52 disc is about
// 9800). Six unknowns (p0 and m), three numbers per sensor, so eight sensors
// give 24 equations.
//
// The fit is "variable projection": the field is LINEAR in m, so for any
// trial position the best moment is one 3x3 linear solve, and the only
// non-linear search left is over the three position coordinates
// (Levenberg-Marquardt, numeric Jacobian). That keeps mm-sized and
// 10^4-sized unknowns out of the same matrix and needs no starting guess for
// the moment at all.
//
// With 3-axis sensors in a plane there is no above/below mirror ambiguity, but
// the magnet cannot be under the board, so z is kept positive anyway.
//
// No Arduino in here - plain C++ and <math.h>, so `pio test -e native` runs it
// on the host and it drops into any firmware unchanged.
// ---------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>

#include "Vec3.h"

#define MAGFIT_MAX_SENSORS 16

// Fit limits (mm). The magnet is looked for inside this box, board frame.
#define MAGFIT_Z_MIN 0.5f
#define MAGFIT_Z_MAX 150.0f
#define MAGFIT_XY_MARGIN 100.0f // how far outside the sensor footprint

// The lattice search (magFitCoarse, and the start of every cold start): a
// lattice of trial positions this far apart, this far outside the sensor
// footprint and over this range of heights, then 3x3x3 lattices of half the
// step round the best, down to the final step. About 250 trial positions.
#define MAGFIT_COARSE_STEP_MM 25.0f
#define MAGFIT_COARSE_MARGIN_MM 45.0f
#define MAGFIT_COARSE_Z_MIN_MM 4.0f
#define MAGFIT_COARSE_Z_MAX_MM 110.0f
#define MAGFIT_COARSE_FINAL_MM 3.0f
// A cold start refined from the lattice's best point that fits this well
// (as a fraction of the misfit limit, in cost) is taken; worse, and the old
// seeds are tried too.
#define MAGFIT_COARSE_GOOD_ENOUGH 0.25f
// ...and one that came out lower than this always is (false minima live there).
#define MAGFIT_COARSE_LOW_MM 12.0f

struct MagFitResult {
    bool valid;     // the fit converged and explains the readings
    Vec3 position;  // magnet centre, mm, board frame
    Vec3 moment;    // dipole moment, mT*mm^3 (points S pole -> N pole)
    float strength; // |moment|
    float residual; // RMS misfit per axis, mT
    float signal;   // RMS reading per axis, mT (residual/signal = fit quality)
    Vec3 sigma;     // 1-sigma error bar on position, mm, per axis (see below)
    int iterations;
    // A cold start in slices (magFitSolveStep): 0 = none under way, k >= 1 =
    // the seeds from seed k-1 are still to try; the best so far and its cost.
    int coldStage;
    Vec3 coldBest;
    float coldCost;
};

// The error bar. At the answer, the fit knows how much every reading would
// change per millimetre the magnet moved (its Jacobian J) and how wrong the
// readings still are (the residual: sensor noise plus everything the model
// does not describe). Ordinary least-squares error propagation turns the two
// into a position covariance, s^2 (J^T J)^-1, and sigma is the square root of
// its diagonal. It is what makes a fix usable: a magnet 10 mm over a sensor
// and one 40 mm up in the weak far field can both "fit", and their sigmas
// differ by a factor of fifty. It counts the residual as random, so an error
// that is the same in every frame (a sensor in the wrong place) shows up only
// partly; read it as "at least this uncertain".
//
// The residual is not allowed to claim less than this much error per reading,
// so a lucky fit cannot report an impossibly small bar. About half of one
// averaged frame's noise on the bench array.
#define MAGFIT_NOISE_FLOOR_MT 0.005f

// The forward model: field (mT) at `sensor` from a dipole `moment` at `magnet`.
Vec3 magFitDipoleField( Vec3 sensor, Vec3 magnet, Vec3 moment );

// Fit a dipole to `count` readings. sensors[i] is where sensor i sits (mm),
// fields[i] its reading (mT, board frame, ambient already subtracted), and
// use[i] (may be null = all) leaves dead sensors out.
//
// If result->valid is true on entry its position is the starting guess (warm
// start - the normal case while tracking, a few iterations). Otherwise the
// search starts over the strongest readings at a few heights. This one
// runs a cold start to its end in one call; result->coldStage is reset.
//
// Returns result->valid. maxMisfit is the residual/signal ratio above which
// the fit is called invalid (0.25 is a reasonable start: a real magnet near
// the array fits to a few percent, and junk does not fit at all).
bool magFitSolve( const Vec3* sensors, const Vec3* fields, const bool* use, int count,
                  float maxMisfit, MagFitResult* result );

// The same, one slice at a time: a warm start is one call; a cold start is
// the lattice and a refinement in the first call and, if that came out
// poor or low, one seed per call after it (result->coldStage says one is
// under way; the caller calls again next frame rather than waiting). A
// whole cold start held the loop 25-38 ms on the CH32H417 (longer than an
// LED frame period) and showed on the supply; a slice is a few ms.
bool magFitSolveStep( const Vec3* sensors, const Vec3* fields, const bool* use, int count,
                      float maxMisfit, MagFitResult* result );

// The same, for a magnet whose strength |moment| is already known - which a
// probe's is: it carries one magnet. With the strength free, a magnet a little
// higher and a little stronger looks almost the same to the array as one a
// little lower and weaker, and that trade-off is where most of the height
// noise (and some of the scale error toward the edges) comes from. Pinning the
// strength removes it: five unknowns instead of six.
//
// It runs the free fit above first (that is the robust part, and the only part
// that ever starts cold) and then refines position and direction with the
// strength held near `strength` - held softly, as one extra equation, so a
// magnet that is a few percent off (temperature, a different sample) still
// fits. result->strength comes back as what the fit settled on.
bool magFitSolveKnownStrength( const Vec3* sensors, const Vec3* fields, const bool* use, int count,
                               float maxMisfit, float strength, MagFitResult* result );

// Roughly where is the magnet, without iterating: the point of a coarse
// lattice (MAGFIT_COARSE_*) that explains the readings best, with the moment
// solved at each point. It cannot get stuck and cannot fail to answer, so it
// is what to ask when the fit above will not converge - a probe far away in
// a weak field, or seen by too few sensors - and its sigma says how rough the
// answer is. It is also the first thing every cold start does. Fills in the
// same fields as magFitSolve; result->valid only says there were readings.
bool magFitCoarse( const Vec3* sensors, const Vec3* fields, const bool* use, int count, MagFitResult* result );

// Solve the n x n system a x = b in place, n <= MAGFIT_MAX_PARAMS (Gaussian
// elimination, partial pivoting; the answer is left in b). false if singular.
#define MAGFIT_MAX_PARAMS 6
bool magFitSolveLinear( float a[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ], float b[ MAGFIT_MAX_PARAMS ], int n );

#endif // MAGFIT_H
