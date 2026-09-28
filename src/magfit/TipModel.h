// SPDX-License-Identifier: MIT
#ifndef TIPMODEL_H
#define TIPMODEL_H
// ---------------------------------------------------------------------------
// Where the probe's POINT is, from where its magnet is and which way its
// shaft leans. One number - the menu's "tip", the magnet's distance up the
// shaft - is the rigid answer: point = magnet - tip x shaft. It is not the
// whole answer: the dipole fit's position for a leaning magnet is biased
// along the lean (a real magnet is not a point, the sensors' gains are not
// exact), and not the same along the rows as across them. So the model
// (2026-09-28, Kevin: "we need a better system for computing the correct
// row when the probe is held at an angle, maybe we do another calibration")
// is a horizontal 2 x 2 on the shaft's lean plus a vertical distance:
//
//   point.xy = magnet.xy - A h        h = (shaft.x, shaft.y), |h| = sin(tilt)
//   point.z  = magnet.z  - dz shaft.z
//
// A = tip x I and dz = tip is today's model exactly. It is LEARNED by the
// lean calibration: the point resting in one hole, the probe held still
// straight up and then leaning several ways; every sample is the magnet and
// the shaft, the hole's own place is an unknown too (three per hole), so
// the flow needs neither the grid nor the surface map. Linear least
// squares, a little ridge on A and dz, refused when the leans are too alike
// (A wants three directions) or there is no upright sample (dz wants a
// spread in shaft.z). Pure; host-tested (test/test_tipmodel).
// ---------------------------------------------------------------------------
#include "Vec3.h"

#define TIPMODEL_MAX_SAMPLES 24
#define TIPMODEL_MAX_HOLES 4
#define TIPMODEL_UNKNOWNS ( 3 * TIPMODEL_MAX_HOLES + 5 )
#define TIPMODEL_RIDGE 1e-4f
#define TIPMODEL_MIN_LEAN_SPREAD 0.05f // the smaller eigenvalue of sum(h h^T) at least this: three leans of 25 deg in three directions give ~0.15
#define TIPMODEL_MIN_Z_SPREAD 0.08f    // max - min of shaft.z at least this: upright against a 25 deg lean is 0.09
#define TIPMODEL_MAX_TIP_MM 60.0f

struct TipModel {
    bool valid;   // learned; else the scalar tip applies
    float a[ 4 ]; // a00 a01 a10 a11
    float dz;
};

struct TipSample {
    Vec3 magnet, shaft; // the shaft a unit vector up the probe
    int hole;           // 0 .. TIPMODEL_MAX_HOLES - 1: samples in the same hole share its place
};

struct TipFitReport {
    int samples, holes;
    float rmsMm, worstMm;         // the points' scatter about their holes with the model
    float leanMinDeg, leanMaxDeg; // the samples' tilts
    float tipMm;                  // (a00 + a11) / 2: the tip for the menu
    bool scalar;                  // only the plain tip could be pinned (three to five samples, or leans all one way): A = tip I, dz = tip
    const char* note;             // why it is the plain one, when it is (nullptr otherwise)
    const char* refused;          // nullptr when fitted, else why not
};

void tipModelClear( TipModel* m );
// The point for this magnet and shaft: the model when valid, else the scalar.
Vec3 tipModelPoint( const TipModel* m, float tipMm, Vec3 magnet, Vec3 shaft );
// Fit the samples: the full model from six or more that lean three ways
// with an upright among them; else the plain tip alone (as the console's Q
// always did) from three or more that lean at all; false (and
// report->refused) when not even that.
bool tipModelFit( const TipSample* samples, int count, TipModel* out, TipFitReport* report );

#endif // TIPMODEL_H
