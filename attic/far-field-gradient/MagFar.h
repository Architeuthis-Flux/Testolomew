// SPDX-License-Identifier: MIT
#ifndef MAGFAR_H
#define MAGFAR_H
// ---------------------------------------------------------------------------
// Roughly where is the magnet, without a fit. For a probe held well away from
// the array (weak fields, the dipole fit failing or not worth 50 ms), and as
// a starting point that lands the fit near the answer at once.
//
// The trick (Nara, Suzuki and Ando, IEEE Trans. Magn. 2006): a dipole's field
// is homogeneous of degree -3 in the vector r from the magnet to the point
// where it is measured, so by Euler's theorem the field's gradient tensor G
// (G[i][j] = dB_i/dx_j) and the field B at ONE point give that vector outright:
//
//     G r = -3 B      =>      r = -3 G^-1 B,     magnet = point - r
//
// No iteration, no starting guess, no false minima. What it needs is G, and a
// flat array of 3-axis sensors gives all of it: the in-plane derivatives
// dB/dx and dB/dy by fitting a plane to each component across the sensors,
// and the rest from Maxwell in empty space - the field is curl-free
// (dBx/dz = dBz/dx, dBy/dz = dBz/dy, dBx/dy = dBy/dx) and divergence-free
// (dBz/dz = -dBx/dx - dBy/dy). The plane fit is a finite difference over the
// sensor spacing, so it is only right when the magnet is far compared with
// the spacing: at two spacings out the answer is within about a fifth of the
// distance, closer in it drifts, and that is what the error bar says.
//
// If the magnet's strength is known there is a second, cruder handle: |B| at
// the centre of the array is between m/r^3 and 2m/r^3 whatever way the magnet
// points, so r is known within a factor of 2^(1/3) = 1.26.
//
// No Arduino in here; host-tested.
// ---------------------------------------------------------------------------
#include <stdbool.h>

#include "Vec3.h"

#define MAGFAR_MAX_SENSORS 16

// The estimate is not offered nearer than this many array half-diagonals from
// the array's centre: the gradient is a finite difference and the answer
// breaks down inside that.
#define MAGFAR_MIN_DISTANCE_FACTOR 1.0f

// Error bar as a fraction of the distance (from simulation, see the tests), and
// its floor in mm.
#define MAGFAR_SIGMA_FRACTION 0.2f
#define MAGFAR_SIGMA_FLOOR_MM 5.0f

struct MagFarEstimate {
    bool valid;      // an answer worth having (see the limits above)
    Vec3 position;   // where the magnet roughly is, mm
    float sigmaMm;   // about how far off that could be (1 sigma, any axis)
    float distanceMm; // from the array's centre
    Vec3 centre;     // the array's centre, where B and G are taken
    Vec3 field;      // B at the centre, from the plane fit
    float gradientMt; // |G| (Frobenius), mT/mm - how much signal the gradient had
    float rangeMm;   // distance from |B| and the known strength, 0 if not known
};

// The estimate from `count` readings (mT, board frame, ambient removed) at
// `sensors` (mm); use[i] may be null. knownStrength (mT*mm^3, 0 = unknown)
// only adds rangeMm. Returns out->valid.
bool magFarEstimate( const Vec3* sensors, const Vec3* fields, const bool* use, int count, float knownStrength, MagFarEstimate* out );

#endif // MAGFAR_H
