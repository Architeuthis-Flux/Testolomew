// SPDX-License-Identifier: MIT
#ifndef SURFACEMAP_H
#define SURFACEMAP_H
// ---------------------------------------------------------------------------
// The breadboard's surface as a height map over the sensor plane, learned
// from the row calibration's taps (the probe resting in a hole: the point's
// x, y and its resting z). One number - the menu's "surface" - is the
// surface at the taps' centre; this is how much higher or lower it runs
// elsewhere: a plane from three to five taps (a board sitting at a tilt to
// the sensor PCB), a quadratic from six or more (a bow, and the dipole
// fit's own height bias, which is not the same over a corner sensor as
// over the middle). 2026-09-28, Kevin: "the reported height seems to change
// across the surface, let's do a learn height calibration so we can
// account for that".
//
// The fit is least squares on {1, dx, dy, dx^2, dx dy, dy^2} about the
// taps' centroid, a little ridge on the diagonal so a thin set of taps
// (the twelve are on four lines) still solves; outside the taps' box the
// map is held at the box's edge, so a quadratic never runs away where
// nothing was measured. Pure; host-tested (test/test_surfacemap).
// ---------------------------------------------------------------------------
#include "Vec3.h"

#define SURFACEMAP_TERMS 6
#define SURFACEMAP_PLANE_FROM 3     // taps for a plane...
#define SURFACEMAP_QUADRATIC_FROM 6 // ...and for a quadratic
#define SURFACEMAP_RIDGE 1e-4f      // of the normal matrix's diagonal, for a thin set of taps
#define SURFACEMAP_MARGIN_MM 5.0f   // the map holds its edge value this far beyond the taps' box

struct SurfaceMap {
    int terms; // 0 flat (nothing learned), 3 a plane, 6 a quadratic
    int count; // taps it was fitted to
    float centreX, centreY;
    float coef[ SURFACEMAP_TERMS ]; // z = coef[0] + coef[1] dx + coef[2] dy + coef[3] dx^2 + coef[4] dx dy + coef[5] dy^2
    float minX, maxX, minY, maxY;   // the taps' box (with the margin)
    float worstMm;                  // the tap the fit misses by most
};

void surfaceMapClear( SurfaceMap* m );
// Fit the taps' resting heights: their (x, y, z). Returns the terms used.
int surfaceMapFit( SurfaceMap* m, const Vec3* taps, int count );
// The surface height at (x, y) relative to the taps' centre (0 there; 0
// everywhere when nothing is learned).
float surfaceMapOffset( const SurfaceMap* m, float x, float y );
// The surface height at the centre itself (coef[0]; what "surface" means).
float surfaceMapCentreZ( const SurfaceMap* m );

#endif // SURFACEMAP_H
