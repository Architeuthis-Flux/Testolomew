// SPDX-License-Identifier: MIT
#ifndef MAG_OFFSET_FILTER_H
#define MAG_OFFSET_FILTER_H
// ---------------------------------------------------------------------------
// One sensor's zero as a state with a doubt: a per-axis random-walk Kalman
// filter. The offset is what the sensor reads with nothing near; the
// variance is how sure of that the filter is. Every frame the doubt grows by
// the walk (a zero drifts with temperature: a TMAG5273's 3-10 uT/degC); a
// frame explained as "nothing here" updates it with the reading against the
// noise alone; a frame with a field no dipole explains updates it against a
// wider sigma (the caller's: the field's own size), so a static lone-sensor
// error is absorbed on the filter's own time and a real probe is not; a
// frame a dipole explains holds it. The steady gain is about sqrt(q/R) a
// frame: for a TMAG5273 (0.012 mT noise, 0.00025 mT/sqrt(s) walk) a clean
// step is absorbed with a 5 s time constant, a 0.1 mT unexplained one at
// 40 s.
//
// Pure: no Arduino; tested under test/test_magoffset.
// ---------------------------------------------------------------------------
#include "Vec3.h"

struct MagOffsetFilter {
    Vec3 offset;   // mT, in the sensor's own frame
    Vec3 variance; // mT^2, per axis
    float walk;    // mT per sqrt(s): how fast the doubt grows while nothing updates it
};

void magOffsetInit( MagOffsetFilter* f, Vec3 offset, float sigmaMt, float walkMtPerSqrtS );
void magOffsetPredict( MagOffsetFilter* f, float dtS );                 // the doubt grows: every frame
void magOffsetUpdate( MagOffsetFilter* f, Vec3 reading, Vec3 sigmaMt ); // a reading of nothing (or of something unexplained, with sigma widened)

#endif // MAG_OFFSET_FILTER_H
