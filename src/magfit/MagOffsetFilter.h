// SPDX-License-Identifier: MIT
#ifndef MAG_OFFSET_FILTER_H
#define MAG_OFFSET_FILTER_H
// ---------------------------------------------------------------------------
// One sensor's zero as a state with a doubt: a per-axis random-walk Kalman
// filter. The offset is what the sensor reads with nothing near; the
// variance is how sure of that the filter is. Every frame the doubt grows by
// the walk (a zero drifts with temperature: a TMAG5273's 3-10 uT/degC); a
// frame explained as "nothing here" updates it with the reading against the
// noise alone; a frame a dipole explains, or one with a field no dipole
// explains that is still within its hold, leaves it (the locator decides
// which, MagLocator::keepZeros: the filter only follows or holds). The
// steady gain is about sqrt(q/R) a frame: the locator sets the walk so that
// a reading of nothing is followed with MAG_OFFSET_FOLLOW_S.
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
