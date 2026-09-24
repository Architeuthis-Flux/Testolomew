// SPDX-License-Identifier: MIT
#include "MagOffsetFilter.h"

void magOffsetInit( MagOffsetFilter* f, Vec3 offset, float sigmaMt, float walkMtPerSqrtS ) {
    f->offset = offset;
    float v = sigmaMt * sigmaMt;
    f->variance = { v, v, v };
    f->walk = walkMtPerSqrtS;
}

void magOffsetPredict( MagOffsetFilter* f, float dtS ) {
    float q = f->walk * f->walk * dtS;
    f->variance.x += q;
    f->variance.y += q;
    f->variance.z += q;
}

static void updateAxis( float* offset, float* variance, float reading, float sigma ) {
    float r = sigma * sigma;
    float k = *variance / ( *variance + r );
    *offset += k * ( reading - *offset );
    *variance *= 1.0f - k;
}

void magOffsetUpdate( MagOffsetFilter* f, Vec3 reading, Vec3 sigmaMt ) {
    updateAxis( &f->offset.x, &f->variance.x, reading.x, sigmaMt.x );
    updateAxis( &f->offset.y, &f->variance.y, reading.y, sigmaMt.y );
    updateAxis( &f->offset.z, &f->variance.z, reading.z, sigmaMt.z );
}
