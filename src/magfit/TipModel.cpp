// SPDX-License-Identifier: MIT
#include "TipModel.h"

#include <math.h>

void tipModelClear( TipModel* m ) {
    m->valid = false;
    m->a[ 0 ] = m->a[ 1 ] = m->a[ 2 ] = m->a[ 3 ] = 0.0f;
    m->dz = 0.0f;
}

Vec3 tipModelPoint( const TipModel* m, float tipMm, Vec3 magnet, Vec3 shaft ) {
    if ( m == nullptr || !m->valid ) {
        Vec3 p = { magnet.x - tipMm * shaft.x, magnet.y - tipMm * shaft.y, magnet.z - tipMm * shaft.z };
        return p;
    }
    Vec3 p = { magnet.x - ( m->a[ 0 ] * shaft.x + m->a[ 1 ] * shaft.y ), magnet.y - ( m->a[ 2 ] * shaft.x + m->a[ 3 ] * shaft.y ), magnet.z - m->dz * shaft.z };
    return p;
}

// Gaussian elimination with pivoting, in double: n x n, in place.
static bool solve( double a[ TIPMODEL_UNKNOWNS ][ TIPMODEL_UNKNOWNS ], double* b, int n ) {
    for ( int col = 0; col < n; col++ ) {
        int pivot = col;
        for ( int r = col + 1; r < n; r++ )
            if ( fabs( a[ r ][ col ] ) > fabs( a[ pivot ][ col ] ) )
                pivot = r;
        if ( fabs( a[ pivot ][ col ] ) < 1e-12 )
            return false;
        for ( int c = 0; c < n; c++ ) {
            double t = a[ col ][ c ];
            a[ col ][ c ] = a[ pivot ][ c ];
            a[ pivot ][ c ] = t;
        }
        double t = b[ col ];
        b[ col ] = b[ pivot ];
        b[ pivot ] = t;
        for ( int r = col + 1; r < n; r++ ) {
            double k = a[ r ][ col ] / a[ col ][ col ];
            for ( int c = col; c < n; c++ )
                a[ r ][ c ] -= k * a[ col ][ c ];
            b[ r ] -= k * b[ col ];
        }
    }
    for ( int r = n - 1; r >= 0; r-- ) {
        for ( int c = r + 1; c < n; c++ )
            b[ r ] -= a[ r ][ c ] * b[ c ];
        b[ r ] /= a[ r ][ r ];
    }
    return true;
}

bool tipModelFit( const TipSample* samples, int count, TipModel* out, TipFitReport* report ) {
    TipFitReport r = { };
    r.refused = nullptr;
    r.samples = count;
    tipModelClear( out );
    if ( count > TIPMODEL_MAX_SAMPLES )
        count = TIPMODEL_MAX_SAMPLES;
    int holes = 0;
    for ( int i = 0; i < count; i++ )
        if ( samples[ i ].hole >= holes )
            holes = samples[ i ].hole + 1;
    r.holes = holes;
    // The spread: the leans' scatter (for A) and the shaft.z span (for dz).
    float sxx = 0.0f, sxy = 0.0f, syy = 0.0f, zMin = 2.0f, zMax = -2.0f;
    r.leanMinDeg = 90.0f;
    r.leanMaxDeg = 0.0f;
    for ( int i = 0; i < count; i++ ) {
        const Vec3& s = samples[ i ].shaft;
        sxx += s.x * s.x;
        sxy += s.x * s.y;
        syy += s.y * s.y;
        zMin = s.z < zMin ? s.z : zMin;
        zMax = s.z > zMax ? s.z : zMax;
        float lean = acosf( s.z > 1.0f ? 1.0f : ( s.z < -1.0f ? -1.0f : s.z ) ) * 180.0f / (float)M_PI;
        r.leanMinDeg = lean < r.leanMinDeg ? lean : r.leanMinDeg;
        r.leanMaxDeg = lean > r.leanMaxDeg ? lean : r.leanMaxDeg;
    }
    float tr = sxx + syy, det = sxx * syy - sxy * sxy;
    float disc = sqrtf( tr * tr * 0.25f - det > 0.0f ? tr * tr * 0.25f - det : 0.0f );
    float smaller = tr * 0.5f - disc;
    // The shafts' scatter about their mean, for the plain tip (the old Q's rule).
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    for ( int i = 0; i < count; i++ ) {
        mx += samples[ i ].shaft.x;
        my += samples[ i ].shaft.y;
        mz += samples[ i ].shaft.z;
    }
    mx /= count > 0 ? count : 1;
    my /= count > 0 ? count : 1;
    mz /= count > 0 ? count : 1;
    float scatter = 0.0f;
    for ( int i = 0; i < count; i++ ) {
        const Vec3& s = samples[ i ].shaft;
        scatter += ( s.x - mx ) * ( s.x - mx ) + ( s.y - my ) * ( s.y - my ) + ( s.z - mz ) * ( s.z - mz );
    }
    int unknowns = 3 * holes + 5;
    r.scalar = false;
    r.note = nullptr;
    if ( holes < 1 || holes > TIPMODEL_MAX_HOLES || count < 3 ) {
        r.refused = "too few samples: three or more (six, leaning five ways, for the lean bias too)";
    } else if ( count * 3 < unknowns + 3 ) {
        r.scalar = true;
        r.note = "fewer than six samples: the plain tip only";
    } else if ( smaller < TIPMODEL_MIN_LEAN_SPREAD ) {
        r.scalar = true;
        r.note = "the leans were too alike to tell the lean bias: the plain tip only (lean three ways, 30 degrees or so)";
    } else if ( zMax - zMin < TIPMODEL_MIN_Z_SPREAD ) {
        r.scalar = true;
        r.note = "no upright sample among the leans: the plain tip only";
    }
    if ( r.refused == nullptr && r.scalar && scatter < 0.05f ) {
        r.refused = "the angles were too alike to tell: spread them (upright, leaning one way, leaning another)";
    }
    if ( r.refused != nullptr ) {
        if ( report != nullptr )
            *report = r;
        return false;
    }
    // Normal equations. Unknowns: [px py pz] per hole, then a00 a01 a10 a11, dz
    // - or, for the plain tip, the one t (A = t I, dz = t).
    static double n[ TIPMODEL_UNKNOWNS ][ TIPMODEL_UNKNOWNS ];
    static double b[ TIPMODEL_UNKNOWNS ];
    int A = 3 * holes, DZ = A + 4;
    if ( r.scalar )
        unknowns = A + 1;
    for ( int i = 0; i < unknowns; i++ ) {
        b[ i ] = 0.0;
        for ( int j = 0; j < unknowns; j++ )
            n[ i ][ j ] = 0.0;
    }
    for ( int i = 0; i < count; i++ ) {
        const TipSample& sm = samples[ i ];
        int P = 3 * sm.hole;
        // Three rows: x, y, z. Each row: (index, coefficient) pairs and its rhs.
        int idx[ 3 ][ 3 ] = { { P, A, A + 1 }, { P + 1, A + 2, A + 3 }, { P + 2, DZ, -1 } };
        double coef[ 3 ][ 3 ] = { { 1.0, sm.shaft.x, sm.shaft.y }, { 1.0, sm.shaft.x, sm.shaft.y }, { 1.0, sm.shaft.z, 0.0 } };
        if ( r.scalar ) {
            int sidx[ 3 ][ 3 ] = { { P, A, -1 }, { P + 1, A, -1 }, { P + 2, A, -1 } };
            double scoef[ 3 ][ 3 ] = { { 1.0, sm.shaft.x, 0.0 }, { 1.0, sm.shaft.y, 0.0 }, { 1.0, sm.shaft.z, 0.0 } };
            for ( int row = 0; row < 3; row++ )
                for ( int u = 0; u < 3; u++ ) {
                    idx[ row ][ u ] = sidx[ row ][ u ];
                    coef[ row ][ u ] = scoef[ row ][ u ];
                }
        }
        double rhs[ 3 ] = { sm.magnet.x, sm.magnet.y, sm.magnet.z };
        for ( int row = 0; row < 3; row++ ) {
            for ( int u = 0; u < 3; u++ ) {
                if ( idx[ row ][ u ] < 0 )
                    continue;
                b[ idx[ row ][ u ] ] += coef[ row ][ u ] * rhs[ row ];
                for ( int v = 0; v < 3; v++ ) {
                    if ( idx[ row ][ v ] < 0 )
                        continue;
                    n[ idx[ row ][ u ] ][ idx[ row ][ v ] ] += coef[ row ][ u ] * coef[ row ][ v ];
                }
            }
        }
    }
    double scale = 0.0;
    for ( int i = A; i < unknowns; i++ )
        scale += n[ i ][ i ];
    scale /= unknowns - A;
    for ( int i = A; i < unknowns; i++ )
        n[ i ][ i ] += TIPMODEL_RIDGE * scale; // a whisper on A and dz, never on the holes
    if ( !solve( n, b, unknowns ) ) {
        r.refused = "the samples do not pin the model (a solve failed)";
        if ( report != nullptr )
            *report = r;
        return false;
    }
    TipModel m;
    m.valid = true;
    if ( r.scalar ) {
        float t = (float)b[ A ];
        m.a[ 0 ] = m.a[ 3 ] = t;
        m.a[ 1 ] = m.a[ 2 ] = 0.0f;
        m.dz = t;
    } else {
        for ( int k = 0; k < 4; k++ )
            m.a[ k ] = (float)b[ A + k ];
        m.dz = (float)b[ DZ ];
    }
    r.tipMm = 0.5f * ( m.a[ 0 ] + m.a[ 3 ] );
    if ( r.tipMm < -1.0f || r.tipMm > TIPMODEL_MAX_TIP_MM || m.dz < -1.0f || m.dz > TIPMODEL_MAX_TIP_MM ) {
        r.refused = "the solve says a tip that is not a probe's: keep the point in the one hole and try again";
        if ( report != nullptr )
            *report = r;
        return false;
    }
    // The scatter of the points about their holes with the model.
    float sum = 0.0f;
    r.worstMm = 0.0f;
    for ( int i = 0; i < count; i++ ) {
        int P = 3 * samples[ i ].hole;
        Vec3 p = tipModelPoint( &m, 0.0f, samples[ i ].magnet, samples[ i ].shaft );
        float dx = p.x - (float)b[ P ], dy = p.y - (float)b[ P + 1 ], dz = p.z - (float)b[ P + 2 ];
        float e = sqrtf( dx * dx + dy * dy + dz * dz );
        sum += e * e;
        r.worstMm = e > r.worstMm ? e : r.worstMm;
    }
    r.rmsMm = sqrtf( sum / count );
    *out = m;
    if ( report != nullptr )
        *report = r;
    return true;
}
