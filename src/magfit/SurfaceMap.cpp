// SPDX-License-Identifier: MIT
#include "SurfaceMap.h"

#include <math.h>

#include "MagFit.h" // magFitSolveLinear

void surfaceMapClear( SurfaceMap* m ) {
    m->terms = 0;
    m->count = 0;
    m->centreX = m->centreY = 0.0f;
    for ( int k = 0; k < SURFACEMAP_TERMS; k++ )
        m->coef[ k ] = 0.0f;
    m->minX = m->maxX = m->minY = m->maxY = 0.0f;
    m->worstMm = 0.0f;
}

static void basis( float dx, float dy, int terms, float* b ) {
    b[ 0 ] = 1.0f;
    b[ 1 ] = dx;
    b[ 2 ] = dy;
    if ( terms > 3 ) {
        b[ 3 ] = dx * dx;
        b[ 4 ] = dx * dy;
        b[ 5 ] = dy * dy;
    }
}

static bool solve( SurfaceMap* m, const Vec3* taps, int count, int terms ) {
    float a[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ];
    float rhs[ MAGFIT_MAX_PARAMS ];
    for ( int r = 0; r < terms; r++ ) {
        rhs[ r ] = 0.0f;
        for ( int c = 0; c < terms; c++ )
            a[ r ][ c ] = 0.0f;
    }
    for ( int i = 0; i < count; i++ ) {
        float b[ SURFACEMAP_TERMS ];
        basis( taps[ i ].x - m->centreX, taps[ i ].y - m->centreY, terms, b );
        for ( int r = 0; r < terms; r++ ) {
            rhs[ r ] += b[ r ] * taps[ i ].z;
            for ( int c = 0; c < terms; c++ )
                a[ r ][ c ] += b[ r ] * b[ c ];
        }
    }
    for ( int r = 1; r < terms; r++ )
        a[ r ][ r ] *= 1.0f + SURFACEMAP_RIDGE; // never the constant: the centre's height is what it is
    if ( !magFitSolveLinear( a, rhs, terms ) )
        return false;
    for ( int k = 0; k < SURFACEMAP_TERMS; k++ )
        m->coef[ k ] = k < terms ? rhs[ k ] : 0.0f;
    m->terms = terms;
    return true;
}

int surfaceMapFit( SurfaceMap* m, const Vec3* taps, int count ) {
    surfaceMapClear( m );
    if ( count < SURFACEMAP_PLANE_FROM || taps == nullptr )
        return 0;
    float sx = 0.0f, sy = 0.0f;
    m->minX = m->maxX = taps[ 0 ].x;
    m->minY = m->maxY = taps[ 0 ].y;
    for ( int i = 0; i < count; i++ ) {
        sx += taps[ i ].x;
        sy += taps[ i ].y;
        m->minX = taps[ i ].x < m->minX ? taps[ i ].x : m->minX;
        m->maxX = taps[ i ].x > m->maxX ? taps[ i ].x : m->maxX;
        m->minY = taps[ i ].y < m->minY ? taps[ i ].y : m->minY;
        m->maxY = taps[ i ].y > m->maxY ? taps[ i ].y : m->maxY;
    }
    m->centreX = sx / count;
    m->centreY = sy / count;
    m->minX -= SURFACEMAP_MARGIN_MM;
    m->maxX += SURFACEMAP_MARGIN_MM;
    m->minY -= SURFACEMAP_MARGIN_MM;
    m->maxY += SURFACEMAP_MARGIN_MM;
    m->count = count;
    int terms = count >= SURFACEMAP_QUADRATIC_FROM ? SURFACEMAP_QUADRATIC_FROM : SURFACEMAP_PLANE_FROM;
    if ( !solve( m, taps, count, terms ) && ( terms == SURFACEMAP_PLANE_FROM || !solve( m, taps, count, SURFACEMAP_PLANE_FROM ) ) ) {
        surfaceMapClear( m ); // the taps do not span a surface (all in a line)
        return 0;
    }
    m->worstMm = 0.0f;
    for ( int i = 0; i < count; i++ ) {
        float miss = fabsf( taps[ i ].z - ( m->coef[ 0 ] + surfaceMapOffset( m, taps[ i ].x, taps[ i ].y ) ) );
        if ( miss > m->worstMm )
            m->worstMm = miss;
    }
    return m->terms;
}

float surfaceMapOffset( const SurfaceMap* m, float x, float y ) {
    if ( m->terms == 0 )
        return 0.0f;
    if ( x < m->minX )
        x = m->minX;
    if ( x > m->maxX )
        x = m->maxX;
    if ( y < m->minY )
        y = m->minY;
    if ( y > m->maxY )
        y = m->maxY;
    float b[ SURFACEMAP_TERMS ];
    basis( x - m->centreX, y - m->centreY, m->terms, b );
    float z = 0.0f;
    for ( int k = 1; k < m->terms; k++ )
        z += m->coef[ k ] * b[ k ];
    return z;
}

float surfaceMapCentreZ( const SurfaceMap* m ) {
    return m->coef[ 0 ];
}
