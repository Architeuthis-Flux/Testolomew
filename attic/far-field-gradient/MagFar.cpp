// SPDX-License-Identifier: MIT
#include "MagFar.h"

#include <math.h>

// Solve the 3x3 system a x = b (Cramer's rule). false if a is singular.
static bool solve3( const float a[ 3 ][ 3 ], const float b[ 3 ], float x[ 3 ] ) {
    float c00 = a[ 1 ][ 1 ] * a[ 2 ][ 2 ] - a[ 1 ][ 2 ] * a[ 2 ][ 1 ];
    float c01 = a[ 1 ][ 2 ] * a[ 2 ][ 0 ] - a[ 1 ][ 0 ] * a[ 2 ][ 2 ];
    float c02 = a[ 1 ][ 0 ] * a[ 2 ][ 1 ] - a[ 1 ][ 1 ] * a[ 2 ][ 0 ];
    float det = a[ 0 ][ 0 ] * c00 + a[ 0 ][ 1 ] * c01 + a[ 0 ][ 2 ] * c02;
    if ( fabsf( det ) < 1e-30f ) {
        return false;
    }
    float inv = 1.0f / det;
    x[ 0 ] = inv * ( b[ 0 ] * c00 + a[ 0 ][ 1 ] * ( a[ 1 ][ 2 ] * b[ 2 ] - b[ 1 ] * a[ 2 ][ 2 ] ) + a[ 0 ][ 2 ] * ( b[ 1 ] * a[ 2 ][ 1 ] - a[ 1 ][ 1 ] * b[ 2 ] ) );
    x[ 1 ] = inv * ( a[ 0 ][ 0 ] * ( b[ 1 ] * a[ 2 ][ 2 ] - a[ 1 ][ 2 ] * b[ 2 ] ) + b[ 0 ] * c01 + a[ 0 ][ 2 ] * ( a[ 1 ][ 0 ] * b[ 2 ] - b[ 1 ] * a[ 2 ][ 0 ] ) );
    x[ 2 ] = inv * ( a[ 0 ][ 0 ] * ( a[ 1 ][ 1 ] * b[ 2 ] - b[ 1 ] * a[ 2 ][ 1 ] ) + a[ 0 ][ 1 ] * ( b[ 1 ] * a[ 2 ][ 0 ] - a[ 1 ][ 0 ] * b[ 2 ] ) + b[ 0 ] * c02 );
    return true;
}

bool magFarEstimate( const Vec3* sensors, const Vec3* fields, const bool* use, int count, float knownStrength, MagFarEstimate* out ) {
    MagFarEstimate e = { };
    *out = e;
    if ( count > MAGFAR_MAX_SENSORS ) {
        count = MAGFAR_MAX_SENSORS;
    }

    // The array's centre, and its half-diagonal (how far apart the sensors are).
    int used = 0;
    float cx = 0, cy = 0, cz = 0;
    for ( int i = 0; i < count; i++ ) {
        if ( use != nullptr && !use[ i ] )
            continue;
        used++;
        cx += sensors[ i ].x;
        cy += sensors[ i ].y;
        cz += sensors[ i ].z;
    }
    if ( used < 3 ) {
        return false;
    }
    cx /= used;
    cy /= used;
    cz /= used;
    float halfDiagonalSq = 0.0f;
    for ( int i = 0; i < count; i++ ) {
        if ( use != nullptr && !use[ i ] )
            continue;
        float dx = sensors[ i ].x - cx, dy = sensors[ i ].y - cy;
        if ( dx * dx + dy * dy > halfDiagonalSq )
            halfDiagonalSq = dx * dx + dy * dy;
    }

    // Fit a plane B_k = a_k + gx_k (x - cx) + gy_k (y - cy) to each component.
    // The normal matrix is the same for all three, so it is built once.
    float n[ 3 ][ 3 ] = { { 0 } };
    float rhs[ 3 ][ 3 ] = { { 0 } }; // per component: sum b, sum b dx, sum b dy
    for ( int i = 0; i < count; i++ ) {
        if ( use != nullptr && !use[ i ] )
            continue;
        float dx = sensors[ i ].x - cx, dy = sensors[ i ].y - cy;
        float row[ 3 ] = { 1.0f, dx, dy };
        float b[ 3 ] = { fields[ i ].x, fields[ i ].y, fields[ i ].z };
        for ( int a = 0; a < 3; a++ ) {
            for ( int c = 0; c < 3; c++ ) {
                n[ a ][ c ] += row[ a ] * row[ c ];
            }
            for ( int k = 0; k < 3; k++ ) {
                rhs[ k ][ a ] += row[ a ] * b[ k ];
            }
        }
    }
    float plane[ 3 ][ 3 ]; // per component: a, gx, gy
    for ( int k = 0; k < 3; k++ ) {
        if ( !solve3( n, rhs[ k ], plane[ k ] ) ) {
            return false; // the sensors are in a line: no plane
        }
    }
    e.centre = { cx, cy, cz };
    e.field = { plane[ 0 ][ 0 ], plane[ 1 ][ 0 ], plane[ 2 ][ 0 ] };

    // The gradient tensor. The two mixed in-plane derivatives are one number
    // (curl-free), so their two estimates are averaged; the out-of-plane
    // column comes from Maxwell.
    float gxx = plane[ 0 ][ 1 ], gyy = plane[ 1 ][ 2 ];
    float gxy = 0.5f * ( plane[ 0 ][ 2 ] + plane[ 1 ][ 1 ] );
    float gzx = plane[ 2 ][ 1 ], gzy = plane[ 2 ][ 2 ];
    float g[ 3 ][ 3 ] = {
        { gxx, gxy, gzx },
        { gxy, gyy, gzy },
        { gzx, gzy, -gxx - gyy },
    };
    float norm = 0.0f;
    for ( int a = 0; a < 3; a++ )
        for ( int c = 0; c < 3; c++ )
            norm += g[ a ][ c ] * g[ a ][ c ];
    e.gradientMt = sqrtf( norm );

    // Euler: G r = -3 B, r from the magnet to the centre.
    float minus3b[ 3 ] = { -3.0f * e.field.x, -3.0f * e.field.y, -3.0f * e.field.z };
    float r[ 3 ];
    bool solved = solve3( g, minus3b, r );
    if ( solved ) {
        e.position = { cx - r[ 0 ], cy - r[ 1 ], cz - r[ 2 ] };
        e.distanceMm = sqrtf( r[ 0 ] * r[ 0 ] + r[ 1 ] * r[ 1 ] + r[ 2 ] * r[ 2 ] );
    }

    // The crude range from the field's size, if the strength is known: |B| is
    // m/r^3 broadside and 2m/r^3 end-on; the geometric mean splits the difference.
    float bMag = sqrtf( e.field.x * e.field.x + e.field.y * e.field.y + e.field.z * e.field.z );
    if ( knownStrength > 0.0f && bMag > 0.0f ) {
        e.rangeMm = cbrtf( 1.414f * knownStrength / bMag );
    }

    // Worth having? Above the board, and far enough out for the finite
    // difference to mean something.
    e.valid = solved && e.position.z > 0.0f && e.distanceMm * e.distanceMm >= MAGFAR_MIN_DISTANCE_FACTOR * MAGFAR_MIN_DISTANCE_FACTOR * halfDiagonalSq;
    e.sigmaMm = MAGFAR_SIGMA_FRACTION * e.distanceMm;
    if ( e.sigmaMm < MAGFAR_SIGMA_FLOOR_MM ) {
        e.sigmaMm = MAGFAR_SIGMA_FLOOR_MM;
    }
    *out = e;
    return e.valid;
}
