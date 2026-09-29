// SPDX-License-Identifier: MIT
#include "Paint.h"

#include <math.h>

float paintReach( int size ) {
    if ( size < 0 )
        size = 0;
    if ( size > PAINT_BRUSH_MAX )
        size = PAINT_BRUSH_MAX;
    return size + 0.1f; // the LED itself, or that many rows out (a little slack for the grid's rounding)
}

// 1 up to half a row out, then down to 0.6 at one row, then 0.15 a row at
// a time (the look the old brush had beyond one row), never below the edge
// weight while within reach, 0 beyond it.
float paintWeight( float dRows, int size ) {
    if ( dRows < 0.0f )
        dRows = -dRows;
    if ( dRows > paintReach( size ) )
        return 0.0f;
    float w;
    if ( dRows <= 0.5f ) {
        w = 1.0f;
    } else if ( dRows <= 1.0f ) {
        w = 1.0f - 0.8f * ( dRows - 0.5f );
    } else {
        w = 0.6f - 0.15f * ( dRows - 1.0f );
    }
    return w < PAINT_EDGE_WEIGHT ? PAINT_EDGE_WEIGHT : w;
}

void paintStrokeBegin( PaintStroke* stroke ) {
    stroke->down = true;
    for ( int i = 0; i < PROBELED_MAX; i++ )
        stroke->base[ i ] = 0.0f;
}

void paintStrokeEnd( PaintStroke* stroke ) {
    stroke->down = false;
}

void paintPickerMove( float* hueDeg, float* sat, float dx, float dy ) {
    float rad = *hueDeg * ( 3.14159265f / 180.0f );
    float x = *sat * cosf( rad ) + dx, y = *sat * sinf( rad ) + dy;
    float r = sqrtf( x * x + y * y );
    if ( r > 1.0f ) {
        // Against the rim: what would have gone further out goes round it
        // instead, toward where the stick points, and no further than that.
        float at = atan2f( y, x ), want = atan2f( dy, dx );
        float diff = want - at;
        while ( diff > 3.14159265f )
            diff -= 2.0f * 3.14159265f;
        while ( diff < -3.14159265f )
            diff += 2.0f * 3.14159265f;
        float step = r - 1.0f; // radii beyond the rim = radians round it
        if ( step > fabsf( diff ) )
            step = fabsf( diff );
        at += diff > 0.0f ? step : -step;
        x = cosf( at );
        y = sinf( at );
        r = 1.0f;
    }
    *sat = r;
    if ( r > 0.02f ) // at the centre the hue is whatever it was
        *hueDeg = atan2f( y, x ) * ( 180.0f / 3.14159265f );
    if ( *hueDeg < 0.0f )
        *hueDeg += 360.0f;
    if ( *hueDeg >= 359.5f ) // the menu item (and so the saved value) runs 0-359
        *hueDeg = 0.0f;
}

void paintClear( ProbeLedPaint* paint, PaintStroke* stroke ) {
    probeLedPaintClear( paint );
    stroke->down = false;
    for ( int i = 0; i < PROBELED_MAX; i++ )
        stroke->base[ i ] = 0.0f;
}

void paintDab( ProbeLedPaint* paint, PaintStroke* stroke, const LedLayout* layout, int centre, const PaintBrush* brush, int skipLed ) {
    if ( centre < 0 || centre >= layout->count )
        return;
    float reach = paintReach( brush->size );
    for ( int k = 0; k < layout->count; k++ ) {
        if ( k == skipLed )
            continue;
        float da = layout->along[ k ] - layout->along[ centre ];
        float dc = ( layout->acrossMm[ k ] - layout->acrossMm[ centre ] ) / 2.54f; // across at the hole pitch too (the rails are LEDs too)
        float d = sqrtf( da * da + dc * dc );
        if ( d > reach )
            continue;
        if ( brush->erase ) {
            stroke->base[ k ] = 0.0f;
            paint->level[ k ] = 0.0f;
            continue;
        }
        float w = paintWeight( d, brush->size );
        if ( w < stroke->base[ k ] )
            continue; // this stroke has already given the LED more
        stroke->base[ k ] = w;
        paint->level[ k ] = w * brush->bright;
        paint->r[ k ] = brush->r;
        paint->g[ k ] = brush->g;
        paint->b[ k ] = brush->b;
    }
}
