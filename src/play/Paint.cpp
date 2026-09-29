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

static void paintPickerWrap( float* hueDeg ) {
    while ( *hueDeg < 0.0f )
        *hueDeg += 360.0f;
    while ( *hueDeg >= 359.5f ) // the menu item (and so the saved value) runs 0-359
        *hueDeg -= 360.0f;
    if ( *hueDeg < 0.0f )
        *hueDeg = 0.0f;
}

void paintPickerMove( float* hueDeg, float* sat, int* cycling, float dx, float dy ) {
    float d = sqrtf( dx * dx + dy * dy );
    if ( *cycling != 0 ) {
        // Round the rim at the stick's tilt, whichever way it points.
        *hueDeg += ( *cycling > 0 ? d : -d ) * ( 180.0f / 3.14159265f );
        *sat = 1.0f;
        paintPickerWrap( hueDeg );
        return;
    }
    float rad = *hueDeg * ( 3.14159265f / 180.0f );
    float x = *sat * cosf( rad ) + dx, y = *sat * sinf( rad ) + dy;
    float r = sqrtf( x * x + y * y );
    if ( r > 1.0f ) {
        // Against the rim. The push in the marker's own frame: out along it
        // and round it; not inward, so the cycle starts - the way a sideways
        // push says, anticlockwise for one straight out - and this frame's
        // travel is the whole tilt, round.
        float cs = cosf( rad ), sn = sinf( rad );
        float radial = dx * cs + dy * sn, tangential = -dx * sn + dy * cs;
        if ( radial >= 0.0f && d > 0.0f ) {
            *cycling = fabsf( tangential ) > 0.5f * d ? ( tangential > 0.0f ? 1 : -1 ) : 1;
            *hueDeg += ( *cycling > 0 ? d : -d ) * ( 180.0f / 3.14159265f );
            *sat = 1.0f;
            paintPickerWrap( hueDeg );
            return;
        }
        x /= r;
        y /= r;
        r = 1.0f;
    }
    *sat = r;
    if ( r > 0.02f ) // at the centre the hue is whatever it was
        *hueDeg = atan2f( y, x ) * ( 180.0f / 3.14159265f );
    paintPickerWrap( hueDeg );
}

void paintPickerRelease( int* cycling ) {
    *cycling = 0;
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
