// SPDX-License-Identifier: MIT
#ifndef PAINT_H
#define PAINT_H
// ---------------------------------------------------------------------------
// The paint brush: what a dab of the brush does to the paint layer
// (ProbeLedPaint, ProbeLeds.h).
//
// The rule is NEWEST WINS. A stroke starts when the point comes down
// (paintStrokeBegin) and its high-water mark - how much of the brush each
// LED has had in THIS stroke - starts at nothing, so the first dab paints
// over whatever was there, colour and level, and within the stroke the
// brush's centre (weight 1) always beats an edge that passed earlier
// (weight less), while an edge passing later never dims a centre. So a line
// drawn along a row comes out as that row's LEDs at full, a stroke over an
// old one recolours it, and the soft edge stays soft.
//
// The weight falls continuously from 1 under the point to 0.15 at the
// brush's edge (paintWeight): no step, so a point sliding between two holes
// does not flicker the level.
//
// No Arduino in here; host-tested (test/test_paint).
// ---------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>

#include "ProbeLeds.h"

#define PAINT_EDGE_WEIGHT 0.15f // the least an LED at the brush's edge gets
#define PAINT_BRUSH_MAX 3       // rows around the LED under the point

struct PaintBrush {
    int size;         // 0 = the one LED, 1-3 = that many rows around it
    bool erase;       // take the paint away instead
    float bright;     // the paint's level under the point (0-1)
    uint8_t r, g, b;  // the colour
};

struct PaintStroke {
    bool down;                   // the point is on the board
    float base[ PROBELED_MAX ];  // how much of the brush each LED has had this stroke
};

// The brush's weight at `dRows` from its centre for a brush of `size`: 1 at
// the centre, 0.6 a row out, falling to PAINT_EDGE_WEIGHT at the edge, 0 beyond.
float paintWeight( float dRows, int size );
// How far the brush reaches, in rows.
float paintReach( int size );

void paintStrokeBegin( PaintStroke* stroke ); // the point came down: a new stroke
void paintStrokeEnd( PaintStroke* stroke );

// One dab of the brush centred on LED `centre` of `layout`. `skipLed` (-1 =
// none) is left alone (the target game's LED).
void paintDab( ProbeLedPaint* paint, PaintStroke* stroke, const LedLayout* layout, int centre, const PaintBrush* brush, int skipLed );

// Wipe the paint and the stroke.
void paintClear( ProbeLedPaint* paint, PaintStroke* stroke );

// The colour wheel's marker moved by (dx, dy) in radii (the joystick's tilt
// times the pick speed and the frame): hue round, saturation out, the rim at
// 1. Against the rim the move's excess beyond it becomes travel ROUND the
// rim toward where the stick points (2026-09-28, Kevin: "moving the analog
// stick up against the edge of the color wheel, it keeps moving around the
// edge"), so a stick held out to any side takes the marker round to that
// hue and stops there; at the centre the hue is whatever it was.
void paintPickerMove( float* hueDeg, float* sat, float dx, float dy );

#endif // PAINT_H
