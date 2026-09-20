// SPDX-License-Identifier: MIT
#ifndef STICKDPAD_H
#define STICKDPAD_H
// ---------------------------------------------------------------------------
// The analog stick as a four-way (the menus' up/down/left/right), done the
// way game UIs do it:
//
//  - AXIAL thresholds, not radial: a four-way only ever wants one of four
//    vectors, and the axis with the larger deflection is the only one that
//    counts (a diagonal push is one direction, never two).
//  - HYSTERESIS: a direction starts past onAt and ends back inside offAt.
//  - A short HOLD before a crossing counts (holdMs): a click of the stick's
//    own button tilts the stick a little, and the button going down while
//    a crossing is waiting cancels it - so a click never moves the cursor
//    first. A flick still counts within holdMs.
//  - RE-ARM after a release (rearmMs): a stick let go from full tilt
//    springs back THROUGH the centre and rings for a moment, which at a
//    low threshold fires the opposite direction (Godot's UI navigation hit
//    exactly this when its threshold went from 0.5 to 0.2). An axis whose
//    direction has just ended is disarmed until it has sat inside the off
//    band for rearmMs; a deliberate second push takes longer than that.
//
// Feed it the linear stick every tick; read down[]. No Arduino in here;
// host-tested (test/test_input).
// ---------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>

#define STICK_DPAD_ON 0.25f     // a direction past this much of the raw travel (the setting "joy menu at")
#define STICK_DPAD_OFF 0.15f    // ...and over once back inside this (the setting "joy menu off")
#define STICK_DPAD_HOLD_MS 30   // a crossing has to last this long (a click's tilt does not)
#define STICK_DPAD_REARM_MS 80  // after a direction ends, that axis has to rest inside the off band this long

enum StickDpadDirection {
    STICK_UP,
    STICK_DOWN,
    STICK_LEFT,
    STICK_RIGHT
};

struct StickDpad {
    float onAt, offAt;
    uint32_t holdMs, rearmMs;
    int axis; // the axis with a direction on: -1 none, 0 x, 1 y
    int sign; // its sign, -1 / +1
    int pendingAxis, pendingSign; // a crossing waiting out holdMs (-1 = none)
    uint32_t pendingSinceMs;
    bool inside[ 2 ]; // per axis: inside the off band, since
    uint32_t insideSinceMs[ 2 ];
    bool armed[ 2 ];
    bool down[ 4 ]; // STICK_UP (+y), STICK_DOWN, STICK_LEFT, STICK_RIGHT (+x)
};

void stickDpadInit( StickDpad* d, float onAt, float offAt, uint32_t holdMs, uint32_t rearmMs );
// One sample: the stick, linear, -1..1 each way (+x right, +y up), and
// whether the stick's own button is down. Updates down[].
void stickDpadFeed( StickDpad* d, float x, float y, bool pressDown, uint32_t nowMs );

#endif // STICKDPAD_H
