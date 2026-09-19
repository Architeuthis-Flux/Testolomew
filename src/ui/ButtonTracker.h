// SPDX-License-Identifier: MIT
#ifndef BUTTONTRACKER_H
#define BUTTONTRACKER_H
// ---------------------------------------------------------------------------
// One control's raw level, over time, turned into the events of
// InputEvent.h: a port of JumperlOS's EncoderClickTracker::poll
// (RotaryEncoder.cpp) with auto-repeat added for the direction keys.
//
// Debounce is by settling: the level has to hold debounceMs before it
// counts (0 = at once, for a level that is already debounced, like the nav
// stick decoder's). Then: PRESS on the down edge; while down, a repeating
// control fires REPEAT after BUTTON_REPEAT_DELAY_MS and then every
// BUTTON_REPEAT_MS (BUTTON_REPEAT_FAST_MS after BUTTON_REPEAT_FAST_AFTER of
// them: a number lever that runs), a non-repeating one fires HOLD at
// BUTTON_HOLD_MS and LONG_HOLD at BUTTON_LONG_HOLD_MS, once each; on the
// up edge, CLICK then RELEASE if nothing fired while it was down, else
// RELEASE alone. No Arduino in here; host-tested (test/test_input).
// ---------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>

#include "InputEvent.h"

#define BUTTON_DEBOUNCE_MS 20
#define BUTTON_REPEAT_DELAY_MS 400 // first repeat while held
#define BUTTON_REPEAT_MS 80        // then this often
#define BUTTON_REPEAT_FAST_MS 40   // ...and this often after
#define BUTTON_REPEAT_FAST_AFTER 20 // this many repeats
#define BUTTON_HOLD_MS 500
#define BUTTON_LONG_HOLD_MS 1500

struct ButtonTracker {
    uint32_t debounceMs;
    bool autoRepeat;
    bool raw, down; // the last raw level, and the debounced state
    uint32_t changedMs, downSinceMs, nextRepeatMs;
    bool holdFired, longFired;
    int repeats; // REPEATs fired in this press
};

void buttonInit( ButtonTracker* b, uint32_t debounceMs, bool autoRepeat );

// One sample. Returns how many events it made (0-2) in `out`, in order:
// PRESS; or CLICK, RELEASE / RELEASE; else at most one of LONG_HOLD, HOLD, REPEAT.
int buttonFeed( ButtonTracker* b, bool rawDown, uint32_t nowMs, InputEventKind out[ 2 ] );

#endif // BUTTONTRACKER_H
