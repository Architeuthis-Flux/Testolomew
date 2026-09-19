// SPDX-License-Identifier: MIT
#include "ButtonTracker.h"

void buttonInit( ButtonTracker* b, uint32_t debounceMs, bool autoRepeat ) {
    b->debounceMs = debounceMs;
    b->autoRepeat = autoRepeat;
    b->raw = b->down = false;
    b->changedMs = b->downSinceMs = b->nextRepeatMs = 0;
    b->holdFired = b->longFired = false;
    b->repeats = 0;
}

int buttonFeed( ButtonTracker* b, bool rawDown, uint32_t nowMs, InputEventKind out[ 2 ] ) {
    if ( rawDown != b->raw ) {
        b->raw = rawDown;
        b->changedMs = nowMs;
    }
    if ( rawDown != b->down && (int32_t)( nowMs - b->changedMs ) >= (int32_t)b->debounceMs ) {
        b->down = rawDown;
        if ( rawDown ) {
            b->downSinceMs = nowMs;
            b->nextRepeatMs = nowMs + BUTTON_REPEAT_DELAY_MS;
            b->holdFired = b->longFired = false;
            b->repeats = 0;
            out[ 0 ] = IN_PRESS;
            return 1;
        }
        // Up: a tap if nothing fired while it was down.
        if ( !b->holdFired && b->repeats == 0 ) {
            out[ 0 ] = IN_CLICK;
            out[ 1 ] = IN_RELEASE;
            return 2;
        }
        out[ 0 ] = IN_RELEASE;
        return 1;
    }
    if ( !b->down ) {
        return 0;
    }
    uint32_t held = nowMs - b->downSinceMs;
    if ( b->autoRepeat ) {
        if ( (int32_t)( nowMs - b->nextRepeatMs ) >= 0 ) {
            b->repeats++;
            b->nextRepeatMs = nowMs + ( b->repeats >= BUTTON_REPEAT_FAST_AFTER ? BUTTON_REPEAT_FAST_MS : BUTTON_REPEAT_MS );
            out[ 0 ] = IN_REPEAT;
            return 1;
        }
        return 0;
    }
    if ( !b->longFired && held >= BUTTON_LONG_HOLD_MS ) {
        b->longFired = true;
        b->holdFired = true;
        out[ 0 ] = IN_LONG_HOLD;
        return 1;
    }
    if ( !b->holdFired && held >= BUTTON_HOLD_MS ) {
        b->holdFired = true;
        out[ 0 ] = IN_HOLD;
        return 1;
    }
    return 0;
}
