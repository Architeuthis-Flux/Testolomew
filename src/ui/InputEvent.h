// SPDX-License-Identifier: MIT
#ifndef INPUTEVENT_H
#define INPUTEVENT_H
// ---------------------------------------------------------------------------
// What the controls say, as events: which control, and what it did. Shared
// by the input service (which makes them), the UI shell (which routes them)
// and the host tests. No Arduino in here.
//
// One press of a control is: PRESS on the way down; then, while it stays
// down, either REPEATs (a direction key held: the cursor keeps moving) or,
// for a control that does not repeat, HOLD at BUTTON_HOLD_MS and LONG_HOLD
// at BUTTON_LONG_HOLD_MS; then on the way up RELEASE - preceded by CLICK
// if it came up before any REPEAT or HOLD. So a CLICK is a short tap and
// nothing else, a HOLD is never also a CLICK, and a screen that acts on
// CLICK is not also acted on by the screen that took the HOLD.
// ---------------------------------------------------------------------------

enum InputControl {
    IN_NAV_UP,
    IN_NAV_DOWN,
    IN_NAV_LEFT,
    IN_NAV_RIGHT,
    IN_NAV_PRESS,
    IN_JOY_UP, // the joystick as a four-way
    IN_JOY_DOWN,
    IN_JOY_LEFT,
    IN_JOY_RIGHT,
    IN_JOY_PRESS,
    IN_BTN_A,
    IN_BTN_B,
    IN_CONTROL_COUNT
};

enum InputEventKind {
    IN_PRESS,     // the control went down
    IN_RELEASE,   // ...and up again
    IN_REPEAT,    // still down: fires like a press, at the repeat rate (directions)
    IN_HOLD,      // down BUTTON_HOLD_MS: fires once
    IN_CLICK,     // came up before a REPEAT or HOLD: a tap (fires just before the RELEASE)
    IN_LONG_HOLD, // down BUTTON_LONG_HOLD_MS: fires once
    IN_EVENT_KIND_COUNT
};

struct InputEvent {
    InputControl control;
    InputEventKind kind;
};

#endif // INPUTEVENT_H
