// SPDX-License-Identifier: MIT
#ifndef INPUT_H
#define INPUT_H
// ---------------------------------------------------------------------------
// The physical controls: a navigation stick with a centre press, an analog
// joystick with a press, and two buttons - read, debounced and turned into
// the events of InputEvent.h (press, click, repeat, hold, long hold,
// release) by one ButtonTracker per control. The joystick is both an analog
// pair (for orbiting the camera) and, with hysteresis, a second four-way
// (for menus).
//
// The nav stick (ALPS RKJXM1015004) has four direction contacts, a diagonal
// closing two neighbours, and a push contact (PIN_NAV_PRESS) that closes on
// the centre push AND, as wired here, on every tilt as well. So the push
// contact says only "the stick is doing something"; what it is doing is
// decoded from the four direction contacts, read as one PATTERN and taken
// only once it has held still for the debounce time (contacts do not close
// on the same millisecond) and then INPUT_NAV_DIRECTION_MS more:
//   - a direction (one contact) or a diagonal (two) that holds that long is
//     a direction, whatever the push contact does - though with the push
//     contact closed it has to hold INPUT_NAV_DIRECTION_WITH_PUSH_MS, since
//     a centre push wobbles the stick into a direction for a while first;
//   - the push contact closed that long with NO direction contact closed is
//     the press;
//   - once one is decided the other is ignored until it releases: a push's
//     wobble into a direction is not a direction, a tilt's closing of the
//     push contact is not a press.
// With no push pin (-1) three or four direction contacts at once count as
// the press instead, and a press then stays on until at most one is left.
// The decoder's outputs go to their trackers with no further debounce
// (the guards are the debounce); the joystick press and the buttons settle
// INPUT_DEBOUNCE_MS. The two guards are settings (direction ms, push guard
// ms), so the feel can be tuned on the board.
//
// Pins are in BoardPins.h (PIN_NAV_*, PIN_JOY_*, PIN_BTN_*), -1 = not
// fitted. The same events can be typed on the serial console: arrow keys =
// the nav stick, Enter = its press, Tab = button A, ` (backtick) = button
// B, , . = joystick left/right, ; ' = joystick down/up, / = joystick press.
// The console hands those characters here first (consoleSetKeySink) and
// never sees them as commands. And the :key / :joy verbs (and the host
// simulator) drive the same simulation surface with holds and releases.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "ButtonTracker.h"
#include "InputEvent.h"
#include "JumperlOS.h"
#include "StickDpad.h"

#define INPUT_PERIOD_US 5000
#define INPUT_DEBOUNCE_MS BUTTON_DEBOUNCE_MS
#define INPUT_HOLD_MS BUTTON_HOLD_MS
#define INPUT_JOY_DEAD 0.04f  // the inner dead zone: this much of the travel round the centre is nothing (a scaled radial one: no step at its edge)
#define INPUT_JOY_OUTER 0.97f // ...and this much is full (a cheap stick does not quite reach the rails)
#define INPUT_EMULATED_NUDGE_MS 150 // a typed joystick key holds the stick this long
#define INPUT_TAP_MS 40             // a typed key or :key tap holds the control this long (and the next waits as long)
#define INPUT_NAV_SETTLE_MS 10      // the nav contacts' pattern has to hold still this long (contact bounce)
// The nav decoder's guards, the defaults of the settings "direction ms" and
// "push guard ms" (a saved value wins over these until a reset). One or
// two contacts must hold INPUT_NAV_DIRECTION_MS to be a direction, and
// INPUT_NAV_DIRECTION_WITH_PUSH_MS while the push contact is closed too;
// the push alone must hold INPUT_NAV_DIRECTION_MS to be the press. What
// they guard against, from the 2026-09-18 contact traces
// (tools/hostsim/navtest.cpp): on a tilt the push contact closes up to
// 15 ms BEFORE the direction contact, so a push-alone guard under 20 ms
// decodes tilts as presses; on a centre push the stick wobbles into a
// direction contact for 30-100 ms first, so a push guard under that
// decodes some presses as a step. 60/130 was safe and 170 ms late with
// the tracker's debounce on top; 20/50 (2026-09-19) is the fast end that
// still passes navtest - a wobble past 50 ms is a step, the trade Kevin
// took. 10/20 was tried and decodes tilts as presses.
#define INPUT_NAV_DIRECTION_MS 20
#define INPUT_NAV_DIRECTION_WITH_PUSH_MS 50
#define INPUT_NAV_TRACE 32          // nav pattern changes remembered for `J`
#define INPUT_EVENTS 32             // the event ring; a full one drops its oldest (counted: `j`)

// Where a control's level comes from: its pin, and the simulation on top.
struct InputSource {
    int pin; // -1 = not fitted
    bool activeLow;
    bool emulated;          // a tap being played
    uint32_t emulatedUntil; // ...until this time
    int taps;               // taps waiting to be played (each is a press and a release)
    uint32_t nextTapMs;     // not before this
    bool simDown;           // held by simPress() / simHold()
    uint32_t simUntil;      // ...until this time (0 = until simRelease())
};

class Input : public Service {
  public:
    static Input& getInstance( );

    Input( const Input& ) = delete;
    Input& operator=( const Input& ) = delete;

    void begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "Input"; }
    ServicePriority getPriority( ) const override { return ServicePriority::CRITICAL; }
    uint32_t periodUs( ) const override { return INPUT_PERIOD_US; }

    // Events, oldest first. false when there are none.
    bool next( InputEvent* e );
    bool held( InputControl c ) const { return trackers[ c ].down; }
    uint32_t heldMask( ) const; // bit c set while control c is down (debounced)
    uint32_t dropped = 0;       // events lost to a full ring

    // The joystick, -1..1 each way, + = right / up, the dead zone taken out
    // radially and the rest scaled to fill the range: joyRawX/Y linear (the
    // four-way's, StickDpad.h), joyX/Y with an expo on the magnitude (fine
    // near the centre: the camera's).
    float joyX = 0.0f, joyY = 0.0f;
    float joyRawX = 0.0f, joyRawY = 0.0f;
    bool joystickFitted = false;
    float joyFullScale = 4095.0f; // what analogRead() returns at full deflection
    StickDpad dpad;

    // The feel (the Settings menu's controls page, saved - a saved value
    // wins over the defines above until a reset): the decoder's guards in
    // ms, and the stick's four-way thresholds.
    float navDirectionMs = INPUT_NAV_DIRECTION_MS;
    float navPushGuardMs = INPUT_NAV_DIRECTION_WITH_PUSH_MS;
    float joyMenuAt = STICK_DPAD_ON;
    float joyMenuOff = STICK_DPAD_OFF;

    // Console `j`: everything as read, for checking the wiring; `J`: what the
    // nav stick's contacts did lately.
    void printInputs( Stream* out ) const;
    void printNavTrace( Stream* out ) const;

    // From the console's key sink.
    bool takeKey( char c );
    // Set by the UI while it wants Enter as the nav press (an overlay is
    // open); otherwise Enter stays the console's.
    bool uiWantsEnter = false;

    // The simulation surface: what the typed keys, the :key / :joy verbs and
    // the host simulator drive. A simulated control counts exactly as the
    // physical one (the same debounce, repeat and hold), and while a
    // simulated joystick is active the real one is not read.
    void simPress( InputControl c );                   // down until simRelease()
    void simRelease( InputControl c );
    void simTap( InputControl c );                     // a press and a release; several queue up and play one after another
    void simHold( InputControl c, uint32_t ms );       // down for this long
    void simJoystick( float x, float y, uint32_t ms ); // -1..1 each way; ms 0 = until the next call
    void simJoystickOff( );
    bool simActive( ) const;
    // For the host tests: which pin (and polarity) a control reads.
    void setPin( InputControl c, int pin, bool activeLow );

  private:
    Input( ) = default;

    InputSource sources[ IN_CONTROL_COUNT ];
    ButtonTracker trackers[ IN_CONTROL_COUNT ];
    InputEvent events[ INPUT_EVENTS ];
    int eventHead = 0, eventCount = 0;
    int escape = 0; // arrow-key escape sequence state
    bool lastWasCr = false;
    // The nav stick's four contacts as a pattern (bit 0 up, 1 down, 2 left, 3 right).
    uint8_t navRaw = 0, navStable = 0, navDecoded = 0;
    uint32_t navChangedMs = 0, navStableSinceMs = 0;
    bool navPressed = false; // the decoded press, with its release hysteresis
    bool navPushRaw = false, navPushStable = false;
    uint32_t navPushChangedMs = 0, navPushStableSinceMs = 0;
    bool navDirectionOn = false; // a direction/diagonal is decided and still held
    // What the contacts did, for `J`: the last changes of the raw pattern.
    uint8_t navTracePattern[ INPUT_NAV_TRACE ]; // bits 0-3 the contacts, bit 4 the push
    uint8_t navTraceLast = 0;
    uint32_t navTraceMs[ INPUT_NAV_TRACE ];
    int navTraceHead = 0, navTraceCount = 0;
    float simJoyX = 0.0f, simJoyY = 0.0f;
    bool simJoyOn = false;
    uint32_t simJoyUntil = 0; // 0 = until simJoystickOff()

    void post( InputControl c, InputEventKind k );
    bool pinDown( int c ) const;
    void decodeNav( uint32_t now, bool navDown[ 5 ] );
};

extern Input& input;

#endif // INPUT_H
