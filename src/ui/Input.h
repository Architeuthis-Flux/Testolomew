// SPDX-License-Identifier: MIT
#ifndef INPUT_H
#define INPUT_H
// ---------------------------------------------------------------------------
// The physical controls: a navigation stick with a centre press, an analog
// joystick with a press, and two buttons - read, debounced and turned into
// events (press, release, repeat while held, long hold) that the UI
// consumes. The joystick is both an analog pair (for orbiting the camera) and,
// with hysteresis, a second four-way (for menus).
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
//
// None of them is wired yet: pins are in BoardPins.h (PIN_NAV_*, PIN_JOY_*,
// PIN_BTN_*), -1 = not fitted, and until they are the same events can be
// typed on the serial console: arrow keys = the nav stick, Enter = its
// press, Tab = button A, ` (backtick) = button B, , . = joystick left/right,
// ; ' = joystick down/up, / = joystick press. The console hands those
// characters here first (consoleSetKeySink) and never sees them as commands.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"

#define INPUT_PERIOD_US 5000
#define INPUT_DEBOUNCE_MS 20
#define INPUT_REPEAT_DELAY_MS 400   // first repeat while held
#define INPUT_REPEAT_MS 80          // then this often
#define INPUT_HOLD_MS 700           // a long hold
#define INPUT_JOY_DEAD 0.05f        // fraction of travel ignored round the centre
#define INPUT_JOY_ON 0.6f           // as a four-way: past this = pressed...
#define INPUT_JOY_OFF 0.4f          // ...back inside this = released
#define INPUT_EMULATED_NUDGE_MS 150 // a typed joystick key holds the stick this long
#define INPUT_NAV_DIRECTION_MS 60           // one or two nav contacts must hold this long to be a direction (a push closes all four, but not at once)
#define INPUT_NAV_DIRECTION_WITH_PUSH_MS 130 // ...and this long while the push contact is closed: a centre push's wobble into a direction lasts up to about 100 ms
#define INPUT_NAV_TRACE 32          // nav pattern changes remembered for `J`
#define INPUT_EVENTS 16

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
    IN_PRESS,
    IN_RELEASE,
    IN_REPEAT, // still held: fires like a press, at the repeat rate
    IN_HOLD    // held INPUT_HOLD_MS: fires once
};

struct InputEvent {
    InputControl control;
    InputEventKind kind;
};

struct InputButton {
    int pin; // -1 = not fitted
    bool activeLow;
    bool raw, state; // last raw read, debounced state
    uint32_t changedMs, pressedMs, nextRepeatMs;
    bool holdFired;
    bool emulated;          // pressed from the console (a tap being played)
    uint32_t emulatedUntil; // ...until this time
    int taps;               // typed presses waiting to be played (each is a press and a release)
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
    bool held( InputControl c ) const { return buttons[ c ].state; }

    // The joystick, -1..1 each way, dead zone taken out, + = right / up.
    float joyX = 0.0f, joyY = 0.0f;
    bool joystickFitted = false;
    float joyFullScale = 4095.0f; // what analogRead() returns at full deflection

    // Console `j`: everything as read, for checking the wiring; `J`: what the
    // nav stick's contacts did lately.
    void printInputs( Stream* out ) const;
    void printNavTrace( Stream* out ) const;

    // From the console's key sink.
    bool takeKey( char c );

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
    // Bit c set while control c is down (debounced).
    uint32_t heldMask( ) const;
    // Set by the UI while a menu is open: Enter on the console is then the
    // nav press (otherwise it stays the console's).
    bool uiWantsEnter = false;

  private:
    Input( ) = default;

    InputButton buttons[ IN_CONTROL_COUNT ];
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
    void feed( InputControl c, bool down, uint32_t now );
    void emulate( InputControl c, uint32_t now );
};

extern Input& input;

#endif // INPUT_H
