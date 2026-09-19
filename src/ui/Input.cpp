// SPDX-License-Identifier: MIT
#include "Input.h"

#include "BoardPins.h"
#include "Console.h"

Input& input = Input::getInstance( );

Input& Input::getInstance( ) {
    static Input instance;
    return instance;
}

static bool keySink( char c ) {
    return input.takeKey( c );
}

static void onInputs( Stream* out ) {
    input.printInputs( out );
}

static void onNavTrace( Stream* out ) {
    input.printNavTrace( out );
}

void Input::begin( ) {
    static const int pins[ IN_CONTROL_COUNT ] = { PIN_NAV_UP, PIN_NAV_DOWN, PIN_NAV_LEFT, PIN_NAV_RIGHT, PIN_NAV_PRESS,
                                                  -1, -1, -1, -1, PIN_JOY_PRESS,
                                                  PIN_BTN_A, PIN_BTN_B };
    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        InputButton b = { };
        b.pin = pins[ c ];
        b.activeLow = true;
        buttons[ c ] = b;
        if ( b.pin >= 0 ) {
            pinMode( b.pin, INPUT_PULLUP ); // ASSUMPTION: switches to ground; the chip's pull-ups are enough
        }
    }
    joystickFitted = PIN_JOY_X >= 0 && PIN_JOY_Y >= 0;
    if ( joystickFitted ) {
        pinMode( PIN_JOY_X, INPUT );
        pinMode( PIN_JOY_Y, INPUT );
        // The Arduino default is 10 bits, whatever the converter has; ask for
        // the converter's 12 and scale by whatever came back (a mismatch here
        // reads as a stick jammed hard over: it did, on 2026-09-18).
        analogReadResolution( 12 );
        joyFullScale = (float)( ( 1 << analogReadResolutionBits( ) ) - 1 );
    }
    consoleSetKeySink( keySink );
    consoleAddCommand( 'j', "the controls as read: joystick raw and scaled, nav stick contacts, buttons", onInputs );
    consoleAddCommand( 'J', "the nav stick's last 32 contact changes, with timing (press it first, then J)", onNavTrace );
}

void Input::printNavTrace( Stream* out ) const {
    char line[ 100 ];
    out->println( "nav stick contact changes, oldest first (ms since the previous one; up down left right, push):" );
    for ( int n = 0; n < navTraceCount; n++ ) {
        int i = ( navTraceHead - navTraceCount + n + INPUT_NAV_TRACE ) % INPUT_NAV_TRACE;
        int prev = ( i - 1 + INPUT_NAV_TRACE ) % INPUT_NAV_TRACE;
        uint8_t p = navTracePattern[ i ];
        unsigned long dt = n == 0 ? 0 : (unsigned long)( navTraceMs[ i ] - navTraceMs[ prev ] );
        snprintf( line, sizeof( line ), "  +%6lu ms  %d %d %d %d  push %d%s", dt, ( p >> 0 ) & 1, ( p >> 1 ) & 1, ( p >> 2 ) & 1, ( p >> 3 ) & 1, ( p >> 4 ) & 1, p == 0 ? "  (released)" : "" );
        out->println( line );
    }
    if ( navTraceCount == 0 ) {
        out->println( "  (none yet)" );
    }
}

void Input::printInputs( Stream* out ) const {
    char line[ 200 ];
    if ( joystickFitted ) {
        snprintf( line, sizeof( line ), "joystick raw x %d y %d of %.0f (centre %.0f)  ->  x %+.2f y %+.2f (dead zone %.0f %%)  press %s",
                  analogRead( PIN_JOY_X ), analogRead( PIN_JOY_Y ), joyFullScale, 0.5f * joyFullScale, joyX, joyY, INPUT_JOY_DEAD * 100.0f,
                  buttons[ IN_JOY_PRESS ].state ? "DOWN" : "up" );
    } else {
        snprintf( line, sizeof( line ), "joystick not fitted (PIN_JOY_X/Y are -1); typed: x %+.2f y %+.2f", joyX, joyY );
    }
    out->println( line );
    snprintf( line, sizeof( line ), "nav stick contacts up %d down %d left %d right %d push %s (stable pattern 0x%x, decoded: %s%s%s%s%s)  buttons A %s B %s",
              ( navRaw >> 0 ) & 1, ( navRaw >> 1 ) & 1, ( navRaw >> 2 ) & 1, ( navRaw >> 3 ) & 1,
              buttons[ IN_NAV_PRESS ].pin < 0 ? "(no pin)" : ( navPushRaw ? "1" : "0" ), navStable,
              navPressed ? "PRESS" : "", buttons[ IN_NAV_UP ].state ? "up " : "", buttons[ IN_NAV_DOWN ].state ? "down " : "",
              buttons[ IN_NAV_LEFT ].state ? "left " : "", buttons[ IN_NAV_RIGHT ].state ? "right " : "",
              buttons[ IN_BTN_A ].state ? "DOWN" : "up", buttons[ IN_BTN_B ].state ? "DOWN" : "up" );
    out->println( line );
}

void Input::post( InputControl c, InputEventKind k ) {
    if ( eventCount >= INPUT_EVENTS ) {
        return; // dropped: nobody is reading them
    }
    events[ ( eventHead + eventCount ) % INPUT_EVENTS ] = { c, k };
    eventCount++;
}

bool Input::next( InputEvent* e ) {
    if ( eventCount == 0 ) {
        return false;
    }
    *e = events[ eventHead ];
    eventHead = ( eventHead + 1 ) % INPUT_EVENTS;
    eventCount--;
    return true;
}

// One control's debounced state, and the events it makes.
void Input::feed( InputControl c, bool down, uint32_t now ) {
    InputButton& b = buttons[ c ];
    if ( down != b.raw ) {
        b.raw = down;
        b.changedMs = now;
    }
    if ( down != b.state && (int32_t)( now - b.changedMs ) >= (int32_t)INPUT_DEBOUNCE_MS ) {
        b.state = down;
        if ( down ) {
            b.pressedMs = now;
            b.nextRepeatMs = now + INPUT_REPEAT_DELAY_MS;
            b.holdFired = false;
            post( c, IN_PRESS );
        } else {
            post( c, IN_RELEASE );
        }
    }
    if ( b.state ) {
        if ( (int32_t)( now - b.nextRepeatMs ) >= 0 ) {
            b.nextRepeatMs = now + INPUT_REPEAT_MS;
            post( c, IN_REPEAT );
        }
        if ( !b.holdFired && (int32_t)( now - b.pressedMs ) >= (int32_t)INPUT_HOLD_MS ) {
            b.holdFired = true;
            post( c, IN_HOLD );
        }
    }
}

// A typed key: a tap - pressed, then released a moment later (the console
// cannot say when a key goes up). Several typed in a row are played one
// after another, so "down down down" is three presses.
void Input::emulate( InputControl c, uint32_t now ) {
    (void)now;
    buttons[ c ].taps++;
}

bool Input::takeKey( char c ) {
    uint32_t now = millis( );
    // Arrow keys arrive as ESC [ A/B/C/D.
    if ( escape == 0 && c == 27 ) {
        escape = 1;
        return true;
    }
    if ( escape == 1 ) {
        escape = c == '[' ? 2 : 0;
        return true; // ESC followed by something else: both dropped
    }
    if ( escape == 2 ) {
        escape = 0;
        switch ( c ) {
        case 'A':
            emulate( IN_NAV_UP, now );
            return true;
        case 'B':
            emulate( IN_NAV_DOWN, now );
            return true;
        case 'C':
            emulate( IN_NAV_RIGHT, now );
            return true;
        case 'D':
            emulate( IN_NAV_LEFT, now );
            return true;
        default:
            return true;
        }
    }
    bool afterCr = lastWasCr;
    lastWasCr = c == '\r';
    switch ( c ) {
    case '\r':
    case '\n':
        // Enter is the nav press only while the UI wants it (a menu is open);
        // otherwise the console ignores newlines anyway. A terminal that sends
        // CR LF is one press, not two.
        if ( uiWantsEnter ) {
            if ( !( c == '\n' && afterCr ) ) {
                emulate( IN_NAV_PRESS, now );
            }
            return true;
        }
        return false;
    case '\t':
        emulate( IN_BTN_A, now );
        return true;
    case '`':
        emulate( IN_BTN_B, now );
        return true;
    case '/':
        emulate( IN_JOY_PRESS, now );
        return true;
    case ',':
        emulatedJoyX = -0.8f;
        emulatedJoyUntil = now + INPUT_EMULATED_NUDGE_MS;
        return true;
    case '.':
        emulatedJoyX = 0.8f;
        emulatedJoyUntil = now + INPUT_EMULATED_NUDGE_MS;
        return true;
    case ';':
        emulatedJoyY = -0.8f;
        emulatedJoyUntil = now + INPUT_EMULATED_NUDGE_MS;
        return true;
    case '\'':
        emulatedJoyY = 0.8f;
        emulatedJoyUntil = now + INPUT_EMULATED_NUDGE_MS;
        return true;
    default:
        return false;
    }
}

static float deadZone( float v ) {
    float a = v < 0 ? -v : v;
    if ( a < INPUT_JOY_DEAD ) {
        return 0.0f;
    }
    float s = ( a - INPUT_JOY_DEAD ) / ( 1.0f - INPUT_JOY_DEAD );
    s = s * s; // expo: fine control near the centre
    return v < 0 ? -s : s;
}

ServiceStatus Input::service( ) {
    uint32_t now = millis( );

    // The joystick: real if fitted, else whatever was typed.
    if ( joystickFitted ) {
        // ASSUMPTION: centre at half scale (`j` shows the raw readings).
        float half = 0.5f * joyFullScale;
        float x = ( analogRead( PIN_JOY_X ) - half ) / half;
        float y = ( analogRead( PIN_JOY_Y ) - half ) / half;
        if ( JOY_X_REVERSED )
            x = -x;
        if ( JOY_Y_REVERSED )
            y = -y;
        joyX = deadZone( x );
        joyY = deadZone( y );
    } else {
        if ( (int32_t)( now - emulatedJoyUntil ) >= 0 ) {
            emulatedJoyX = emulatedJoyY = 0.0f;
        }
        joyX = emulatedJoyX;
        joyY = emulatedJoyY;
    }

    // The nav stick: its four contacts as one pattern, decoded once it has
    // held still for the debounce time (see Input.h).
    uint8_t pattern = 0;
    for ( int c = IN_NAV_UP; c <= IN_NAV_RIGHT; c++ ) {
        const InputButton& b = buttons[ c ];
        if ( b.pin >= 0 && ( digitalRead( b.pin ) == LOW ) == b.activeLow ) {
            pattern |= (uint8_t)( 1 << ( c - IN_NAV_UP ) );
        }
    }
    bool pushNow = buttons[ IN_NAV_PRESS ].pin >= 0 && ( digitalRead( buttons[ IN_NAV_PRESS ].pin ) == LOW ) == buttons[ IN_NAV_PRESS ].activeLow;
    uint8_t traced = pattern | ( pushNow ? 0x10 : 0 );
    if ( traced != navTraceLast ) {
        navTraceLast = traced;
        navTracePattern[ navTraceHead ] = traced;
        navTraceMs[ navTraceHead ] = now;
        navTraceHead = ( navTraceHead + 1 ) % INPUT_NAV_TRACE;
        if ( navTraceCount < INPUT_NAV_TRACE )
            navTraceCount++;
    }
    if ( pattern != navRaw ) {
        navRaw = pattern;
        navChangedMs = now;
    }
    if ( pattern != navStable && (int32_t)( now - navChangedMs ) >= (int32_t)INPUT_DEBOUNCE_MS ) {
        navStable = pattern;
        navStableSinceMs = now;
    }
    // The push contact, debounced here (it is decoded, not fed straight to
    // the button machinery).
    if ( buttons[ IN_NAV_PRESS ].pin >= 0 ) {
        if ( pushNow != navPushRaw ) {
            navPushRaw = pushNow;
            navPushChangedMs = now;
        }
        if ( pushNow != navPushStable && (int32_t)( now - navPushChangedMs ) >= (int32_t)INPUT_DEBOUNCE_MS ) {
            navPushStable = pushNow;
            navPushStableSinceMs = now;
        }
    }
    // Decoding. Three or four contacts is the push, at once. One or two is a
    // direction (or a diagonal) only once it has held INPUT_NAV_DIRECTION_MS
    // - or, with the push contact closed, the longer
    // INPUT_NAV_DIRECTION_WITH_PUSH_MS: a centre push wobbles the stick into
    // a direction contact for up to a hundred-odd milliseconds before it
    // settles on the push alone, and that wobble was coming out as a
    // direction just before the press (2026-09-18). A tilt holds its
    // contact far longer than that. Nothing is released straight away.
    int closed = 0;
    for ( int k = 0; k < 4; k++ )
        closed += ( navStable >> k ) & 1;
    uint32_t hold = navPushStable ? INPUT_NAV_DIRECTION_WITH_PUSH_MS : INPUT_NAV_DIRECTION_MS;
    if ( closed >= 3 || closed == 0 || (int32_t)( now - navStableSinceMs ) >= (int32_t)hold ) {
        navDecoded = navStable;
    }
    int decodedClosed = 0;
    for ( int k = 0; k < 4; k++ )
        decodedClosed += ( navDecoded >> k ) & 1;
    bool navDown[ 5 ] = { false, false, false, false, false };
    if ( buttons[ IN_NAV_PRESS ].pin >= 0 ) {
        // Decide one thing at a time (see Input.h). A direction that has
        // held its time wins while it lasts; the push contact alone, with no
        // direction contact closed, for its time, is the press while it lasts.
        if ( navDirectionOn ) {
            if ( closed == 0 )
                navDirectionOn = false;
        } else if ( navPressed ) {
            if ( !navPushStable )
                navPressed = false;
        } else if ( decodedClosed > 0 && closed > 0 ) {
            navDirectionOn = true;
        } else if ( navPushStable && closed == 0 && (int32_t)( now - navPushStableSinceMs ) >= (int32_t)INPUT_NAV_DIRECTION_MS ) {
            navPressed = true;
        }
        if ( navDirectionOn ) {
            for ( int k = 0; k < 4; k++ )
                navDown[ k ] = ( navStable >> k ) & 1; // follow the contacts as they are (a roll from up to up+right)
        }
        navDown[ 4 ] = navPressed;
    } else {
        if ( decodedClosed >= 3 ) {
            navPressed = true; // no push pin: the centre push closing all four is the press
        } else if ( decodedClosed <= 1 ) {
            navPressed = false;
        } // two closed while leaving a press: still the press, not a diagonal
        navDown[ 4 ] = navPressed;
        if ( !navPressed ) {
            for ( int k = 0; k < 4; k++ )
                navDown[ k ] = ( navDecoded >> k ) & 1;
        }
    }

    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        InputButton& b = buttons[ c ];
        bool down = false;
        if ( c >= IN_NAV_UP && c <= IN_NAV_PRESS ) {
            down = navDown[ c - IN_NAV_UP ];
        } else if ( c >= IN_JOY_UP && c <= IN_JOY_RIGHT ) {
            // The stick as a four-way, with hysteresis.
            float v = c == IN_JOY_UP ? joyY : ( c == IN_JOY_DOWN ? -joyY : ( c == IN_JOY_RIGHT ? joyX : -joyX ) );
            down = b.state ? v > INPUT_JOY_OFF : v > INPUT_JOY_ON;
        } else if ( b.pin >= 0 ) {
            down = ( digitalRead( b.pin ) == LOW ) == b.activeLow;
        }
        if ( !b.emulated && b.taps > 0 && (int32_t)( now - b.nextTapMs ) >= 0 ) {
            b.emulated = true;
            b.emulatedUntil = now + INPUT_DEBOUNCE_MS + 30;
            b.taps--;
        }
        if ( b.emulated ) {
            if ( (int32_t)( now - b.emulatedUntil ) < 0 ) {
                down = true;
            } else {
                b.emulated = false;
                b.nextTapMs = now + INPUT_DEBOUNCE_MS + 30;
            }
        }
        feed( (InputControl)c, down, now );
    }

    lastStatus = eventCount > 0 ? ServiceStatus::BUSY : ServiceStatus::IDLE;
    return lastStatus;
}
