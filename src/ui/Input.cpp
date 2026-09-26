// SPDX-License-Identifier: MIT
#include "Input.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

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

// :key <control> [tap|down|up|hold|xN] and :joy <x> <y> [ms] | off: the
// controls from the console, through the same path as the physical ones.
static const char* const controlNames[ IN_CONTROL_COUNT ] = { "up", "down", "left", "right", "press", "jup", "jdown", "jleft", "jright", "jpress", "a", "b" };

static int controlByName( const char* name ) {
    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        if ( strcmp( controlNames[ c ], name ) == 0 )
            return c;
    }
    return -1;
}

static void onKeyVerb( int argc, char** argv, Stream* out ) {
    if ( argc < 2 ) {
        consoleErr( out, "usage: :key <up|down|left|right|press|jup|jdown|jleft|jright|jpress|a|b> [tap|down|up|hold|xN]" );
        return;
    }
    int c = controlByName( argv[ 1 ] );
    if ( c < 0 ) {
        consoleErr( out, "no such control (up down left right press jup jdown jleft jright jpress a b)" );
        return;
    }
    const char* how = argc >= 3 ? argv[ 2 ] : "tap";
    char line[ 64 ];
    if ( strcmp( how, "tap" ) == 0 ) {
        input.simTap( (InputControl)c );
    } else if ( strcmp( how, "down" ) == 0 ) {
        input.simPress( (InputControl)c );
    } else if ( strcmp( how, "up" ) == 0 ) {
        input.simRelease( (InputControl)c );
    } else if ( strcmp( how, "hold" ) == 0 ) {
        input.simHold( (InputControl)c, BUTTON_HOLD_MS + 200 );
    } else if ( how[ 0 ] == 'x' && how[ 1 ] >= '0' && how[ 1 ] <= '9' ) {
        int n = atoi( how + 1 );
        for ( int k = 0; k < n && k < 64; k++ )
            input.simTap( (InputControl)c );
    } else {
        consoleErr( out, "the second word is tap, down, up, hold or xN" );
        return;
    }
    snprintf( line, sizeof( line ), "key %s %s", controlNames[ c ], how );
    consoleOk( out, line );
}

static void onJoyVerb( int argc, char** argv, Stream* out ) {
    if ( argc >= 2 && strcmp( argv[ 1 ], "off" ) == 0 ) {
        input.simJoystickOff( );
        consoleOk( out, "joy off" );
        return;
    }
    if ( argc < 3 ) {
        consoleErr( out, "usage: :joy <x> <y> [ms] (-1..1 each way; no ms = until :joy off) | :joy off" );
        return;
    }
    float x = atof( argv[ 1 ] ), y = atof( argv[ 2 ] );
    uint32_t ms = argc >= 4 ? (uint32_t)atol( argv[ 3 ] ) : 0;
    input.simJoystick( x, y, ms );
    char line[ 64 ];
    snprintf( line, sizeof( line ), "joy %.2f %.2f %lu ms", x, y, (unsigned long)ms );
    consoleOk( out, line );
}

void Input::begin( ) {
    static const int pins[ IN_CONTROL_COUNT ] = { PIN_NAV_UP, PIN_NAV_DOWN, PIN_NAV_LEFT, PIN_NAV_RIGHT, PIN_NAV_PRESS,
                                                  -1, -1, -1, -1, PIN_JOY_PRESS,
                                                  PIN_BTN_A, PIN_BTN_B };
    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        InputSource s = { };
        s.pin = pins[ c ];
        s.activeLow = true; // the stick's contacts and the buttons: switches to ground on the chip's pull-ups
        bool pullup = true;
        if ( c == IN_JOY_PRESS ) {
            s.activeLow = JOY_PRESS_ACTIVE_LOW != 0; // as wired (BoardPins.h)
            pullup = JOY_PRESS_PULLUP != 0;
        }
        sources[ c ] = s;
        if ( s.pin >= 0 ) {
            pinMode( s.pin, pullup ? INPUT_PULLUP : INPUT );
        }
        // The nav decoder's outputs and the joystick's four-way are debounced
        // already (the guards, the hysteresis): no more on top, so a tilt
        // counts as soon as it is decoded. The joystick press and the
        // buttons settle INPUT_DEBOUNCE_MS. The directions repeat, the
        // presses and buttons hold.
        bool direction = ( c >= IN_NAV_UP && c <= IN_NAV_RIGHT ) || ( c >= IN_JOY_UP && c <= IN_JOY_RIGHT );
        bool decoded = c <= IN_NAV_PRESS || ( c >= IN_JOY_UP && c <= IN_JOY_RIGHT );
        buttonInit( &trackers[ c ], decoded ? 0 : INPUT_DEBOUNCE_MS, direction );
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
    stickDpadInit( &dpad, joyMenuAt, joyMenuOff, STICK_DPAD_HOLD_MS, STICK_DPAD_REARM_MS );
    consoleSetKeySink( keySink );
    consoleAddCommand( 'j', "the controls as read: joystick raw and scaled, nav stick contacts, buttons", onInputs );
    consoleAddCommand( 'J', "the nav stick's last 32 contact changes, with timing (press it first, then J)", onNavTrace );
    consoleAddVerb( "key", "<control> [tap|down|up|hold|xN]", "a control from here: up down left right press jup jdown jleft jright jpress a b", CONSOLE_CHANGES, onKeyVerb );
    consoleAddVerb( "joy", "<x> <y> [ms] | off", "the joystick from here, -1..1 each way, in place of the stick", CONSOLE_CHANGES, onJoyVerb );
}

// ---- the simulation surface ------------------------------------------------------

void Input::simPress( InputControl c ) {
    sources[ c ].simDown = true;
    sources[ c ].simUntil = 0;
}

void Input::simRelease( InputControl c ) {
    sources[ c ].simDown = false;
    sources[ c ].simUntil = 0;
}

void Input::simHold( InputControl c, uint32_t ms ) {
    uint32_t until = millis( ) + ms;
    sources[ c ].simDown = true;
    sources[ c ].simUntil = until == 0 ? 1 : until;
}

// A tap: pressed, then released a moment later. Several in a row are played
// one after another, so "down down down" is three presses.
void Input::simTap( InputControl c ) {
    sources[ c ].taps++;
}

void Input::simJoystick( float x, float y, uint32_t ms ) {
    simJoyX = x < -1.0f ? -1.0f : ( x > 1.0f ? 1.0f : x );
    simJoyY = y < -1.0f ? -1.0f : ( y > 1.0f ? 1.0f : y );
    simJoyOn = true;
    uint32_t until = millis( ) + ms;
    simJoyUntil = ms == 0 ? 0 : ( until == 0 ? 1 : until );
}

void Input::simJoystickOff( ) {
    simJoyOn = false;
    simJoyX = simJoyY = 0.0f;
}

bool Input::simActive( ) const {
    if ( simJoyOn )
        return true;
    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        if ( sources[ c ].simDown || sources[ c ].emulated || sources[ c ].taps > 0 )
            return true;
    }
    return false;
}

void Input::setPin( InputControl c, int pin, bool activeLow ) {
    sources[ c ].pin = pin;
    sources[ c ].activeLow = activeLow;
}

uint32_t Input::heldMask( ) const {
    uint32_t mask = 0;
    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        if ( trackers[ c ].down )
            mask |= 1u << c;
    }
    return mask;
}

// ---- reports -----------------------------------------------------------------------

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
        snprintf( line, sizeof( line ), "joystick raw x %d y %d of %.0f (centre followed to %.0f %.0f)  ->  linear x %+.2f y %+.2f, expo x %+.2f y %+.2f (dead zone %.0f %%, full at %.0f %%)  press %s (pin %s, %s)",
                  analogRead( PIN_JOY_X ), analogRead( PIN_JOY_Y ), joyFullScale, joyCentreX, joyCentreY, joyRawX, joyRawY, joyX, joyY, joyDead * 100.0f, INPUT_JOY_OUTER * 100.0f,
                  trackers[ IN_JOY_PRESS ].down ? "DOWN" : "up", pinDown( IN_JOY_PRESS ) ? "reads pressed" : "reads released", sources[ IN_JOY_PRESS ].activeLow ? "active low" : "active high" );
    } else {
        snprintf( line, sizeof( line ), "joystick not fitted (PIN_JOY_X/Y are -1); typed: x %+.2f y %+.2f", joyX, joyY );
    }
    out->println( line );
    snprintf( line, sizeof( line ), "nav stick contacts up %d down %d left %d right %d push %s (stable pattern 0x%x, decoded: %s%s%s%s%s)  buttons A %s B %s",
              ( navRaw >> 0 ) & 1, ( navRaw >> 1 ) & 1, ( navRaw >> 2 ) & 1, ( navRaw >> 3 ) & 1,
              sources[ IN_NAV_PRESS ].pin < 0 ? "(no pin)" : ( navPushRaw ? "1" : "0" ), navStable,
              navPressed ? "PRESS" : "", trackers[ IN_NAV_UP ].down ? "up " : "", trackers[ IN_NAV_DOWN ].down ? "down " : "",
              trackers[ IN_NAV_LEFT ].down ? "left " : "", trackers[ IN_NAV_RIGHT ].down ? "right " : "",
              trackers[ IN_BTN_A ].down ? "DOWN" : "up", trackers[ IN_BTN_B ].down ? "DOWN" : "up" );
    out->println( line );
    snprintf( line, sizeof( line ), "events: %d waiting, %lu dropped (ring of %d); held mask 0x%03lx%s", eventCount, (unsigned long)dropped, INPUT_EVENTS, (unsigned long)heldMask( ),
              simActive( ) ? "; a simulated control is active" : "" );
    out->println( line );
    if ( joystickFitted ) {
        // The single ADC samples' spread since the last look: at rest, more
        // than a few counts is the reference (the 3.3 V rail) moving under
        // the conversions, not the hand.
        Input* self = const_cast<Input*>( this );
        snprintf( line, sizeof( line ), "joystick ADC spread since the last j: x %d..%d (%d counts), y %d..%d (%d counts) over %lu samples; %.1f %% of travel is %d counts", joyMinX, joyMaxX,
                  joyMaxX - joyMinX, joyMinY, joyMaxY, joyMaxY - joyMinY, (unsigned long)joySamples, 100.0f * joyDead, (int)( joyDead * 0.5f * joyFullScale ) );
        out->println( line );
        self->joyMinX = self->joyMinY = 4095;
        self->joyMaxX = self->joyMaxY = 0;
        self->joySamples = 0;
    }
    snprintf( line, sizeof( line ), "feel: a nav direction after %.0f ms (%.0f with the push contact closed), the press after %.0f ms alone, contacts settle %d ms; buttons %d ms bounce",
              navDirectionMs, navPushGuardMs, navDirectionMs, INPUT_NAV_SETTLE_MS, INPUT_DEBOUNCE_MS );
    out->println( line );
    snprintf( line, sizeof( line ), "      the stick is a menu direction past %.2f (over inside %.2f), one axis at a time, after %lu ms (a click cancels it), re-armed after %lu ms at rest%s%s",
              dpad.onAt, dpad.offAt, (unsigned long)dpad.holdMs, (unsigned long)dpad.rearmMs, dpad.armed[ 0 ] ? "" : "; x disarmed", dpad.armed[ 1 ] ? "" : "; y disarmed" );
    out->println( line );
}

// ---- events ------------------------------------------------------------------------

// A full ring drops its OLDEST: the newest event is the one that says what
// the hand is doing now.
void Input::post( InputControl c, InputEventKind k ) {
    if ( eventCount >= INPUT_EVENTS ) {
        eventHead = ( eventHead + 1 ) % INPUT_EVENTS;
        eventCount--;
        dropped++;
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

// ---- typed keys --------------------------------------------------------------------

bool Input::takeKey( char c ) {
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
            simTap( IN_NAV_UP );
            return true;
        case 'B':
            simTap( IN_NAV_DOWN );
            return true;
        case 'C':
            simTap( IN_NAV_RIGHT );
            return true;
        case 'D':
            simTap( IN_NAV_LEFT );
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
        // Enter is the nav press only while the UI wants it (an overlay is
        // open); otherwise the console ignores newlines anyway. A terminal
        // that sends CR LF is one press, not two.
        if ( uiWantsEnter ) {
            if ( !( c == '\n' && afterCr ) ) {
                simTap( IN_NAV_PRESS );
            }
            return true;
        }
        return false;
    case '\t':
        simTap( IN_BTN_A );
        return true;
    case '`':
        simTap( IN_BTN_B );
        return true;
    case '/':
        simTap( IN_JOY_PRESS );
        return true;
    case ',':
        simJoystick( -0.8f, simJoyOn ? simJoyY : 0.0f, INPUT_EMULATED_NUDGE_MS );
        return true;
    case '.':
        simJoystick( 0.8f, simJoyOn ? simJoyY : 0.0f, INPUT_EMULATED_NUDGE_MS );
        return true;
    case ';':
        simJoystick( simJoyOn ? simJoyX : 0.0f, -0.8f, INPUT_EMULATED_NUDGE_MS );
        return true;
    case '\'':
        simJoystick( simJoyOn ? simJoyX : 0.0f, 0.8f, INPUT_EMULATED_NUDGE_MS );
        return true;
    default:
        return false;
    }
}

// ---- reading -----------------------------------------------------------------------

// The stick's dead zone, radial and scaled (a circle round the centre is
// nothing, the ring from there to the outer edge is stretched to 0..1, so
// there is no step at the dead zone's edge and no snap to the axes), into
// the linear pair; the expo pair squares the magnitude (fine control near
// the centre for the camera) and keeps the direction.
static void shapeStick( float x, float y, float dead, float* linearX, float* linearY, float* expoX, float* expoY ) {
    if ( dead < 0.0f )
        dead = 0.0f;
    if ( dead > INPUT_JOY_OUTER - 0.05f )
        dead = INPUT_JOY_OUTER - 0.05f;
    float mag = sqrtf( x * x + y * y );
    if ( mag < dead || mag <= 0.0f ) {
        *linearX = *linearY = *expoX = *expoY = 0.0f;
        return;
    }
    float s = ( mag - dead ) / ( INPUT_JOY_OUTER - dead );
    if ( s > 1.0f )
        s = 1.0f;
    float ux = x / mag, uy = y / mag;
    *linearX = ux * s;
    *linearY = uy * s;
    *expoX = ux * s * s;
    *expoY = uy * s * s;
}

// One axis: three conversions, the median (a transient on one is thrown
// out), and the spread of the singles kept for `j`.
static int readAxis( int pin, int* minSeen, int* maxSeen ) {
    int a = analogRead( pin ), b = analogRead( pin ), c = analogRead( pin );
    int lo = a < b ? a : b, hi = a < b ? b : a;
    if ( lo < *minSeen )
        *minSeen = lo;
    if ( hi > *maxSeen )
        *maxSeen = hi;
    if ( c < *minSeen )
        *minSeen = c;
    if ( c > *maxSeen )
        *maxSeen = c;
    return c < lo ? lo : ( c > hi ? hi : c ); // the median of three
}

// The stick's centre follows a reading that has held still (within a few
// counts) for INPUT_JOY_CENTRE_STILL_MS near the centre it has: a resting
// stick that came back off-centre is centred again within a few seconds; a
// hand holding a direction moves more than that, or holds further out.
void Input::followCentre( int rx, int ry, uint32_t now, float half ) {
    const int stillCounts = 40;
    bool still = rx - joyStillX <= stillCounts && joyStillX - rx <= stillCounts && ry - joyStillY <= stillCounts && joyStillY - ry <= stillCounts;
    if ( !still ) {
        joyStillX = rx;
        joyStillY = ry;
        joyStillSinceMs = now;
        return;
    }
    float dx = ( rx - joyCentreX ) / half, dy = ( ry - joyCentreY ) / half;
    if ( dx * dx + dy * dy > INPUT_JOY_CENTRE_WITHIN * INPUT_JOY_CENTRE_WITHIN ) {
        return; // held out: a direction, not a rest
    }
    if ( now - joyStillSinceMs < INPUT_JOY_CENTRE_STILL_MS ) {
        return;
    }
    float alpha = ( INPUT_PERIOD_US * 1e-6f ) / INPUT_JOY_CENTRE_TAU_S;
    joyCentreX += alpha * ( rx - joyCentreX );
    joyCentreY += alpha * ( ry - joyCentreY );
}

bool Input::pinDown( int c ) const {
    const InputSource& s = sources[ c ];
    return s.pin >= 0 && ( digitalRead( s.pin ) == LOW ) == s.activeLow;
}

// The nav stick: its four contacts as one pattern, decoded once it has held
// still for the debounce time (see Input.h). Fills navDown[ 0-3 ] with the
// directions and navDown[ 4 ] with the press.
void Input::decodeNav( uint32_t now, bool navDown[ 5 ] ) {
    uint8_t pattern = 0;
    for ( int c = IN_NAV_UP; c <= IN_NAV_RIGHT; c++ ) {
        if ( pinDown( c ) ) {
            pattern |= (uint8_t)( 1 << ( c - IN_NAV_UP ) );
        }
    }
    bool pushNow = pinDown( IN_NAV_PRESS );
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
    if ( pattern != navStable && (int32_t)( now - navChangedMs ) >= (int32_t)INPUT_NAV_SETTLE_MS ) {
        navStable = pattern;
        navStableSinceMs = now;
    }
    // The push contact, debounced here (it is decoded, not fed straight to
    // the button machinery).
    if ( sources[ IN_NAV_PRESS ].pin >= 0 ) {
        if ( pushNow != navPushRaw ) {
            navPushRaw = pushNow;
            navPushChangedMs = now;
        }
        if ( pushNow != navPushStable && (int32_t)( now - navPushChangedMs ) >= (int32_t)INPUT_NAV_SETTLE_MS ) {
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
    uint32_t directionMs = (uint32_t)( navDirectionMs < 0.0f ? 0.0f : navDirectionMs );
    uint32_t hold = navPushStable ? (uint32_t)( navPushGuardMs < 0.0f ? 0.0f : navPushGuardMs ) : directionMs;
    if ( closed >= 3 || closed == 0 || (int32_t)( now - navStableSinceMs ) >= (int32_t)hold ) {
        navDecoded = navStable;
    }
    int decodedClosed = 0;
    for ( int k = 0; k < 4; k++ )
        decodedClosed += ( navDecoded >> k ) & 1;
    for ( int k = 0; k < 5; k++ )
        navDown[ k ] = false;
    if ( sources[ IN_NAV_PRESS ].pin >= 0 ) {
        // Decide one thing at a time (see Input.h). A direction that has
        // held its time wins while it lasts; the push contact alone, with no
        // direction contact closed, for its time, is the press while it lasts.
        if ( navDirectionOn ) {
            if ( closed == 0 ) {
                navDirectionOn = false;
                // The push contact outlives a tilt's direction contact by a
                // few milliseconds: that tail is not a press. The push-alone
                // clock starts now.
                navPushStableSinceMs = now;
            }
        } else if ( navPressed ) {
            if ( !navPushStable )
                navPressed = false;
        } else if ( decodedClosed > 0 && closed > 0 ) {
            navDirectionOn = true;
        } else if ( navPushStable && closed == 0 && (int32_t)( now - navPushStableSinceMs ) >= (int32_t)directionMs ) {
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
}

ServiceStatus Input::service( ) {
    uint32_t now = millis( );

    // The joystick: a simulated one while there is one, else the real one if
    // fitted, else centred.
    if ( simJoyOn && simJoyUntil != 0 && (int32_t)( now - simJoyUntil ) >= 0 ) {
        simJoystickOff( );
    }
    if ( simJoyOn ) {
        joyRawX = joyX = simJoyX; // a simulated stick is what it says, both ways
        joyRawY = joyY = simJoyY;
    } else if ( joystickFitted ) {
        // The centre: half scale to start with, then followed slowly while
        // the stick sits still near it (`j` shows the raw readings and the centre).
        float half = 0.5f * joyFullScale;
        int rx = readAxis( PIN_JOY_X, &joyMinX, &joyMaxX );
        int ry = readAxis( PIN_JOY_Y, &joyMinY, &joyMaxY );
        joySamples += 3;
        followCentre( rx, ry, now, half );
        float x = ( rx - joyCentreX ) / half;
        float y = ( ry - joyCentreY ) / half;
        if ( JOY_X_REVERSED )
            x = -x;
        if ( JOY_Y_REVERSED )
            y = -y;
        shapeStick( x, y, joyDead, &joyRawX, &joyRawY, &joyX, &joyY );
    } else {
        joyX = joyY = joyRawX = joyRawY = 0.0f;
    }

    bool navDown[ 5 ];
    decodeNav( now, navDown );

    // Every control's level: the pins and the decoder, with the simulation
    // on top (taps played one after another, and holds).
    bool level[ IN_CONTROL_COUNT ];
    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        InputSource& s = sources[ c ];
        bool down = false;
        if ( c >= IN_NAV_UP && c <= IN_NAV_PRESS ) {
            down = navDown[ c - IN_NAV_UP ];
        } else if ( c < IN_JOY_UP || c > IN_JOY_RIGHT ) {
            down = pinDown( c );
        }
        if ( !s.emulated && s.taps > 0 && (int32_t)( now - s.nextTapMs ) >= 0 ) {
            s.emulated = true;
            s.emulatedUntil = now + INPUT_TAP_MS;
            s.taps--;
        }
        if ( s.emulated ) {
            if ( (int32_t)( now - s.emulatedUntil ) < 0 ) {
                down = true;
            } else {
                s.emulated = false;
                s.nextTapMs = now + INPUT_TAP_MS;
            }
        }
        if ( s.simDown && s.simUntil != 0 && (int32_t)( now - s.simUntil ) >= 0 ) {
            s.simDown = false;
        }
        if ( s.simDown ) {
            down = true;
        }
        level[ c ] = down;
    }
    // The stick as a four-way (StickDpad.h): the settings may have moved.
    dpad.onAt = joyMenuAt;
    dpad.offAt = joyMenuOff < joyMenuAt ? joyMenuOff : 0.6f * joyMenuAt; // 'off' has to be under 'on' to be any use
    stickDpadFeed( &dpad, joyRawX, joyRawY, level[ IN_JOY_PRESS ], now );
    for ( int c = IN_JOY_UP; c <= IN_JOY_RIGHT; c++ ) {
        level[ c ] = level[ c ] || dpad.down[ c - IN_JOY_UP ]; // (a :key jup tap on top)
    }

    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        InputEventKind made[ 2 ];
        int n = buttonFeed( &trackers[ c ], level[ c ], now, made );
        for ( int k = 0; k < n; k++ ) {
            post( (InputControl)c, made[ k ] );
        }
    }

    lastStatus = eventCount > 0 ? ServiceStatus::BUSY : ServiceStatus::IDLE;
    return lastStatus;
}
