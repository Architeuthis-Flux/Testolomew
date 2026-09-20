// SPDX-License-Identifier: MIT
#include "UiShell.h"

#include <math.h>
#include <string.h>

static void noteChange( UiShell* s ) {
    // The focus moved: whatever is still down is nobody's until it is let
    // go, and the stick is nobody's until it has been centred.
    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        s->swallow[ c ] = s->held[ c ];
    }
    s->joyArmed = false;
    s->generation++;
}

void uiShellInit( UiShell* s, const UiApp* apps, int appCount, int firstApp, int settingsCell ) {
    s->apps = apps;
    s->appCount = appCount;
    s->app = -1;
    s->previousApp = -1;
    s->settingsCell = settingsCell;
    s->settingsIcon = nullptr;
    s->depth = 0;
    homeInit( &s->home, appCount + 1 );
    menuInit( &s->menu );
    s->confirmItem = -1;
    s->resultTitle[ 0 ] = '\0';
    s->resultScroll = s->resultLines = s->resultVisible = 0;
    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        s->held[ c ] = s->swallow[ c ] = false;
    }
    s->joyArmed = false;
    s->lastInputMs = 0;
    s->generation = 1;
    s->absoluteJoystick = false;
    s->absolutePeak = 0.0f;
    s->absoluteFrozen = false;
    s->menuRows = 9;
    s->menuScrollTop = 0;
    if ( firstApp >= 0 && firstApp < appCount ) {
        s->app = firstApp;
        if ( apps[ firstApp ].enter != nullptr )
            apps[ firstApp ].enter( );
    }
}

PaneKind uiShellTop( const UiShell* s ) {
    return s->depth == 0 ? PANE_APP : s->stack[ s->depth - 1 ];
}

bool uiShellOverlayOpen( const UiShell* s ) {
    return s->depth > 0;
}

const UiApp* uiShellApp( const UiShell* s ) {
    return s->app >= 0 && s->app < s->appCount ? &s->apps[ s->app ] : nullptr;
}

int uiShellHomeCellApp( const UiShell* s, int cell ) {
    if ( cell == s->settingsCell )
        return -1;
    int app = cell < s->settingsCell ? cell : cell - 1;
    return app >= 0 && app < s->appCount ? app : -1;
}

static int homeCellOfApp( const UiShell* s, int app ) {
    return app < s->settingsCell ? app : app + 1;
}

static void push( UiShell* s, PaneKind kind ) {
    if ( s->depth < UISHELL_MAX_DEPTH ) {
        s->stack[ s->depth++ ] = kind;
    }
    noteChange( s );
}

static void pop( UiShell* s ) {
    if ( s->depth > 0 ) {
        s->depth--;
    }
    noteChange( s );
}

void uiShellCloseAll( UiShell* s ) {
    s->depth = 0;
    s->confirmItem = -1;
    noteChange( s );
}

void uiShellSelectApp( UiShell* s, int app ) {
    if ( app < 0 || app >= s->appCount ) {
        return;
    }
    if ( app != s->app ) {
        const UiApp* old = uiShellApp( s );
        if ( old != nullptr && old->exit != nullptr )
            old->exit( );
        s->previousApp = s->app;
        s->app = app;
        if ( s->apps[ app ].enter != nullptr )
            s->apps[ app ].enter( );
    }
    uiShellCloseAll( s );
}

void uiShellOpenHome( UiShell* s ) {
    if ( uiShellTop( s ) == PANE_HOME )
        return;
    s->home.cursor = s->app >= 0 ? homeCellOfApp( s, s->app ) : 0;
    push( s, PANE_HOME );
}

void uiShellOpenMenu( UiShell* s ) {
    if ( uiShellTop( s ) == PANE_MENU )
        return;
    push( s, PANE_MENU );
}

void uiShellShowResult( UiShell* s, const char* title, int lines, int visible ) {
    strncpy( s->resultTitle, title == nullptr ? "" : title, sizeof( s->resultTitle ) - 1 );
    s->resultTitle[ sizeof( s->resultTitle ) - 1 ] = '\0';
    s->resultLines = lines;
    s->resultVisible = visible;
    s->resultScroll = 0;
    if ( uiShellTop( s ) != PANE_RESULT )
        push( s, PANE_RESULT );
    else
        noteChange( s );
}

// B held anywhere above the app: everything closes and Home opens.
static void homeFromAnywhere( UiShell* s ) {
    uiShellCloseAll( s );
    uiShellOpenHome( s );
}

static bool isDirection( InputControl c ) {
    return ( c >= IN_NAV_UP && c <= IN_NAV_RIGHT ) || ( c >= IN_JOY_UP && c <= IN_JOY_RIGHT );
}

static MenuKey directionKey( InputControl c ) {
    switch ( c ) {
    case IN_NAV_UP:
    case IN_JOY_UP:
        return MENUKEY_UP;
    case IN_NAV_DOWN:
    case IN_JOY_DOWN:
        return MENUKEY_DOWN;
    case IN_NAV_LEFT:
    case IN_JOY_LEFT:
        return MENUKEY_LEFT;
    default:
        return MENUKEY_RIGHT;
    }
}

// The presses select on their way DOWN in the panes (a click - the release
// of a short press - felt like nothing happening; the press's own click and
// release are then swallowed by the focus change, or ignored).
static bool isSelect( InputControl c ) {
    return c == IN_NAV_PRESS || c == IN_JOY_PRESS || c == IN_BTN_A;
}

// The menu's answer to an ENTER: an action to run now, or a question first.
static int menuAction( UiShell* s, int action ) {
    if ( action == MENU_AT_ROOT ) {
        pop( s );
        return -1;
    }
    if ( action < 0 )
        return -1;
    if ( s->menu.items[ action ].confirm ) {
        s->confirmItem = action;
        push( s, PANE_CONFIRM );
        return -1;
    }
    return action;
}

static bool isJoystickDirection( InputControl c ) {
    return c >= IN_JOY_UP && c <= IN_JOY_RIGHT;
}

int uiShellEvent( UiShell* s, InputEvent e, uint32_t nowMs ) {
    InputControl c = e.control;
    s->lastInputMs = nowMs == 0 ? 1 : nowMs;
    if ( e.kind == IN_PRESS )
        s->held[ c ] = true;
    if ( e.kind == IN_RELEASE )
        s->held[ c ] = false;
    if ( s->swallow[ c ] ) {
        // Nobody's until it is let go.
        if ( e.kind == IN_RELEASE )
            s->swallow[ c ] = false;
        return -1;
    }
    if ( s->absoluteJoystick && isJoystickDirection( c ) && ( uiShellTop( s ) == PANE_HOME || uiShellTop( s ) == PANE_MENU ) ) {
        return -1; // the stick's position steers these (uiShellTick), not its four-way
    }
    bool press = e.kind == IN_PRESS || e.kind == IN_REPEAT;
    bool repeat = e.kind == IN_REPEAT;
    s->generation++;

    switch ( uiShellTop( s ) ) {
    case PANE_APP: {
        if ( c == IN_BTN_A ) {
            if ( e.kind == IN_CLICK )
                uiShellOpenHome( s );
            else if ( e.kind == IN_HOLD )
                uiShellOpenMenu( s );
            return -1;
        }
        if ( c == IN_BTN_B ) {
            if ( e.kind == IN_CLICK ) {
                if ( s->previousApp >= 0 && s->previousApp < s->appCount )
                    uiShellSelectApp( s, s->previousApp );
                else
                    uiShellOpenHome( s );
            } else if ( e.kind == IN_HOLD ) {
                uiShellOpenHome( s );
            }
            return -1;
        }
        const UiApp* app = uiShellApp( s );
        if ( app != nullptr && app->event != nullptr )
            app->event( &e );
        return -1;
    }
    case PANE_HOME:
        if ( isDirection( c ) && press ) {
            homeKey( &s->home, directionKey( c ) );
        } else if ( isSelect( c ) && e.kind == IN_PRESS ) {
            int app = uiShellHomeCellApp( s, s->home.cursor );
            if ( app >= 0 ) {
                uiShellSelectApp( s, app );
            } else {
                pop( s ); // Settings: the menu in Home's place
                uiShellOpenMenu( s );
            }
        } else if ( c == IN_BTN_B && e.kind == IN_CLICK ) {
            pop( s );
        }
        return -1;
    case PANE_MENU:
        if ( isDirection( c ) && press ) {
            return menuAction( s, menuKey( &s->menu, directionKey( c ), repeat ) );
        }
        if ( isSelect( c ) && e.kind == IN_PRESS ) {
            return menuAction( s, menuKey( &s->menu, MENUKEY_ENTER, false ) );
        }
        if ( c == IN_BTN_B ) {
            if ( e.kind == IN_CLICK )
                return menuAction( s, menuKey( &s->menu, MENUKEY_BACK, false ) );
            if ( e.kind == IN_HOLD )
                homeFromAnywhere( s );
        }
        return -1;
    case PANE_CONFIRM:
        if ( isSelect( c ) && e.kind == IN_PRESS ) {
            int item = s->confirmItem;
            s->confirmItem = -1;
            pop( s );
            return item; // yes
        }
        if ( c == IN_BTN_B ) {
            if ( e.kind == IN_CLICK ) {
                s->confirmItem = -1;
                pop( s ); // no
            } else if ( e.kind == IN_HOLD ) {
                s->confirmItem = -1;
                homeFromAnywhere( s );
            }
        }
        return -1;
    case PANE_RESULT:
        if ( isDirection( c ) && press ) {
            MenuKey k = directionKey( c );
            int most = s->resultLines - s->resultVisible;
            if ( most < 0 )
                most = 0;
            if ( k == MENUKEY_UP && s->resultScroll < most )
                s->resultScroll++;
            if ( k == MENUKEY_DOWN && s->resultScroll > 0 )
                s->resultScroll--;
        } else if ( ( isSelect( c ) && e.kind == IN_PRESS ) || ( c == IN_BTN_B && e.kind == IN_CLICK ) ) {
            pop( s );
        } else if ( c == IN_BTN_B && e.kind == IN_HOLD ) {
            homeFromAnywhere( s );
        }
        return -1;
    }
    return -1;
}

void uiShellMenuWindow( UiShell* s ) {
    int visible = menuVisibleCount( &s->menu );
    int rows = visible < s->menuRows ? visible : s->menuRows;
    if ( s->menu.cursor < s->menuScrollTop )
        s->menuScrollTop = s->menu.cursor;
    if ( s->menu.cursor >= s->menuScrollTop + rows )
        s->menuScrollTop = s->menu.cursor - rows + 1;
    if ( s->menuScrollTop > visible - rows )
        s->menuScrollTop = visible - rows < 0 ? 0 : visible - rows;
    if ( s->menuScrollTop < 0 )
        s->menuScrollTop = 0;
}

// One axis of the stick into three bands (0, 1, 2) with hysteresis round
// the band edges at a third of the travel.
static int stickBand( float v, int current ) {
    const float edge = 0.33f, slack = 0.06f;
    if ( current == 0 )
        return v > -edge + slack ? ( v > edge + slack ? 2 : 1 ) : 0;
    if ( current == 2 )
        return v < edge - slack ? ( v < -edge - slack ? 0 : 1 ) : 2;
    return v < -edge - slack ? 0 : ( v > edge + slack ? 2 : 1 );
}

#define ABSOLUTE_CENTRE 0.12f  // inside this the stick is home again
#define ABSOLUTE_RETRACT 0.15f // this much back from its furthest and it is on its way home

// The stick's position as the cursor, while it is deflected and going out.
static void steerAbsolute( UiShell* s, float x, float y ) {
    float mag = sqrtf( x * x + y * y );
    if ( mag > 1.0f )
        mag = 1.0f;
    if ( mag < ABSOLUTE_CENTRE ) {
        s->absolutePeak = 0.0f;
        s->absoluteFrozen = false;
        return;
    }
    if ( mag > s->absolutePeak )
        s->absolutePeak = mag;
    if ( s->absoluteFrozen || s->absolutePeak - mag > ABSOLUTE_RETRACT ) {
        s->absoluteFrozen = true; // coming back: the cursor stays where it was pointed
        return;
    }
    if ( uiShellTop( s ) == PANE_HOME ) {
        int column = stickBand( x, s->home.cursor % HOME_COLUMNS );
        int row = stickBand( -y, s->home.cursor / HOME_COLUMNS );
        int rows = ( s->home.count + HOME_COLUMNS - 1 ) / HOME_COLUMNS;
        if ( row >= rows )
            row = rows - 1;
        int cell = row * HOME_COLUMNS + column;
        if ( cell >= s->home.count )
            cell = s->home.count - 1;
        if ( cell != s->home.cursor ) {
            s->home.cursor = cell;
            s->generation++;
        }
    } else if ( uiShellTop( s ) == PANE_MENU ) {
        int visible = menuVisibleCount( &s->menu );
        if ( visible == 0 )
            return;
        uiShellMenuWindow( s );
        int rows = visible < s->menuRows ? visible : s->menuRows;
        // Up is the top row: the stick's height over the rows on screen,
        // with a little slack before the cursor moves a row.
        float pos = ( 1.0f - y ) * 0.5f * rows; // 0 at the top .. rows at the bottom
        int r = s->menu.cursor - s->menuScrollTop;
        const float slack = 0.15f;
        if ( pos < r - slack )
            r = (int)( pos + slack );
        else if ( pos >= r + 1.0f + slack )
            r = (int)( pos - slack );
        if ( r < 0 )
            r = 0;
        if ( r > rows - 1 )
            r = rows - 1;
        int cursor = s->menuScrollTop + r;
        if ( cursor != s->menu.cursor ) {
            s->menu.cursor = cursor;
            s->generation++;
        }
    }
}

void uiShellTick( UiShell* s, uint32_t nowMs, float dtS, float joyX, float joyY, float joyRawX, float joyRawY, const bool* heldNow ) {
    // A release the ring lost: the raw state says it is up, so it is.
    if ( heldNow != nullptr ) {
        for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
            if ( !heldNow[ c ] && ( s->held[ c ] || s->swallow[ c ] ) ) {
                s->held[ c ] = false;
                s->swallow[ c ] = false;
            }
        }
    }
    // Left alone: the overlays close, remembering where they were.
    if ( s->depth > 0 && s->lastInputMs != 0 && nowMs - s->lastInputMs >= UISHELL_IDLE_MS ) {
        uiShellCloseAll( s );
        s->lastInputMs = nowMs == 0 ? 1 : nowMs;
    }
    if ( s->absoluteJoystick && s->depth > 0 ) {
        if ( joyRawX != 0.0f || joyRawY != 0.0f )
            s->lastInputMs = nowMs == 0 ? 1 : nowMs; // the stick counts as input
        steerAbsolute( s, joyRawX, joyRawY );
    }
    // The stick is the app's only with nothing open, and only once it has
    // been seen centred since the focus last changed.
    bool centred = joyX == 0.0f && joyY == 0.0f;
    if ( !s->joyArmed && centred )
        s->joyArmed = true;
    bool stickIsApps = s->depth == 0 && s->joyArmed;
    const UiApp* app = uiShellApp( s );
    if ( app != nullptr && app->tick != nullptr ) {
        app->tick( dtS, stickIsApps ? joyX : 0.0f, stickIsApps ? joyY : 0.0f );
    }
}
