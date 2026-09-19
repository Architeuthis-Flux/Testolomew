// SPDX-License-Identifier: MIT
#include "UiShell.h"

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
        } else if ( isSelect( c ) && e.kind == IN_CLICK ) {
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
        if ( isSelect( c ) && e.kind == IN_CLICK ) {
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
        if ( isSelect( c ) && e.kind == IN_CLICK ) {
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
        } else if ( ( isSelect( c ) || c == IN_BTN_B ) && e.kind == IN_CLICK ) {
            pop( s );
        } else if ( c == IN_BTN_B && e.kind == IN_HOLD ) {
            homeFromAnywhere( s );
        }
        return -1;
    }
    return -1;
}

void uiShellTick( UiShell* s, uint32_t nowMs, float dtS, float joyX, float joyY, const bool* heldNow ) {
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
