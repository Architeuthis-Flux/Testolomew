// SPDX-License-Identifier: MIT
#ifndef UISHELL_H
#define UISHELL_H
// ---------------------------------------------------------------------------
// Who has the controls: the one focus owner. The screen is an app (UiApp.h)
// with a stack of overlays on it - Home (the grid of apps), the Settings
// menu (Menu.h), a Confirm question, a Result panel (the log tail after an
// action) - and every event goes to the top of the stack and nowhere else.
//
// The vocabulary, the same everywhere:
//   in an app       nav/joystick, their presses: the app's own; A click: Home;
//                   A hold: Settings; B click: the previous app, else Home;
//                   B hold: Home
//   Home            up/down/left/right move, a press (or A) selects, B closes
//   Settings page   up/down move, LEFT/RIGHT CHANGE THE VALUE IN PLACE, a
//                   press enters / runs / flips / cycles, B backs a page (at
//                   the root: closes), B hold closes everything and opens Home
//   Confirm         a press or A: yes; B: no
//   Result          up/down scroll; a press, A or B dismiss
//   20 s idle       overlays close (cursors and the page remembered)
// A and B never reach an app.
//
// The leak fix: when a pane closes or the app changes while a control is
// down, that control is SWALLOWED until its release - its repeats and its
// release never reach whoever is now on top - and the joystick is not
// handed to the app until it has been centred once. So a held nav-left
// that backs out of the menu does not go on panning the camera, and a
// flick that closes the menu does not orbit the scene. A release that got
// lost (the ring dropped it) is healed from the raw held state each tick.
//
// The apps are known through their table only. The probe, the LEDs and
// the paint go on underneath whatever is open: overlays take only the
// buttons and the joystick. No Arduino in here; host-tested (test_ui).
// ---------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>

#include "HomeGrid.h"
#include "InputEvent.h"
#include "Menu.h"
#include "UiApp.h"

#define UISHELL_MAX_DEPTH 4
#define UISHELL_IDLE_MS 20000
#define UISHELL_RESULT_TITLE 24

enum PaneKind {
    PANE_APP, // the base: no overlay
    PANE_HOME,
    PANE_MENU,
    PANE_CONFIRM,
    PANE_RESULT
};

struct UiShell {
    const UiApp* apps;
    int appCount;
    int app;         // the app on the screen
    int previousApp; // -1 = none: B goes Home instead
    int settingsCell; // which Home cell is Settings (the others are the apps in table order)
    const uint8_t* settingsIcon; // its icon (may be nullptr)
    PaneKind stack[ UISHELL_MAX_DEPTH ]; // the overlays, bottom first
    int depth;                           // 0 = the app alone
    HomeGrid home;
    Menu menu;
    int confirmItem; // the ACTION the Confirm pane asks about
    char resultTitle[ UISHELL_RESULT_TITLE ];
    int resultScroll; // lines back from the newest
    int resultLines;  // how many lines the result has
    int resultVisible;
    bool held[ IN_CONTROL_COUNT ];    // down, as the events say
    bool swallow[ IN_CONTROL_COUNT ]; // ...and dropped until released
    bool joyArmed;                    // the stick has been centred since the last focus change
    uint32_t lastInputMs;
    uint32_t generation; // bumps on anything that changes the picture
};

void uiShellInit( UiShell* s, const UiApp* apps, int appCount, int firstApp, int settingsCell );

// One event. Returns the index of a menu ACTION the caller must run now (a
// confirmed one, or one that needs no confirmation), else -1.
int uiShellEvent( UiShell* s, InputEvent e, uint32_t nowMs );
// Every UI tick: heals lost releases from the raw held state (heldNow, one
// bool per control, may be nullptr), closes idle overlays, and ticks the
// app - with the stick only when it is the app's.
void uiShellTick( UiShell* s, uint32_t nowMs, float dtS, float joyX, float joyY, const bool* heldNow );

void uiShellSelectApp( UiShell* s, int app ); // swaps the base pane and closes the overlays
void uiShellOpenHome( UiShell* s );
void uiShellOpenMenu( UiShell* s );
void uiShellShowResult( UiShell* s, const char* title, int lines, int visible ); // over whatever is open
void uiShellCloseAll( UiShell* s );
PaneKind uiShellTop( const UiShell* s );
bool uiShellOverlayOpen( const UiShell* s );
const UiApp* uiShellApp( const UiShell* s );
int uiShellHomeCellApp( const UiShell* s, int cell ); // the app a Home cell leads to, -1 for Settings

#endif // UISHELL_H
