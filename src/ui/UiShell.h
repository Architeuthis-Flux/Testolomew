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
//   Home            up/down/left/right move, a press (or A), on its way down,
//                   selects; B closes
//   Home            (opens with the cursor on Settings, the centre cell;
//                   a held direction walks the cells no faster than
//                   stepRepeatMs, as up/down walk a menu page - the raw
//                   repeat, every 80 ms, skipped rows on a long tilt)
//   Settings page   up/down move, LEFT/RIGHT CHANGE THE VALUE IN PLACE, a
//                   press enters a page or runs an action on its way down;
//                   a toggle flips and a choice cycles on the CLICK (the
//                   release of a short press), so a press HELD on a toggle,
//                   a number or a choice tweaks it (below) without ever
//                   touching it (an accessor's setter with side effects -
//                   the tracker reset, the chain darkened - must not run
//                   for a look at the value; the review, 2026-09-25), B backs
//                   a page (at the root: closes), B hold closes everything
//                   (the page is kept: the menu opens again where it was
//                   last used, even after B backed it out to the root)
//   Tweak           the menu hidden, the app showing, the item's label and
//                   value along the bottom: UP/DOWN change it (up = more),
//                   LEFT/RIGHT move to the neighbouring items (the axes the
//                   other way round from the page), the joystick is the
//                   app's (it orbits the scene being looked at); a press or
//                   B: the menu again; B hold: everything closes.
//   Confirm         a press or A: yes; B: no; B hold: everything closes
//   Result          up/down scroll; a press, A or B dismiss
//   20 s idle       overlays close (cursors and the page remembered) - not
//                   a tweak, which is deliberate and ends on B
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
#define UISHELL_STEP_REPEAT_MS 300.0f // a held direction steps the cursor no faster than this (the menu's "cursor repeat"; the raw repeat is every 80 ms: "way less touchy", Kevin 2026-09-25)
#define UISHELL_RESULT_TITLE 24

enum PaneKind {
    PANE_APP, // the base: no overlay
    PANE_HOME,
    PANE_MENU,
    PANE_CONFIRM,
    PANE_RESULT,
    PANE_TWEAK // a menu value edited over the app (the menu beneath it is not drawn)
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
    int selectPressItem; // the menu item a select was pressed on (-1 none): its hold tweaks that item, not what the press led to
    char resultTitle[ UISHELL_RESULT_TITLE ];
    int resultScroll; // lines back from the newest
    int resultLines;  // how many lines the result has
    int resultVisible;
    bool held[ IN_CONTROL_COUNT ];    // down, as the events say
    bool swallow[ IN_CONTROL_COUNT ]; // ...and dropped until released
    bool joyArmed;                    // the stick has been centred since the last focus change
    uint32_t lastInputMs;
    uint32_t generation; // bumps on anything that changes the picture
    // Absolute joystick (a setting): while it is deflected, the stick's
    // position IS the cursor - on Home the cell it points at (three bands
    // each way), on a menu page the row among those on screen - and its
    // four-way events are ignored there. Let go and the cursor stays where
    // it was pointed: the stick springs back THROUGH the middle, so the
    // cursor follows only while the stick is going out, and freezes once
    // it starts coming back until it has reached the centre. Off: the
    // four-way steps.
    bool absoluteJoystick;
    float absolutePeak;  // the furthest the stick has been since it last left the centre
    bool absoluteFrozen; // ...and it is on its way back: the cursor stays where it was pointed
    float stepRepeatMs;  // a held direction steps the cursor (Home: any way; a page or a tweak: up/down) no faster than this; left/right on a page keep the raw repeat, which is what makes a number run
    uint32_t lastStepMs; // when the cursor last stepped on a held direction
    int menuRows;        // rows a menu page shows at once (the Ui says)...
    int ( *pageRowsTaken )( const char* page ); // ...less what a page's preview takes (may be null; the modules': the colours page shows its mapping under its items) - uiShellPageRows
    int menuScrollTop;   // the first row shown (kept here so the stick and the drawing agree)
};

void uiShellInit( UiShell* s, const UiApp* apps, int appCount, int firstApp, int settingsCell );

// One event. Returns the index of a menu ACTION the caller must run now (a
// confirmed one, or one that needs no confirmation), else -1.
int uiShellEvent( UiShell* s, InputEvent e, uint32_t nowMs );
// Every UI tick: heals lost releases from the raw held state (heldNow, one
// bool per control, may be nullptr), closes idle overlays, steers the
// panes with the raw stick in absolute mode, and ticks the app - with the
// (shaped) stick only when it is the app's.
void uiShellTick( UiShell* s, uint32_t nowMs, float dtS, float joyX, float joyY, float joyRawX, float joyRawY, const bool* heldNow );
// The menu page's window: menuScrollTop brought to where the cursor is.
void uiShellMenuWindow( UiShell* s );
// Rows the current page shows at once: menuRows less its preview's, never under three.
int uiShellPageRows( const UiShell* s );

void uiShellSelectApp( UiShell* s, int app ); // swaps the base pane and closes the overlays
void uiShellOpenHome( UiShell* s );
void uiShellOpenMenu( UiShell* s );     // where it was last used (menuResume)
void uiShellOpenMenuRoot( UiShell* s ); // at the root page
void uiShellShowResult( UiShell* s, const char* title, int lines, int visible ); // over whatever is open
void uiShellCloseAll( UiShell* s );
PaneKind uiShellTop( const UiShell* s );
bool uiShellOverlayOpen( const UiShell* s );
const UiApp* uiShellApp( const UiShell* s );
int uiShellHomeCellApp( const UiShell* s, int cell ); // the app a Home cell leads to, -1 for Settings

#endif // UISHELL_H
