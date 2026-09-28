// SPDX-License-Identifier: MIT
#ifndef UI_H
#define UI_H
// ---------------------------------------------------------------------------
// The on-screen UI service: the controls into the shell (UiShell.h), the
// shell's panes onto the canvas, and the console's window on both (:screen,
// :ui). Nothing in here knows an app: the apps are a table (UiApp.h) given
// at begin(), and the Settings menu is built into shell.menu by whoever
// knows the modules (src/apps/SettingsMenu.cpp).
//
// Controls (physical, or typed on the console: Input.h), the same everywhere:
//   in an app       the nav stick, the joystick and their presses are the
//                   app's own; A click: Home; A hold: Settings; B click: the
//                   previous app, else Home; B hold: Home
//   Home            up/down/left/right move, press (or A) selects, B closes
//   Settings        up/down move, left/right change the value in place, press
//                   enters / runs / flips / cycles, press held tweaks a value
//                   over the app, B backs a page (closes at the root), B hold
//                   closes everything (UiShell.h has the whole vocabulary)
//   Confirm         press or A: yes; B: no
//   Result          up/down scroll; press, A or B dismiss
//   20 s idle       the overlays close
//
// Drawing: the Display service asks the apps framework for a frame; that
// calls draw() here, which draws the app (unless Home covers it) and then
// the overlays bottom-up. frameStamp() tells it whether anything changed
// for an app that is not animated.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "UiApp.h"
#include "UiLayout.h"
#include "UiShell.h"

#define UI_PERIOD_US 10000

class GFXcanvas16;

class Ui : public Service {
  public:
    static Ui& getInstance( );

    Ui( const Ui& ) = delete;
    Ui& operator=( const Ui& ) = delete;

    // The apps, which one to start on, and which Home cell is Settings.
    void begin( const UiApp* apps, int appCount, int firstApp, int settingsCell );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "Ui"; }
    ServicePriority getPriority( ) const override { return ServicePriority::HIGH; }
    uint32_t periodUs( ) const override { return UI_PERIOD_US; }

    // A frame: the app and the overlays.
    void draw( GFXcanvas16* canvas );
    // 0 = always redraw (an animated app with nothing opaque over it); else
    // a value that changes when the picture would.
    uint32_t frameStamp( );
    // ...of which, the part the controls change (the shell and the app), as
    // against the console's output flowing into a result panel.
    uint32_t inputStamp( );
    bool overlayOpen( ) const { return uiShellOverlayOpen( &shell ); }

    // After an action ran and printed: its output, the log's tail, over
    // whatever is open (until B).
    void showResult( const char* title );
    // A value item's help (Menu.h), wrapped to the panel's width into a
    // buffer of its own (not the log: a stream printing meanwhile would
    // scroll it away), as a Result titled with its label.
    void showHelp( int item );
    char helpLines[ UI_HELP_LINES ][ UI_RESULT_COLS + 1 ];
    int helpCount = 0; // lines in use while a help result is open (shell.resultLimit says so)

    // :screen - what the screen shows, as text. screenExtra (may be null)
    // adds the modules' lines (the probe, the play state...).
    void printScreen( Stream* out );
    void ( *screenExtra )( Stream* out ) = nullptr;
    // A page's preview (the modules': the colours page shows its mapping),
    // drawn under the page's items and over a tweak's strip, in the rows
    // shell.pageRowsTaken says the page gives it.
    void ( *menuPreview )( GFXcanvas16* canvas, const char* page, int x, int y, int w, int h ) = nullptr;

    UiShell shell;

  private:
    Ui( ) = default;

    uint32_t lastUs = 0;

    void runAction( int index );
    void drawHome( GFXcanvas16* canvas );
    void drawMenu( GFXcanvas16* canvas );
    void drawConfirm( GFXcanvas16* canvas );
    void drawResult( GFXcanvas16* canvas );
    void drawTweak( GFXcanvas16* canvas );
};

extern Ui& ui;

// A 24 x 24 one-bit icon (UIAPP_ICON_BYTES) at `scale`, in `colour`.
void uiDrawIcon( GFXcanvas16* canvas, int x, int y, const uint8_t* bits, int scale, uint16_t colour );
// An item's value as the menu shows it (drawMenu and :screen use the same).
void uiMenuItemValue( const Menu* m, int index, char* value, int size );

#endif // UI_H
