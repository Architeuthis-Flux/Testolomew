// SPDX-License-Identifier: MIT
#ifndef UI_H
#define UI_H
// ---------------------------------------------------------------------------
// The on-screen UI: what the controls do on each screen, the menu that can do
// everything the serial console can, and the log screen that shows what the
// console printed.
//
// Controls (physical or typed on the console, see Input.h):
//   button A        open / close the menu
//   button B        next screen (3D scene / breadboard LEDs / log); in the menu: back
//   nav switch      3D: pan the view (the target slides in the board plane);
//                   LEDs: nothing yet; log: scroll; menu: move / enter / back
//   nav press       3D: next camera mode; log: back to the newest line; menu: enter
//   joystick        3D: orbit (left/right = yaw, up/down = elevation), with
//                   the press held: zoom (up/down)
//   joystick press  3D: reset the view (hold: home + fixed mode)
//
// The menu is built at boot: the settings that matter as typed items (the
// tracker, the cursor, the camera, the LEDs), and every console command as
// an item under "commands" - one whose help text says <number> asks for it
// first - run through the console's own handler with the output going to the
// log screen (and the serial port). Nothing here knows how a command works.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "MagView.h"
#include "Menu.h"

#define UI_PERIOD_US 10000
#define UI_ORBIT_DEG_PER_S 90.0f // full joystick
#define UI_ZOOM_PER_S 1.5f       // zoom factor per second at full joystick
#define UI_PAN_MM 4.0f           // per nav press / repeat
#define UI_MENU_ROWS 9           // items shown at once (18 px rows of size-2 text)
#define UI_LOG_TEXT 1            // the log screen's text size: 1 = 6 x 8 px characters, 40 to a line
#define UI_LOG_ROWS 28           // lines of it above the footer

class GFXcanvas16;

class Ui : public Service {
  public:
    static Ui& getInstance( );

    Ui( const Ui& ) = delete;
    Ui& operator=( const Ui& ) = delete;

    void begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "Ui"; }
    ServicePriority getPriority( ) const override { return ServicePriority::HIGH; }
    uint32_t periodUs( ) const override { return UI_PERIOD_US; }

    // Drawn by MagView into its framebuffer: the menu panel when it is open,
    // the log when that screen is up.
    bool menuOpen( ) const { return menuShown; }
    void openMenu( );
    void closeMenu( );
    void drawMenu( GFXcanvas16* canvas );
    void drawLog( GFXcanvas16* canvas );

    Menu menu;
    bool stripOn = false; // the LEDs page's "chain on" (mirrors ProbeLedService::strip; N toggles it)
    // The settings module has written saved values into the menu's
    // variables: carry the choice/toggle ones into the modules. The items
    // it does not save (row mode, chain on, V5 stream, play mode: modes, not
    // settings) are read from the modules first, or their compile-time
    // values would be applied over what the modules boot with (2026-09-19:
    // every boot switched row mode and the LED chain off this way).
    void settingsLoaded( );
    // Console commands registered after begin() (the settings module's), into the commands page.
    void addNewCommands( );
    int logScroll = 0;      // lines back from the newest that the log screen shows
    uint32_t menuEdits = 0; // bumps on every menu key (the display redraws the menu on it)
    // The menu from the console (:ui): a key, as if pressed.
    void menuKeyFromConsole( MenuKey key );
    // What the screen shows, as text (:screen): the screen, the panes, the
    // menu page with its items as drawn, the play and probe state.
    void printScreen( Stream* out );

  private:
    Ui( ) = default;

    uint32_t lastUs = 0;
    bool menuShown = false;
    int cameraModeChoice = 0;
    int cursorModeChoice = 0;
    int ledLayoutChoice = 0;
    bool trackerOn = true;
    bool ledStream = false;
    bool rowMode = false;
    int commandsSubmenu = -1; // the "commands" page, and how many console commands it lists so far
    int commandsAdded = 0;
    int scrollTop = 0; // first menu row shown

    void buildMenu( );
    void applyChoices( );
    void runAction( int index );
    void handleMenuKey( MenuKey key, bool repeat );
    void handleScreenEvent( int control, int kind );
};

extern Ui& ui;

#endif // UI_H
