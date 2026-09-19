// SPDX-License-Identifier: MIT
#ifndef UIAPP_H
#define UIAPP_H
// ---------------------------------------------------------------------------
// An app: one of the things the screen can be (the 3D view, the LEDs, the
// terminal, the paint app...). The table of them (src/apps/Apps.cpp) is the
// shape of JumperlOS's apps[] with ContextEntry's hooks: everything is a
// plain function pointer, none of them blocks, and the shell (UiShell.h)
// knows apps only through this. An app gets the events the shell does not
// keep for itself (never A or B, never a control an overlay took), a tick
// with the joystick (zeroed while an overlay is open, and until the stick
// has been centred once after a change of focus), and a draw call.
// ---------------------------------------------------------------------------
#include <stdint.h>

#include "InputEvent.h"

class GFXcanvas16;

#define UIAPP_ICON_BYTES 72 // 24 x 24 bits, row-major, most significant bit first

struct UiApp {
    const char* name;
    const uint8_t* icon;                                   // UIAPP_ICON_BYTES of 1-bit art for the Home grid (may be nullptr)
    void ( *enter )( );                                    // it becomes the screen
    void ( *exit )( );                                     // another app does
    void ( *tick )( float dtS, float joyX, float joyY ); // every UI tick, with the stick (0 while it is not the app's)
    void ( *draw )( GFXcanvas16* canvas );                 // a frame
    bool ( *event )( const InputEvent* e );                // a control; true = taken
    uint32_t ( *generation )( );                           // bumps when a redraw is due; 0 = always redraw
};

#endif // UIAPP_H
