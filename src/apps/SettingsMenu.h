// SPDX-License-Identifier: MIT
#ifndef SETTINGSMENU_H
#define SETTINGSMENU_H
// ---------------------------------------------------------------------------
// The Settings menu: every setting that matters as a typed item (the
// tracker, the cursor, the camera, the LEDs, the rows, the play modes, the
// magnet, the sensors), and every console command as an item under
// "commands" - run through the console's own handler with the output going
// to the log (and shown in a Result panel over the page). The values bind
// to the modules: a plain value by its pointer, one with a side effect
// through a get/set pair whose setter compares before it writes. The
// page and item labels are the settings module's keys (page/label), so
// they are kept as they are.
// ---------------------------------------------------------------------------
#include "Menu.h"

void settingsMenuBuild( Menu* m );
// Console commands registered after the build (the settings module's own),
// into the commands page.
void settingsMenuAddNewCommands( );

#endif // SETTINGSMENU_H
