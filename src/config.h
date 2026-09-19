// SPDX-License-Identifier: MIT
#ifndef CONFIG_H
#define CONFIG_H
// ---------------------------------------------------------------------------
// Which experiments are in the build. 1 = on, 0 = off.
//
// A module is a folder under src/ holding one or more Services. Turning one
// off here means setup() never begins it or registers it, so it does nothing
// and touches no pins; its files still compile, so it cannot quietly rot.
// Dependencies are noted; main.cpp checks them.
// ---------------------------------------------------------------------------

#define MODULE_MAG_ARRAY 1   // src/magarray - the TMAG5273 array
#define MODULE_MAG_LOCATOR 1 // src/magfit   - dipole fit -> probe position   (needs MAG_ARRAY)
#define MODULE_MAG_VIEW 1    // src/display  - the magnet in 3D on the LCD    (needs MAG_LOCATOR)
#define MODULE_ROW_COUNT 1   // src/rowcount - which breadboard row is it over (needs MAG_LOCATOR)
#define MODULE_PROBE_LEDS 1  // src/probeled - the probe cursor on the breadboard's LEDs (needs MAG_LOCATOR; the LCD previews it)
#define MODULE_UI 1          // src/ui       - controls, on-screen menu, log screen, camera (needs MAG_VIEW)
#define MODULE_SETTINGS 1    // src/settings - the menu's settings and the row anchors kept in flash (needs UI)
#define MODULE_PLAY 1        // src/play     - paint, target game and a draw screen, to see the tracking with (needs PROBE_LEDS)

#define CONSOLE_BAUD 115200

#endif // CONFIG_H
