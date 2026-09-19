// SPDX-License-Identifier: MIT
#ifndef APPS_H
#define APPS_H
// ---------------------------------------------------------------------------
// The apps: what the screen can be, as a table (UiApp.h) the way JumperlOS
// keeps its apps[]. Each lives in its own file here (AppView.cpp,
// AppLeds.cpp, ...) and knows its modules; src/ui knows only the table.
// appsBegin() packs the icons (Icons.h), sets the Settings menu up
// (SettingsMenu.h), starts the UI on the first app and hands the Display
// its draw function; `e` steps to the next app and prints the draw times.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "Camera.h"
#include "UiApp.h"

class GFXcanvas16;

enum UiAppId {
    APP_VIEW,
    APP_LEDS,
    APP_TERMINAL,
    APP_DRAW,
    APP_COUNT
};
#define APPS_SETTINGS_CELL APP_COUNT // Settings sits after the apps on Home

extern UiApp apps[ APP_COUNT ];

void appsBegin( );
// The Display's drawFn: the UI's frame, when there is something new.
bool appsDrawFrame( GFXcanvas16* canvas, uint32_t nowMs );
// The app from the console: by id.
void appsOpen( int app );

// ---- the apps' own functions (the table's rows) ----
// View (AppView.cpp): the 3D scene and its camera.
extern Camera viewCamera;
void viewBegin( );
void viewDraw( GFXcanvas16* canvas );
void viewTick( float dtS, float joyX, float joyY );
bool viewEvent( const InputEvent* e );
// LEDs (AppLeds.cpp).
void ledsDraw( GFXcanvas16* canvas );
// Terminal (AppTerminal.cpp): the log.
void terminalDraw( GFXcanvas16* canvas );
bool terminalEvent( const InputEvent* e );
uint32_t terminalGeneration( );
// Draw (AppDraw.cpp): the paint app.
void drawBegin( );
void drawEnter( );
void drawExit( );
void drawDraw( GFXcanvas16* canvas );
void drawTick( float dtS, float joyX, float joyY );
bool drawEvent( const InputEvent* e );

#endif // APPS_H
