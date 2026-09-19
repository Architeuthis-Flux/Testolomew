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
    APP_TARGET,
    APP_ROWS,
    APP_CALIBRATE,
    APP_INFO,
    APP_COUNT
};
#define APPS_SETTINGS_CELL 7 // Home: View, LEDs, Terminal, Draw, Target, Rows, Calibrate, Settings, Info

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
void drawBoardMap( GFXcanvas16* canvas, bool brushRing ); // the board from above with the paint (the Target app uses it too)
// Target (AppTarget.cpp): the target game.
void targetEnter( );
void targetExit( );
void targetDraw( GFXcanvas16* canvas );
bool targetEvent( const InputEvent* e );
// Rows (AppRows.cpp): the counted row, large.
void rowsDraw( GFXcanvas16* canvas );
bool rowsEvent( const InputEvent* e );
uint32_t rowsGeneration( );
// Calibrate (AppCalibrate.cpp): the twelve taps.
void calibrateEnter( );
void calibrateExit( );
void calibrateDraw( GFXcanvas16* canvas );
bool calibrateEvent( const InputEvent* e );
uint32_t calibrateGeneration( );
// Info (AppInfo.cpp): the build, the sensors, the frame rate, the services.
void infoDraw( GFXcanvas16* canvas );
uint32_t infoGeneration( );

#endif // APPS_H
