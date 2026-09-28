// SPDX-License-Identifier: MIT
#ifndef PLAY_H
#define PLAY_H
// ---------------------------------------------------------------------------
// Things to do with the probe that show how well it is tracked.
//
// PAINT: with the point on the board, the LED under it takes the paint
// colour and keeps it, so a line drawn along a row should come out as that
// row's LEDs and nothing else; "clear" wipes it (the menu's, W, or the nav
// stick's centre held in the Draw app). The Draw app (src/apps/AppDraw.cpp)
// IS the paint app: going to it starts painting, leaving it stops, and the
// drawing stays. The brush itself is Paint.h:
// newest wins - a stroke paints over what was there, and within a stroke
// the centre beats the edge. The LCD's draw screen IS the
// paint app: going to it starts painting, leaving it stops, and the drawing
// stays - off the LEDs while away, back on them on return - until cleared.
// The draw screen shows the drawing as the LEDs have it - each painted hole
// a square in its colour on a map of the board, the brush's ring round the
// point - so the LCD and the breadboard agree. The colour is a hue and a
// saturation (the menu's, saved), picked on the LCD's draw screen from a
// colour wheel the joystick moves a marker over; the joystick's click
// toggles draw/erase, the nav stick's up/down set how bright the paint is
// and left/right how wide the brush (0 = the one LED, 1-3 = that many rows
// around it, the outer ones softer). The draw screen shows the same
// LEDs over a top-down map of the board.
//
// TARGET: one LED lights green somewhere on the board; touch it. The score
// is how long it took and how far from the LED's centre the point landed
// (the tracker's word for where it landed, so a miss is the tracker's or
// the hand's, and the mean over many is the accuracy of the whole thing);
// then the next one, somewhere else. The LED screen's status line and the
// console keep the tally ("play" page: score info; `w` prints it).
//
// The paint layer lives in ProbeLeds.h (ProbeLedPaint) and is handed to the
// renderer by ProbeLedService; this service only decides what goes in it.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "Paint.h"
#include "ProbeLeds.h"

#define PLAY_PERIOD_US 10000
#define PLAY_TOUCH_MM 2.0f        // the point is on the board below this height (the default; the menu's "touch mm", saved). 2026-09-19: 1 mm missed touches toward the middle of the board
#define PLAY_TOUCH_RELEASE_MM 0.7f // ...and stays "on" until this much higher: no flicker at the line
#define PLAY_WITHIN_ROWS 1.0f // a hole is "under" the point within this (a whole pitch: a point 2 mm into the channel still paints hole 1, 2026-09-19)
#define PLAY_TRACE_W 240
#define PLAY_TRACE_H 240
#define PLAY_TRACE_MID_Y 92    // the channel line's row on the draw screen: the map sits in the upper half, the settings below
#define PLAY_BRUSH_MAX PAINT_BRUSH_MAX // rows around the LED under the point
#define PLAY_BRIGHT_STEP 0.05f // the nav stick's up/down
#define PLAY_PICK_PER_S 4.0f   // the joystick's marker speed across the wheel, radii a second (rim to rim in half a second at full tilt; the stick's expo keeps small tilts fine)

enum PlayMode {
    PLAY_OFF,
    PLAY_PAINT,
    PLAY_TARGET,
    PLAY_MODE_COUNT
};
extern const char* const playModeNames[ PLAY_MODE_COUNT ];

// Hue (degrees) and saturation (0-1) at full value to red, green, blue.
void playHsvToRgb( float hueDeg, float sat, uint8_t* r, uint8_t* g, uint8_t* b );

class PlayService : public Service {
  public:
    static PlayService& getInstance( );

    PlayService( const PlayService& ) = delete;
    PlayService& operator=( const PlayService& ) = delete;

    void begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "Play"; }
    ServicePriority getPriority( ) const override { return ServicePriority::NORMAL; }
    uint32_t periodUs( ) const override { return PLAY_PERIOD_US; }

    // Settings (the menu's play page; saved but for the mode and erase).
    int mode = PLAY_OFF;
    float paintHue = 324.0f;       // degrees round the wheel (the bench's on 2026-09-27; 0 until then)
    float paintSat = 1.0f;         // 0 = white at the centre, 1 = the rim
    float paintBright = 0.5f;      // the paint's level (the LEDs' own brightness levers still apply)
    float brushSize = 0.0f;        // rows around the LED under the point (a number item, whole)
    bool erase = false;            // the joystick's click: painting takes the paint away instead
    float touchMm = PLAY_TOUCH_MM; // the point paints below this height above the surface

    ProbeLedPaint paint;
    PaintStroke stroke = { }; // the stroke under way (Paint.h: newest wins within it)
    // The colour as it is now, and the wheel: the marker's place is the colour
    // (x = sat cos hue, y = sat sin hue, y up), so moving it picks the colour.
    void paintColour( uint8_t* r, uint8_t* g, uint8_t* b ) const { playHsvToRgb( paintHue, paintSat, r, g, b ); }
    void movePicker( float dx, float dy );
    void setPaintBright( float level ); // the brush's, for the next strokes; what is painted keeps its level
    // Board place -> draw screen pixel: the map the draw screen plots the LEDs on.
    static void tracePlace( float along, float acrossMm, int* x, int* y );

    // The target game's tally.
    int targetLed = -1;
    uint32_t hits = 0, misses = 0;
    float meanMs = 0.0f, meanMissMm = 0.0f, lastMs = 0.0f, lastMissMm = 0.0f;
    uint32_t targetSinceMs = 0;

    void clearPaint( );
    void newTarget( );
    void printScore( Stream* out ) const;

  private:
    PlayService( ) = default;
    uint8_t targetWasR = 0, targetWasG = 0, targetWasB = 0; // the paint under the target LED, put back when it moves on
    float targetWasLevel = 0.0f;
    uint32_t layoutSeen = 0;      // probeLeds.layoutGeneration the paint was made for
    void releaseTarget( );
    bool wasTouching = false;
    uint32_t rng = 0x2545F491u;
    void paintAt( float along, float acrossMm );
};

extern PlayService& play;

#endif // PLAY_H
