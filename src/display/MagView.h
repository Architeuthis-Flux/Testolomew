// SPDX-License-Identifier: MIT
#ifndef MAGVIEW_H
#define MAGVIEW_H
// ---------------------------------------------------------------------------
// The sensed magnet in 3D, on the LCD.
//
// What is drawn: the board plane as a 10 mm grid, each sensor with its field
// vector (length is logarithmic - the field spans three decades), and the
// magnet as a red/blue bar along its axis (red = north) with a drop line and a
// cross on the board under it, so height reads at a glance. A fading trail
// follows the magnet. Numbers along the top, status along the bottom.
//
// The camera (src/ui/Camera.h) looks at the middle of the array from a fixed
// viewpoint to start with; the controls orbit, pan and zoom it, and it can
// sway or spin (parallax is what makes a wireframe read as 3D on a flat
// panel), look straight down, follow the probe, or ride on its point.
//
// Timing: two framebuffers. One is on its way to the panel in two DMA bands
// that run on their own while the loop goes on reading sensors; the next
// frame is drawn into the other meanwhile (a few milliseconds, in one tick).
// A tick otherwise only starts a band or notices one has finished, so the
// sensors keep their 100 Hz and the panel gets 40-50 frames a second.
//
// The breadboard LEDs come first: a draw holds the loop for as long as it
// takes, and the LED service (50 Hz, its own DMA) must not be made late by
// one, so a frame is not started when the LED service is due sooner than
// this screen's last draw took (it is drawn on the next tick instead). Text
// goes through FastDraw.h, not GFX's print, which is what made a draw take
// tens of milliseconds. `e` reports each screen's draw time.
//
// With the row counter in row mode (console r) the grid becomes the breadboard's
// rows, and the counted row is written large.
//
// Console: v = next camera mode (fixed / sway / spin / top / follow / POV),
// e = next screen (3D scene / the breadboard LEDs / the log / the draw screen,
// which in paint mode carries the colour wheel and the brush's state).
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "Camera.h"
#include "JumperlOS.h"
#include "Vec3.h"

#define MAGVIEW_BAND_ROWS 120 // rows per DMA band (two per frame; the push runs while the sensors are read)
#define MAGVIEW_TRAIL_POINTS 48
#define MAGVIEW_TRAIL_PERIOD_MS 40 // a trail point this often, ~2 s of history
#define MAGVIEW_LOG_REDRAW_MS 200  // the log screen (all text, 30 ms to draw) no more often than this
#define MAGVIEW_STARVE_US 60000    // a screen not drawn this long is drawn whatever the LED timing says
#define MAGVIEW_TEXT 2             // text size: 2 = 12 x 16 px characters, 20 to a line on a 240 px panel
#define MAGVIEW_CHAR_W ( 6 * MAGVIEW_TEXT )
#define MAGVIEW_LINE_H ( 8 * MAGVIEW_TEXT )
#define MAGVIEW_COLUMNS ( LCD_WIDTH / MAGVIEW_CHAR_W )
#define MAGVIEW_ELEVATION_DEG 32.0f // the home viewpoint
#define MAGVIEW_YAW_DEG -25.0f

// What the panel shows.
enum MagViewScreen {
    MAGVIEW_SCREEN_SCENE, // the 3D view
    MAGVIEW_SCREEN_LEDS,  // the breadboard's LEDs, as the probe cursor would light them
    MAGVIEW_SCREEN_LOG,   // what the console printed
    MAGVIEW_SCREEN_DRAW,  // the point's path over a top-down map of the board (the play module's trace)
    MAGVIEW_SCREEN_COUNT
};

class GFXcanvas16;

class MagView : public Service {
  public:
    static MagView& getInstance( );

    MagView( const MagView& ) = delete;
    MagView& operator=( const MagView& ) = delete;

    // false if the panel or the framebuffer could not be set up (the service
    // then idles).
    bool begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "MagView"; }
    ServicePriority getPriority( ) const override { return ServicePriority::NORMAL; }
    uint32_t periodUs( ) const override { return 3000; }

    void nextCamera( );
    void nextScreen( );
    float fps( ) const { return framesPerSecond; }
    MagViewScreen screen = MAGVIEW_SCREEN_SCENE;
    Camera cam; // the 3D view's camera (src/ui/Camera.h); the UI's controls drive it

    // How long a draw of each screen takes (microseconds): the last one, an
    // average that follows, and the longest since boot. `e` prints them.
    uint32_t drawLastUs[ MAGVIEW_SCREEN_COUNT ] = { 0 };
    uint32_t drawAvgUs[ MAGVIEW_SCREEN_COUNT ] = { 0 };
    uint32_t drawMaxUs[ MAGVIEW_SCREEN_COUNT ] = { 0 };
    uint32_t clearUs[ 2 ] = { 0, 0 }; // a fillScreen of each canvas, timed at begin(): says which memory it landed in
    uint32_t yields = 0;              // draws put off because the LED service was due first
    uint32_t lastDrawStartUs = 0;     // when a draw last ran (the starvation guard)

    // For a screen dump (DumpService): `hold` stops new frames going to the
    // panel (the push under way finishes; drawing goes on into the other
    // buffer); frozen() says the shown frame is complete and will stay.
    // copyShownRow() gives a row of it in native RGB565 - the push swaps
    // the bytes of the shown buffer in place for the wire (ST7789.h), so
    // the copy swaps them back iff that has happened (shownPushed).
    bool hold = false;
    bool shownPushed = false; // the shown buffer has been through a push (and is byte-swapped)
    bool frozen( ) const { return hold && pushRow < 0; }
    void copyShownRow( int y, uint16_t* dst ) const;

  private:
    MagView( ) = default;

    GFXcanvas16* canvas = nullptr; // the frame being drawn
    GFXcanvas16* shown = nullptr;  // the frame on its way to the panel
    int pushRow = -1;              // next row of `shown` to send, -1 = nothing in flight
    bool drawn = false;            // `canvas` holds a frame not yet sent

    // The projection's copy of the camera, taken each frame.
    Vec3 target = { 0, 0, 0 }; // what the camera looks at
    float zoom = 2.0f;         // pixels per mm at the target
    float cameraDistance = 300.0f;
    float sinYaw = 0, cosYaw = 1, sinElevation = 0, cosElevation = 1;
    uint32_t lastFrameMs = 0;
    uint32_t lastLogStamp = 0xffffffffu; // the log screen as last drawn
    uint32_t lastLogDrawMs = 0;
    int lastDrawnScreen = -1;

    Vec3 trail[ MAGVIEW_TRAIL_POINTS ];
    int trailCount = 0;
    int trailHead = 0;
    uint32_t lastTrailMs = 0;

    uint32_t fpsWindowStartMs = 0;

    int fpsWindowFrames = 0;
    float framesPerSecond = 0.0f;

    void benchmark( );
    void aimCamera( uint32_t nowMs );
    bool project( Vec3 world, int* sx, int* sy ) const; // false = behind the camera
    void line3d( Vec3 a, Vec3 b, uint16_t color );
    bool tryDraw( );                  // a timed drawFrame, unless the LEDs are due first
    bool drawFrame( uint32_t nowMs ); // false = nothing new to show
    void drawBoard( );
    void drawRows( );
    void drawSensors( );
    void drawMagnet( );
    void drawMagnetBar( Vec3 centre, Vec3 axis, bool flipped, float halfLengthMm );
    void ring( Vec3 centre, float rx, float ry, uint16_t color );
    void drawText( );
    void drawLedScreen( );
    void drawTraceScreen( );
    void drawColourWheel( ); // on the draw screen, in paint mode
};

extern MagView& magView;

#endif // MAGVIEW_H
