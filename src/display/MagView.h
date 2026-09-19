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
// The framebuffers, the DMA push and when a frame is drawn are the Display
// service's (Display.h); this draws into the canvas it is handed, as its
// drawFn. Text goes through FastDraw.h, not GFX's print. `e` reports each
// screen's draw time.
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
#include "Vec3.h"

#define MAGVIEW_TRAIL_POINTS 48
#define MAGVIEW_TRAIL_PERIOD_MS 40 // a trail point this often, ~2 s of history
#define MAGVIEW_LOG_REDRAW_MS 200  // the log screen (all text, 30 ms to draw) no more often than this
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

class MagView {
  public:
    static MagView& getInstance( );

    MagView( const MagView& ) = delete;
    MagView& operator=( const MagView& ) = delete;

    // The console commands and the camera's home viewpoint (the display
    // itself is the Display service's).
    void begin( );

    void nextCamera( );
    void nextScreen( );
    MagViewScreen screen = MAGVIEW_SCREEN_SCENE;
    Camera cam; // the 3D view's camera (src/ui/Camera.h); the UI's controls drive it

    // A frame into `canvas`. false = nothing new to show (the log screen
    // unchanged). Installed as the Display's drawFn (magViewDraw).
    bool drawFrame( GFXcanvas16* canvas, uint32_t nowMs );

  private:
    MagView( ) = default;

    GFXcanvas16* canvas = nullptr; // the frame being drawn (the Display's, per call)

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

    void aimCamera( uint32_t nowMs );
    bool project( Vec3 world, int* sx, int* sy ) const; // false = behind the camera
    void line3d( Vec3 a, Vec3 b, uint16_t color );
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

// The Display's drawFn: the current screen (and its slot).
bool magViewDraw( GFXcanvas16* canvas, uint32_t nowMs );

extern MagView& magView;

#endif // MAGVIEW_H
