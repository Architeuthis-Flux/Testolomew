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
// The camera orbits the middle of the array; by default it sways a little,
// because parallax is what makes a wireframe read as 3D on a flat panel.
//
// Timing: one frame is drawn into the framebuffer in a single tick (about a
// millisecond), then sent to the panel a band of rows per tick, so no tick
// holds the loop for more than ~5 ms and the sensors keep their 100 Hz.
//
// With the row counter in row mode (console r) the grid becomes the breadboard's
// rows, and the counted row is written large.
//
// Console: v = cycle the camera (sway / fixed / spin / top-down).
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "Vec3.h"

#define MAGVIEW_BAND_ROWS 15 // rows sent per tick: half of what measured 4.8 ms, so the sensors get read in between
#define MAGVIEW_TRAIL_POINTS 48
#define MAGVIEW_TRAIL_PERIOD_MS 40 // a trail point this often, ~2 s of history
#define MAGVIEW_ELEVATION_DEG 32.0f
#define MAGVIEW_YAW_DEG -25.0f
#define MAGVIEW_SWAY_DEG 18.0f

enum MagViewCamera {
    MAGVIEW_SWAY,
    MAGVIEW_FIXED,
    MAGVIEW_SPIN,
    MAGVIEW_TOP,
    MAGVIEW_CAMERA_COUNT
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
    float fps( ) const { return framesPerSecond; }

  private:
    MagView( ) = default;

    GFXcanvas16* canvas = nullptr;
    int pushRow = -1; // next row to send, -1 = draw a new frame

    MagViewCamera camera = MAGVIEW_FIXED;
    Vec3 target = { 0, 0, 0 }; // what the camera looks at (middle of the array)
    float zoom = 2.0f;         // pixels per mm at the target
    float cameraDistance = 300.0f;
    float sinYaw = 0, cosYaw = 1, sinElevation = 0, cosElevation = 1;

    Vec3 trail[ MAGVIEW_TRAIL_POINTS ];
    int trailCount = 0;
    int trailHead = 0;
    uint32_t lastTrailMs = 0;

    uint32_t fpsWindowStartMs = 0;

    int fpsWindowFrames = 0;
    float framesPerSecond = 0.0f;

    void aimCamera( uint32_t nowMs );
    void project( Vec3 world, int* sx, int* sy ) const;
    void line3d( Vec3 a, Vec3 b, uint16_t color );
    void drawFrame( uint32_t nowMs );
    void drawBoard( );
    void drawRows( );
    void drawSensors( );
    void drawMagnet( );
    void drawMagnetBar( Vec3 centre, Vec3 axis, bool flipped, float halfLengthMm );
    void drawText( );
};

extern MagView& magView;

#endif // MAGVIEW_H
