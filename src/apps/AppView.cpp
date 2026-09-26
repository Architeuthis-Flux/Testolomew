// SPDX-License-Identifier: MIT
// The View app: the sensed magnet in 3D (the scene that was MagView).
//
// What is drawn: the board plane as a 10 mm grid, each sensor with its field
// vector (length is logarithmic - the field spans three decades), and the
// magnet as a red/blue bar along its axis (red = north) with a drop line and a
// cross on the board under it, so height reads at a glance. A fading trail
// follows the magnet. Numbers along the top, status along the bottom. With
// the row counter in row mode the grid becomes the breadboard's rows, and
// the counted row is written large.
//
// The camera (src/ui/Camera.h) looks at the middle of the array from a fixed
// viewpoint to start with; the controls orbit, pan and zoom it, and it can
// sway or spin (parallax is what makes a wireframe read as 3D on a flat
// panel), look straight down, follow the probe, or ride on its point.
//
// Controls: the nav stick pans; its press clicks to the next camera mode
// and held resets the view; the joystick orbits (yaw/elevation), and with
// its press held zooms; the joystick's press clicked resets the view, held
// goes home in the fixed mode. Console: v = next camera mode.
#include <Adafruit_GFX.h>
#include <math.h>

#include "Apps.h"
#include "BoardPins.h"
#include "Console.h"
#include "Display.h"
#include "FastDraw.h"
#include "MagArray.h"
#include "MagLocator.h"
#include "ProbeLeds.h" // probeLedHue: the poles' colours from their hues
#include "UiLayout.h"
#include "config.h"
#include "Input.h"
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif

#define COLOR_BACKGROUND RGB565( 0, 0, 0 )
#define COLOR_GRID RGB565( 30, 40, 60 )
#define COLOR_BOARD RGB565( 70, 100, 150 )
#define COLOR_SURFACE RGB565( 60, 140, 90 ) // the breadboard's surface, magLocator.boardZ above the sensors
#define COLOR_SENSOR_OK RGB565( 60, 220, 90 )
#define COLOR_SENSOR_LOST RGB565( 230, 50, 50 )
#define COLOR_FIELD RGB565( 0, 200, 230 )
#define COLOR_NORTH RGB565( 255, 60, 50 )
#define COLOR_SOUTH RGB565( 60, 110, 255 )
#define COLOR_SHADOW RGB565( 240, 200, 40 )
#define COLOR_TIP RGB565( 255, 255, 255 )
#define COLOR_TEXT RGB565( 220, 220, 220 )
#define COLOR_TEXT_DIM RGB565( 120, 120, 120 )
#define COLOR_WARNING RGB565( 255, 140, 0 )
#define COLOR_ERROR RGB565( 200, 80, 200 )
#define COLOR_ROW RGB565( 40, 50, 40 )
#define COLOR_ROW_FIFTH RGB565( 80, 100, 80 )
#define COLOR_ROW_CURRENT RGB565( 250, 250, 250 )
#define COLOR_SURE RGB565( 80, 255, 110 )
#define COLOR_UNSURE RGB565( 255, 220, 40 )

#define BOARD_MARGIN_MM 15.0f // board outline this far beyond the outermost sensors
#define GRID_PITCH_MM 10.0f
#define MAGNET_HALF_LENGTH_MM 5.0f
#define VIEW_DEG_TO_RAD ( (float)M_PI / 180.0f )


#define VIEW_ORBIT_DEG_PER_S 90.0f // full joystick
#define VIEW_ZOOM_PER_S 1.5f       // zoom factor per second at full joystick
#define VIEW_PAN_MM 4.0f           // per nav press / repeat
#define VIEW_ELEVATION_DEG 32.0f   // the home viewpoint
#define VIEW_YAW_DEG -25.0f
#define VIEW_TRAIL_POINTS 48
#define VIEW_TRAIL_PERIOD_MS 40 // a trail point this often, ~2 s of history

Camera viewCamera;
ViewStyle viewStyle = { true, MAGNET_HALF_LENGTH_MM, 3.0f, 225.0f, 1.0f }; // the colours page: the poles shown, the bar half length, the hues of COLOR_NORTH / COLOR_SOUTH as they were, the field arrows at full

static GFXcanvas16* canvas = nullptr; // the frame being drawn
// The projection's copy of the camera, taken each frame.
static Vec3 target = { 0, 0, 0 }; // what the camera looks at
static float zoom = 2.0f;         // pixels per mm at the target
static float cameraDistance = 300.0f;
static float sinYaw = 0, cosYaw = 1, sinElevation = 0, cosElevation = 1;
static uint32_t lastFrameMs = 0;
static Vec3 trail[ VIEW_TRAIL_POINTS ];
static int trailCount = 0;
static int trailHead = 0;
static uint32_t lastTrailMs = 0;

static void aimCamera( uint32_t nowMs );
static bool project( Vec3 world, int* sx, int* sy );
static void line3d( Vec3 a, Vec3 b, uint16_t color );
static void drawBoard( );
static void drawRows( );
static void drawSensors( );
static void drawMagnet( );
static void drawMagnetBar( Vec3 centre, Vec3 axis, bool flipped, float halfLengthMm );
static void ring( Vec3 centre, float rx, float ry, uint16_t color );
static void drawText( );

static void onCamera( Stream* out ) {
    (void)out;
    cameraNextMode( &viewCamera );
}

// Once, after the array is up: the console command and the camera's home
// viewpoint - the middle of the array, a little above the board, zoomed so
// the board outline fills most of the panel's width.
void viewBegin( ) {
    consoleAddCommand( 'v', "next camera mode (fixed / sway / spin / top / follow / POV)", onCamera );
    float xMin = 1e9f, xMax = -1e9f, yMin = 1e9f, yMax = -1e9f;
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        const Vec3& p = magArray.position[ i ];
        if ( p.x < xMin )
            xMin = p.x;
        if ( p.x > xMax )
            xMax = p.x;
        if ( p.y < yMin )
            yMin = p.y;
        if ( p.y > yMax )
            yMax = p.y;
    }
    Vec3 middle = { 0.5f * ( xMin + xMax ), 0.5f * ( yMin + yMax ), 8.0f };
    float span = ( xMax - xMin ) > ( yMax - yMin ) ? ( xMax - xMin ) : ( yMax - yMin );
    span += 2.0f * BOARD_MARGIN_MM + 40.0f;
    cameraInit( &viewCamera, middle, VIEW_YAW_DEG, VIEW_ELEVATION_DEG, 4.0f * span, LCD_WIDTH / span );
}

// ---- camera ----------------------------------------------------------------

static void aimCamera( uint32_t nowMs ) {
    float dtS = lastFrameMs == 0 ? 0.03f : ( nowMs - lastFrameMs ) * 0.001f;
    lastFrameMs = nowMs;
    // The probe for the follow / POV modes: the track when there is one, else the fix.
    const MagTrack& track = magLocator.track;
    const MagProbeFix& fix = magLocator.fix;
    bool tracked = track.enabled && ( track.state == MAGTRACK_TRACKING || track.state == MAGTRACK_COASTING );
    bool haveProbe = tracked || fix.valid;
    Vec3 tip = tracked ? track.viewTip : fix.tip;
    Vec3 shaft = tracked ? track.shaft : fix.shaft;
    cameraUpdate( &viewCamera, dtS, nowMs * 0.001f, haveProbe, tip, shaft );
    target = viewCamera.target;
    zoom = viewCamera.zoom;
    cameraDistance = viewCamera.distance;
    sinYaw = sinf( viewCamera.yawDeg * VIEW_DEG_TO_RAD );
    cosYaw = cosf( viewCamera.yawDeg * VIEW_DEG_TO_RAD );
    sinElevation = sinf( viewCamera.elevationDeg * VIEW_DEG_TO_RAD );
    cosElevation = cosf( viewCamera.elevationDeg * VIEW_DEG_TO_RAD );
}

// Board frame (mm) -> screen pixel. The camera sits `cameraDistance` from the
// target, `elevation` above the board plane, swung `yaw` around the vertical.
static bool project( Vec3 world, int* sx, int* sy ) {
    float dx = world.x - target.x;
    float dy = world.y - target.y;
    float dz = world.z - target.z;

    float x1 = dx * cosYaw + dy * sinYaw; // swing about the vertical
    float y1 = -dx * sinYaw + dy * cosYaw;

    float up = y1 * sinElevation + dz * cosElevation;    // screen up
    float depth = y1 * cosElevation - dz * sinElevation; // away from the camera

    // Behind the camera (which the POV camera, sitting at the probe's point,
    // has plenty of): not drawn, else it would appear mirrored.
    float range = cameraDistance + depth;
    bool inFront = range > 5.0f;
    if ( range < 20.0f ) {
        range = 20.0f;
    }
    float scale = zoom * cameraDistance / range;

    float fx = LCD_WIDTH * 0.5f + scale * x1;
    float fy = LCD_HEIGHT * 0.55f - scale * up;
    // Far off-screen lines cost GFX a pixel loop each; keep them bounded.
    if ( fx < -2000.0f )
        fx = -2000.0f;
    if ( fx > 2000.0f )
        fx = 2000.0f;
    if ( fy < -2000.0f )
        fy = -2000.0f;
    if ( fy > 2000.0f )
        fy = 2000.0f;
    *sx = (int)fx;
    *sy = (int)fy;
    return inFront;
}

static void line3d( Vec3 a, Vec3 b, uint16_t color ) {
    int ax, ay, bx, by;
    bool aFront = project( a, &ax, &ay );
    bool bFront = project( b, &bx, &by );
    if ( !aFront || !bFront ) {
        return; // no clipping, just culling: a line with an end behind the camera is left out
    }
    fastLine( canvas, ax, ay, bx, by, color );
}

// ---- the scene -------------------------------------------------------------

static void drawBoard( ) {
    float xMin = 1e9f, xMax = -1e9f, yMin = 1e9f, yMax = -1e9f;
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        const Vec3& p = magArray.position[ i ];
        if ( p.x < xMin )
            xMin = p.x;
        if ( p.x > xMax )
            xMax = p.x;
        if ( p.y < yMin )
            yMin = p.y;
        if ( p.y > yMax )
            yMax = p.y;
    }
    xMin -= BOARD_MARGIN_MM;
    xMax += BOARD_MARGIN_MM;
    yMin -= BOARD_MARGIN_MM;
    yMax += BOARD_MARGIN_MM;

    bool rows = false;
#if MODULE_ROW_COUNT
    rows = rowCounter.active;
#endif
    if ( rows ) {
        drawRows( );
    } else {
        for ( float x = ceilf( xMin / GRID_PITCH_MM ) * GRID_PITCH_MM; x < xMax; x += GRID_PITCH_MM ) {
            line3d( { x, yMin, 0 }, { x, yMax, 0 }, COLOR_GRID );
        }
        for ( float y = ceilf( yMin / GRID_PITCH_MM ) * GRID_PITCH_MM; y < yMax; y += GRID_PITCH_MM ) {
            line3d( { xMin, y, 0 }, { xMax, y, 0 }, COLOR_GRID );
        }
    }
    line3d( { xMin, yMin, 0 }, { xMax, yMin, 0 }, COLOR_BOARD );
    line3d( { xMax, yMin, 0 }, { xMax, yMax, 0 }, COLOR_BOARD );
    line3d( { xMax, yMax, 0 }, { xMin, yMax, 0 }, COLOR_BOARD );
    line3d( { xMin, yMax, 0 }, { xMin, yMin, 0 }, COLOR_BOARD );
    // The breadboard's surface, where the pointer lands and below which the
    // point cannot be: the same outline that many mm up, tied to the sensor
    // plane at the corners so the height reads in 3D.
    float zs = magLocator.boardZ;
    line3d( { xMin, yMin, zs }, { xMax, yMin, zs }, COLOR_SURFACE );
    line3d( { xMax, yMin, zs }, { xMax, yMax, zs }, COLOR_SURFACE );
    line3d( { xMax, yMax, zs }, { xMin, yMax, zs }, COLOR_SURFACE );
    line3d( { xMin, yMax, zs }, { xMin, yMin, zs }, COLOR_SURFACE );
    line3d( { xMin, yMin, 0 }, { xMin, yMin, zs }, COLOR_SURFACE );
    line3d( { xMax, yMin, 0 }, { xMax, yMin, zs }, COLOR_SURFACE );
    line3d( { xMax, yMax, 0 }, { xMax, yMax, zs }, COLOR_SURFACE );
    line3d( { xMin, yMax, 0 }, { xMin, yMax, zs }, COLOR_SURFACE );
}

// Row mode: instead of the 10 mm grid, the breadboard - a line along every
// row of both halves, from its first hole to its fifth, every fifth row
// brighter and the counted one white. While calibrating, a ring round the hole
// that is wanted.
static void drawRows( ) {
#if MODULE_ROW_COUNT
    const RowGrid& grid = rowCounter.grid;
    const float inner = ROWGRID_INNER_HOLE_MM - 0.5f * ROWGRID_PITCH_MM;
    const float outer = inner + ROWGRID_HOLES_PER_ROW * ROWGRID_PITCH_MM;
    for ( int row = 1; row <= 2 * ROWGRID_ROWS_PER_HALF; row++ ) {
        bool bottom = row > ROWGRID_ROWS_PER_HALF;
        float along = (float)( bottom ? row - ROWGRID_ROWS_PER_HALF : row );
        float side = bottom ? -1.0f : 1.0f;
        uint16_t color = row % 5 == 0 ? COLOR_ROW_FIFTH : COLOR_ROW;
        if ( rowCounter.reading.valid && row == rowCounter.reading.row && !rowCounter.calibrating( ) ) {
            color = COLOR_ROW_CURRENT;
        }
        Vec3 a = rowGridToBoard( &grid, along, side * inner ), b = rowGridToBoard( &grid, along, side * outer );
        a.z = b.z = magLocator.boardZ; // the rows lie on the surface, not the sensor plane
        line3d( a, b, color );
    }
    if ( rowCounter.calibrating( ) ) {
        int row, hole, sx, sy;
        rowCounter.calibrationTarget( &row, &hole );
        RowPlace wanted = rowGridHolePlace( row, hole );
        Vec3 at = rowGridToBoard( &grid, wanted.along, wanted.acrossMm );
        at.z = magLocator.boardZ;
        if ( project( at, &sx, &sy ) ) {
            canvas->drawCircle( sx, sy, 5, COLOR_ROW_CURRENT );
            canvas->drawCircle( sx, sy, 6, COLOR_ROW_CURRENT );
        }
    }
#endif
}

static void drawSensors( ) {
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        const Vec3& p = magArray.position[ i ];
        bool ok = magArray.sensor( i ).ok;
        int sx, sy;
        if ( !project( p, &sx, &sy ) ) {
            continue;
        }
        fastFillRect( canvas, sx - 2, sy - 2, 5, 5, !ok ? COLOR_SENSOR_LOST : ( magArray.saturated[ i ] ? COLOR_WARNING : COLOR_SENSOR_OK ) );

        if ( !ok || !magArray.baselineReady( ) ) {
            continue;
        }
        const Vec3& b = magArray.field[ i ];
        float magnitude = sqrtf( b.x * b.x + b.y * b.y + b.z * b.z );
        if ( magnitude < 0.05f ) {
            continue;
        }
        // 0.05 mT -> 1.5 mm, 5 mT -> 10 mm, 80 mT -> 16 mm, times the colours page's "field arrows" (0: none)
        if ( viewStyle.fieldArrows <= 0.0f )
            continue;
        float length = viewStyle.fieldArrows * 5.0f * log10f( 1.0f + magnitude / 0.05f );
        float k = length / magnitude;
        line3d( p, { p.x + k * b.x, p.y + k * b.y, p.z + k * b.z }, COLOR_FIELD );
    }
}

// A magnet as a bar along `axis`: north half in the north hue, south half in
// the south hue (the colours page; red and blue to start with), three pixels
// thick; with the poles off, a thin plain bar. `flipped` = its north pole is at the -axis end.
static void drawMagnetBar( Vec3 centre, Vec3 axis, bool flipped, float halfLengthMm ) {
    float h = flipped ? -halfLengthMm : halfLengthMm;
    Vec3 north = { centre.x + h * axis.x, centre.y + h * axis.y, centre.z + h * axis.z };
    Vec3 south = { centre.x - h * axis.x, centre.y - h * axis.y, centre.z - h * axis.z };
    int cx, cy, nx, ny, sx, sy;
    if ( !project( centre, &cx, &cy ) || !project( north, &nx, &ny ) || !project( south, &sx, &sy ) ) {
        return;
    }
    if ( !viewStyle.poles ) {
        fastLine( canvas, nx, ny, sx, sy, COLOR_TEXT_DIM );
        canvas->drawCircle( cx, cy, 2, COLOR_TEXT );
        return;
    }
    uint8_t r, g, b;
    probeLedHue( viewStyle.northHueDeg, 1.0f, &r, &g, &b );
    uint16_t northColour = RGB565( r, g, b );
    probeLedHue( viewStyle.southHueDeg, 1.0f, &r, &g, &b );
    uint16_t southColour = RGB565( r, g, b );
    for ( int d = -1; d <= 1; d++ ) {
        fastLine( canvas, cx + d, cy, nx + d, ny, northColour );
        fastLine( canvas, cx, cy + d, nx, ny + d, northColour );
        fastLine( canvas, cx + d, cy, sx + d, sy, southColour );
        fastLine( canvas, cx, cy + d, sx, sy + d, southColour );
    }
    canvas->drawCircle( cx, cy, halfLengthMm > 3.0f ? 4 : 2, COLOR_TEXT );
}

// A circle on a horizontal plane, for error bars.
static void ring( Vec3 centre, float rx, float ry, uint16_t color ) {
    const int segments = 20;
    for ( int k = 0; k < segments; k++ ) {
        float a0 = k * 2.0f * (float)M_PI / segments, a1 = ( k + 1 ) * 2.0f * (float)M_PI / segments;
        line3d( { centre.x + rx * cosf( a0 ), centre.y + ry * sinf( a0 ), centre.z }, { centre.x + rx * cosf( a1 ), centre.y + ry * sinf( a1 ), centre.z }, color );
    }
}

static void drawMagnet( ) {
    const MagProbeFix& fix = magLocator.fix;
    const MagTrack& track = magLocator.track;

    // The trail, oldest first, fading in.
    for ( int n = 1; n < trailCount; n++ ) {
        int older = ( trailHead - trailCount + n - 1 + 2 * VIEW_TRAIL_POINTS ) % VIEW_TRAIL_POINTS;
        int newer = ( older + 1 ) % VIEW_TRAIL_POINTS;
        uint8_t level = (uint8_t)( 40 + 160 * n / trailCount );
        line3d( trail[ older ], trail[ newer ], RGB565( level, level * 3 / 4, 0 ) );
    }

    // With the tracker on, what is drawn is the TRACK: the filtered magnet,
    // its shaft, and the cursor on the surface - carried on through a gap in
    // the fixes (dimmer), and for a far probe just a soft ring of "about here".
    bool tracked = track.enabled && track.state != MAGTRACK_NONE;
    if ( tracked && track.state == MAGTRACK_ROUGH ) {
        Vec3 under = { track.position.x, track.position.y, magLocator.boardZ };
        ring( under, track.sigma.x, track.sigma.y, COLOR_ERROR );
        ring( under, 0.5f * track.sigma.x, 0.5f * track.sigma.y, COLOR_ERROR );
        line3d( under, track.position, COLOR_GRID );
        int cx, cy;
        if ( project( track.position, &cx, &cy ) ) {
            canvas->drawCircle( cx, cy, 3, COLOR_TEXT_DIM );
        }
        return;
    }
    if ( !tracked && !fix.valid ) {
        return;
    }
    Vec3 m = tracked ? track.viewPosition : fix.magnet;
    Vec3 axis = tracked ? track.shaft : fix.axis;
    Vec3 tip = tracked ? track.viewTip : fix.tip;
    Vec3 cursor = tracked ? track.cursor : fix.pointer;
    Vec3 sigma = tracked ? track.sigma : fix.sigma;
    bool coasting = tracked && track.state == MAGTRACK_COASTING;

    // The error bar, drawn where it matters: a 2-sigma ellipse on the surface
    // round the cursor, and a 2-sigma bar on the height.
    float barX = tracked ? track.cursorSigmaMm : sigma.x, barY = tracked ? track.cursorSigmaMm : sigma.y;
    ring( cursor, 2.0f * barX, 2.0f * barY, COLOR_ERROR );
    line3d( { m.x, m.y, m.z - 2.0f * sigma.z }, { m.x, m.y, m.z + 2.0f * sigma.z }, COLOR_ERROR );

    // Where it points: the shaft carried on from the point to the surface
    // (dotted), a cross there, and a drop to the sensor plane.
    const Vec3& p = cursor;
    for ( float f = 0.0f; f + 0.1f < 1.0f; f += 0.2f ) {
        line3d( { tip.x + f * ( p.x - tip.x ), tip.y + f * ( p.y - tip.y ), tip.z + f * ( p.z - tip.z ) },
                { tip.x + ( f + 0.1f ) * ( p.x - tip.x ), tip.y + ( f + 0.1f ) * ( p.y - tip.y ), tip.z + ( f + 0.1f ) * ( p.z - tip.z ) }, COLOR_SHADOW );
    }
    uint16_t cross = coasting ? COLOR_TEXT_DIM : COLOR_SHADOW;
    line3d( { p.x - 3, p.y, p.z }, { p.x + 3, p.y, p.z }, cross );
    line3d( { p.x, p.y - 3, p.z }, { p.x, p.y + 3, p.z }, cross );
    for ( float z = 0.0f; z + 1.0f < p.z; z += 2.0f ) {
        line3d( { p.x, p.y, z }, { p.x, p.y, z + 1.0f }, COLOR_GRID );
    }

    // The magnet: a bar along its axis, north half red, south half blue,
    // three pixels thick. While coasting, a thin grey one.
    if ( coasting ) {
        float h = viewStyle.poleMm;
        Vec3 n = { m.x + h * axis.x, m.y + h * axis.y, m.z + h * axis.z };
        Vec3 sth = { m.x - h * axis.x, m.y - h * axis.y, m.z - h * axis.z };
        line3d( n, sth, COLOR_TEXT_DIM );
    } else {
        drawMagnetBar( m, fix.axis, false, viewStyle.poleMm );
    }

    // The probe's point, down the shaft from the magnet.
    if ( magLocator.tipOffsetMm != 0.0f ) {
        int cx, cy, tx, ty;
        if ( project( m, &cx, &cy ) && project( tip, &tx, &ty ) ) {
            fastLine( canvas, cx, cy, tx, ty, COLOR_TIP );
            canvas->fillCircle( tx, ty, 2, COLOR_TIP );
        }
    }
}

// Text is drawn at UI_TEXT (2: 12 x 16 px characters, 20 to a line on
// this panel), the counted row at twice that. Every line is written to fit.
static void textAt( GFXcanvas16* canvas, int x, int y, int size, uint16_t color, const char* text ) {
    fastText( canvas, x, y, size, color, text );
}

static void drawText( ) {
    const MagProbeFix& fix = magLocator.fix;
    char line[ 48 ];
    const int T = UI_TEXT, H = UI_LINE_H;

    if ( fix.valid ) {
        snprintf( line, sizeof( line ), "x%.1f y%.1f z%.1f", fix.magnet.x, fix.magnet.y, fix.magnet.z );
        textAt( canvas, 2, 2, T, COLOR_TEXT, line );
        snprintf( line, sizeof( line ), "+-%.1f %s%.0f %.0f%%%s", fix.errorMm, fix.rough ? "rough " : "tilt", fix.rough ? fix.errorMm : fix.tiltDeg, fix.misfit * 100.0f,
                  magLocator.knownStrength > 0.0f ? " =" : "" );
        textAt( canvas, 2, 2 + H, T, COLOR_TEXT_DIM, line );
#if MODULE_ROW_COUNT
        // Row mode: the counted row, large, coloured by how sure the track (or
        // this fix) is of it; under it the offset, error bar and chance.
        const RowReading& reading = rowCounter.reading;
        if ( rowCounter.active && reading.valid && !rowCounter.calibrating( ) ) {
            float sure = reading.tracked ? reading.trackConfidence : reading.confidence;
            uint16_t colour = sure > 0.95f ? COLOR_SURE : ( sure > 0.68f ? COLOR_UNSURE : COLOR_WARNING );
            if ( reading.row > 0 ) {
                snprintf( line, sizeof( line ), "row %d", reading.row );
            } else {
                snprintf( line, sizeof( line ), "off end" );
            }
            textAt( canvas, 2, 2 + 2 * H + 2, 2 * T, colour, line );
            float offset = reading.tracked ? reading.trackPlace.along - floorf( reading.trackPlace.along + 0.5f ) : reading.offsetRows;
            float bar = reading.tracked ? reading.trackSigmaRows : reading.sigmaRows;
            snprintf( line, sizeof( line ), "%+.2f +-%.2f %.0f%% h%d", offset, bar, sure * 100.0f, reading.hole );
            textAt( canvas, 2, 2 + 4 * H + 6, T, COLOR_TEXT_DIM, line );
        }
#endif
    } else if ( magLocator.track.enabled && magLocator.track.state != MAGTRACK_NONE ) {
        const MagTrack& track = magLocator.track;
        bool rough = track.state == MAGTRACK_ROUGH;
        snprintf( line, sizeof( line ), rough ? "about x%.0f y%.0f z%.0f" : "x%.1f y%.1f z%.1f", track.position.x, track.position.y, track.position.z );
        textAt( canvas, 2, 2, T, rough ? COLOR_WARNING : COLOR_TEXT, line );
        snprintf( line, sizeof( line ), "+-%.0f mm %s", track.sigma.x, rough ? "rough" : "coasting" );
        textAt( canvas, 2, 2 + H, T, rough ? COLOR_WARNING : COLOR_TEXT_DIM, line );
    } else {
        const char* one = nullptr;
        const char* two = nullptr;
        uint16_t colour = COLOR_WARNING;
        if ( !boardVioIs3V3( ) ) {
            one = "VIO rail is not";
            two = "3.3 V";
        } else if ( magArray.sensorsOk( ) == 0 ) {
            one = "no sensors";
            two = "answering";
        } else if ( !magArray.baselineReady( ) ) {
            one = "zeroing - keep";
            two = "the magnet away";
        } else if ( magLocator.baselinePolluted ) {
            one = "zeroed WITH the";
            two = "magnet: move, z";
        } else if ( fix.present && fix.seenBy + fix.faintBy < MAGLOC_MIN_SENSORS ) {
            snprintf( line, sizeof( line ), "%.2f mT, %d sensor%s", fix.peakMt, fix.seenBy + fix.faintBy, fix.seenBy + fix.faintBy == 1 ? "" : "s" );
            one = line;
            two = "see it: need 3";
        } else if ( fix.present ) {
            snprintf( line, sizeof( line ), "%.2f mT no fix", fix.peakMt );
            one = line;
            two = "";
        } else {
            one = "no magnet";
            two = "";
            colour = COLOR_TEXT_DIM;
        }
        textAt( canvas, 2, 2, T, colour, one );
        textAt( canvas, 2, 2 + H, T, colour, two );
    }

#if MODULE_ROW_COUNT
    // Calibrating: which hole is wanted, and a bar that fills while the tap is taken.
    if ( rowCounter.calibrating( ) ) {
        int row, hole;
        rowCounter.calibrationTarget( &row, &hole );
        fastFillRect( canvas, 0, 2 + 2 * H, LCD_WIDTH, 4 * H, COLOR_BACKGROUND );
        snprintf( line, sizeof( line ), "tap row %d", row );
        textAt( canvas, 2, 2 + 2 * H + 2, 2 * T, COLOR_ROW_CURRENT, line );
        snprintf( line, sizeof( line ), "%s hole  %d/%d", hole == 1 ? "inner" : "outer", rowCounter.calibrationStepNumber( ) + 1, ROWGRID_CALIBRATION_TARGETS );
        textAt( canvas, 2, 2 + 4 * H + 6, T, COLOR_TEXT, line );
        int barY = 2 + 5 * H + 8;
        fastRect( canvas, 4, barY, LCD_WIDTH - 8, 8, COLOR_TEXT_DIM );
        fastFillRect( canvas, 4, barY, (int)( ( LCD_WIDTH - 8 ) * rowCounter.tapProgress( ) ), 8, COLOR_SURE );
    }
#endif

    // A simulated probe (:probe) is marked, so a screen shot says so.
    if ( magLocator.simProbeActive( ) ) {
        textAt( canvas, LCD_WIDTH - 3 * UI_CHAR_W - 2, 2, T, COLOR_WARNING, "SIM" );
    }
    // Two lines at the bottom: the array and frame rate; the camera, the
    // track's state and the cursor mode.
    static const char* trackNames[ 4 ] = { "-", "rough", "coast", "track" };
    snprintf( line, sizeof( line ), "%d/%d sens %2.0ffps s%.1f", magArray.sensorsOk( ), magArray.sensorCount( ), display.fps( ), magLocator.boardZ ); // s = the surface's height
    textAt( canvas, 2, LCD_HEIGHT - 2 * H - 2, T, COLOR_TEXT_DIM, line );
    snprintf( line, sizeof( line ), "%s %s %s", cameraModeNames[ viewCamera.mode ], magLocator.track.enabled ? trackNames[ magLocator.track.state ] : "raw",
              magLocator.track.cursorMode == MAGCURSOR_UNDER ? "under" : "aim" );
    textAt( canvas, 2, LCD_HEIGHT - H - 2, T, COLOR_TEXT_DIM, line );
}


// ---- the app -----------------------------------------------------------------------

void viewDraw( GFXcanvas16* into ) {
    canvas = into;
    uint32_t nowMs = millis( );
    const MagProbeFix& fix = magLocator.fix;
    // Keep the trail: a point per period while there is a fix, and let it
    // drain away at the same rate once there is not.
    if ( nowMs - lastTrailMs >= VIEW_TRAIL_PERIOD_MS ) {
        lastTrailMs = nowMs;
        const MagTrack& track = magLocator.track;
        bool tracked = track.enabled && ( track.state == MAGTRACK_TRACKING || track.state == MAGTRACK_COASTING );
        if ( tracked || fix.valid ) {
            trail[ trailHead ] = tracked ? track.viewPosition : fix.magnet;
            trailHead = ( trailHead + 1 ) % VIEW_TRAIL_POINTS;
            if ( trailCount < VIEW_TRAIL_POINTS ) {
                trailCount++;
            }
        } else if ( trailCount > 0 ) {
            trailCount--;
        }
    }

    aimCamera( nowMs );
    drawBoard( );
    drawSensors( );
    drawMagnet( );
    drawText( );
}

// The joystick, continuously: orbit, or zoom with the stick pressed.
void viewTick( float dtS, float joyX, float joyY ) {
    if ( joyX == 0.0f && joyY == 0.0f )
        return;
    if ( input.held( IN_JOY_PRESS ) ) {
        float f = 1.0f + joyY * VIEW_ZOOM_PER_S * dtS;
        cameraZoom( &viewCamera, f );
    } else {
        cameraOrbit( &viewCamera, joyX * VIEW_ORBIT_DEG_PER_S * dtS, joyY * VIEW_ORBIT_DEG_PER_S * dtS );
    }
}

bool viewEvent( const InputEvent* e ) {
    bool press = e->kind == IN_PRESS || e->kind == IN_REPEAT;
    switch ( e->control ) {
    case IN_NAV_LEFT:
        if ( press )
            cameraPan( &viewCamera, -VIEW_PAN_MM, 0.0f );
        return true;
    case IN_NAV_RIGHT:
        if ( press )
            cameraPan( &viewCamera, VIEW_PAN_MM, 0.0f );
        return true;
    case IN_NAV_UP:
        if ( press )
            cameraPan( &viewCamera, 0.0f, VIEW_PAN_MM );
        return true;
    case IN_NAV_DOWN:
        if ( press )
            cameraPan( &viewCamera, 0.0f, -VIEW_PAN_MM );
        return true;
    case IN_NAV_PRESS:
        if ( e->kind == IN_CLICK )
            cameraNextMode( &viewCamera );
        else if ( e->kind == IN_HOLD )
            cameraReset( &viewCamera );
        return true;
    case IN_JOY_PRESS:
        if ( e->kind == IN_CLICK ) {
            cameraReset( &viewCamera );
        } else if ( e->kind == IN_HOLD ) {
            cameraReset( &viewCamera );
            cameraSetMode( &viewCamera, CAMERA_FIXED );
        }
        return true;
    default:
        return false;
    }
}
