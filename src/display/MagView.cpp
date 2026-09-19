// SPDX-License-Identifier: MIT
#include "MagView.h"

#include <Adafruit_GFX.h>
#include <math.h>

#include "BoardPins.h"
#include "Console.h"
#include "Display.h"
#include "FastDraw.h"
#include "MagArray.h"
#include "MagLocator.h"
#include "ST7789.h"
#include "config.h"
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif
#if MODULE_PROBE_LEDS
#include "ProbeLedService.h"
#endif
#if MODULE_PLAY
#include "Play.h"
#endif
#if MODULE_UI
#include "Ui.h"
#include "UiStream.h"
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

MagView& magView = MagView::getInstance( );

MagView& MagView::getInstance( ) {
    static MagView instance;
    return instance;
}

static void onCamera( Stream* out ) {
    (void)out;
    magView.nextCamera( );
}

void MagView::nextCamera( ) {
    cameraNextMode( &cam );
}

static void onScreen( Stream* out ) {
    magView.nextScreen( );
    static const char* names[ MAGVIEW_SCREEN_COUNT ] = { "3D scene", "the breadboard's LEDs", "the log", "the draw screen" };
    char line[ 160 ];
    snprintf( line, sizeof( line ), "screen: %s  (%.0f frames/s on the last one)", names[ magView.screen ], display.fps( ) );
    out->println( line );
    // The cost of each screen, so a slow one is seen for what it is: the
    // loop stops for a draw, and the LEDs after it wait.
    static const char* const slots[ MAGVIEW_SCREEN_COUNT ] = { "scene", "LEDs", "log", "draw" };
    display.printStats( out, slots, MAGVIEW_SCREEN_COUNT );
}

void MagView::nextScreen( ) {
    screen = (MagViewScreen)( ( screen + 1 ) % MAGVIEW_SCREEN_COUNT );
}

void MagView::begin( ) {
    consoleAddCommand( 'v', "next camera mode (fixed / sway / spin / top / follow / POV)", onCamera );
    consoleAddCommand( 'e', "next screen: 3D scene / breadboard LEDs / log / draw", onScreen );

    // Look at the middle of the array, a little above the board, zoomed so the
    // board outline fills most of the panel's width.
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
    cameraInit( &cam, middle, MAGVIEW_YAW_DEG, MAGVIEW_ELEVATION_DEG, 4.0f * span, LCD_WIDTH / span );
}

bool magViewDraw( GFXcanvas16* canvas, uint32_t nowMs ) {
    display.slot = magView.screen;
    return magView.drawFrame( canvas, nowMs );
}

// ---- camera ----------------------------------------------------------------

void MagView::aimCamera( uint32_t nowMs ) {
    float dtS = lastFrameMs == 0 ? 0.03f : ( nowMs - lastFrameMs ) * 0.001f;
    lastFrameMs = nowMs;
    // The probe for the follow / POV modes: the track when there is one, else the fix.
    const MagTrack& track = magLocator.track;
    const MagProbeFix& fix = magLocator.fix;
    bool tracked = track.enabled && ( track.state == MAGTRACK_TRACKING || track.state == MAGTRACK_COASTING );
    bool haveProbe = tracked || fix.valid;
    Vec3 tip = tracked ? track.viewTip : fix.tip;
    Vec3 shaft = tracked ? track.shaft : fix.shaft;
    cameraUpdate( &cam, dtS, nowMs * 0.001f, haveProbe, tip, shaft );
    target = cam.target;
    zoom = cam.zoom;
    cameraDistance = cam.distance;
    sinYaw = sinf( cam.yawDeg * VIEW_DEG_TO_RAD );
    cosYaw = cosf( cam.yawDeg * VIEW_DEG_TO_RAD );
    sinElevation = sinf( cam.elevationDeg * VIEW_DEG_TO_RAD );
    cosElevation = cosf( cam.elevationDeg * VIEW_DEG_TO_RAD );
}

// Board frame (mm) -> screen pixel. The camera sits `cameraDistance` from the
// target, `elevation` above the board plane, swung `yaw` around the vertical.
bool MagView::project( Vec3 world, int* sx, int* sy ) const {
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

void MagView::line3d( Vec3 a, Vec3 b, uint16_t color ) {
    int ax, ay, bx, by;
    bool aFront = project( a, &ax, &ay );
    bool bFront = project( b, &bx, &by );
    if ( !aFront || !bFront ) {
        return; // no clipping, just culling: a line with an end behind the camera is left out
    }
    fastLine( canvas, ax, ay, bx, by, color );
}

// ---- the scene -------------------------------------------------------------

void MagView::drawBoard( ) {
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
void MagView::drawRows( ) {
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

void MagView::drawSensors( ) {
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
        // 0.05 mT -> 1.5 mm, 5 mT -> 10 mm, 80 mT -> 16 mm
        float length = 5.0f * log10f( 1.0f + magnitude / 0.05f );
        float k = length / magnitude;
        line3d( p, { p.x + k * b.x, p.y + k * b.y, p.z + k * b.z }, COLOR_FIELD );
    }
}

// A magnet as a bar along `axis`: north half red, south half blue. `flipped` = its north pole is at the -axis end.
void MagView::drawMagnetBar( Vec3 centre, Vec3 axis, bool flipped, float halfLengthMm ) {
    float h = flipped ? -halfLengthMm : halfLengthMm;
    Vec3 north = { centre.x + h * axis.x, centre.y + h * axis.y, centre.z + h * axis.z };
    Vec3 south = { centre.x - h * axis.x, centre.y - h * axis.y, centre.z - h * axis.z };
    int cx, cy, nx, ny, sx, sy;
    if ( !project( centre, &cx, &cy ) || !project( north, &nx, &ny ) || !project( south, &sx, &sy ) ) {
        return;
    }
    for ( int d = -1; d <= 1; d++ ) {
        fastLine( canvas, cx + d, cy, nx + d, ny, COLOR_NORTH );
        fastLine( canvas, cx, cy + d, nx, ny + d, COLOR_NORTH );
        fastLine( canvas, cx + d, cy, sx + d, sy, COLOR_SOUTH );
        fastLine( canvas, cx, cy + d, sx, sy + d, COLOR_SOUTH );
    }
    canvas->drawCircle( cx, cy, halfLengthMm > 3.0f ? 4 : 2, COLOR_TEXT );
}

// A circle on a horizontal plane, for error bars.
void MagView::ring( Vec3 centre, float rx, float ry, uint16_t color ) {
    const int segments = 20;
    for ( int k = 0; k < segments; k++ ) {
        float a0 = k * 2.0f * (float)M_PI / segments, a1 = ( k + 1 ) * 2.0f * (float)M_PI / segments;
        line3d( { centre.x + rx * cosf( a0 ), centre.y + ry * sinf( a0 ), centre.z }, { centre.x + rx * cosf( a1 ), centre.y + ry * sinf( a1 ), centre.z }, color );
    }
}

void MagView::drawMagnet( ) {
    const MagProbeFix& fix = magLocator.fix;
    const MagTrack& track = magLocator.track;

    // The trail, oldest first, fading in.
    for ( int n = 1; n < trailCount; n++ ) {
        int older = ( trailHead - trailCount + n - 1 + 2 * MAGVIEW_TRAIL_POINTS ) % MAGVIEW_TRAIL_POINTS;
        int newer = ( older + 1 ) % MAGVIEW_TRAIL_POINTS;
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
        Vec3 n = { m.x + MAGNET_HALF_LENGTH_MM * axis.x, m.y + MAGNET_HALF_LENGTH_MM * axis.y, m.z + MAGNET_HALF_LENGTH_MM * axis.z };
        Vec3 sth = { m.x - MAGNET_HALF_LENGTH_MM * axis.x, m.y - MAGNET_HALF_LENGTH_MM * axis.y, m.z - MAGNET_HALF_LENGTH_MM * axis.z };
        line3d( n, sth, COLOR_TEXT_DIM );
    } else {
        drawMagnetBar( m, fix.axis, false, MAGNET_HALF_LENGTH_MM );
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

// Text is drawn at MAGVIEW_TEXT (2: 12 x 16 px characters, 20 to a line on
// this panel), the counted row at twice that. Every line is written to fit.
static void textAt( GFXcanvas16* canvas, int x, int y, int size, uint16_t color, const char* text ) {
    fastText( canvas, x, y, size, color, text );
}

void MagView::drawText( ) {
    const MagProbeFix& fix = magLocator.fix;
    char line[ 48 ];
    const int T = MAGVIEW_TEXT, H = MAGVIEW_LINE_H;

    if ( fix.valid ) {
        snprintf( line, sizeof( line ), "x%.1f y%.1f z%.1f", fix.magnet.x, fix.magnet.y, fix.magnet.z );
        textAt( canvas, 2, 2, T, COLOR_TEXT, line );
        snprintf( line, sizeof( line ), "+-%.1f %s%.0f %.0f%%%s", fix.errorMm, fix.rough ? "rough " : "tilt", fix.rough ? fix.errorMm : fix.tiltDeg, fix.misfit * 100.0f,
                  magLocator.learning( ) ? " ?" : ( magLocator.knownStrength > 0.0f ? " =" : "" ) );
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
        textAt( canvas, LCD_WIDTH - 3 * MAGVIEW_CHAR_W - 2, 2, T, COLOR_WARNING, "SIM" );
    }
    // Two lines at the bottom: the array and frame rate; the camera, the
    // track's state and the cursor mode.
    static const char* trackNames[ 4 ] = { "-", "rough", "coast", "track" };
    snprintf( line, sizeof( line ), "%d/%d sens %2.0ffps s%.1f", magArray.sensorsOk( ), magArray.sensorCount( ), display.fps( ), magLocator.boardZ ); // s = the surface's height
    textAt( canvas, 2, LCD_HEIGHT - 2 * H - 2, T, COLOR_TEXT_DIM, line );
    snprintf( line, sizeof( line ), "%s %s %s", cameraModeNames[ cam.mode ], magLocator.track.enabled ? trackNames[ magLocator.track.state ] : "raw",
              magLocator.track.cursorMode == MAGCURSOR_UNDER ? "under" : "aim" );
    textAt( canvas, 2, LCD_HEIGHT - H - 2, T, COLOR_TEXT_DIM, line );
}

bool MagView::drawFrame( GFXcanvas16* into, uint32_t nowMs ) {
    canvas = into;
    const MagProbeFix& fix = magLocator.fix;
#if MODULE_UI
    // The log screen is all text, which GFX draws slowly (a full screen of
    // it measured 30 ms): only redraw it when something changed, and no more
    // than every MAGVIEW_LOG_REDRAW_MS even then (a CSV stream into the log
    // would otherwise hold the loop most of the time). A change of screen,
    // of the menu or of the scroll always redraws.
    if ( screen == MAGVIEW_SCREEN_LOG ) {
        uint32_t stamp = uiStream.logGeneration( ) + 7919u * (uint32_t)ui.logScroll + 104729u * ui.menuEdits + ( ui.menuOpen( ) ? 1u : 0u );
        bool sameScreen = lastDrawnScreen == (int)screen;
        if ( sameScreen && ( stamp == lastLogStamp || nowMs - lastLogDrawMs < MAGVIEW_LOG_REDRAW_MS ) ) {
            return false;
        }
        lastLogStamp = stamp;
        lastLogDrawMs = nowMs;
    }
    lastDrawnScreen = (int)screen;
#endif

    // Keep the trail: a point per period while there is a fix, and let it
    // drain away at the same rate once there is not.
    if ( nowMs - lastTrailMs >= MAGVIEW_TRAIL_PERIOD_MS ) {
        lastTrailMs = nowMs;
        const MagTrack& track = magLocator.track;
        bool tracked = track.enabled && ( track.state == MAGTRACK_TRACKING || track.state == MAGTRACK_COASTING );
        if ( tracked || fix.valid ) {
            trail[ trailHead ] = tracked ? track.viewPosition : fix.magnet;
            trailHead = ( trailHead + 1 ) % MAGVIEW_TRAIL_POINTS;
            if ( trailCount < MAGVIEW_TRAIL_POINTS ) {
                trailCount++;
            }
        } else if ( trailCount > 0 ) {
            trailCount--;
        }
    }

    aimCamera( nowMs );
    canvas->fillScreen( COLOR_BACKGROUND );
    if ( screen == MAGVIEW_SCREEN_LEDS ) {
        drawLedScreen( );
    } else if ( screen == MAGVIEW_SCREEN_DRAW ) {
        drawTraceScreen( );
    } else if ( screen == MAGVIEW_SCREEN_LOG ) {
#if MODULE_UI
        ui.drawLog( canvas );
#endif
    } else {
        drawBoard( );
        drawSensors( );
        drawMagnet( );
        drawText( );
    }
#if MODULE_UI
    if ( ui.menuOpen( ) ) {
        ui.drawMenu( canvas );
    }
#endif
    return true;
}

// The drawing over the board from above: every hole a dot, the painted
// ones squares in their colour at their level (as the LEDs have them), the
// brush's ring round the point as hollow squares in the paint colour, the
// point itself a cross. The map is the LED screen's: 7 px a row.
void MagView::drawTraceScreen( ) {
#if MODULE_PLAY
    const LedLayout& layout = probeLeds.layout;
    const ProbeLedFrame& frame = probeLeds.frame;
    const ProbeLedBrush& brush = probeLeds.brush;
    for ( int i = 0; i < layout.count; i++ ) {
        if ( layout.kind[ i ] != PROBELED_HOLE )
            continue;
        int x, y;
        PlayService::tracePlace( layout.along[ i ], layout.acrossMm[ i ], &x, &y );
        float painted = play.paint.level[ i ];
        if ( painted > 0.0f ) {
            // As lit: the paint's colour at its level (the LEDs' own gamma
            // is theirs; the panel shows the level as it is).
            float k = painted > 1.0f ? 1.0f : painted;
            fastFillRect( canvas, x - 2, y - 2, 5, 5, RGB565( (uint8_t)( play.paint.r[ i ] * k ), (uint8_t)( play.paint.g[ i ] * k ), (uint8_t)( play.paint.b[ i ] * k ) ) );
        } else {
            fastFillRect( canvas, x, y, 1, 1, COLOR_GRID );
        }
        // The brush ring (what the LED renderer is showing as the cursor).
        if ( brush.active && frame.target[ i ] > 0.0f && frame.target[ i ] != painted ) {
            fastRect( canvas, x - 3, y - 3, 7, 7, RGB565( brush.r, brush.g, brush.b ) );
        }
    }
    const ProbeLedInput& in = probeLeds.input;
    if ( in.state != PROBELED_NONE ) {
        int x, y;
        PlayService::tracePlace( in.haveUnder ? in.underAlong : in.along, in.haveUnder ? in.underAcrossMm : in.acrossMm, &x, &y );
        uint16_t c = in.heightMm < play.touchMm ? COLOR_SURE : COLOR_SHADOW;
        fastFillRect( canvas, x - 4, y, 9, 1, c );
        fastFillRect( canvas, x, y - 4, 1, 9, c );
    }
    char line[ 40 ];
    snprintf( line, sizeof( line ), "%s: %s", playModeNames[ play.mode ], in.state == PROBELED_NONE ? "no probe" : ( in.heightMm < play.touchMm ? "touching" : "lifted" ) ); // 20 columns
    textAt( canvas, 2, 2, MAGVIEW_TEXT, COLOR_TEXT_DIM, line );
    if ( play.mode == PLAY_PAINT ) {
        drawColourWheel( );
    }
    if ( play.mode == PLAY_TARGET ) {
        snprintf( line, sizeof( line ), "%lu hit %.2fs %.1fmm", (unsigned long)play.hits, play.meanMs * 1e-3f, play.meanMissMm );
        textAt( canvas, 2, LCD_HEIGHT - MAGVIEW_LINE_H - 2, MAGVIEW_TEXT, COLOR_TEXT_DIM, line );
    }
#else
    textAt( canvas, 4, 4, MAGVIEW_TEXT, COLOR_TEXT_DIM, "MODULE_PLAY is off" );
#endif
}

// The paint's colour wheel, bottom right of the draw screen: hue round it,
// saturation out from the white centre, the marker where the paint colour
// is (the joystick moves it), a swatch of the colour at its brightness, and
// the brush and draw/erase state. The wheel's pixels are worked out once.
#define WHEEL_R 26
#define WHEEL_CX ( LCD_WIDTH - WHEEL_R - 4 )
#define WHEEL_CY ( LCD_HEIGHT - WHEEL_R - 4 )
#define WHEEL_PANEL_Y ( LCD_HEIGHT - 5 * MAGVIEW_LINE_H ) // the settings, five lines to the wheel's left (the map is above, PLAY_TRACE_MID_Y)
static uint16_t wheelPixels[ ( 2 * WHEEL_R + 1 ) * ( 2 * WHEEL_R + 1 ) ];
static bool wheelReady = false;

void MagView::drawColourWheel( ) {
#if MODULE_PLAY
    if ( !wheelReady ) {
        for ( int dy = -WHEEL_R; dy <= WHEEL_R; dy++ ) {
            for ( int dx = -WHEEL_R; dx <= WHEEL_R; dx++ ) {
                float r = sqrtf( (float)( dx * dx + dy * dy ) ) / WHEEL_R;
                uint16_t c = 0;
                if ( r <= 1.0f ) {
                    float hue = atan2f( (float)-dy, (float)dx ) * ( 180.0f / 3.14159265f ); // y up
                    uint8_t cr, cg, cb;
                    playHsvToRgb( hue, r, &cr, &cg, &cb );
                    c = RGB565( cr, cg, cb );
                    if ( c == 0 )
                        c = 1; // 0 is the "outside" mark
                }
                wheelPixels[ ( dy + WHEEL_R ) * ( 2 * WHEEL_R + 1 ) + dx + WHEEL_R ] = c;
            }
        }
        wheelReady = true;
    }
    uint16_t* buffer = canvas->getBuffer( );
    for ( int dy = -WHEEL_R; dy <= WHEEL_R; dy++ ) {
        int y = WHEEL_CY + dy;
        if ( y < 0 || y >= LCD_HEIGHT )
            continue;
        for ( int dx = -WHEEL_R; dx <= WHEEL_R; dx++ ) {
            uint16_t c = wheelPixels[ ( dy + WHEEL_R ) * ( 2 * WHEEL_R + 1 ) + dx + WHEEL_R ];
            int x = WHEEL_CX + dx;
            if ( c != 0 && x >= 0 && x < LCD_WIDTH )
                buffer[ y * LCD_WIDTH + x ] = c;
        }
    }
    // The marker: a ring where the colour is.
    float rad = play.paintHue * ( 3.14159265f / 180.0f );
    int mx = WHEEL_CX + (int)( play.paintSat * WHEEL_R * cosf( rad ) + 0.5f );
    int my = WHEEL_CY - (int)( play.paintSat * WHEEL_R * sinf( rad ) + 0.5f );
    fastRect( canvas, mx - 3, my - 3, 7, 7, RGB565( 0, 0, 0 ) );
    fastRect( canvas, mx - 2, my - 2, 5, 5, RGB565( 255, 255, 255 ) );
    // Every setting of the play page, to the wheel's left: the colour, the
    // brush's brightness and width, draw or erase (with the swatch: the
    // colour as it will be painted), and how to clear. What the controls do
    // is in the README; the joystick, its click and the nav stick are it.
    uint8_t r, g, b;
    play.paintColour( &r, &g, &b );
    float k = play.erase ? 0.0f : play.paintBright;
    char line[ 24 ];
    int y = WHEEL_PANEL_Y;
    snprintf( line, sizeof( line ), "hue %3d sat %.2f", (int)( play.paintHue + 0.5f ) % 360, play.paintSat );
    textAt( canvas, 2, y, MAGVIEW_TEXT, COLOR_TEXT_DIM, line );
    y += MAGVIEW_LINE_H;
    snprintf( line, sizeof( line ), "bright %3d%%", (int)( play.paintBright * 100.0f + 0.5f ) );
    textAt( canvas, 2, y, MAGVIEW_TEXT, COLOR_TEXT_DIM, line );
    y += MAGVIEW_LINE_H;
    snprintf( line, sizeof( line ), "brush %d", (int)( play.brushSize + 0.5f ) );
    textAt( canvas, 2, y, MAGVIEW_TEXT, COLOR_TEXT_DIM, line );
    int sx = 2 + 7 * MAGVIEW_CHAR_W + 8, sy = y + 2; // the swatch after "brush 1", then draw / ERASE
    fastFillRect( canvas, sx, sy, 12, 12, RGB565( (uint8_t)( r * k ), (uint8_t)( g * k ), (uint8_t)( b * k ) ) );
    fastRect( canvas, sx - 1, sy - 1, 14, 14, play.erase ? COLOR_SHADOW : COLOR_TEXT_DIM );
    textAt( canvas, sx + 18, y, MAGVIEW_TEXT, play.erase ? COLOR_SHADOW : COLOR_TEXT, play.erase ? "ERASE" : "draw" );
    y += MAGVIEW_LINE_H;
    snprintf( line, sizeof( line ), "touch %.1f mm", play.touchMm );
    textAt( canvas, 2, y, MAGVIEW_TEXT, COLOR_TEXT_DIM, line );
    y += MAGVIEW_LINE_H;
    textAt( canvas, 2, y, MAGVIEW_TEXT, COLOR_TEXT_DIM, "hold nav: clear" );
#endif
}

// The breadboard's LEDs from above: rows along the panel, holes across it,
// each LED a square in the colour it would be lit, as the probe cursor
// renderer has it. Rows 1-30 at the top, 31-60 below the channel.
void MagView::drawLedScreen( ) {
#if MODULE_PROBE_LEDS
    const LedLayout& layout = probeLeds.layout;
    const ProbeLedFrame& frame = probeLeds.frame;
    const int cell = LCD_WIDTH / ( PROBELED_ROWS + 2 ); // 7 px per row
    const float pxPerMm = cell / 2.54f;
    const int x0 = ( LCD_WIDTH - PROBELED_ROWS * cell ) / 2;
    const int yMid = LCD_HEIGHT / 2 + 8;
    for ( int i = 0; i < layout.count; i++ ) {
        int x = x0 + (int)( ( layout.along[ i ] - 1.0f ) * cell );
        int y = yMid - (int)( layout.acrossMm[ i ] * pxPerMm );
        // The LCD is not an LED: no gamma, so the dim end of the bell shows.
        float level = frame.level[ i ] > 1.0f ? 1.0f : frame.level[ i ];
        uint8_t r = (uint8_t)( frame.r[ i ] * level ), g = (uint8_t)( frame.g[ i ] * level ), b = (uint8_t)( frame.b[ i ] * level );
        uint16_t colour = level > 0.02f ? RGB565( r, g, b ) : ( layout.kind[ i ] == PROBELED_RAIL ? RGB565( 24, 20, 20 ) : RGB565( 28, 32, 40 ) );
        fastFillRect( canvas, x, y - cell / 2 + 1, cell - 1, cell - 1, colour );
    }
    fastFillRect( canvas, x0, yMid, PROBELED_ROWS * cell, 1, COLOR_GRID );
    textAt( canvas, x0, (int)( yMid - 8 * pxPerMm * 2.54f - 28 ), MAGVIEW_TEXT, COLOR_TEXT_DIM, "1" );
    textAt( canvas, x0 + 28 * cell, (int)( yMid - 8 * pxPerMm * 2.54f - 28 ), MAGVIEW_TEXT, COLOR_TEXT_DIM, "30" );

    char line[ 60 ];
    const ProbeLedInput& in = probeLeds.input;
    static const char* names[ 4 ] = { "no probe", "far away", "coasting", "tracking" };
    if ( in.state == PROBELED_NONE ) {
        textAt( canvas, 2, 2, MAGVIEW_TEXT, COLOR_TEXT_DIM, names[ 0 ] );
    } else {
        int rowNumber = (int)floorf( in.along + 0.5f ) + ( in.acrossMm < 0.0f ? PROBELED_ROWS : 0 );
        snprintf( line, sizeof( line ), "%s row %d", names[ in.state ], rowNumber );
        textAt( canvas, 2, 2, MAGVIEW_TEXT, COLOR_TEXT, line );
        snprintf( line, sizeof( line ), "+-%.2f %.0f%% up %.0fmm%s", in.sigmaRows, in.confidence * 100.0f, in.heightMm, in.haveUnder ? " *" : "" );
        textAt( canvas, 2, 2 + MAGVIEW_LINE_H, MAGVIEW_TEXT, COLOR_TEXT_DIM, line );
    }
    snprintf( line, sizeof( line ), "%s %d LEDs %2.0f fps", probeLeds.v5 ? "V5" : "V6", layout.count, display.fps( ) );
    textAt( canvas, 2, LCD_HEIGHT - MAGVIEW_LINE_H - 2, MAGVIEW_TEXT, COLOR_TEXT_DIM, line );
#else
    canvas->setCursor( 4, 4 );
    canvas->setTextColor( COLOR_TEXT_DIM );
    canvas->print( "MODULE_PROBE_LEDS is off" );
#endif
}
