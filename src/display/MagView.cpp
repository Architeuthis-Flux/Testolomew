// SPDX-License-Identifier: MIT
#include "MagView.h"

#include <Adafruit_GFX.h>
#include <math.h>

#include "BoardPins.h"
#include "Console.h"
#include "MagArray.h"
#include "MagLocator.h"
#include "ST7789.h"
#include "config.h"
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif

#define COLOR_BACKGROUND RGB565( 0, 0, 0 )
#define COLOR_GRID RGB565( 30, 40, 60 )
#define COLOR_BOARD RGB565( 70, 100, 150 )
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
    camera = (MagViewCamera)( ( camera + 1 ) % MAGVIEW_CAMERA_COUNT );
}

bool MagView::begin( ) {
    consoleAddCommand( 'v', "cycle the 3D view camera (sway / fixed / spin / top)", onCamera );

    if ( !st7789Begin( ) ) {
        return false;
    }
    canvas = new GFXcanvas16( LCD_WIDTH, LCD_HEIGHT );
    if ( canvas == nullptr || canvas->getBuffer( ) == nullptr ) {
        canvas = nullptr;
        return false;
    }
    canvas->setTextWrap( false );

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
    target = { 0.5f * ( xMin + xMax ), 0.5f * ( yMin + yMax ), 8.0f };
    float span = ( xMax - xMin ) > ( yMax - yMin ) ? ( xMax - xMin ) : ( yMax - yMin );
    span += 2.0f * BOARD_MARGIN_MM + 40.0f;
    zoom = LCD_WIDTH / span;
    cameraDistance = 4.0f * span;
    return true;
}

// ---- camera ----------------------------------------------------------------

void MagView::aimCamera( uint32_t nowMs ) {
    float t = nowMs / 1000.0f;
    float yaw = MAGVIEW_YAW_DEG;
    float elevation = MAGVIEW_ELEVATION_DEG;
    switch ( camera ) {
    case MAGVIEW_SWAY:
        yaw += MAGVIEW_SWAY_DEG * sinf( t * 2.0f * (float)M_PI / 6.0f );
        break;
    case MAGVIEW_SPIN:
        yaw = fmodf( t * 30.0f, 360.0f );
        break;
    case MAGVIEW_TOP:
        yaw = 0.0f;
        elevation = 89.9f;
        break;
    default:
        break;
    }
    sinYaw = sinf( yaw * VIEW_DEG_TO_RAD );
    cosYaw = cosf( yaw * VIEW_DEG_TO_RAD );
    sinElevation = sinf( elevation * VIEW_DEG_TO_RAD );
    cosElevation = cosf( elevation * VIEW_DEG_TO_RAD );
}

// Board frame (mm) -> screen pixel. The camera sits `cameraDistance` from the
// target, `elevation` above the board plane, swung `yaw` around the vertical.
void MagView::project( Vec3 world, int* sx, int* sy ) const {
    float dx = world.x - target.x;
    float dy = world.y - target.y;
    float dz = world.z - target.z;

    float x1 = dx * cosYaw + dy * sinYaw; // swing about the vertical
    float y1 = -dx * sinYaw + dy * cosYaw;

    float up = y1 * sinElevation + dz * cosElevation;    // screen up
    float depth = y1 * cosElevation - dz * sinElevation; // away from the camera

    float range = cameraDistance + depth;
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
}

void MagView::line3d( Vec3 a, Vec3 b, uint16_t color ) {
    int ax, ay, bx, by;
    project( a, &ax, &ay );
    project( b, &bx, &by );
    canvas->drawLine( ax, ay, bx, by, color );
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
        line3d( rowGridToBoard( &grid, along, side * inner ), rowGridToBoard( &grid, along, side * outer ), color );
    }
    if ( rowCounter.calibrating( ) ) {
        int row, hole, sx, sy;
        rowCounter.calibrationTarget( &row, &hole );
        RowPlace wanted = rowGridHolePlace( row, hole );
        project( rowGridToBoard( &grid, wanted.along, wanted.acrossMm ), &sx, &sy );
        canvas->drawCircle( sx, sy, 5, COLOR_ROW_CURRENT );
        canvas->drawCircle( sx, sy, 6, COLOR_ROW_CURRENT );
    }
#endif
}

void MagView::drawSensors( ) {
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        const Vec3& p = magArray.position[ i ];
        bool ok = magArray.sensor( i ).ok;
        int sx, sy;
        project( p, &sx, &sy );
        canvas->fillRect( sx - 2, sy - 2, 5, 5, !ok ? COLOR_SENSOR_LOST : ( magArray.saturated[ i ] ? COLOR_WARNING : COLOR_SENSOR_OK ) );

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
    project( centre, &cx, &cy );
    project( north, &nx, &ny );
    project( south, &sx, &sy );
    for ( int d = -1; d <= 1; d++ ) {
        canvas->drawLine( cx + d, cy, nx + d, ny, COLOR_NORTH );
        canvas->drawLine( cx, cy + d, nx, ny + d, COLOR_NORTH );
        canvas->drawLine( cx + d, cy, sx + d, sy, COLOR_SOUTH );
        canvas->drawLine( cx, cy + d, sx, sy + d, COLOR_SOUTH );
    }
    canvas->drawCircle( cx, cy, halfLengthMm > 3.0f ? 4 : 2, COLOR_TEXT );
}

void MagView::drawMagnet( ) {
    const MagProbeFix& fix = magLocator.fix;

    // The trail, oldest first, fading in.
    for ( int n = 1; n < trailCount; n++ ) {
        int older = ( trailHead - trailCount + n - 1 + 2 * MAGVIEW_TRAIL_POINTS ) % MAGVIEW_TRAIL_POINTS;
        int newer = ( older + 1 ) % MAGVIEW_TRAIL_POINTS;
        uint8_t level = (uint8_t)( 40 + 160 * n / trailCount );
        line3d( trail[ older ], trail[ newer ], RGB565( level, level * 3 / 4, 0 ) );
    }

    if ( !fix.valid ) {
        return;
    }
    const Vec3& m = fix.magnet;

    // The error bar, drawn where it matters: a 2-sigma ellipse on the board
    // plane around the point under the magnet, and a 2-sigma bar on the height.
    const int segments = 20;
    for ( int k = 0; k < segments; k++ ) {
        float a0 = k * 2.0f * (float)M_PI / segments, a1 = ( k + 1 ) * 2.0f * (float)M_PI / segments;
        line3d( { fix.pointer.x + 2.0f * fix.sigma.x * cosf( a0 ), fix.pointer.y + 2.0f * fix.sigma.y * sinf( a0 ), fix.pointer.z },
                { fix.pointer.x + 2.0f * fix.sigma.x * cosf( a1 ), fix.pointer.y + 2.0f * fix.sigma.y * sinf( a1 ), fix.pointer.z }, COLOR_ERROR );
    }
    line3d( { m.x, m.y, m.z - 2.0f * fix.sigma.z }, { m.x, m.y, m.z + 2.0f * fix.sigma.z }, COLOR_ERROR );

    // Where it points: the shaft carried on from the point to the board's
    // surface (dotted), a cross there, and a drop to the sensor plane if the
    // board sits above it.
    const Vec3& p = fix.pointer;
    for ( float f = 0.0f; f + 0.1f < 1.0f; f += 0.2f ) {
        line3d( { fix.tip.x + f * ( p.x - fix.tip.x ), fix.tip.y + f * ( p.y - fix.tip.y ), fix.tip.z + f * ( p.z - fix.tip.z ) },
                { fix.tip.x + ( f + 0.1f ) * ( p.x - fix.tip.x ), fix.tip.y + ( f + 0.1f ) * ( p.y - fix.tip.y ), fix.tip.z + ( f + 0.1f ) * ( p.z - fix.tip.z ) }, COLOR_SHADOW );
    }
    line3d( { p.x - 3, p.y, p.z }, { p.x + 3, p.y, p.z }, COLOR_SHADOW );
    line3d( { p.x, p.y - 3, p.z }, { p.x, p.y + 3, p.z }, COLOR_SHADOW );
    for ( float z = 0.0f; z + 1.0f < p.z; z += 2.0f ) {
        line3d( { p.x, p.y, z }, { p.x, p.y, z + 1.0f }, COLOR_GRID );
    }

    // The magnet: a bar along its axis, north half red, south half blue,
    // three pixels thick.
    drawMagnetBar( m, fix.axis, false, MAGNET_HALF_LENGTH_MM );

    // The probe's point, down the shaft from the magnet.
    if ( magLocator.tipOffsetMm != 0.0f ) {
        int cx, cy, tx, ty;
        project( m, &cx, &cy );
        project( fix.tip, &tx, &ty );
        canvas->drawLine( cx, cy, tx, ty, COLOR_TIP );
        canvas->fillCircle( tx, ty, 2, COLOR_TIP );
    }
}

void MagView::drawText( ) {
    const MagProbeFix& fix = magLocator.fix;
    char line[ 48 ];

    canvas->setTextSize( 1 );
    canvas->setTextColor( COLOR_TEXT );
    if ( fix.valid ) {
        snprintf( line, sizeof( line ), "x%6.1f y%6.1f z%6.1f  +-%.1f mm", fix.magnet.x, fix.magnet.y, fix.magnet.z, fix.errorMm );
        canvas->setCursor( 4, 4 );
        canvas->print( line );
        snprintf( line, sizeof( line ), "tilt %2.0f  m %5.0f%s  misfit %2.0f%%", fix.tiltDeg, fix.strength,
                  magLocator.learning( ) ? "?" : ( magLocator.knownStrength > 0.0f ? "=" : "" ), fix.misfit * 100.0f );
        canvas->setCursor( 4, 16 );
        canvas->setTextColor( COLOR_TEXT_DIM );
        canvas->print( line );
#if MODULE_ROW_COUNT
        // Row mode: the counted row, large, coloured by how sure this frame's
        // fix is of its row; under it that fix's offset, error bar and chance.
        const RowReading& reading = rowCounter.reading;
        if ( rowCounter.active && reading.valid && !rowCounter.calibrating( ) ) {
            canvas->setTextSize( 3 );
            canvas->setTextColor( reading.confidence > 0.95f ? COLOR_SURE : ( reading.confidence > 0.68f ? COLOR_UNSURE : COLOR_WARNING ) );
            if ( reading.row > 0 ) {
                snprintf( line, sizeof( line ), "row %d", reading.row );
            } else {
                snprintf( line, sizeof( line ), "off end" );
            }
            canvas->setCursor( 4, 30 );
            canvas->print( line );
            canvas->setTextSize( 1 );
            snprintf( line, sizeof( line ), "%+.2f hole %d  +-%.2f rows  %.0f%% sure", reading.offsetRows, reading.hole, reading.sigmaRows, reading.confidence * 100.0f );
            canvas->setCursor( 4, 58 );
            canvas->print( line );
        }
#endif
    } else {
        canvas->setCursor( 4, 4 );
        if ( !boardVioIs3V3( ) ) {
            canvas->setTextColor( COLOR_WARNING );
            canvas->print( "VIO rail is not 3.3 V" );
        } else if ( magArray.sensorsOk( ) == 0 ) {
            canvas->setTextColor( COLOR_WARNING );
            canvas->print( "no sensors answering" );
        } else if ( !magArray.baselineReady( ) ) {
            canvas->setTextColor( COLOR_WARNING );
            canvas->print( "zeroing - keep the magnet away" );
        } else if ( fix.present && fix.seenBy + fix.faintBy < MAGLOC_MIN_SENSORS ) {
            canvas->setTextColor( COLOR_WARNING );
            snprintf( line, sizeof( line ), "%.2f mT, noticed by %d sensor%s: need %d", fix.peakMt, fix.seenBy + fix.faintBy, fix.seenBy + fix.faintBy == 1 ? "" : "s", MAGLOC_MIN_SENSORS );
            canvas->print( line );
        } else if ( fix.present ) {
            canvas->setTextColor( COLOR_WARNING );
            snprintf( line, sizeof( line ), "%.2f mT, no fix (%.0f%%, +-%.0f mm)", fix.peakMt, fix.misfit * 100.0f, fix.errorMm > 99.0f ? 99.0f : fix.errorMm );
            canvas->print( line );
        } else {
            canvas->setTextColor( COLOR_TEXT_DIM );
            canvas->print( "no magnet" );
        }
    }

#if MODULE_ROW_COUNT
    // Calibrating: which hole is wanted, and a bar that fills while the tap is taken.
    if ( rowCounter.calibrating( ) ) {
        int row, hole;
        rowCounter.calibrationTarget( &row, &hole );
        canvas->fillRect( 0, 28, LCD_WIDTH, 50, COLOR_BACKGROUND );
        canvas->setTextSize( 3 );
        canvas->setTextColor( COLOR_ROW_CURRENT );
        snprintf( line, sizeof( line ), "tap row %d", row );
        canvas->setCursor( 4, 30 );
        canvas->print( line );
        canvas->setTextSize( 1 );
        snprintf( line, sizeof( line ), "%s   %d/%d", hole == 1 ? "hole next to the channel" : "outermost hole", rowCounter.calibrationStepNumber( ) + 1, ROWGRID_CALIBRATION_TARGETS );
        canvas->setCursor( 4, 58 );
        canvas->print( line );
        canvas->drawRect( 4, 70, LCD_WIDTH - 8, 6, COLOR_TEXT_DIM );
        canvas->fillRect( 4, 70, (int)( ( LCD_WIDTH - 8 ) * rowCounter.tapProgress( ) ), 6, COLOR_SURE );
    }
#endif

    static const char* cameraNames[ MAGVIEW_CAMERA_COUNT ] = { "sway", "fixed", "spin", "top" };
    snprintf( line, sizeof( line ), "%d/%d sensors  %2.0f fps  fit %lu us  %s", magArray.sensorsOk( ), magArray.sensorCount( ),
              framesPerSecond, (unsigned long)fix.fitUs, cameraNames[ camera ] );
    canvas->setTextColor( COLOR_TEXT_DIM );
    canvas->setCursor( 4, LCD_HEIGHT - 12 );
    canvas->print( line );
}

void MagView::drawFrame( uint32_t nowMs ) {
    const MagProbeFix& fix = magLocator.fix;

    // Keep the trail: a point per period while there is a fix, and let it
    // drain away at the same rate once there is not.
    if ( nowMs - lastTrailMs >= MAGVIEW_TRAIL_PERIOD_MS ) {
        lastTrailMs = nowMs;
        if ( fix.valid ) {
            trail[ trailHead ] = fix.magnet;
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
    drawBoard( );
    drawSensors( );
    drawMagnet( );
    drawText( );
}

ServiceStatus MagView::service( ) {
    if ( canvas == nullptr ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }

    if ( pushRow < 0 ) {
        drawFrame( millis( ) );
        pushRow = 0;
    } else {
        int rows = LCD_HEIGHT - pushRow;
        if ( rows > MAGVIEW_BAND_ROWS ) {
            rows = MAGVIEW_BAND_ROWS;
        }
        st7789PushRows( canvas->getBuffer( ), pushRow, rows );
        pushRow += rows;
        if ( pushRow >= LCD_HEIGHT ) {
            pushRow = -1;
            fpsWindowFrames++;
            uint32_t now = millis( );
            if ( now - fpsWindowStartMs >= 1000 ) {
                framesPerSecond = fpsWindowFrames * 1000.0f / ( now - fpsWindowStartMs );
                fpsWindowStartMs = now;
                fpsWindowFrames = 0;
            }
        }
    }

    lastStatus = ServiceStatus::BUSY;
    return lastStatus;
}
