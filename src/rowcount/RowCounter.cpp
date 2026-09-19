// SPDX-License-Identifier: MIT
#include "RowCounter.h"

#include <math.h>

#include "Console.h"
#include "MagLocator.h"

RowCounter& rowCounter = RowCounter::getInstance( );

RowCounter& RowCounter::getInstance( ) {
    static RowCounter instance;
    return instance;
}

// R is followed by a row number (consoleReadNumber). 0 = nothing usable was typed.
static int readRowNumber( Stream* out ) {
    long number = consoleReadNumber( out, "row number, then Enter: ", ROWCOUNT_NUMBER_TIMEOUT_MS );
    if ( number < 1 || number > 2 * ROWGRID_ROWS_PER_HALF ) {
        out->println( "no row number (1-60) - nothing changed" );
        return 0;
    }
    return (int)number;
}

static const char* holeName( int hole ) {
    return hole == 1 ? "hole next to the channel" : ( hole == ROWGRID_HOLES_PER_ROW ? "outermost hole" : "hole" );
}

static void onToggle( Stream* out ) { rowCounter.toggle( out ); }

static void onHold( Stream* out ) { rowCounter.startHold( ROWHOLD_TEST, 0, out ); }

static void onAnchor( Stream* out ) {
    int row = readRowNumber( out );
    if ( row > 0 ) {
        rowCounter.startHold( ROWHOLD_ANCHOR, row, out );
    }
}

static void onCalibrate( Stream* out ) { rowCounter.startCalibration( out ); }

static void onForget( Stream* out ) { rowCounter.forgetAnchors( out ); }

void RowCounter::begin( ) {
    static const RowGrid atBoot = ROWCOUNT_GRID_AT_BOOT;
    grid = atBoot;
    consoleAddCommand( 'r', "row mode: which breadboard row the probe is over (toggle)", onToggle );
    consoleAddCommand( 'c', "calibrate the rows: tap the 12 holes it asks for (again = cancel)", onCalibrate );
    consoleAddCommand( 'R', "one more anchor: the probe is on row <number>, hole next to the channel (R30<Enter>)", onAnchor );
    consoleAddCommand( 'C', "forget the row anchors: back to the grid the firmware boots with", onForget );
    consoleAddCommand( 'h', "hold-still test: 2 s of fixes -> row, scatter, step since the last test", onHold );
}

void RowCounter::toggle( Stream* out ) {
    active = !active;
    saidNoFix = false;
    if ( !active ) {
        out->println( "row mode off" );
        return;
    }
    char line[ 240 ];
    Vec3 row1 = rowGridToBoard( &grid, 1.0f, 0.0f );
    snprintf( line, sizeof( line ), "row mode on: rows 1-30 over 31-60, %d anchor%s. Rows 1/31 are level with x %.1f y %.1f mm on the channel line, counting up at %.1f deg; the fit reads %+.1f %% along, %+.1f %% across.",
              anchorCount, anchorCount == 1 ? "" : "s", row1.x, row1.y, rowGridAngleDeg( &grid ), ( rowGridScaleAlong( &grid ) - 1.0f ) * 100.0f, ( rowGridScaleAcross( &grid ) - 1.0f ) * 100.0f );
    out->println( line );
    out->println( "c = calibrate by tapping 12 holes, R<row> = add one anchor, C = forget them, h = hold-still test." );
}

// ---- anchors -------------------------------------------------------------------

// A second anchor on the same hole replaces the first.
void RowCounter::addAnchor( int row, int hole, Vec3 position, float sigmaMm ) {
    int at = anchorCount;
    for ( int k = 0; k < anchorCount; k++ ) {
        if ( anchors[ k ].row == row && anchors[ k ].hole == hole ) {
            at = k;
        }
    }
    if ( at >= ROWCOUNT_MAX_ANCHORS ) {
        return;
    }
    anchors[ at ] = { row, hole, position, sigmaMm };
    if ( at == anchorCount ) {
        anchorCount++;
    }
}

void RowCounter::forgetAnchors( Stream* out ) {
    static const RowGrid atBoot = ROWCOUNT_GRID_AT_BOOT;
    grid = atBoot;
    anchorCount = 0;
    haveLastHold = false;
    out->println( "anchors forgotten: the grid is the one the firmware boots with" );
}

void RowCounter::restoreAnchors( const RowAnchor* list, int count ) {
    anchorCount = 0;
    for ( int k = 0; k < count && k < ROWCOUNT_MAX_ANCHORS; k++ ) {
        anchors[ anchorCount++ ] = list[ k ];
    }
    Stream* out = console.port( );
    if ( out != nullptr ) {
        char line[ 80 ];
        snprintf( line, sizeof( line ), "row anchors: %d put back from the saved settings", anchorCount );
        out->println( line );
    }
    fitToAnchors( out );
}

void RowCounter::fitToAnchors( Stream* out ) {
    static const char* kindName[] = { "nothing", "moved into place only - angle and scale need anchors 10 mm or more apart", "place, angle and scale", "place, angle, a scale each way and skew" };
    char line[ 240 ];

    RowFitReport report = rowGridFit( &grid, anchors, anchorCount, nullptr );
    haveLastHold = false;

    snprintf( line, sizeof( line ), "grid fitted to %d anchor%s (%s).", anchorCount, anchorCount == 1 ? "" : "s", kindName[ report.kind ] );
    out->println( line );
    out->println( "  row hole        x       y       z   +/-mm    misses by: along (rows)  across (mm)" );
    for ( int k = 0; k < anchorCount; k++ ) {
        const RowAnchor& a = anchors[ k ];
        RowPlace wanted = rowGridHolePlace( a.row, a.hole );
        RowPlace got = rowGridPlace( &grid, a.position );
        snprintf( line, sizeof( line ), "  %3d  %3d   %6.2f  %6.2f  %6.2f   %5.2f                   %+6.2f       %+6.2f", a.row, a.hole, a.position.x, a.position.y, a.position.z, a.sigmaMm,
                  got.along - wanted.along, got.acrossMm - wanted.acrossMm );
        out->println( line );
    }
    if ( anchorCount > 1 ) {
        snprintf( line, sizeof( line ), "  the anchors miss their holes by %.2f mm rms, worst %.2f mm (row %d). Half a row is 1.27 mm.", report.rmsMm, report.worstMm, anchors[ report.worst ].row );
        out->println( line );
    }
    snprintf( line, sizeof( line ), "  the magnet fit reads %+.1f %% along the rows and %+.1f %% across; rows count up at %.1f deg from board +x.",
              ( rowGridScaleAlong( &grid ) - 1.0f ) * 100.0f, ( rowGridScaleAcross( &grid ) - 1.0f ) * 100.0f, rowGridAngleDeg( &grid ) );
    out->println( line );
    snprintf( line, sizeof( line ), "  to keep it (RowCounter.h): #define ROWCOUNT_GRID_AT_BOOT { %.5ff, %.5ff, %.3ff, %.5ff, %.5ff, %.3ff }", grid.ax, grid.ay, grid.a0, grid.cx, grid.cy, grid.c0 );
    out->println( line );
}

// ---- the guided calibration ----------------------------------------------------

void RowCounter::startCalibration( Stream* out ) {
    if ( calibrating( ) ) {
        calibrationStep = -1;
        out->println( "calibration cancelled: nothing changed" );
        return;
    }
    calibrationStep = 0;
    anchorCount = 0;
    stillCount = 0;
    haveLastTap = false;
    // Wherever the probe is right now is not the first tap, however still it
    // is held while the instructions are read: it has to go somewhere else
    // (or be lifted away) first.
    leftStart = !magLocator.fix.valid;
    startPosition = magLocator.fix.rawPointer;
    touchZ = ROWCOUNT_TOUCH_Z_MM;
    active = true; // the LCD shows which hole is wanted
    out->println( "calibrating the rows: rest the probe in each hole it asks for and keep it still for a second, until it asks for the next." );
    out->println( "the holes: rows 1, 15, 30, then 60, 45, 31 - each at the hole next to the channel, then the outermost hole. c = cancel." );
    int row, hole;
    calibrationTarget( &row, &hole );
    char line[ 80 ];
    snprintf( line, sizeof( line ), "  1/%d: row %d, %s", ROWGRID_CALIBRATION_TARGETS, row, holeName( hole ) );
    out->println( line );
}

// Called with every new fix while calibrating.
void RowCounter::watchForTap( Stream* out ) {
    const MagProbeFix& fix = magLocator.fix;
    // Until the board's height is known the pointer runs on to the sensor plane,
    // 15 mm too far: the point itself is the better tap until then.
    Vec3 p = touchZ > 0.0f ? fix.rawPointer : fix.rawTip;

    // Still where it was?
    float tolerance = 3.0f * fix.errorXyMm;
    tolerance = tolerance < ROWCOUNT_TAP_STILL_MIN_MM ? ROWCOUNT_TAP_STILL_MIN_MM : tolerance;
    tolerance = tolerance > ROWCOUNT_TAP_STILL_MAX_MM ? ROWCOUNT_TAP_STILL_MAX_MM : tolerance;
    if ( stillCount > 0 ) {
        float dx = p.x - stillSum.x / stillCount, dy = p.y - stillSum.y / stillCount, dz = fix.rawTip.z - stillSum.z / stillCount;
        if ( dx * dx + dy * dy > tolerance * tolerance || fabsf( dz ) > 2.0f * tolerance ) {
            // Off the spot. A few such fixes are a glitch and are left out; a run of them is the probe moving.
            if ( ++strayCount > ROWCOUNT_TAP_STRAYS ) {
                stillCount = 0;
            }
            return;
        }
    }
    strayCount = 0;
    // Resting on the board (once it is known how high that is), and not still on the last hole?
    bool down = touchZ <= 0.0f || fix.rawTip.z < touchZ + ROWCOUNT_TOUCH_MARGIN_MM;
    float apartSq = ( p.x - lastTap.x ) * ( p.x - lastTap.x ) + ( p.y - lastTap.y ) * ( p.y - lastTap.y );
    bool moved = !haveLastTap || apartSq > ROWCOUNT_TAP_APART_MM * ROWCOUNT_TAP_APART_MM;
    if ( !leftStart ) {
        float dx = p.x - startPosition.x, dy = p.y - startPosition.y, dz = p.z - startPosition.z;
        leftStart = dx * dx + dy * dy + dz * dz > ROWCOUNT_TAP_APART_MM * ROWCOUNT_TAP_APART_MM;
        moved = false;
    }
    if ( !down || !moved ) {
        stillCount = 0;
        return;
    }

    if ( stillCount == 0 ) {
        stillSum = { 0, 0, 0 };
    }
    if ( stillCount <= ROWCOUNT_TAP_SETTLE_FIXES ) { // the measurement starts over until the settling second is done
        tapSum = { 0, 0, 0 };
        tapSigmaSum = 0.0f;
        tapCount = 0;
    }
    stillCount++;
    stillSum = { stillSum.x + p.x, stillSum.y + p.y, stillSum.z + fix.rawTip.z };
    tapSum = { tapSum.x + p.x, tapSum.y + p.y, tapSum.z + fix.rawTip.z }; // x, y of the pointer; z of the point itself (the resting height)
    tapSigmaSum += fix.errorXyMm;
    tapCount++;
    if ( stillCount < ROWCOUNT_TAP_FIXES ) {
        return;
    }

    // A tap.
    int row, hole;
    calibrationTarget( &row, &hole );
    Vec3 mean = { tapSum.x / tapCount, tapSum.y / tapCount, tapSum.z / tapCount };
    addAnchor( row, hole, mean, tapSigmaSum / tapCount );
    lastTap = mean;
    haveLastTap = true;
    stillCount = 0;
    if ( touchZ <= 0.0f ) {
        touchZ = mean.z; // the first tap says how high the board holds the magnet
        magLocator.boardZ = touchZ;
    }

    char line[ 160 ];
    snprintf( line, sizeof( line ), "       got it: x %.2f y %.2f z %.2f mm, +/-%.2f", mean.x, mean.y, mean.z, tapSigmaSum / tapCount );
    out->println( line );
    calibrationStep++;
    if ( calibrationStep >= ROWGRID_CALIBRATION_TARGETS ) {
        finishCalibration( out );
        return;
    }
    calibrationTarget( &row, &hole );
    snprintf( line, sizeof( line ), "  %d/%d: row %d, %s", calibrationStep + 1, ROWGRID_CALIBRATION_TARGETS, row, holeName( hole ) );
    out->println( line );
}

void RowCounter::finishCalibration( Stream* out ) {
    calibrationStep = -1;

    // The height the board holds the magnet at: the middle of the taps' heights.
    float heights[ ROWCOUNT_MAX_ANCHORS ];
    for ( int k = 0; k < anchorCount; k++ ) {
        int at = k;
        while ( at > 0 && heights[ at - 1 ] > anchors[ k ].position.z ) {
            heights[ at ] = heights[ at - 1 ];
            at--;
        }
        heights[ at ] = anchors[ k ].position.z;
    }
    touchZ = heights[ anchorCount / 2 ];
    magLocator.boardZ = touchZ; // the probe now points at the board's surface, not the sensor plane

    fitToAnchors( out );
    char line[ 200 ];
    snprintf( line, sizeof( line ), "  resting on the board the magnet is %.1f mm up (taps ranged %.1f to %.1f - the spread is the fit's height error): #define ROWCOUNT_TOUCH_Z_MM %.1ff",
              touchZ, heights[ 0 ], heights[ anchorCount - 1 ], touchZ );
    out->println( line );
}

// ---- holds: the hold-still test and R ---------------------------------------------

void RowCounter::startHold( RowHoldPurpose purpose, int row, Stream* out ) {
    if ( calibrating( ) ) {
        out->println( "calibrating - c cancels it" );
        return;
    }
    holdPurpose = purpose;
    holdRow = row;
    holdCount = 0;
    holdSettle = 0;
    holdSum = { 0, 0, 0 };
    holdSigmaSum = 0.0f;
    holdSigmaMmSum = 0.0f;
    holdMisfitSum = 0.0f;
    holdStartMs = millis( );
    out->println( "hold it still..." );
}

void RowCounter::collect( ) {
    const MagProbeFix& fix = magLocator.fix;
    if ( holdSettle < ROWCOUNT_HOLD_SETTLE_FIXES ) {
        holdSettle++; // the first second is the hand settling: not measured
        return;
    }
    held[ holdCount++ ] = reading.place.along;
    Vec3 p = touchZ > 0.0f ? fix.rawPointer : fix.rawTip; // as in watchForTap()
    holdSum.x += p.x;
    holdSum.y += p.y;
    holdSum.z += fix.rawTip.z;
    holdSigmaSum += reading.sigmaRows;
    holdSigmaMmSum += fix.errorXyMm;
    holdMisfitSum += fix.misfit;
}

// Standard deviation of values[0..count) about their mean.
static float scatterOf( const float* values, int count, float* meanOut ) {
    float mean = 0.0f;
    for ( int i = 0; i < count; i++ ) {
        mean += ( values[ i ] - values[ 0 ] ) / count; // sum differences: small numbers keep their digits
    }
    mean += values[ 0 ];
    float sumSq = 0.0f;
    for ( int i = 0; i < count; i++ ) {
        sumSq += ( values[ i ] - mean ) * ( values[ i ] - mean );
    }
    if ( meanOut != nullptr ) {
        *meanOut = mean;
    }
    return count > 1 ? sqrtf( sumSq / ( count - 1 ) ) : 0.0f;
}

void RowCounter::finishHold( Stream* out ) {
    RowHoldPurpose purpose = holdPurpose;
    holdPurpose = ROWHOLD_NONE;
    char line[ 240 ];

    int n = holdCount;
    Vec3 mean = { holdSum.x / n, holdSum.y / n, holdSum.z / n };
    float scatter = scatterOf( held, n, nullptr );

    if ( purpose == ROWHOLD_ANCHOR ) {
        snprintf( line, sizeof( line ), "row %d, %s: x %.2f y %.2f z %.2f mm, scatter while holding %.2f rows.", holdRow, holeName( 1 ), mean.x, mean.y, mean.z, scatter );
        out->println( line );
        addAnchor( holdRow, 1, mean, holdSigmaMmSum / n );
        fitToAnchors( out );
        return;
    }

    // The hold-still test.
    RowPlace place = rowGridPlace( &grid, mean );
    bool bottom = place.acrossMm < 0.0f;
    int row = rowGridRow( place.along, bottom );
    int nearest = (int)floorf( place.along + 0.5f );
    int called = 0;
    for ( int i = 0; i < n; i++ ) {
        if ( (int)floorf( held[ i ] + 0.5f ) == nearest ) {
            called++;
        }
    }
    // What averaging buys: the scatter of means of ROWCOUNT_BLOCK_FIXES fixes.
    float blockMean[ ROWCOUNT_HOLD_FIXES / ROWCOUNT_BLOCK_FIXES ];
    int blocks = n / ROWCOUNT_BLOCK_FIXES;
    for ( int b = 0; b < blocks; b++ ) {
        scatterOf( held + b * ROWCOUNT_BLOCK_FIXES, ROWCOUNT_BLOCK_FIXES, &blockMean[ b ] );
    }
    float blockScatter = scatterOf( blockMean, blocks, nullptr );
    float seconds = ( millis( ) - holdStartMs ) / 1000.0f;
    float shown = place.along + ( bottom ? ROWGRID_ROWS_PER_HALF : 0 ); // 44.02 rather than "14.02 on the near half"

    snprintf( line, sizeof( line ), "hold test: %d fixes in %.1f s at x %.2f y %.2f z %.2f mm, misfit %.0f %%", n, seconds, mean.x, mean.y, mean.z, holdMisfitSum / n * 100.0f );
    out->println( line );
    if ( row == 0 ) {
        snprintf( line, sizeof( line ), "  %.2f rows along: off the end of the breadboard", place.along );
    } else {
        snprintf( line, sizeof( line ), "  %.2f -> row %d, %+.2f rows (%+.2f mm) off its centre; %.1f mm from the channel = hole %d", shown, row, place.along - nearest,
                  ( place.along - nearest ) * ROWGRID_PITCH_MM, fabsf( place.acrossMm ), rowGridHole( place.acrossMm ) );
    }
    out->println( line );
    snprintf( line, sizeof( line ), "  scatter of single fixes %.2f rows = %.2f mm (the fit's own error bar said %.2f rows); averaged %d at a time %.2f rows",
              scatter, scatter * ROWGRID_PITCH_MM, holdSigmaSum / n, ROWCOUNT_BLOCK_FIXES, blockScatter );
    out->println( line );
    snprintf( line, sizeof( line ), "  single fixes that called this row: %d of %d (%.0f %%)", called, n, 100.0f * called / n );
    out->println( line );
    if ( haveLastHold ) {
        float moved = place.along - lastHold.along;
        snprintf( line, sizeof( line ), "  moved since the last hold test: %+.2f rows (%+.2f mm) along, %+.2f mm across", moved, moved * ROWGRID_PITCH_MM, place.acrossMm - lastHold.acrossMm );
        out->println( line );
    }
    haveLastHold = true;
    lastHold = place;
}

// ---- the service -----------------------------------------------------------------

ServiceStatus RowCounter::service( ) {
    const MagProbeFix& fix = magLocator.fix;
    Stream* out = console.port( );
    uint32_t now = millis( );

    if ( holdPurpose != ROWHOLD_NONE && now - holdStartMs > ROWCOUNT_HOLD_TIMEOUT_MS ) {
        if ( out != nullptr ) {
            char line[ 120 ];
            snprintf( line, sizeof( line ), "gave up: only %d of %d fixes in %d s (after a second's settling). l says why there is no fix.", holdCount, ROWCOUNT_HOLD_FIXES, ROWCOUNT_HOLD_TIMEOUT_MS / 1000 );
            out->println( line );
        }
        holdPurpose = ROWHOLD_NONE;
    }

    const MagTrack& track = magLocator.track;
    bool tracked = track.enabled && ( track.state == MAGTRACK_TRACKING || track.state == MAGTRACK_COASTING );
    // A rough fix (the probe far up or off the edge) counts no row.
    bool rough = track.enabled ? track.state == MAGTRACK_ROUGH : fix.rough;

    if ( !fix.valid || rough ) {
        if ( tracked && !rough ) {
            followTrack( ); // the counted row rides the coasting track through a gap, and stays shown
        } else {
            reading.valid = false;
            reading.tracked = false;
        }
        if ( now - lastFixMs > ROWCOUNT_TAP_GAP_MS ) {
            stillCount = 0;   // a tap survives a dropped frame or two, not a lifted probe
            leftStart = true; // ...and a probe lifted right away has certainly left where it was
        }
        if ( active && !saidNoFix && out != nullptr && now - lastFixMs > ROWCOUNT_NO_FIX_AFTER_MS ) {
            out->println( rough ? "row: too far to say (rough fix)" : "row: no fix" );
            saidNoFix = true;
        }
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    if ( fix.count == lastFixCount ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    lastFixCount = fix.count;
    lastFixMs = now;
    saidNoFix = false;

    // This frame's raw fix, with its own error bar.
    // (The bar is the magnet's. With a tip offset the tip's is a little wider.)
    reading.place = rowGridPlace( &grid, fix.rawPointer );
    reading.offsetRows = reading.place.along - floorf( reading.place.along + 0.5f );
    reading.hole = rowGridHole( reading.place.acrossMm );
    reading.sigmaRows = rowGridSigmaAlong( &grid, fix.sigma );
    reading.sigmaAcrossMm = rowGridSigmaAcross( &grid, fix.sigma );
    reading.confidence = rowGridConfidence( reading.place, reading.sigmaRows, reading.sigmaAcrossMm );
    reading.touching = touchZ > 0.0f && fix.rawTip.z < touchZ + ROWCOUNT_TOUCH_MARGIN_MM;

    // The counted row follows the tracker's cursor (or, with the tracker off,
    // the smoothed fix) and needs a clear step to change, along the board or
    // across the channel.
    if ( tracked ) {
        followTrack( );
    } else {
        reading.tracked = false;
        countRow( rowGridPlace( &grid, fix.pointer ), !reading.valid );
    }
    reading.valid = true;

    if ( out != nullptr ) {
        if ( calibrating( ) ) {
            watchForTap( out );
        } else if ( holdPurpose != ROWHOLD_NONE ) {
            collect( );
            if ( holdCount >= ROWCOUNT_HOLD_FIXES ) {
                finishHold( out );
            }
        }
        if ( active && !calibrating( ) && now - lastPrintMs >= ROWCOUNT_PRINT_PERIOD_MS ) {
            lastPrintMs = now;
            printReading( out );
        }
    }

    lastStatus = ServiceStatus::BUSY;
    return lastStatus;
}

// The counted row from a place on the breadboard, with hysteresis unless it
// is starting fresh.
void RowCounter::countRow( RowPlace place, bool fresh ) {
    if ( fresh || fabsf( place.along - countedAlong ) > 0.5f + ROWCOUNT_HYSTERESIS_ROWS ) {
        countedAlong = (int)floorf( place.along + 0.5f );
    }
    if ( fresh || fabsf( place.acrossMm ) > ROWCOUNT_HALF_HYSTERESIS_MM ) {
        countedBottom = place.acrossMm < 0.0f;
    }
    reading.row = rowGridRow( (float)countedAlong, countedBottom );
}

// The tracker's cursor in breadboard terms, and the counted row from it.
void RowCounter::followTrack( ) {
    const MagTrack& track = magLocator.track;
    bool fresh = !reading.tracked;
    reading.tracked = true;
    reading.trackPlace = rowGridPlace( &grid, track.cursor );
    Vec3 bar = { track.cursorSigmaMm * 0.7071f, track.cursorSigmaMm * 0.7071f, track.sigma.z };
    reading.trackSigmaRows = rowGridSigmaAlong( &grid, bar );
    reading.trackConfidence = rowGridConfidence( reading.trackPlace, reading.trackSigmaRows, rowGridSigmaAcross( &grid, bar ) );
    countRow( reading.trackPlace, fresh );
}

void RowCounter::printReading( Stream* out ) const {
    const MagProbeFix& fix = magLocator.fix;
    char line[ 280 ];
    char counted[ 8 ] = "off";
    if ( reading.row > 0 ) {
        snprintf( counted, sizeof( counted ), "%3d", reading.row );
    }
    float shown = reading.place.along + ( reading.place.acrossMm < 0.0f ? ROWGRID_ROWS_PER_HALF : 0 );
    char tracked[ 48 ] = "";
    if ( reading.tracked ) {
        float t = reading.trackPlace.along + ( reading.trackPlace.acrossMm < 0.0f ? ROWGRID_ROWS_PER_HALF : 0 );
        snprintf( tracked, sizeof( tracked ), "  track %6.2f +/-%.2f %3.0f %%", t, reading.trackSigmaRows, reading.trackConfidence * 100.0f );
    }
    snprintf( line, sizeof( line ), "row %s   this fix %6.2f (%+.2f) hole %d  +/-%.2f rows  %3.0f %% sure%s   x %.1f y %.1f z %.1f%s  misfit %.0f %%  seen by %d+%d faint",
              counted, shown, reading.offsetRows, reading.hole, reading.sigmaRows, reading.confidence * 100.0f, tracked,
              fix.rawPointer.x, fix.rawPointer.y, fix.rawTip.z, touchZ <= 0.0f ? "" : ( reading.touching ? " down" : " up" ), fix.misfit * 100.0f, fix.seenBy, fix.faintBy );
    out->println( line );
}
