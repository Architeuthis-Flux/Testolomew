// SPDX-License-Identifier: MIT
#include "ProbeCursor.h"

#include <stdlib.h>
#include <string.h>

#include "Commands.h"        // requestLedShow
#include "GraphicOverlays.h" // graphicOverlayState

ProbeCursor& probeCursor = ProbeCursor::getInstance( );

ProbeCursor& ProbeCursor::getInstance( ) {
    static ProbeCursor instance;
    return instance;
}

void ProbeCursor::begin( Stream* stream ) {
    link = stream;
    ledLayoutV5( &layout );
    probeLedDefaultStyle( &style );
    probeLedClear( &frame, layout.count );
    lineLength = 0;
}

// Gather characters into lines; each complete line is parsed.
void ProbeCursor::readLink( ) {
    while ( link->available( ) ) {
        char c = (char)link->read( );
        if ( c == '\n' || c == '\r' ) {
            if ( lineLength > 0 ) {
                line[ lineLength ] = 0;
                if ( parseLine( line ) ) {
                    linesParsed++;
                    lastLineMs = millis( );
                } else if ( strncmp( line, "cursor,", 7 ) == 0 ) {
                    linesBad++;
                }
            }
            lineLength = 0;
        } else if ( lineLength < PROBECURSOR_LINE_MAX - 1 ) {
            line[ lineLength++ ] = c;
        } else {
            lineLength = 0; // too long: not ours
        }
    }
}

// cursor,state,along,across,sigmaRows,sigmaAcross,confidence,height,haveUnder,underAlong,underAcross
bool ProbeCursor::parseLine( const char* text ) {
    if ( strncmp( text, "cursor,", 7 ) != 0 ) {
        return false;
    }
    float v[ 10 ];
    const char* p = text + 7;
    for ( int i = 0; i < 10; i++ ) {
        char* end;
        v[ i ] = strtof( p, &end );
        if ( end == p ) {
            return false;
        }
        p = end;
        if ( i < 9 ) {
            if ( *p != ',' )
                return false;
            p++;
        }
    }
    // Only sane numbers: finite, and within what a breadboard can hold (a NaN
    // would poison every LED's smoothed level for good).
    for ( int i = 0; i < 10; i++ ) {
        if ( v[ i ] != v[ i ] || v[ i ] > 1e4f || v[ i ] < -1e4f ) {
            return false;
        }
    }
    int state = (int)v[ 0 ];
    if ( state < 0 || state > PROBELED_TRACKING ) {
        return false;
    }
    if ( v[ 3 ] < 0.0f || v[ 4 ] < 0.0f ) {
        return false;
    }
    input.state = (ProbeLedState)state;
    input.along = v[ 1 ];
    input.acrossMm = v[ 2 ];
    input.sigmaRows = v[ 3 ];
    input.sigmaAcrossMm = v[ 4 ];
    input.confidence = v[ 5 ];
    input.heightMm = v[ 6 ];
    input.haveUnder = v[ 7 ] != 0.0f;
    input.underAlong = v[ 8 ];
    input.underAcrossMm = v[ 9 ];
    return true;
}

// The frame into the overlay: overlay row 1..5 = the top half's holes from
// the outer edge in (hole 5 .. hole 1), 6..10 = the bottom half's from the
// channel out (hole 1 .. hole 5); column = the breadboard row 1..30. Dark
// LEDs are 0 = transparent, so the nets show through.
void ProbeCursor::paint( ) {
    static uint32_t colors[ MAX_OVERLAY_PIXELS ];
    bool anything = false;
    for ( int k = 0; k < MAX_OVERLAY_PIXELS; k++ ) {
        colors[ k ] = 0;
    }
    for ( int i = 0; i < layout.count; i++ ) {
        if ( layout.kind[ i ] != PROBELED_HOLE ) {
            continue; // the rails are not in the overlay
        }
        uint8_t r, g, b;
        probeLedRgb( &frame, i, &r, &g, &b );
        if ( ( r | g | b ) == 0 ) {
            continue;
        }
        int row = layout.row[ i ];
        int hole = layout.hole[ i ];
        int overlayRow = row <= PROBELED_ROWS ? 6 - hole : 5 + hole;
        int col = row <= PROBELED_ROWS ? row : row - PROBELED_ROWS;
        colors[ ( overlayRow - 1 ) * MAX_OVERLAY_WIDTH + ( col - 1 ) ] = ( (uint32_t)r << 16 ) | ( (uint32_t)g << 8 ) | b;
        anything = true;
    }
    if ( anything ) {
        // addOverlay updates an overlay of the same name in place (it copies
        // the colours), so there is nothing to remove first.
        graphicOverlayState.addOverlay( PROBECURSOR_OVERLAY, 1, 1, MAX_OVERLAY_WIDTH, MAX_OVERLAY_HEIGHT, colors );
        requestLedShow( -1 ); // clear-first, as every overlay poster does (a cell that went dark is otherwise never cleared)
        overlayShown = true;
    } else if ( overlayShown ) {
        graphicOverlayState.removeOverlay( PROBECURSOR_OVERLAY );
        requestLedShow( -1 );
        overlayShown = false;
    }
}

ServiceStatus ProbeCursor::service( ) {
    if ( link == nullptr ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    readLink( );

    uint32_t now = micros( );
    float dtS = lastRenderUs == 0 ? 0.04f : ( now - lastRenderUs ) * 1e-6f;
    lastRenderUs = now;

    ProbeLedInput in = input;
    if ( millis( ) - lastLineMs > PROBECURSOR_STALE_MS ) {
        in.state = PROBELED_NONE; // the array board went quiet: fade out
    }
    probeLedRender( &layout, &in, &style, dtS, &frame );
    paint( );

    lastStatus = ServiceStatus::BUSY;
    return lastStatus;
}
