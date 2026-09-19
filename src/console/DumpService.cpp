// SPDX-License-Identifier: MIT
#include "DumpService.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Console.h"
#include "config.h"
#if MODULE_MAG_VIEW
#include "Display.h"
#include "ST7789.h"
#endif
#if MODULE_PROBE_LEDS
#include "ProbeLedService.h"
#endif
#if MODULE_UI
#include "UiStream.h"
#endif
#if __has_include( <ch32h4_spi.h> )
#include "SerialDma.h"
#define DUMP_SERIAL_DMA 1
#else
#define DUMP_SERIAL_DMA 0
#endif

DumpService& dump = DumpService::getInstance( );

DumpService& DumpService::getInstance( ) {
    static DumpService instance;
    return instance;
}

// Room in the console's transmit ring (everything, on the host).
static size_t txRoom( ) {
#if DUMP_SERIAL_DMA
    size_t pending = serialDmaPending( );
    return pending >= SERIALDMA_RING ? 0 : SERIALDMA_RING - pending;
#else
    return 1u << 30;
#endif
}

static void logOff( bool* was ) {
#if MODULE_UI
    *was = uiStream.logToScreen;
    uiStream.logToScreen = false;
#else
    *was = true;
#endif
}

static void logRestore( bool was ) {
#if MODULE_UI
    uiStream.logToScreen = was;
#else
    (void)was;
#endif
}

// ---- colours to characters ------------------------------------------------------

// A hue letter for a colour: r y g c b m, upper case when bright, w/W for
// white and greys, . for dark.
static char hueLetter( int r, int g, int b ) {
    int hi = r > g ? ( r > b ? r : b ) : ( g > b ? g : b );
    int lo = r < g ? ( r < b ? r : b ) : ( g < b ? g : b );
    if ( hi < 12 )
        return '.';
    char c;
    if ( hi - lo < hi / 3 ) {
        c = 'w';
    } else if ( r == hi ) {
        c = g > b ? ( g > r / 2 ? 'y' : 'r' ) : ( b > r / 2 ? 'm' : 'r' );
    } else if ( g == hi ) {
        c = r > b ? ( r > g / 2 ? 'y' : 'g' ) : ( b > g / 2 ? 'c' : 'g' );
    } else {
        c = r > g ? ( r > b / 2 ? 'm' : 'b' ) : ( g > b / 2 ? 'c' : 'b' );
    }
    return hi >= 128 ? (char)( c - 'a' + 'A' ) : c;
}

static const char* const ramp = " .:-=+*#%@"; // ten levels, dark to bright

// ---- the screen -----------------------------------------------------------------

#if MODULE_MAG_VIEW
static uint16_t rowBuffer[ 4 ][ LCD_WIDTH ]; // the cell's rows, native RGB565
#endif

int DumpService::asciiLine( int y ) {
#if MODULE_MAG_VIEW
    const int cw = LCD_WIDTH / DUMP_ASCII_W, ch = LCD_HEIGHT / DUMP_ASCII_H; // 3 x 4
    for ( int k = 0; k < ch; k++ ) {
        display.copyShownRow( y * ch + k, rowBuffer[ k ] );
    }
    int n = 0;
    for ( int x = 0; x < DUMP_ASCII_W; x++ ) {
        // The cell's 5/6/5-bit sums, expanded to 8 bits once (a division per
        // channel per cell, not per pixel: the tick has to stay short).
        int r5 = 0, g6 = 0, b5 = 0;
        for ( int k = 0; k < ch; k++ ) {
            const uint16_t* px = rowBuffer[ k ] + x * cw;
            for ( int j = 0; j < cw; j++ ) {
                uint16_t c = px[ j ];
                r5 += c >> 11;
                g6 += ( c >> 5 ) & 63;
                b5 += c & 31;
            }
        }
        int r = r5 * 255 / ( 31 * cw * ch );
        int g = g6 * 255 / ( 63 * cw * ch );
        int b = b5 * 255 / ( 31 * cw * ch );
        int lum = ( 299 * r + 587 * g + 114 * b ) / 1000;
        char out;
        if ( colour ) {
            int hi = r > g ? ( r > b ? r : b ) : ( g > b ? g : b );
            int lo = r < g ? ( r < b ? r : b ) : ( g < b ? g : b );
            out = hi - lo > 48 ? hueLetter( r, g, b ) : ramp[ lum * 9 / 255 ];
        } else {
            out = ramp[ lum * 9 / 255 ];
        }
        line[ n++ ] = out;
    }
    line[ n ] = '\0';
    return n;
#else
    (void)y;
    return -1;
#endif
}

int DumpService::rleLine( int y ) {
#if MODULE_MAG_VIEW
    display.copyShownRow( y, rowBuffer[ 0 ] );
    int n = snprintf( line, sizeof( line ), "%d:", y );
    static const char* const hex = "0123456789abcdef";
    int x = 0;
    while ( x < LCD_WIDTH ) {
        uint16_t c = rowBuffer[ 0 ][ x ];
        int run = 0;
        while ( x < LCD_WIDTH && rowBuffer[ 0 ][ x ] == c ) {
            run++;
            x += step;
        }
        // "<run>x<hex4>," by hand: a text-heavy row is 200 runs, and snprintf
        // for each was most of the tick.
        if ( n >= (int)sizeof( line ) - 12 ) {
            break;
        }
        if ( run >= 100 )
            line[ n++ ] = (char)( '0' + run / 100 );
        if ( run >= 10 )
            line[ n++ ] = (char)( '0' + ( run / 10 ) % 10 );
        line[ n++ ] = (char)( '0' + run % 10 );
        line[ n++ ] = 'x';
        line[ n++ ] = hex[ ( c >> 12 ) & 15 ];
        line[ n++ ] = hex[ ( c >> 8 ) & 15 ];
        line[ n++ ] = hex[ ( c >> 4 ) & 15 ];
        line[ n++ ] = hex[ c & 15 ];
        if ( x < LCD_WIDTH ) {
            line[ n++ ] = ',';
        }
    }
    line[ n ] = '\0';
    return n;
#else
    (void)y;
    return -1;
#endif
}

static const char* const b64chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int DumpService::b64Line( int y ) {
#if MODULE_MAG_VIEW
    display.copyShownRow( y, rowBuffer[ 0 ] );
    int n = snprintf( line, sizeof( line ), "%d:", y );
    // Little-endian bytes of every step-th pixel, three bytes to four characters.
    uint8_t bytes[ LCD_WIDTH * 2 ];
    int count = 0;
    for ( int x = 0; x < LCD_WIDTH; x += step ) {
        bytes[ count++ ] = (uint8_t)( rowBuffer[ 0 ][ x ] & 0xff );
        bytes[ count++ ] = (uint8_t)( rowBuffer[ 0 ][ x ] >> 8 );
    }
    for ( int i = 0; i < count; i += 3 ) {
        uint32_t v = (uint32_t)bytes[ i ] << 16;
        if ( i + 1 < count )
            v |= (uint32_t)bytes[ i + 1 ] << 8;
        if ( i + 2 < count )
            v |= bytes[ i + 2 ];
        line[ n++ ] = b64chars[ ( v >> 18 ) & 63 ];
        line[ n++ ] = b64chars[ ( v >> 12 ) & 63 ];
        line[ n++ ] = i + 1 < count ? b64chars[ ( v >> 6 ) & 63 ] : '=';
        line[ n++ ] = i + 2 < count ? b64chars[ v & 63 ] : '=';
    }
    line[ n ] = '\0';
    return n;
#else
    (void)y;
    return -1;
#endif
}

// ---- the LEDs -------------------------------------------------------------------

#define DUMP_LEDS_PER_LINE 20

int DumpService::ledsHexLine( int n ) {
#if MODULE_PROBE_LEDS
    int first = n * DUMP_LEDS_PER_LINE;
    if ( first >= probeLeds.layout.count )
        return -1;
    int len = snprintf( line, sizeof( line ), "%d:", first );
    for ( int i = first; i < first + DUMP_LEDS_PER_LINE && i < probeLeds.layout.count; i++ ) {
        uint8_t r, g, b;
        probeLedRgb( &probeLeds.frame, i, &r, &g, &b );
        len += snprintf( line + len, sizeof( line ) - len, "%s%02x%02x%02x", i == first ? "" : " ", r, g, b );
    }
    return len;
#else
    (void)n;
    return -1;
#endif
}

int DumpService::ledsMapLine( int n ) {
#if MODULE_PROBE_LEDS
    const LedLayout& layout = probeLeds.layout;
    if ( n >= layout.count )
        return -1;
    return snprintf( line, sizeof( line ), "%d %.2f %.2f %s %d %d", n, layout.along[ n ], layout.acrossMm[ n ], layout.kind[ n ] == PROBELED_RAIL ? "rail" : "hole", layout.row[ n ], layout.hole[ n ] );
#else
    (void)n;
    return -1;
#endif
}

// The grid: printed at once (a kilobyte), columns are rows 1-30 along the
// board, lines the hole/rail positions across it, top half first.
#if MODULE_PROBE_LEDS
static void printLedGrid( Stream* out ) {
    const LedLayout& layout = probeLeds.layout;
    const ProbeLedFrame& frame = probeLeds.frame;
    // The distinct across positions, sorted from + (rows 1-30) to -.
    float lines[ 24 ];
    int lineCount = 0;
    for ( int i = 0; i < layout.count && lineCount < 24; i++ ) {
        bool seen = false;
        for ( int k = 0; k < lineCount; k++ ) {
            if ( fabsf( lines[ k ] - layout.acrossMm[ i ] ) < 0.3f )
                seen = true;
        }
        if ( !seen )
            lines[ lineCount++ ] = layout.acrossMm[ i ];
    }
    for ( int a = 0; a < lineCount; a++ ) {
        for ( int b = a + 1; b < lineCount; b++ ) {
            if ( lines[ b ] > lines[ a ] ) {
                float t = lines[ a ];
                lines[ a ] = lines[ b ];
                lines[ b ] = t;
            }
        }
    }
    char text[ 80 ];
    const ProbeLedInput& in = probeLeds.input;
    static const char* const names[ 4 ] = { "none", "rough", "coasting", "tracking" };
    out->println( "leds{" );
    if ( in.state == PROBELED_NONE ) {
        snprintf( text, sizeof( text ), "layout %s %d LEDs cursor: none", probeLeds.v5 ? "V5" : "V6", layout.count );
    } else {
        int rowNumber = (int)floorf( in.along + 0.5f ) + ( in.acrossMm < 0.0f ? PROBELED_ROWS : 0 );
        snprintf( text, sizeof( text ), "layout %s %d LEDs cursor: %s row %d along %.2f across %.1f +-%.2f rows %.0f%% up %.1f mm%s", probeLeds.v5 ? "V5" : "V6", layout.count, names[ in.state ],
                  rowNumber, in.along, in.acrossMm, in.sigmaRows, in.confidence * 100.0f, in.heightMm, in.haveUnder ? " (pointed)" : "" );
    }
    out->println( text );
    if ( probeLeds.brush.active ) {
        snprintf( text, sizeof( text ), "brush: radius %d level %.2f colour %02x%02x%02x", probeLeds.brush.radiusRows, probeLeds.brush.level, probeLeds.brush.r, probeLeds.brush.g, probeLeds.brush.b );
        out->println( text );
    }
    out->println( "        1    5    10   15   20   25   30" );
    for ( int k = 0; k < lineCount; k++ ) {
        char cells[ PROBELED_ROWS + 1 ];
        for ( int c = 0; c < PROBELED_ROWS; c++ )
            cells[ c ] = ' ';
        cells[ PROBELED_ROWS ] = '\0';
        bool rail = false;
        for ( int i = 0; i < layout.count; i++ ) {
            if ( fabsf( layout.acrossMm[ i ] - lines[ k ] ) >= 0.3f )
                continue;
            int column = (int)( layout.along[ i ] + 0.5f ) - 1;
            if ( column < 0 || column >= PROBELED_ROWS )
                continue;
            rail = rail || layout.kind[ i ] == PROBELED_RAIL;
            uint8_t r, g, b;
            probeLedRgb( &frame, i, &r, &g, &b );
            cells[ column ] = hueLetter( r, g, b );
        }
        snprintf( text, sizeof( text ), "%+6.1f%c %s", lines[ k ], rail ? 'r' : ' ', cells );
        out->println( text );
    }
    out->println( "}" );
}
#endif

// ---- the service ----------------------------------------------------------------

bool DumpService::start( DumpKind what, int stepArg, bool colourArg, Stream* io ) {
    if ( kind != DUMP_NONE || io == nullptr ) {
        return false;
    }
    kind = what;
    step = stepArg < 1 ? 1 : ( stepArg > 8 ? 8 : stepArg );
    colour = colourArg;
    out = io;
    row = 0;
    startedMs = millis( );
    buildMaxUs = printMaxUs = 0;
    logOff( &wasLogging );
    char head[ 80 ];
    switch ( kind ) {
    case DUMP_ASCII:
        out->println( "screen:ascii{" );
        snprintf( head, sizeof( head ), "w %d h %d cell %dx%d %s", DUMP_ASCII_W, DUMP_ASCII_H, LCD_WIDTH / DUMP_ASCII_W, LCD_HEIGHT / DUMP_ASCII_H, colour ? "colour" : "grey" );
        out->println( head );
        break;
    case DUMP_RLE:
    case DUMP_B64:
        out->println( "screen:dump{" );
        snprintf( head, sizeof( head ), "w %d h %d step %d %s", LCD_WIDTH, LCD_HEIGHT, step, kind == DUMP_RLE ? "rle" : "b64" );
        out->println( head );
        break;
    case DUMP_LEDS_HEX:
        out->println( "leds:hex{" );
#if MODULE_PROBE_LEDS
        snprintf( head, sizeof( head ), "layout %s %d LEDs, 20 a line from the index given, gamma applied", probeLeds.v5 ? "V5" : "V6", probeLeds.layout.count );
        out->println( head );
#endif
        break;
    case DUMP_LEDS_MAP:
        out->println( "leds:map{" );
#if MODULE_PROBE_LEDS
        snprintf( head, sizeof( head ), "layout %s %d LEDs: index along across kind row hole", probeLeds.v5 ? "V5" : "V6", probeLeds.layout.count );
        out->println( head );
#endif
        break;
    default:
        break;
    }
#if MODULE_MAG_VIEW
    if ( kind == DUMP_ASCII || kind == DUMP_RLE || kind == DUMP_B64 ) {
        display.hold = true;
    }
#endif
    return true;
}

void DumpService::finish( ) {
    if ( kind == DUMP_NONE ) {
        return;
    }
#if MODULE_MAG_VIEW
    display.hold = false;
#endif
    out->println( "}" );
    logRestore( wasLogging );
    kind = DUMP_NONE;
    dumps++;
    if ( buildMaxUs > buildMaxEverUs )
        buildMaxEverUs = buildMaxUs;
    if ( printMaxUs > printMaxEverUs )
        printMaxEverUs = printMaxUs;
}

int DumpService::nextLine( ) {
    switch ( kind ) {
    case DUMP_ASCII:
        return row < DUMP_ASCII_H ? asciiLine( row ) : -1;
    case DUMP_RLE:
        return row < LCD_HEIGHT / step ? rleLine( row * step ) : -1;
    case DUMP_B64:
        return row < LCD_HEIGHT / step ? b64Line( row * step ) : -1;
    case DUMP_LEDS_HEX:
        return ledsHexLine( row );
    case DUMP_LEDS_MAP:
        return ledsMapLine( row );
    default:
        return -1;
    }
}

ServiceStatus DumpService::service( ) {
    if ( kind == DUMP_NONE ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
#if MODULE_MAG_VIEW
    if ( ( kind == DUMP_ASCII || kind == DUMP_RLE || kind == DUMP_B64 ) && !display.frozen( ) && millis( ) - startedMs < DUMP_FREEZE_WAIT_MS ) {
        lastStatus = ServiceStatus::IDLE; // the frame on its way to the panel finishes first
        return lastStatus;
    }
#endif
    // One row, if the ring has room for it and some to spare.
    uint32_t t0 = micros( );
    int len = nextLine( );
    uint32_t t1 = micros( );
    if ( t1 - t0 > buildMaxUs )
        buildMaxUs = t1 - t0;
    if ( len < 0 ) {
        finish( );
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    if ( txRoom( ) < (size_t)len + 2 + DUMP_RING_RESERVE ) {
        lastStatus = ServiceStatus::IDLE; // not yet: the wire is behind
        return lastStatus;
    }
    out->println( line );
    uint32_t t2 = micros( );
    if ( t2 - t1 > printMaxUs )
        printMaxUs = t2 - t1;
    row++;
    lastStatus = ServiceStatus::BUSY;
    return lastStatus;
}

// ---- verbs ----------------------------------------------------------------------

static void onScreenAscii( int argc, char** argv, Stream* out ) {
    bool colour = argc >= 2 && strcmp( argv[ 1 ], "colour" ) == 0;
    if ( !dump.start( DUMP_ASCII, 1, colour, out ) ) {
        consoleErr( out, "a dump is already running" );
    }
}

static void onScreenDump( int argc, char** argv, Stream* out ) {
    DumpKind kind = DUMP_RLE;
    int step = 1;
    for ( int i = 1; i < argc; i++ ) {
        if ( strcmp( argv[ i ], "rle" ) == 0 ) {
            kind = DUMP_RLE;
        } else if ( strcmp( argv[ i ], "b64" ) == 0 ) {
            kind = DUMP_B64;
        } else if ( argv[ i ][ 0 ] >= '1' && argv[ i ][ 0 ] <= '9' ) {
            step = atoi( argv[ i ] );
        } else {
            consoleErr( out, "usage: :screen:dump [rle|b64] [step 1-8]" );
            return;
        }
    }
    if ( !dump.start( kind, step, false, out ) ) {
        consoleErr( out, "a dump is already running" );
    }
}

static void onLeds( int argc, char** argv, Stream* out ) {
    (void)argc;
    (void)argv;
#if MODULE_PROBE_LEDS
    bool was;
    logOff( &was );
    printLedGrid( out );
    logRestore( was );
#else
    consoleErr( out, "no LEDs in this build" );
#endif
}

static void onLedsHex( int argc, char** argv, Stream* out ) {
    (void)argc;
    (void)argv;
    if ( !dump.start( DUMP_LEDS_HEX, 1, false, out ) ) {
        consoleErr( out, "a dump is already running" );
    }
}

static void onLedsMap( int argc, char** argv, Stream* out ) {
    (void)argc;
    (void)argv;
    if ( !dump.start( DUMP_LEDS_MAP, 1, false, out ) ) {
        consoleErr( out, "a dump is already running" );
    }
}

static void onStats( int argc, char** argv, Stream* out ) {
    (void)argc;
    (void)argv;
    bool was;
    logOff( &was );
    out->println( "stats{" );
    jOS.printStats( out );
    char line[ 200 ];
#if MODULE_MAG_VIEW
    display.printStats( out, display.slotNames, display.slotCount );
#endif
#if MODULE_PROBE_LEDS
    if ( probeLeds.strip ) {
        char report[ 160 ];
        ledStripReport( &probeLeds.chain, report, sizeof( report ) );
        snprintf( line, sizeof( line ), "strip: %s; last %.0f mA of %.0f budget, %lu frames scaled", report, probeLeds.stripLastMa, probeLeds.stripBudgetMa( ), (unsigned long)probeLeds.stripScaledFrames );
    } else {
        snprintf( line, sizeof( line ), "strip: off (%s layout, %d LEDs rendered)", probeLeds.v5 ? "V5" : "V6", probeLeds.layout.count );
    }
    out->println( line );
#endif
    snprintf( line, sizeof( line ), "dump: %lu done, max %lu us a tick; the last dump's row build %lu us, print %lu us (since boot %lu / %lu)", (unsigned long)dump.dumps, (unsigned long)dump.maxUs,
              (unsigned long)dump.buildMaxUs, (unsigned long)dump.printMaxUs, (unsigned long)dump.buildMaxEverUs, (unsigned long)dump.printMaxEverUs );
    out->println( line );
    out->println( "}" );
    logRestore( was );
}

void DumpService::begin( ) {
    consoleAddVerb( "stats", "", "service table, display draw times, LED strip statistics", CONSOLE_READS, onStats );
#if MODULE_MAG_VIEW
    consoleAddVerb( "screen:ascii", "[colour]", "the frame as 80x60 characters (a grey ramp, or hue letters)", CONSOLE_READS, onScreenAscii );
    consoleAddVerb( "screen:dump", "[rle|b64] [step]", "every row of the frame as RGB565 (tools/screendump.py makes a PNG)", CONSOLE_READS, onScreenDump );
#endif
#if MODULE_PROBE_LEDS
    consoleAddVerb( "leds", "", "the breadboard's LEDs as a grid of hue letters, with the cursor", CONSOLE_READS, onLeds );
    consoleAddVerb( "leds:hex", "", "every LED's colour as sent, 20 a line", CONSOLE_READS, onLedsHex );
    consoleAddVerb( "leds:map", "", "the LED layout: index, along, across, kind, row, hole", CONSOLE_READS, onLedsMap );
#endif
}
