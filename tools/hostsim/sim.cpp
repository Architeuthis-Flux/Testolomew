// SPDX-License-Identifier: MIT
// The whole firmware on the host, driven by a script.
//
// main.cpp's setup() runs as on the board (against the stubs in this
// folder), then jOS.serviceAll() is called pass after pass with the clock
// (simMicros) advancing SIM_PASS_US a pass, so every service runs at its
// own period as it does on the chip. A WorldService writes the dipole field
// of a simulated magnet into the array's frames. The script is typed on the
// console - letters, keys, and the same :verbs the board answers - with a
// few @directives for the world and for checks:
//
//   :screen                    any verb: typed with Enter, and the run goes on
//                              until its frame closes (or 5 s)
//   e  \t  \e[A  `  /          typed keys (escapes \e \r \n \t), then a moment
//   @run <ms>                  let the loop run
//   @magnet <x> <y> <z> <sx> <sy> <sz>     the magnet's centre and its axis
//   @magnet row <r> <h> [up mm] [lean deg] the point in that hole (leaning +x)
//   @magnet off
//   @move <dx> <dy> <dz> <ms>  glide the magnet by this much over this long
//   @dropout <ms>              no readings for this long
//   @dead <i> <ms>             sensor i not read for this long (the others go on)
//   @bias <i> <x> <y> <z>      a zero error at sensor i: this much (mT, board frame) added to every reading
//   @gain <i> <factor>         sensor i reads this much of the true field (a gain error)
//   @tip <mm>                  the probe's magnet sits this far up the shaft from its point (the world's truth; -1 = as the locator believes)
//   @surface <mm>              where the board's top really is (the @magnet row form rests the point on it; the locator's S is its belief)
//   @strip on|off              pretend a real LED chain is wired (the LED-first rule)
//   @type "text" [n] [gap ms]  type text n times with a gap
//   @expect "text" [ms]        the text must appear in what was printed since
//                              the last script line (waiting up to ms for it)
//   @seed <n>                  the noise
//   @echo <text>
//   @quit
//
// Two verbs exist only here: :screen:png <file> writes the panel as a PNG,
// :screen:verify checks that the dump path (MagView::copyShownRow, what
// :screen:dump sends) gives exactly what the panel received.
//
// Exit code 1 if any @expect failed. `./sim script [-q]`; -q keeps the
// console's output off stdout (the expect lines still print).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

#include "Console.h"
#include "Display.h"
#include "DumpService.h"
#include "EEPROM.h"
#include "Input.h"
#include "JumperlOS.h"
#include "MagArray.h"
#include "MagFit.h"
#include "MagLocator.h"
#include "Apps.h"
#include "Menu.h"
#include "Play.h"
#include "ProbeLedService.h"
#include "RowCounter.h"
#include "RowGrid.h"
#include "ST7789.h"
#include "Settings.h"
#include "Ui.h"
#include "UiStream.h"
#include "config.h"

#define SIM_PASS_US 250
#define SIM_VERB_TIMEOUT_MS 5000

void setup( ); // main.cpp's
extern uint16_t simPanel[ LCD_WIDTH * LCD_HEIGHT ];

static std::string outText; // everything the firmware printed
static size_t mark = 0;     // where the current script line's output starts
static bool quiet = false;
static int expects = 0, failures = 0;
static bool verifyPending = false;

static void onSerialWrite( uint8_t c ) {
    outText.push_back( (char)c );
    if ( !quiet )
        putchar( c );
}

// ---- the world ----------------------------------------------------------------------

static float gauss( float s ) {
    float u1 = ( rand( ) + 1.0f ) / ( (float)RAND_MAX + 2.0f ), u2 = ( rand( ) + 1.0f ) / ( (float)RAND_MAX + 2.0f );
    return s * sqrtf( -2 * logf( u1 ) ) * cosf( 2 * (float)M_PI * u2 );
}

class WorldService : public Service {
  public:
    bool on = false;
    Vec3 magnet = { 0, 0, 0 };
    Vec3 shaft = { 0, 0, 1 };
    float strength = 4200.0f;
    float surfaceZ = MAGLOC_BOARD_Z_MM; // @surface: where the board's top really is (the locator's setting is its belief)
    float tipMm = -1.0f; // @tip: the magnet's centre this far up the shaft from the point (-1 = whatever the locator believes)
    float noise = 0.010f; // a TMAG5273's per-axis noise a frame at the reference (0.011 mT): 0.012 on the bench (2026-09-21), the MMC's 0.0003 by its ratio
    uint64_t dropoutUntilUs = 0;
    uint64_t deadUntilUs[ MAG_SENSOR_COUNT ] = { }; // @dead: one sensor not read until then
    Vec3 bias[ MAG_SENSOR_COUNT ] = { }; // @bias: a zero error at a sensor, added to every frame
    float gain[ MAG_SENSOR_COUNT ]; // @gain: a sensor reads this much of the true field
    WorldService( ) {
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ )
            gain[ i ] = 1.0f;
    }
    // A glide under way.
    bool moving = false;
    Vec3 moveFrom, moveTo;
    uint64_t moveStartUs = 0, moveEndUs = 0;

    ServiceStatus service( ) override {
        if ( moving ) {
            float f = moveEndUs > moveStartUs ? (float)( simMicros - moveStartUs ) / (float)( moveEndUs - moveStartUs ) : 1.0f;
            if ( f >= 1.0f ) {
                f = 1.0f;
                moving = false;
            }
            magnet = { moveFrom.x + f * ( moveTo.x - moveFrom.x ), moveFrom.y + f * ( moveTo.y - moveFrom.y ), moveFrom.z + f * ( moveTo.z - moveFrom.z ) };
        }
        bool dropout = simMicros < dropoutUntilUs;
        Vec3 moment = { strength * shaft.x, strength * shaft.y, strength * shaft.z };
        for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
            Vec3 b = on ? magFitDipoleField( magArray.position[ i ], magnet, moment ) : Vec3{ 0, 0, 0 };
            // Each sensor's noise scaled by its type's (an MMC56x3 is 30-50x quieter than a TMAG5273).
            float n = noise * magArray.noiseMt[ i ] / MAG_WEIGHT_REFERENCE_MT;
            // As the array does it: the reading into raw[], the field less the
            // baseline - so the zero's drift rules (MagArray::driftBaseline)
            // act here as on the board.
            magArray.raw[ i ] = { gain[ i ] * b.x + bias[ i ].x + gauss( n ), gain[ i ] * b.y + bias[ i ].y + gauss( n ), gain[ i ] * b.z + bias[ i ].z + gauss( n ) };
            Vec3 zero = magArray.baselineOf( i );
            magArray.field[ i ] = { magArray.raw[ i ].x - zero.x, magArray.raw[ i ].y - zero.y, magArray.raw[ i ].z - zero.z };
            magArray.fresh[ i ] = !dropout && simMicros >= deadUntilUs[ i ];
            magArray.saturated[ i ] = false;
        }
        magArray.frameCount++;
        return ServiceStatus::BUSY;
    }
    const char* getName( ) const override { return "World"; }
    ServicePriority getPriority( ) const override { return ServicePriority::HIGH; }
    uint32_t periodUs( ) const override { return MAG_FRAME_PERIOD_US; }
};

static WorldService world;

// ---- PNG ----------------------------------------------------------------------------

static uint32_t crcTable[ 256 ];

static void crcInit( ) {
    for ( uint32_t n = 0; n < 256; n++ ) {
        uint32_t c = n;
        for ( int k = 0; k < 8; k++ )
            c = ( c & 1 ) ? 0xedb88320u ^ ( c >> 1 ) : c >> 1;
        crcTable[ n ] = c;
    }
}

static uint32_t crc( const uint8_t* data, size_t n, uint32_t c = 0xffffffffu ) {
    for ( size_t i = 0; i < n; i++ )
        c = crcTable[ ( c ^ data[ i ] ) & 0xff ] ^ ( c >> 8 );
    return c;
}

static void put32( std::vector<uint8_t>& v, uint32_t x ) {
    v.push_back( (uint8_t)( x >> 24 ) );
    v.push_back( (uint8_t)( x >> 16 ) );
    v.push_back( (uint8_t)( x >> 8 ) );
    v.push_back( (uint8_t)x );
}

static void chunk( FILE* f, const char* type, const std::vector<uint8_t>& data ) {
    std::vector<uint8_t> v;
    put32( v, (uint32_t)data.size( ) );
    std::vector<uint8_t> body( type, type + 4 );
    body.insert( body.end( ), data.begin( ), data.end( ) );
    uint32_t c = crc( body.data( ), body.size( ) ) ^ 0xffffffffu;
    fwrite( v.data( ), 1, 4, f );
    fwrite( body.data( ), 1, body.size( ), f );
    v.clear( );
    put32( v, c );
    fwrite( v.data( ), 1, 4, f );
}

// An 8-bit RGB PNG with filter 0 on every row, the deflate stream as stored
// (uncompressed) blocks: no zlib needed, and tools/rgb565png.py reads it.
static bool writePng( const char* path, int w, int h, const uint16_t* rgb565 ) {
    FILE* f = fopen( path, "wb" );
    if ( f == nullptr )
        return false;
    static const uint8_t sig[ 8 ] = { 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n' };
    fwrite( sig, 1, 8, f );
    std::vector<uint8_t> ihdr;
    put32( ihdr, (uint32_t)w );
    put32( ihdr, (uint32_t)h );
    ihdr.push_back( 8 );
    ihdr.push_back( 2 );
    ihdr.push_back( 0 );
    ihdr.push_back( 0 );
    ihdr.push_back( 0 );
    chunk( f, "IHDR", ihdr );
    std::vector<uint8_t> raw;
    for ( int y = 0; y < h; y++ ) {
        raw.push_back( 0 );
        for ( int x = 0; x < w; x++ ) {
            uint16_t c = rgb565[ y * w + x ];
            raw.push_back( (uint8_t)( ( ( c >> 11 ) & 31 ) * 255 / 31 ) );
            raw.push_back( (uint8_t)( ( ( c >> 5 ) & 63 ) * 255 / 63 ) );
            raw.push_back( (uint8_t)( ( c & 31 ) * 255 / 31 ) );
        }
    }
    std::vector<uint8_t> z;
    z.push_back( 0x78 );
    z.push_back( 0x01 );
    size_t pos = 0;
    uint32_t a = 1, b = 0;
    for ( size_t i = 0; i < raw.size( ); i++ ) {
        a = ( a + raw[ i ] ) % 65521;
        b = ( b + a ) % 65521;
    }
    while ( pos < raw.size( ) ) {
        size_t n = raw.size( ) - pos;
        if ( n > 65535 )
            n = 65535;
        bool last = pos + n == raw.size( );
        z.push_back( last ? 1 : 0 );
        z.push_back( (uint8_t)( n & 0xff ) );
        z.push_back( (uint8_t)( n >> 8 ) );
        z.push_back( (uint8_t)( ~n & 0xff ) );
        z.push_back( (uint8_t)( ( ~n >> 8 ) & 0xff ) );
        z.insert( z.end( ), raw.begin( ) + pos, raw.begin( ) + pos + n );
        pos += n;
    }
    put32( z, ( b << 16 ) | a );
    chunk( f, "IDAT", z );
    chunk( f, "IEND", std::vector<uint8_t>( ) );
    fclose( f );
    return true;
}

// ---- the sim-only verbs -----------------------------------------------------------

static void onScreenPng( int argc, char** argv, Stream* out ) {
    if ( argc < 2 ) {
        consoleErr( out, "usage: :screen:png <file>" );
        return;
    }
    if ( !writePng( argv[ 1 ], LCD_WIDTH, LCD_HEIGHT, simPanel ) ) {
        consoleErr( out, "could not write the file" );
        return;
    }
    char line[ 200 ];
    snprintf( line, sizeof( line ), "png %s", argv[ 1 ] );
    consoleOk( out, line );
}

static void onScreenVerify( int argc, char** argv, Stream* out ) {
    (void)argc;
    (void)argv;
    (void)out;
    verifyPending = true; // done by the runner, outside the console's tick
}

// ---- running -----------------------------------------------------------------------

static void pass( ) {
    jOS.serviceAll( );
    simMicros += SIM_PASS_US;
}

static void run( uint32_t ms ) {
    uint64_t until = simMicros + (uint64_t)ms * 1000u;
    while ( simMicros < until )
        pass( );
}

// The dump path against the panel: hold the display, wait for it to hold
// still, compare every row.
static void doVerify( ) {
    display.hold = true;
    uint64_t until = simMicros + 400000u;
    while ( !display.frozen( ) && simMicros < until )
        pass( );
    int bad = 0, firstRow = -1;
    uint16_t row[ LCD_WIDTH ];
    for ( int y = 0; y < LCD_HEIGHT; y++ ) {
        display.copyShownRow( y, row );
        for ( int x = 0; x < LCD_WIDTH; x++ ) {
            if ( row[ x ] != simPanel[ y * LCD_WIDTH + x ] ) {
                if ( firstRow < 0 )
                    firstRow = y;
                bad++;
            }
        }
    }
    display.hold = false;
    char line[ 120 ];
    if ( bad == 0 ) {
        snprintf( line, sizeof( line ), "verify: the dump path matches the panel, %d rows, pushed %d", LCD_HEIGHT, display.shownPushed ? 1 : 0 );
        consoleOk( &uiStream, line );
    } else {
        snprintf( line, sizeof( line ), "verify: %d pixels differ from row %d (pushed %d, frozen %d)", bad, firstRow, display.shownPushed ? 1 : 0, display.frozen( ) ? 1 : 0 );
        consoleErr( &uiStream, line );
    }
}

// Has a frame closed in the output since `from`? A line that is exactly
// "}", or that starts with ok{ or err{.
static bool frameClosed( size_t from ) {
    size_t p = from;
    while ( p < outText.size( ) ) {
        size_t e = outText.find( '\n', p );
        if ( e == std::string::npos )
            break;
        std::string l = outText.substr( p, e - p );
        while ( !l.empty( ) && ( l.back( ) == '\r' || l.back( ) == ' ' ) )
            l.pop_back( );
        if ( l == "}" || l.compare( 0, 3, "ok{" ) == 0 || l.compare( 0, 4, "err{" ) == 0 )
            return true;
        p = e + 1;
    }
    return false;
}

static void typeText( const std::string& text ) {
    Serial.type( text.c_str( ) );
}

// Escapes in typed text: \e \r \n \t \\ and \" .
static std::string unescape( const std::string& s ) {
    std::string r;
    for ( size_t i = 0; i < s.size( ); i++ ) {
        if ( s[ i ] == '\\' && i + 1 < s.size( ) ) {
            char c = s[ ++i ];
            r.push_back( c == 'e' ? 27 : c == 'r' ? '\r' : c == 'n' ? '\n' : c == 't' ? '\t' : c );
        } else {
            r.push_back( s[ i ] );
        }
    }
    return r;
}

static std::vector<std::string> words( const std::string& line ) {
    // Words, with "quoted strings" kept whole (quotes removed).
    std::vector<std::string> w;
    size_t i = 0;
    while ( i < line.size( ) ) {
        while ( i < line.size( ) && line[ i ] == ' ' )
            i++;
        if ( i >= line.size( ) )
            break;
        if ( line[ i ] == '"' ) {
            size_t e = line.find( '"', i + 1 );
            if ( e == std::string::npos )
                e = line.size( );
            w.push_back( line.substr( i + 1, e - i - 1 ) );
            i = e + 1;
        } else {
            size_t e = line.find( ' ', i );
            if ( e == std::string::npos )
                e = line.size( );
            w.push_back( line.substr( i, e - i ) );
            i = e;
        }
    }
    return w;
}

static void note( const char* fmt, const char* a ) {
    char line[ 300 ];
    snprintf( line, sizeof( line ), fmt, a );
    printf( "%s\n", line );
    fflush( stdout );
}

static bool directive( const std::vector<std::string>& w, int lineNo ) {
    const std::string& d = w[ 0 ];
    if ( d == "@run" && w.size( ) >= 2 ) {
        run( (uint32_t)atol( w[ 1 ].c_str( ) ) );
    } else if ( d == "@magnet" && w.size( ) >= 2 ) {
        if ( w[ 1 ] == "off" ) {
            world.on = false;
            world.moving = false;
        } else if ( w[ 1 ] == "row" && w.size( ) >= 4 ) {
            int row = atoi( w[ 2 ].c_str( ) ), hole = atoi( w[ 3 ].c_str( ) );
            float up = w.size( ) >= 5 ? atof( w[ 4 ].c_str( ) ) : 0.0f;
            float lean = ( w.size( ) >= 6 ? atof( w[ 5 ].c_str( ) ) : 0.0f ) * (float)M_PI / 180.0f;
            RowPlace place = rowGridHolePlace( row, hole );
            Vec3 tip = rowGridToBoard( &rowCounter.grid, place.along, place.acrossMm );
            tip.z = world.surfaceZ + up;
            world.shaft = { sinf( lean ), 0.0f, cosf( lean ) };
            { float tt = world.tipMm >= 0.0f ? world.tipMm : magLocator.tipOffsetMm; world.magnet = { tip.x + tt * world.shaft.x, tip.y + tt * world.shaft.y, tip.z + tt * world.shaft.z }; }
            world.on = true;
            world.moving = false;
        } else if ( w.size( ) >= 7 ) {
            world.magnet = { (float)atof( w[ 1 ].c_str( ) ), (float)atof( w[ 2 ].c_str( ) ), (float)atof( w[ 3 ].c_str( ) ) };
            Vec3 s = { (float)atof( w[ 4 ].c_str( ) ), (float)atof( w[ 5 ].c_str( ) ), (float)atof( w[ 6 ].c_str( ) ) };
            float n = sqrtf( s.x * s.x + s.y * s.y + s.z * s.z );
            world.shaft = n > 1e-6f ? Vec3{ s.x / n, s.y / n, s.z / n } : Vec3{ 0, 0, 1 };
            world.on = true;
            world.moving = false;
        } else {
            printf( "line %d: @magnet <x> <y> <z> <sx> <sy> <sz> | row <r> <h> [up] [lean] | off\n", lineNo );
            return false;
        }
    } else if ( d == "@move" && w.size( ) >= 5 ) {
        world.moveFrom = world.magnet;
        world.moveTo = { world.magnet.x + (float)atof( w[ 1 ].c_str( ) ), world.magnet.y + (float)atof( w[ 2 ].c_str( ) ), world.magnet.z + (float)atof( w[ 3 ].c_str( ) ) };
        uint32_t ms = (uint32_t)atol( w[ 4 ].c_str( ) );
        world.moveStartUs = simMicros;
        world.moveEndUs = simMicros + (uint64_t)ms * 1000u;
        world.moving = true;
        run( ms );
    } else if ( d == "@baselines" ) {
        for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
            Vec3 z = magArray.baselineOf( i );
            printf( "baseline %d: %.4f %.4f %.4f  field %.4f %.4f %.4f\n", i, z.x, z.y, z.z, magArray.field[ i ].x, magArray.field[ i ].y, magArray.field[ i ].z );
        }
    } else if ( d == "@dead" && w.size( ) >= 3 ) {
        int i = atoi( w[ 1 ].c_str( ) );
        if ( i >= 0 && i < MAG_SENSOR_COUNT )
            world.deadUntilUs[ i ] = simMicros + (uint64_t)atol( w[ 2 ].c_str( ) ) * 1000u;
    } else if ( d == "@surface" && w.size( ) >= 2 ) {
        world.surfaceZ = (float)atof( w[ 1 ].c_str( ) );
    } else if ( d == "@tip" && w.size( ) >= 2 ) {
        world.tipMm = (float)atof( w[ 1 ].c_str( ) );
    } else if ( d == "@gain" && w.size( ) >= 3 ) {
        int i = atoi( w[ 1 ].c_str( ) );
        if ( i >= 0 && i < MAG_SENSOR_COUNT )
            world.gain[ i ] = (float)atof( w[ 2 ].c_str( ) );
    } else if ( d == "@bias" && w.size( ) >= 5 ) {
        int i = atoi( w[ 1 ].c_str( ) );
        if ( i >= 0 && i < MAG_SENSOR_COUNT )
            world.bias[ i ] = { (float)atof( w[ 2 ].c_str( ) ), (float)atof( w[ 3 ].c_str( ) ), (float)atof( w[ 4 ].c_str( ) ) };
    } else if ( d == "@dropout" && w.size( ) >= 2 ) {
        world.dropoutUntilUs = simMicros + (uint64_t)atol( w[ 1 ].c_str( ) ) * 1000u;
    } else if ( d == "@strip" && w.size( ) >= 2 ) {
        probeLeds.strip = w[ 1 ] == "on";
    } else if ( d == "@seed" && w.size( ) >= 2 ) {
        srand( (unsigned)atol( w[ 1 ].c_str( ) ) );
    } else if ( d == "@type" && w.size( ) >= 2 ) {
        int n = w.size( ) >= 3 ? atoi( w[ 2 ].c_str( ) ) : 1;
        uint32_t gap = w.size( ) >= 4 ? (uint32_t)atol( w[ 3 ].c_str( ) ) : 200;
        std::string text = unescape( w[ 1 ] );
        for ( int k = 0; k < n; k++ ) {
            typeText( text );
            run( gap );
        }
    } else if ( d == "@expect" && w.size( ) >= 2 ) {
        expects++;
        uint32_t ms = w.size( ) >= 3 ? (uint32_t)atol( w[ 2 ].c_str( ) ) : 0;
        uint64_t until = simMicros + (uint64_t)ms * 1000u;
        bool found = outText.find( w[ 1 ], mark ) != std::string::npos;
        while ( !found && simMicros < until ) {
            run( 10 );
            found = outText.find( w[ 1 ], mark ) != std::string::npos;
        }
        if ( found ) {
            note( "expect ok: %s", w[ 1 ].c_str( ) );
        } else {
            failures++;
            char line[ 200 ];
            snprintf( line, sizeof( line ), "EXPECT FAILED (line %d): %s", lineNo, w[ 1 ].c_str( ) );
            note( "%s", line );
        }
    } else if ( d == "@echo" ) {
        std::string text;
        for ( size_t i = 1; i < w.size( ); i++ )
            text += ( i > 1 ? " " : "" ) + w[ i ];
        note( "-- %s", text.c_str( ) );
    } else if ( d == "@quit" ) {
        return false;
    } else {
        printf( "line %d: unknown directive %s\n", lineNo, d.c_str( ) );
        return false;
    }
    return true;
}

int main( int argc, char** argv ) {
    const char* script = nullptr;
    for ( int i = 1; i < argc; i++ ) {
        if ( strcmp( argv[ i ], "-q" ) == 0 )
            quiet = true;
        else
            script = argv[ i ];
    }
    if ( script == nullptr ) {
        fprintf( stderr, "usage: sim <script> [-q]\n" );
        return 2;
    }
    FILE* f = fopen( script, "r" );
    if ( f == nullptr ) {
        fprintf( stderr, "cannot open %s\n", script );
        return 2;
    }
    crcInit( );
    srand( 4 );
    Serial.onWrite = onSerialWrite;

    // The firmware boots as on the board, then the array is told the frames
    // come from the world.
    setup( );
    magArray.useSimulatedFrames( );
    jOS.registerService( &world );
    consoleAddVerb( "screen:png", "<file>", "(host) the panel as a PNG", CONSOLE_READS, onScreenPng );
    consoleAddVerb( "screen:verify", "", "(host) the dump path against the panel", CONSOLE_READS, onScreenVerify );
    run( 50 );
    bool bootOk = rowCounter.active == ROWCOUNT_ROW_MODE_AT_BOOT && ui.shell.menu.count < MENU_MAX_ITEMS - 8;
    printf( "boot: row mode %s (boots %s), chain %s, menu %d/%d items (%s)\n", rowCounter.active ? "on" : "off", ROWCOUNT_ROW_MODE_AT_BOOT ? "on" : "off", probeLeds.strip ? "on" : "off", ui.shell.menu.count,
            MENU_MAX_ITEMS, bootOk ? "PASS" : "FAIL" );
    if ( !bootOk )
        failures++;

    char buffer[ 512 ];
    int lineNo = 0;
    while ( fgets( buffer, sizeof( buffer ), f ) != nullptr ) {
        lineNo++;
        std::string line = buffer;
        while ( !line.empty( ) && ( line.back( ) == '\n' || line.back( ) == '\r' || line.back( ) == ' ' ) )
            line.pop_back( );
        size_t first = line.find_first_not_of( ' ' );
        if ( first == std::string::npos || line[ first ] == '#' )
            continue;
        line = line.substr( first );
        if ( line[ 0 ] == '@' ) {
            std::vector<std::string> w = words( line );
            if ( w[ 0 ] != "@expect" )
                mark = outText.size( );
            if ( !directive( w, lineNo ) ) {
                failures++; // a script that cannot go on has failed (2026-09-21: an unknown directive ended a run green at 28 s)
                break;
            }
            continue;
        }
        mark = outText.size( );
        if ( line[ 0 ] == ':' ) {
            typeText( line + "\r" );
            uint64_t until = simMicros + (uint64_t)SIM_VERB_TIMEOUT_MS * 1000u;
            while ( simMicros < until ) {
                pass( );
                if ( verifyPending ) {
                    verifyPending = false;
                    doVerify( );
                    break;
                }
                if ( frameClosed( mark ) )
                    break;
            }
            run( 10 );
        } else {
            std::string text = unescape( line );
            typeText( text );
            run( 100 + 120 * (uint32_t)text.size( ) );
        }
    }
    fclose( f );
    printf( "sim: %d expects, %d failed, %.1f s simulated\n", expects, failures, simMicros * 1e-6 );
    return failures == 0 ? 0 : 1;
}
