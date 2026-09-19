// SPDX-License-Identifier: MIT
#include "Display.h"

#include <Adafruit_GFX.h>
#include <string.h>
#if __has_include( <ch32h4_itcm.h> )
#include <ch32h4_itcm.h> // __itcm_func: a function copied to the zero-wait ITCM
#else
#define __itcm_func
#endif

#include "Console.h"
#include "FastDraw.h"
#include "ST7789.h"
#include "config.h"
#if MODULE_PROBE_LEDS
#include "ProbeLedService.h"
#endif

Display& display = Display::getInstance( );

Display& Display::getInstance( ) {
    static Display instance;
    return instance;
}

bool Display::begin( ) {
    if ( !st7789Begin( ) ) {
        return false;
    }
    canvas = new GFXcanvas16( LCD_WIDTH, LCD_HEIGHT );
    shown = new GFXcanvas16( LCD_WIDTH, LCD_HEIGHT );
    if ( canvas == nullptr || canvas->getBuffer( ) == nullptr || shown == nullptr || shown->getBuffer( ) == nullptr ) {
        canvas = nullptr;
        return false;
    }
    canvas->setTextWrap( false );
    shown->setTextWrap( false );
    // Which memory the two landed in shows in how long a clear takes (the
    // heap is DTCM first, then the shared SRAM with its wait states).
    uint32_t t0 = micros( );
    canvas->fillScreen( 0 );
    clearUs[ 0 ] = micros( ) - t0;
    t0 = micros( );
    shown->fillScreen( 0 );
    clearUs[ 1 ] = micros( ) - t0;
    benchmark( );
    return true;
}

// What the drawing primitives cost on this chip, printed once at boot: the
// scene is a few thousand line pixels and a few hundred characters, and
// which of those is the time decides what is worth rewriting.
// Two copies of what GFXcanvas16::drawPixel does, one in flash next to this
// code and one in ITCM, to tell an address effect from a library one.
static void __attribute__( ( noinline ) ) pixelFlash( uint16_t* buffer, int16_t x, int16_t y, uint16_t color ) {
    if ( ( x < 0 ) || ( y < 0 ) || ( x >= LCD_WIDTH ) || ( y >= LCD_HEIGHT ) )
        return;
    buffer[ x + y * LCD_WIDTH ] = color;
}
static __itcm_func void pixelItcm( uint16_t* buffer, int16_t x, int16_t y, uint16_t color ) {
    if ( ( x < 0 ) || ( y < 0 ) || ( x >= LCD_WIDTH ) || ( y >= LCD_HEIGHT ) )
        return;
    buffer[ x + y * LCD_WIDTH ] = color;
}

void Display::benchmark( ) {
    Stream* out = console.port( );
    if ( out == nullptr ) {
        return;
    }
    uint16_t* buffer = canvas->getBuffer( );
    uint32_t t0 = micros( );
    for ( int i = 0; i < 10000; i++ ) {
        canvas->drawPixel( i % 240, ( i / 240 ) % 240, 0x1234 );
    }
    uint32_t pixelUs = micros( ) - t0;
    t0 = micros( );
    for ( int i = 0; i < 10000; i++ ) {
        pixelFlash( buffer, i % 240, ( i / 240 ) % 240, 0x1234 );
    }
    uint32_t flashUs = micros( ) - t0;
    t0 = micros( );
    for ( int i = 0; i < 10000; i++ ) {
        pixelItcm( buffer, i % 240, ( i / 240 ) % 240, 0x1234 );
    }
    uint32_t itcmUs = micros( ) - t0;
    t0 = micros( );
    for ( int i = 0; i < 100; i++ ) {
        memset( buffer + i * 240, i, 480 ); // libc, 240 pixels
    }
    uint32_t memsetUs = micros( ) - t0;
    t0 = micros( );
    for ( int i = 0; i < 10000; i++ ) {
        buffer[ ( i % 240 ) + ( ( i / 240 ) % 240 ) * 240 ] = 0x1234;
    }
    uint32_t directUs = micros( ) - t0;
    t0 = micros( );
    for ( int i = 0; i < 100; i++ ) {
        canvas->drawLine( 10, 10 + i, 110, 60 + i, 0x1234 ); // 100 pixels each
    }
    uint32_t lineUs = micros( ) - t0;
    t0 = micros( );
    for ( int i = 0; i < 100; i++ ) {
        fastLine( canvas, 10, 10 + i, 110, 60 + i, 0x1234 );
    }
    uint32_t fastLineUs = micros( ) - t0;
    t0 = micros( );
    for ( int i = 0; i < 400; i++ ) {
        canvas->fillRect( ( i % 30 ) * 7, ( i / 30 ) * 7, 6, 6, 0x1234 );
    }
    uint32_t rectUs = micros( ) - t0;
    t0 = micros( );
    for ( int i = 0; i < 10; i++ ) {
        fastText( canvas, 0, i * 16, 2, 0x1234, "x12.3 y45.6 z78.9 mm" ); // 20 characters
    }
    uint32_t textUs = micros( ) - t0;
    volatile float acc = 0.0f;
    t0 = micros( );
    for ( int i = 0; i < 1000; i++ ) {
        acc += sinf( i * 0.001f );
    }
    uint32_t sinUs = micros( ) - t0;
    canvas->fillScreen( 0 );
    char line[ 260 ];
    snprintf( line, sizeof( line ),
              "Display: GFX drawPixel %lu ns, the same in flash here %lu ns, in ITCM %lu ns, a buffer write %lu ns, memset of 480 B %lu ns, a 100 px line %lu us (GFX) / %lu us (FastDraw), a 6x6 fillRect %.1f us, a 20-char line of size-2 text %lu us, sinf %lu ns",
              (unsigned long)( pixelUs * 1000 / 10000 ), (unsigned long)( flashUs * 1000 / 10000 ), (unsigned long)( itcmUs * 1000 / 10000 ), (unsigned long)( directUs * 1000 / 10000 ),
              (unsigned long)( memsetUs * 1000 / 100 ), (unsigned long)( lineUs / 100 ), (unsigned long)( fastLineUs / 100 ), rectUs / 400.0f, (unsigned long)( textUs / 10 ),
              (unsigned long)( sinUs * 1000 / 1000 ) );
    out->println( line );
}

void Display::printStats( Stream* out, const char* const* slotNames, int slotCount ) {
    char line[ 240 ];
    int n = snprintf( line, sizeof( line ), "display: %.0f fps, draw ms avg/max", framesPerSecond );
    for ( int s = 0; s < slotCount && s < DISPLAY_SLOTS && n < (int)sizeof( line ) - 40; s++ ) {
        if ( drawMaxUs[ s ] == 0 )
            continue;
        if ( slotNames != nullptr && slotNames[ s ] != nullptr ) {
            n += snprintf( line + n, sizeof( line ) - n, " %s %.1f/%.1f", slotNames[ s ], drawAvgUs[ s ] * 1e-3f, drawMaxUs[ s ] * 1e-3f );
        } else {
            n += snprintf( line + n, sizeof( line ) - n, " #%d %.1f/%.1f", s, drawAvgUs[ s ] * 1e-3f, drawMaxUs[ s ] * 1e-3f );
        }
    }
    snprintf( line + n, sizeof( line ) - n, "; clear %lu+%lu us; %lu yields to the LEDs; hold %d", (unsigned long)clearUs[ 0 ], (unsigned long)clearUs[ 1 ], (unsigned long)yields, hold ? 1 : 0 );
    out->println( line );
}

void Display::copyShownRow( int y, uint16_t* dst ) const {
    if ( shown == nullptr || y < 0 || y >= LCD_HEIGHT ) {
        for ( int x = 0; x < LCD_WIDTH; x++ )
            dst[ x ] = 0;
        return;
    }
    const uint16_t* src = shown->getBuffer( ) + (size_t)y * LCD_WIDTH;
    for ( int x = 0; x < LCD_WIDTH; x++ ) {
        uint16_t c = src[ x ];
        dst[ x ] = shownPushed ? (uint16_t)( ( c << 8 ) | ( c >> 8 ) ) : c;
    }
}

// A draw, unless the LED service is due before it would be over: the LEDs
// come first, and the loop cannot take a draw back once it has started. A
// screen that takes longer than the LED period is drawn anyway (it would
// never be shown otherwise). The draw is timed, per slot.
bool Display::tryDraw( ) {
    uint32_t t0 = micros( );
    int s = slot < 0 ? 0 : ( slot >= DISPLAY_SLOTS ? DISPLAY_SLOTS - 1 : slot );
#if MODULE_PROBE_LEDS
    if ( probeLeds.strip ) {
        // The expectation is the running average, not the last draw: one
        // draw stalled by a settings write (a 20 ms flash page) read 18 ms,
        // and 18 ms never fits between LED frames 20 ms apart with a 4 ms
        // frame in them - and only a draw can bring the number down, so the
        // screen froze (2026-09-19, twice). And whatever the numbers say, a
        // screen not drawn for DISPLAY_STARVE_US is drawn now: one late LED
        // frame is nothing next to a display that has stopped.
        uint32_t expect = drawAvgUs[ s ];
        if ( expect > 0 && expect < PROBELED_PERIOD_US && (uint32_t)( t0 - lastDrawStartUs ) < DISPLAY_STARVE_US ) {
            int64_t untilDue = (int64_t)( probeLeds.nextDueUs - micros64( ) );
            if ( untilDue < (int64_t)expect ) {
                yields++;
                return false;
            }
        }
    }
#endif
    lastDrawStartUs = t0;
    if ( drawFn == nullptr || !drawFn( canvas, millis( ) ) ) {
        return false;
    }
    s = slot < 0 ? 0 : ( slot >= DISPLAY_SLOTS ? DISPLAY_SLOTS - 1 : slot ); // the draw may have picked its slot
    uint32_t took = micros( ) - t0;
    drawLastUs[ s ] = took;
    drawAvgUs[ s ] = drawAvgUs[ s ] == 0 ? took : ( drawAvgUs[ s ] * 15 + took ) / 16;
    if ( took > drawMaxUs[ s ] ) {
        drawMaxUs[ s ] = took;
    }
    return true;
}

ServiceStatus Display::service( ) {
    if ( canvas == nullptr ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }

    if ( pushRow >= 0 ) {
        // A frame is on its way. While a band is on the wire, draw the next
        // frame into the other buffer if that is not done yet; otherwise there
        // is nothing to do until the wire is clear.
        if ( st7789PushBusy( ) ) {
            if ( !hold && !drawn && tryDraw( ) ) {
                drawn = true;
                lastStatus = ServiceStatus::BUSY;
                return lastStatus;
            }
            lastStatus = ServiceStatus::IDLE;
            return lastStatus;
        }
        st7789PushFinish( );
        if ( pushRow < LCD_HEIGHT ) {
            int rows = LCD_HEIGHT - pushRow;
            if ( rows > DISPLAY_BAND_ROWS ) {
                rows = DISPLAY_BAND_ROWS;
            }
            st7789PushRowsStart( shown->getBuffer( ), pushRow, rows );
            pushRow += rows;
            lastStatus = ServiceStatus::BUSY;
            return lastStatus;
        }
        // The whole frame is on the panel.
        pushRow = -1;
        shownPushed = true;
        fpsWindowFrames++;
        uint32_t now = millis( );
        if ( now - fpsWindowStartMs >= 1000 ) {
            framesPerSecond = fpsWindowFrames * 1000.0f / ( now - fpsWindowStartMs );
            fpsWindowStartMs = now;
            fpsWindowFrames = 0;
        }
    }

    // Held for a dump: the shown frame stays as it is.
    if ( hold ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    // Nothing in flight: a drawn frame (or one drawn now) starts its way to
    // the panel, and the buffers swap so the next one is drawn elsewhere.
    if ( !drawn && !tryDraw( ) ) {
        lastStatus = ServiceStatus::IDLE; // nothing new to show, or not now
        return lastStatus;
    }
    GFXcanvas16* t = shown;
    shown = canvas;
    canvas = t;
    drawn = false;
    pushRow = 0;
    shownPushed = false;
    st7789PushRowsStart( shown->getBuffer( ), 0, DISPLAY_BAND_ROWS < LCD_HEIGHT ? DISPLAY_BAND_ROWS : LCD_HEIGHT );
    pushRow = DISPLAY_BAND_ROWS < LCD_HEIGHT ? DISPLAY_BAND_ROWS : LCD_HEIGHT;
    lastStatus = ServiceStatus::BUSY;
    return lastStatus;
}
