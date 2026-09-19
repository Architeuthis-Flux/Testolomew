// SPDX-License-Identifier: MIT
#ifndef DUMPSERVICE_H
#define DUMPSERVICE_H
// ---------------------------------------------------------------------------
// The screen and the LEDs, out over the console: what an agent (or a host
// script) looks at instead of the panel.
//
//   :screen:ascii [colour]        the frame as 80 x 60 characters (3 x 4 px a
//                                 cell): a grey ramp, or hue letters with it
//   :screen:dump [rle|b64] [step] every row of the frame as RGB565 - run-length
//                                 hex (the default) or base64 - every step-th
//                                 row and pixel (tools/screendump.py decodes
//                                 both into a PNG)
//   :leds                         the breadboard's LEDs as a grid of hue letters
//   :leds:hex                     every LED's 8-bit colour as sent (gamma applied)
//   :leds:map                     the layout: where every LED sits
//   :stats                        the service table, the display's draw times
//                                 per screen, the LED strip's frame statistics
//
// A dump is thousands of bytes, and the console's DMA ring is 4 KB: printed
// in one go it would wait on the ring and hold the loop (and the LEDs) for
// as long as the wire takes. So this service emits ONE row a tick, and only
// when the ring has room for it and DUMP_RING_RESERVE bytes to spare for
// everyone else, so no tick takes more than a few hundred microseconds and
// nothing ever waits. A screen dump first asks the display to hold its
// shown frame (MagView::hold) and waits until that frame is complete and
// still (frozen), copies it row by row, and lets go at the end. What a dump
// prints never lands in the on-screen log (UiStream::logToScreen).
//
// Every verb answers with one frame: `name{` ... `}`.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"

#define DUMP_PERIOD_US 5000
#define DUMP_RING_RESERVE 256  // bytes of the console ring left for everyone else
#define DUMP_FREEZE_WAIT_MS 300 // longest wait for the display to hold still
#define DUMP_ASCII_W 80
#define DUMP_ASCII_H 60
#define DUMP_LINE_MAX 2048

enum DumpKind {
    DUMP_NONE,
    DUMP_ASCII,   // the screen, 80 x 60 characters
    DUMP_RLE,     // the screen, run-length hex rows
    DUMP_B64,     // the screen, base64 rows
    DUMP_LEDS_HEX, // the LEDs, 20 a line
    DUMP_LEDS_MAP  // the layout, one LED a line
};

class DumpService : public Service {
  public:
    static DumpService& getInstance( );

    DumpService( const DumpService& ) = delete;
    DumpService& operator=( const DumpService& ) = delete;

    void begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "Dump"; }
    ServicePriority getPriority( ) const override { return ServicePriority::LOW; }
    uint32_t periodUs( ) const override { return DUMP_PERIOD_US; }

    // Start one (false if another is under way); it runs to its end by itself.
    bool start( DumpKind kind, int step, bool colour, Stream* out );
    void finish( );
    bool busy( ) const { return kind != DUMP_NONE; }
    uint32_t dumps = 0;
    // The tick, timed: the longest row build and the longest print (us),
    // for the dump under way (reset at start) and since boot.
    uint32_t buildMaxUs = 0, printMaxUs = 0, buildMaxEverUs = 0, printMaxEverUs = 0;

  private:
    DumpService( ) = default;

    DumpKind kind = DUMP_NONE;
    int row = 0;
    int step = 1;
    bool colour = false;
    Stream* out = nullptr;
    bool wasLogging = true;
    uint32_t startedMs = 0;
    char line[ DUMP_LINE_MAX ];

    int nextLine( ); // the next row into line[]; < 0 when there are no more
    int asciiLine( int y );
    int rleLine( int y );
    int b64Line( int y );
    int ledsHexLine( int n );
    int ledsMapLine( int n );
};

extern DumpService& dump;

#endif // DUMPSERVICE_H
