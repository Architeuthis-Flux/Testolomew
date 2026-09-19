// SPDX-License-Identifier: MIT
#ifndef PROBECURSOR_H
#define PROBECURSOR_H
// ---------------------------------------------------------------------------
// JumperlOS module for a V5: the wireless probe's cursor on the breadboard's
// LEDs, fed by a Testolomew array board over a UART.
//
// A V5 has no magnetometer array, so the position comes from the Testolomew
// (its `L` command streams one line per update, 25 Hz):
//
//   cursor,<state>,<along rows>,<across mm>,<sigma rows>,<sigma across mm>,
//          <confidence>,<height mm>,<have under>,<under along>,<under across>
//
// wired console TX -> a V5 UART RX at 115200 (Serial1 on the Nano header, or
// whatever Stream begin() is given). This module parses those lines, renders
// them with the same ProbeLeds renderer the Testolomew previews on its LCD
// (V5 layout: 5+5 holes across the channel, rails left alone), and paints the
// result into a 10 x 30 graphic overlay - the layer JumperlOS draws over the
// nets (GraphicOverlays.h) - so the cursor floats over whatever the board is
// showing. Nothing is drawn while no line has arrived for a second.
//
// To install: copy this folder and ../../src/probeled/ProbeLeds.{h,cpp} into
// JumperlOS's src/ (a folder under src/ is on the include path there too),
// then in main.cpp: probeCursor.begin( &Serial1 ); jOS.registerService(
// &probeCursor ); (Serial1.begin( 115200 ) first). The overlay row/hole
// mapping follows PartLabels: overlay row 1 is the outer edge of the top
// half, row 10 the outer edge of the bottom half.
//
// Not compiled on the Testolomew; compile-checked against JumperlOS on
// 2026-09-17, not yet run on a V5.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "ProbeLeds.h"

#define PROBECURSOR_PERIOD_US 40000 // 25 Hz renders
#define PROBECURSOR_STALE_MS 1000   // no line for this long = nothing to show
// The overlay's name. JumperlOS treats names starting "_GUIDE_" (and a few
// others) as session-only: never saved, and painting them does not mark the
// board state dirty - which matters at 25 paints a second (anything else
// would trigger idle auto-saves and "unsaved edits"). A name of its own would
// go in overlayIsSessionOnly() in GraphicOverlays.cpp.
#define PROBECURSOR_OVERLAY "_GUIDE_PROBE_"
#define PROBECURSOR_LINE_MAX 120

class ProbeCursor : public Service {
  public:
    static ProbeCursor& getInstance( );

    ProbeCursor( const ProbeCursor& ) = delete;
    ProbeCursor& operator=( const ProbeCursor& ) = delete;

    void begin( Stream* link );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "ProbeCursor"; }
    ServicePriority getPriority( ) const override { return ServicePriority::NORMAL; }
    uint32_t periodUs( ) const override { return PROBECURSOR_PERIOD_US; }
    // Keep painting inside modal loops (probe mode, menus), as LedDump does.
    bool inInnerSet( ) const override { return true; }

    ProbeLedInput input = { };
    uint32_t lastLineMs = 0;
    uint32_t linesParsed = 0, linesBad = 0;

  private:
    ProbeCursor( ) = default;

    Stream* link = nullptr;
    LedLayout layout;
    ProbeLedFrame frame;
    ProbeLedStyle style;
    char line[ PROBECURSOR_LINE_MAX ];
    int lineLength = 0;
    uint32_t lastRenderUs = 0;
    bool overlayShown = false;

    void readLink( );
    bool parseLine( const char* text );
    void paint( );
};

extern ProbeCursor& probeCursor;

#endif // PROBECURSOR_H
