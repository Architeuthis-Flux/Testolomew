// SPDX-License-Identifier: MIT
#ifndef DISPLAY_H
#define DISPLAY_H
// ---------------------------------------------------------------------------
// The LCD as a service: two framebuffers, the DMA push, and WHEN a frame is
// drawn. What is drawn is somebody else's (drawFn: the apps framework).
//
// Timing: one framebuffer is on its way to the panel in two DMA bands that
// run on their own while the loop goes on reading sensors; the next frame
// is drawn into the other meanwhile (a few milliseconds, in one tick). A
// tick otherwise only starts a band or notices one has finished, so the
// sensors keep their 100 Hz and the panel gets 40-50 frames a second.
//
// The breadboard LEDs come first: a draw holds the loop for as long as it
// takes, and the LED service (50 Hz, its own DMA) must not be made late by
// one, so a frame is not started when the LED service is due sooner than
// this slot's draws have been taking (the running average - one draw
// stalled by a settings write is not the expectation); it is drawn on the
// next tick instead. Whatever the numbers say, a screen not drawn for
// DISPLAY_STARVE_US is drawn now: one late LED frame is nothing next to a
// display that has stopped. Text goes through FastDraw.h, not GFX's print,
// which is what made a draw take tens of milliseconds.
//
// drawFn returns false when it has nothing new to show (a text screen that
// has not changed): no push, and the slot's draw time is not touched, so
// no-op frames never drag the average down and weaken the yield. `slot`
// is which screen is being drawn (the caller sets it: an app), so each
// keeps its own times for `e` / :stats.
//
// For a screen dump (DumpService): `hold` stops new frames going to the
// panel (the push under way finishes; nothing is drawn meanwhile),
// frozen() says the shown frame is complete and will stay, copyShownRow()
// gives a row of it in native RGB565 - the push swaps the bytes of the
// shown buffer in place for the wire (ST7789.h), so the copy swaps them
// back iff that has happened (shownPushed).
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"

#define DISPLAY_BAND_ROWS 120 // rows per DMA band (two per frame; one DMA block is 136 rows at most)
#define DISPLAY_STARVE_US 60000
#define DISPLAY_SLOTS 12
#define DISPLAY_PERIOD_US 3000

class GFXcanvas16;

typedef bool ( *DisplayDrawFn )( GFXcanvas16* canvas, uint32_t nowMs ); // false = nothing new to show

class Display : public Service {
  public:
    static Display& getInstance( );

    Display( const Display& ) = delete;
    Display& operator=( const Display& ) = delete;

    // false if the panel or the framebuffers could not be set up (the
    // service then idles).
    bool begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "Display"; }
    ServicePriority getPriority( ) const override { return ServicePriority::NORMAL; }
    uint32_t periodUs( ) const override { return DISPLAY_PERIOD_US; }

    DisplayDrawFn drawFn = nullptr;
    int slot = 0; // which screen the draws are for (its own times)
    float fps( ) const { return framesPerSecond; }
    bool ready( ) const { return canvas != nullptr; }

    // How long a draw of each slot takes (microseconds): the last one, an
    // average that follows, and the longest since boot.
    uint32_t drawLastUs[ DISPLAY_SLOTS ] = { 0 };
    uint32_t drawAvgUs[ DISPLAY_SLOTS ] = { 0 };
    uint32_t drawMaxUs[ DISPLAY_SLOTS ] = { 0 };
    uint32_t clearUs[ 2 ] = { 0, 0 }; // a fillScreen of each canvas, timed at begin(): says which memory it landed in
    uint32_t yields = 0;              // draws put off because the LED service was due first
    uint32_t lastDrawStartUs = 0;     // when a draw last ran (the starvation guard)
    // The slots' times, one line (names may be nullptr: numbered). The apps
    // framework sets slotNames / slotCount; :stats uses them.
    void printStats( Stream* out, const char* const* slotNames, int slotCount );
    const char* const* slotNames = nullptr;
    int slotCount = DISPLAY_SLOTS;

    // The dump contract.
    bool hold = false;
    bool shownPushed = false;
    bool frozen( ) const { return hold && pushRow < 0; }
    void copyShownRow( int y, uint16_t* dst ) const;

  private:
    Display( ) = default;

    GFXcanvas16* canvas = nullptr; // the frame being drawn
    GFXcanvas16* shown = nullptr;  // the frame on its way to the panel
    int pushRow = -1;              // next row of `shown` to send, -1 = nothing in flight
    bool drawn = false;            // `canvas` holds a frame not yet sent
    uint32_t fpsWindowStartMs = 0;
    int fpsWindowFrames = 0;
    float framesPerSecond = 0.0f;

    void benchmark( );
    bool tryDraw( ); // a timed drawFn, unless the LEDs are due first
};

extern Display& display;

#endif // DISPLAY_H
