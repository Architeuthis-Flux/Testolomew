// SPDX-License-Identifier: MIT
#ifndef PROBELEDSERVICE_H
#define PROBELEDSERVICE_H
// ---------------------------------------------------------------------------
// Keeps the breadboard-LED picture of the probe up to date (ProbeLeds.h) from
// the tracker, fifty times a second. The test bed has no breadboard LEDs, so
// the picture is shown on the LCD (the LEDs app) and, for a V5 that
// does have them, streamed over the console as one line per update that the
// JumperlOS module in ports/jumperlos/ProbeCursor turns into pixels.
//
// With PIN_LED_STRIP wired (BoardPins.h), the V5 layout's LEDs are also
// driven directly, every tick (50 Hz; a 400-LED frame is 12 ms on the wire,
// and the strip has its own DMA channel so nothing waits for it), scaled by
// the "strip" lever (PROBELED_STRIP_BRIGHTNESS to start) x the style's
// peak: at 1 x 1 the cursor's LED is at the chain's full white (a few LEDs
// at a time, so the current stays small). The 300 hole
// LEDs are pixels 0-299 as JumperlOS numbers them and the 100 rail LEDs
// 300-399 after them, which is one chain on a V5 up to hardware revision 3;
// a revision 4+ V5 has the rails on a second strip (JumperlOS's "top"
// strip, LEDs.cpp: splitLEDs), and sending them on the first chain does
// nothing there - PIN_LED_STRIP_TOP is for that.
//
// This service runs at HIGH priority, before the display: a late LED frame
// is seen on the breadboard, a late LCD frame is not. The Display also puts a
// draw off when this service is due before the draw would be over.
//
// Console: L = stream the cursor for a V5 (toggle), B = board layout V6/V5,
// N = the LED strip on/off (toggle), n = a chase along it (a dot from pixel
// 0 up the whole chain, so the wiring and the pixel order can be seen) with
// the frame statistics.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "LedStrip.h"
#include "ProbeLeds.h"

#define PROBELED_PERIOD_US 20000       // 50 Hz
#define PROBELED_STREAM_EVERY 2        // stream at 25 Hz
#define PROBELED_RAW_HOLD_MS 250       // with the tracker off, the last raw fix's cursor stays (as coasting) this long after a frame with no fix: a fast hand's refused frames were dark ones (2026-09-26)
#define PROBELED_STRIP_BRIGHTNESS 0.96f // the menu's "strip" lever; 1 = the LEDs as bright as they go (the bench's setting on 2026-09-27, taken as the default: 1.0 until then) (the bench's setting on 2026-09-28 evening, taken as the default: 0.62 until then)
// The chain's current, kept under a budget: a frame that would draw more
// than the budget is scaled down whole, so the picture keeps its shape and
// the rail keeps its volts. The budget is the menu's "budget mA" (`stripMaxMa`,
// saved) but never more than PROBELED_STRIP_HARD_MAX_MA, which is the
// ceiling below the settings: nothing the menu, a saved value or a mode
// (paint, with half the board lit) can do gets past it. The model is
// PROBELED_MA_PER_CHANNEL per colour at full; ASSUMPTION for the V5's
// XL-1010RGBC (a 1010 package; a 5050 WS2812B is 20 mA a channel, the small
// ones are quoted 5-12 - 12 taken, the cautious end: a model that reads low
// is a budget that lets the rail sag). A V5 runs its LEDs at 10/255 by
// default (JumperlOS's led_brightness): all 445 white at that is ~200 mA by
// this model. It browned out under this cursor's wide glows at full on
// 2026-09-18 (hence the budget), and again at 600 mA budgeted by the 8 mA
// model under the paint mode on 2026-09-19 (hence the ceiling and the
// model's revision).
#define PROBELED_MA_PER_CHANNEL 12.0f
#define PROBELED_STRIP_MAX_MA 800.0f      // the menu's "budget mA" to start with (2026-09-19: 300 was far too dim with a breadboard on top of the LEDs; 1000 was set after, and the board browned out under it - see the ceiling) (the bench's setting on 2026-09-27, taken as the default: 600 until then - the ceiling PROBELED_STRIP_HARD_MAX_MA is still the most)
#define PROBELED_STRIP_HARD_MAX_MA 800.0f // the ceiling the menu cannot pass: the 2026-09-19 brown-out was ~900 by this model, and the 2026-09-21 boot loop under a lit cursor with the budget at 1000-1100; the LEDs share the board's supply
// ...and a frame's current may rise by at most this much over the last frame's (20 ms): a step is what a supply cannot follow - 0 to 600 mA in one frame
// is the dip that resets the chip - so a cursor lighting up ramps over a few frames (a fall is not limited: less current is never the problem).
#define PROBELED_MAX_STEP_MA 150.0f
#define PROBELED_CHASE_PIXELS_PER_S 80 // the `n` dot's speed: the whole 400-LED chain in five seconds
#define PROBELED_CHASE_MS 5500
#define PROBELED_CHAIN_DMA 8 // DMA1 channels (LedStrip.h)
#define PROBELED_TOP_DMA 1

class ProbeLedService : public Service {
  public:
    static ProbeLedService& getInstance( );

    ProbeLedService( const ProbeLedService& ) = delete;
    ProbeLedService& operator=( const ProbeLedService& ) = delete;

    void begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "ProbeLeds"; }
    ServicePriority getPriority( ) const override { return ServicePriority::HIGH; } // before the display
    uint32_t periodUs( ) const override { return PROBELED_PERIOD_US; }

    LedLayout layout;
    ProbeLedFrame frame;
    ProbeLedStyle style;
    ProbeLedInput input = { };
    ProbeLedInput rawHeld = { }; // with the tracker off: the last live input, and when it was taken (PROBELED_RAW_HOLD_MS)
    uint32_t rawHeldMs = 0;
    bool v5 = false; // which board the layout is for
    bool streaming = false;
    bool strip = false;            // a real LED chain is wired and driven
    bool topStrip = false;         // and a second one, the rails of a revision 4+ V5
    LedStrip chain;                // the breadboard's LEDs (and the rails, on one chain)
    LedStrip top;                  // the rails on their own strip
    uint32_t stripTestUntilMs = 0; // `n`: a chase along the chain, to check the wiring and the order
    uint32_t stripTestStartMs = 0;
    float stripBrightness = PROBELED_STRIP_BRIGHTNESS;
    ProbeLedBrush brush = { false, 0, 0.0f, 0, 0, 0 };            // set by the play module in paint mode: the cursor is then the brush
    uint32_t renderUs = 0, stripUs = 0;                           // the last run's render and strip-send times (the n report)
    uint32_t stripFillUs = 0, stripBudgetUs = 0, stripShowUs = 0; // ...and the send's parts
    uint32_t layoutGeneration = 0;                                // bumps when the layout is rebuilt (B / useV5): whoever indexes LEDs starts over
    float stripMaxMa = PROBELED_STRIP_MAX_MA;                     // the current budget for a frame (the menu's; PROBELED_STRIP_HARD_MAX_MA caps it)
    float stripBudgetMa( ) const { return stripMaxMa < PROBELED_STRIP_HARD_MAX_MA ? stripMaxMa : PROBELED_STRIP_HARD_MAX_MA; }
    uint32_t stripScaledFrames = 0; // frames scaled down to it
    float stripWorstScale = 1.0f;   // the smallest scale applied since the last `n`
    float stripLastMa = 0.0f;       // the last frame's estimated current, after scaling
    // ...and since the last :load report: the least and most a frame drew,
    // the biggest change from one frame to the next, and how many frames
    // went out (a WS2812 keeps its last frame, so the current changes only
    // when a frame does).
    float stripLeastMa = 0.0f, stripMostMa = 0.0f, stripMaxStepMa = 0.0f;
    uint32_t stripSlewedFrames = 0; // frames held back by PROBELED_MAX_STEP_MA
    uint32_t stripFramesSent = 0;
    void resetStripWindow( ) {
        stripLeastMa = stripMostMa = stripLastMa;
        stripMaxStepMa = 0.0f;
        stripFramesSent = 0;
    }

    void useV5( bool on );
    // The chain on or off (off clears it); nothing happens if it is so already.
    void setStrip( bool on, Stream* out );
    void printCursorLine( Stream* out ) const;

  private:
    ProbeLedService( ) = default;
    uint32_t lastUs = 0;
    int tick = 0;
    uint8_t stripRgb[ LEDSTRIP_MAX ][ 3 ]; // a frame's colours before the budget
    void sendStrip( );
    void sendFrame( bool chainFree, bool topFree, int count );
};

extern ProbeLedService& probeLeds;

#endif // PROBELEDSERVICE_H
