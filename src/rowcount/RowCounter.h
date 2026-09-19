// SPDX-License-Identifier: MIT
#ifndef ROWCOUNTER_H
#define ROWCOUNTER_H
// ---------------------------------------------------------------------------
// Which breadboard row is the probe over? The test that matters for the
// magnetic probe: rows are 2.54 mm apart, so the fix has to be good to about a
// millimetre - and stay that good from one end of the board to the other.
//
// The breadboard is the Jumperless one (RowGrid.h): rows 1-30 along the far
// half, 31-60 along the near half, row 31 facing row 1 across the channel.
//
// Takes MagLocator's fixes and the row grid and keeps `reading`: the row, how
// far into it, which hole, the error bar in rows and the chance the row is
// right. MagView shows it large while row mode is on.
//
// Two positions are in play, and they are kept apart on purpose:
//  - the RAW fix of each frame, which is the one the fit's error bar describes.
//    place, sigmaRows and confidence are all about that one fix.
//  - the smoothed fix MagLocator shows. `row`, the counted row, follows that
//    one, and only changes once it is ROWCOUNT_HYSTERESIS_ROWS past a boundary
//    (or ROWCOUNT_HALF_HYSTERESIS_MM past the channel's centre line), so it
//    does not chatter when the probe sits between two rows.
//
// Where the breadboard is comes from ANCHORS: fixes taken with the probe in
// known holes, all fitted together (rowGridFit) - the more of them and the
// further apart, the more the fit can take out: where the breadboard lies, its
// angle, the magnet fit's scale, and with holes spread both ways a scale each
// way. What the anchors still miss by afterwards is printed, per anchor. That
// number is the accuracy of the whole scheme, in mm.
//
// Console:
//   r  row mode on/off: the row large on the LCD, a line here twice a second
//   c  guided calibration: asks for twelve taps - rows 1, 15, 30, 60, 45, 31,
//      each at the hole next to the channel and at the outermost hole - and
//      fits the grid to them. A tap is the probe resting on the board: held
//      still for a second at the height the board puts it at (learned from the
//      first tap). The LCD shows which hole is wanted and a bar filling while
//      the tap is taken. c again = cancel.
//   R  "the probe is on row <number> now, in the hole next to the channel":
//      one more anchor (R1<Enter>, R30<Enter>, R45<Enter> ...), and the grid
//      is fitted again to all of them
//   C  forget every anchor: back to the grid the firmware boots with
//   h  hold-still test: two seconds of fixes -> where, how much scatter, how
//      many of the single fixes called the right row, and how far it moved
//      since the last test. Step the probe one row and press h again: the
//      answer should move one row.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "RowGrid.h"
#include "Vec3.h"

// The grid the firmware boots with: { ax, ay, a0, cx, cy, c0 } (RowGrid.h). As
// it stands: fitted to 66 fixes of the one-magnet probe stepped hole by hole
// along both halves of the breadboard as it lay on the bench array on
// 2026-09-17 - every row then within 0.3 of its number
// (tools/recordings/2026-09-17-row-stepping-anchors.txt). `c` and `R` print a
// line to paste here.
#define ROWCOUNT_GRID_AT_BOOT { 1.00159f, -0.01635f, 9.469f, -0.01970f, 1.06541f, -22.830f }

// The magnet's height when the probe rests on the breadboard, mm. 0 = not
// known yet: the calibration's first tap sets it (and the whole calibration
// then refines it). Ditto, `c` prints the number to put here.
#define ROWCOUNT_TOUCH_Z_MM 17.5f     // the one-magnet probe of 2026-09-17, magnet at its point
#define ROWCOUNT_TOUCH_MARGIN_MM 4.0f // this far above it still counts as resting on the board

#define ROWCOUNT_HYSTERESIS_ROWS 0.1f
#define ROWCOUNT_HALF_HYSTERESIS_MM 1.0f
#define ROWCOUNT_HOLD_SETTLE_FIXES 100 // an R anchor or a hold test: this many fixes (1 s) are let go by first - the hand settling, the smoothing catching up...
#define ROWCOUNT_HOLD_FIXES 200        // ...then this many (2 s) are the measurement
#define ROWCOUNT_BLOCK_FIXES 8         // the hold test also reports the scatter of averages of this many fixes
#define ROWCOUNT_HOLD_TIMEOUT_MS 8000
#define ROWCOUNT_PRINT_PERIOD_MS 500
#define ROWCOUNT_NO_FIX_AFTER_MS 1000   // "no fix" is only said after this long without one (a lifted probe is not news)
#define ROWCOUNT_NUMBER_TIMEOUT_MS 8000 // how long R waits for the row number (the loop is stopped meanwhile)
#define ROWCOUNT_MAX_ANCHORS 24
#define ROWCOUNT_ROW_MODE_AT_BOOT true // boot in row mode (2026-09-18: Kevin wants it the default)

// A tap: the fix stays put for ROWCOUNT_TAP_SETTLE_FIXES (a second: the hand
// settling, the smoothing catching up) and then ROWCOUNT_TAP_MEASURE_FIXES
// more (two seconds), which are the measurement (2026-09-19: Kevin wanted
// the time to place the probe and a longer average; it was 0.4 + 0.4 s).
// "Stays put" is within three of the fix's own error bars, but never tighter
// or looser than these. The next tap has to be at least ROWCOUNT_TAP_APART_MM
// from the last, so resting on a hole is not taken for the next one as well
// (neighbouring targets are 10 mm apart).
// A gap in the fixes shorter than ROWCOUNT_TAP_GAP_MS does not start the tap over.
#define ROWCOUNT_TAP_SETTLE_FIXES 100
#define ROWCOUNT_TAP_MEASURE_FIXES 200
#define ROWCOUNT_TAP_FIXES ( ROWCOUNT_TAP_SETTLE_FIXES + ROWCOUNT_TAP_MEASURE_FIXES )
#define ROWCOUNT_TAP_GAP_MS 300
#define ROWCOUNT_TAP_STRAYS 10 // ...nor do this many fixes in a row off the spot (a glitch; more is the probe moving)
#define ROWCOUNT_TAP_STILL_MIN_MM 1.0f
#define ROWCOUNT_TAP_STILL_MAX_MM 4.0f
#define ROWCOUNT_TAP_APART_MM 5.0f

struct RowReading {
    bool valid;
    int row;             // the counted row, 1-60 (smoothed position, with hysteresis) - what a product would show. 0 = off the end of the breadboard
    RowPlace place;      // this frame's raw fix in breadboard terms: along (rows) and across (mm from the channel)
    float offsetRows;    // how far along, minus the nearest row: -0.5 .. 0.5
    int hole;            // which hole of the row the raw fix is over: 1 = next to the channel .. 5, 0 = channel or rails
    float sigmaRows;     // the raw fix's 1-sigma error bar along the rows, in rows
    float sigmaAcrossMm; // ...and across them
    float confidence;    // chance that the raw fix's nearest row (and half) is the true one, 0..1 (see rowGridConfidence)
    bool touching;       // the magnet is down at the height the board puts it at (false if that height is not known)
    // The same for the tracker's cursor (what the counted row follows when the
    // tracker is on): filtered, and carried through gaps.
    bool tracked; // the cursor below is live (tracking or coasting)
    RowPlace trackPlace;
    float trackSigmaRows;
    float trackConfidence;
};

enum RowHoldPurpose {
    ROWHOLD_NONE,
    ROWHOLD_TEST,
    ROWHOLD_ANCHOR
};

class RowCounter : public Service {
  public:
    static RowCounter& getInstance( );

    RowCounter( const RowCounter& ) = delete;
    RowCounter& operator=( const RowCounter& ) = delete;

    void begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "RowCounter"; }
    ServicePriority getPriority( ) const override { return ServicePriority::NORMAL; }
    uint32_t periodUs( ) const override { return 4000; }

    RowGrid grid;
    RowReading reading = { };
    bool active = ROWCOUNT_ROW_MODE_AT_BOOT; // row mode (r toggles it; a mode, not a saved setting)
    float touchZ = ROWCOUNT_TOUCH_Z_MM;

    void toggle( Stream* out );
    void startHold( RowHoldPurpose purpose, int row, Stream* out );
    void forgetAnchors( Stream* out );

    // The guided calibration, for the console and for MagView's prompt.
    void startCalibration( Stream* out ); // or cancel it, if it is running
    bool calibrating( ) const { return calibrationStep >= 0; }
    int calibrationStepNumber( ) const { return calibrationStep; }
    void calibrationTarget( int* row, int* hole ) const { rowGridCalibrationTarget( calibrationStep, row, hole ); }
    float tapProgress( ) const { return (float)stillCount / ROWCOUNT_TAP_FIXES; } // 0..1 while a tap is being taken

    // The anchors as they stand (for saving), and anchors put back from a
    // save: they replace what there is and the grid is fitted to them.
    int anchorList( const RowAnchor** list ) const {
        *list = anchors;
        return anchorCount;
    }
    void restoreAnchors( const RowAnchor* list, int count );

  private:
    RowCounter( ) = default;

    uint32_t lastFixCount = 0;
    uint32_t lastFixMs = 0;
    int countedAlong = 0;       // the counted row as a place along the board, 1-30...
    bool countedBottom = false; // ...and the half it is on
    uint32_t lastPrintMs = 0;
    bool saidNoFix = false;

    // Anchors, and the grid fitted to them.
    RowAnchor anchors[ ROWCOUNT_MAX_ANCHORS ];
    int anchorCount = 0;
    void addAnchor( int row, int hole, Vec3 position, float sigmaMm );
    void fitToAnchors( Stream* out );

    // A hold: fixes collected while the probe is kept still (h, R).
    RowHoldPurpose holdPurpose = ROWHOLD_NONE;
    int holdRow = 0; // the row number typed with R
    int holdCount = 0;
    int holdSettle = 0; // fixes let go by at the start of a hold (ROWCOUNT_HOLD_SETTLE_FIXES)
    uint32_t holdStartMs = 0;
    float held[ ROWCOUNT_HOLD_FIXES ]; // how far along each raw fix was, rows
    Vec3 holdSum;                      // raw tip positions, summed
    float holdSigmaSum, holdSigmaMmSum, holdMisfitSum;

    bool haveLastHold = false;
    RowPlace lastHold;

    // The guided calibration: which target is wanted (-1 = not calibrating),
    // and the tap detector.
    int calibrationStep = -1;
    int stillCount = 0; // fixes in a row that stayed put
    int strayCount = 0; // fixes in a row that did not
    Vec3 stillSum;      // ...all of them, to judge "stayed put" against
    Vec3 tapSum;        // ...the second half of them: the measurement
    float tapSigmaSum;
    int tapCount = 0;
    bool haveLastTap = false;
    Vec3 lastTap;
    bool leftStart = true; // the probe has moved from where it was when c was pressed
    Vec3 startPosition;

    void watchForTap( Stream* out );
    void finishCalibration( Stream* out );

    void collect( );
    void finishHold( Stream* out );
    void printReading( Stream* out ) const;
    void countRow( RowPlace place, bool fresh );
    void followTrack( );
};

extern RowCounter& rowCounter;

#endif // ROWCOUNTER_H
