// SPDX-License-Identifier: MIT
#ifndef PROBELEDS_H
#define PROBELEDS_H
// ---------------------------------------------------------------------------
// The probe's cursor on the breadboard's own LEDs: a soft spot of light that
// follows the probe and shows, by its size and brightness, how sure the
// tracker is. Sharp fix over a hole: one bright LED and its neighbours barely
// lit. A far fix: the same spot, as wide as its error bar (held at a
// limit), in the same colours - one colours/height mapping for everything
// (2026-09-27; a dim purple glow of its own until then). Nothing: dark.
//
// The renderer knows nothing about pixel wiring. A LedLayout is a table of
// where every LED sits in BREADBOARD terms - along (in rows, 1..30, the same
// number for both halves) and across (mm from the centre line, + on the rows
// 1-30 half) - built once for the board in hand (V6: six holes a side meeting
// at the centre and two rails a side, 16 x 30; V5: five holes a side across a
// 7.62 mm channel and the four rails as JumperlOS wires them). Rendering is
// then the same for any board: for each LED, how much of a bell curve centred
// on the cursor falls on it. The total light is held roughly constant, so a
// wider bell is a dimmer one; confidence dims it further; the height above
// the board tints it.
//
// When the cursor is where the probe POINTS rather than what it is over, a
// tail runs from the cursor back toward the point for tailLength of the
// way - brightest at the cursor, in its colour, fading to the tail hue -
// so the two are never confused (2026-09-26: it ran the whole way from
// under the tip, and was too long).
//
// A moving cursor is swept from where it was last frame to where it is
// (half-row steps, full at the head, PROBELED_SWEEP_TAIL of it at the old
// end): a hand at writing speed moves a row and a half a frame, and the
// bell alone left the LED between two frames' cursors at 6 %, a dotted
// line ("gaps in the LEDs", 2026-09-26). Only from a cursor seen within
// PROBELED_SWEEP_S: a jump after a pause is a jump, not a stroke.
//
// Per-LED smoothing (fast attack, slower decay) leaves a short comet's tail
// behind a moving cursor and keeps the picture calm; the tracker's 1-Euro
// filter has already removed the jitter. A longer decay ("fade") is a
// longer comet.
//
// Looks, each a lever in the style and in the menu's LEDs and colours
// pages: a colour SCHEME - what drives the hue: classic (white on the
// board, blue in the air), height, how sure the row is, the direction the
// probe leans (aim), or a rainbow along the board with time - with the
// whole wheel mapped the style's way (hueTurns turns of it over the scale,
// from hueStartDeg; under a turn a chunk of the spectrum as a gradient,
// over it several rainbows; the height scale liftFullMm) and, in every
// scheme but classic, which keeps its own white-to-blue ramp over the
// height scale and reads none of the wheel's levers, the cursor WHITE with
// the point on the board and all the scheme's colour from colourByMm up.
// What DIMS the cursor (brightBy, by
// brightAmount at the data's far end) and what raises the SPARKLE's density
// (sparkleBy) are chosen from the same data: how unsure the row is, the
// height, the tilt, the speed, or nothing. FULL PEAK (the brightest LED is always the peak,
// whatever the bell's width - otherwise the total light is held and a wide
// bell is a dim one), BLOOM (a wide soft halo round the cursor), SPARKLE
// (twinkles inside the glow, near-white with a random tint each, few with
// the point on the board and many with it lifted), PULSE (the cursor breathes) and a TOUCH
// RING (a ring spreads from the cursor when the point lands on the board).
//
// Output is linear intensity 0..1 per LED with a colour, turned into 8-bit
// RGB through a gamma curve by probeLedRgb(). What drives the LEDs (a WS2812
// chain, the LCD preview, a serial stream to a V5) is not in here.
//
// No Arduino in here; host-tested (test/test_probeleds).
// ---------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>

#define PROBELED_MAX 512
#define PROBELED_ROWS 30

// LED kinds, for drivers that care (the rails may be wired elsewhere).
#define PROBELED_HOLE 0
#define PROBELED_RAIL 1

struct LedLayout {
    int count;
    float along[ PROBELED_MAX ];    // rows (1 = row 1/31 ... 30 = row 30/60), fractional for rail LEDs between rows
    float acrossMm[ PROBELED_MAX ]; // mm from the centre line, + = the rows 1-30 half
    uint8_t kind[ PROBELED_MAX ];
    int16_t row[ PROBELED_MAX ]; // the breadboard row this LED belongs to (1..60), 0 for rails
    int8_t hole[ PROBELED_MAX ]; // 1 = next to the channel ... 6, 0 for rails
};

// V6: rows 1-30 along, six holes a side at 2.54 mm pitch meeting at the centre
// (innermost holes 2.54 mm apart), two rail LEDs a side beyond the sixth hole.
// ASSUMPTION: the rail LEDs sit at hole pitch beyond the outermost hole with
// one empty position between (PROBELED_V6_RAIL_MM); fix from the V6 layout.
#define PROBELED_V6_HOLES 6
#define PROBELED_V6_INNER_MM 1.27f
#define PROBELED_V6_RAIL_MM 19.05f // first rail LED; the second is one pitch further
void ledLayoutV6( LedLayout* layout );

// V5 (JumperlOS LED_COUNT 300 + 100 rail): five holes a side, innermost 7.62 mm
// apart, and 25 rail LEDs per rail in five groups of five with a gap of one
// position between groups (railsToPixelMap in JumperlOS's LEDs.h).
// Positions read off the board (JumperlessV5r8.kicad_pcb, the WS2812B
// footprints): row 1 is at x 32.616, row 30 at 106.276 (2.54 pitch); the
// rail LEDs run from x 33.874 in groups of five at 2.54 with 15.24 from
// group start to group start, so each sits HALF A PITCH along from a row
// (rail LED n is at row 1.5 + 6 (n / 5) + n % 5); the inner rail of each
// pair is 21.59 mm from the channel's centre line (three pitches beyond
// the fifth hole, which is at 13.97) and the outer 24.13. INFERRED from the
// reference designators, to be confirmed by the `n` chase: the chain order
// is rail 0 (pixels 300-324) the top outer rail (D1611-D1635, the rows 1-30
// side, y 102.90), rail 1 the top inner (D1636-D1660), rail 2 the bottom
// inner (D1711-D1735), rail 3 the bottom outer (D1736-D1760), each from the
// row-1 end. The LED order within a row: JumperlOS numbers a row's five
// LEDs (row-1)*5+column and reverses the column for rows 31-60; its
// screenMap (Graphics.cpp) puts column 0 of rows 1-30 at the OUTER edge, so
// column 0 is the outer hole on both halves (PROBELED_V5_COLUMN0_IS_INNER
// 0), which the board agrees with (D1011, row 1 column 0, is the outermost
// top LED; D1311, row 31 column 0, the innermost bottom one). If the cursor
// comes out mirrored across a row on a real V5, flip that one define. (The
// V5 port paints through JumperlOS's overlay, which takes rows and holes,
// so it does not depend on this.)
#define PROBELED_V5_HOLES 5
#define PROBELED_V5_INNER_MM 3.81f
#define PROBELED_V5_RAIL_INNER_MM 21.59f
#define PROBELED_V5_RAIL_OUTER_MM 24.13f
#define PROBELED_V5_RAIL_ALONG0 1.5f // rail LED 0 of each rail, in rows
#define PROBELED_V5_COLUMN0_IS_INNER 0
void ledLayoutV5( LedLayout* layout );

// The V5 pixel index of LED i of a V5 layout, as JumperlOS numbers them:
// holes 0..299, rails 300..399 (rail 0 = the four rails in railsToPixelMap
// order). -1 if not a V5 LED.
int ledLayoutV5Pixel( const LedLayout* layout, int i );
// The rail LEDs alone, for a V5 whose rails are a strip of their own (hardware
// revision 4+): rail r LED n = 25 r + n, -1 for a hole.
int ledLayoutV5RailPixel( const LedLayout* layout, int i );

// What to draw.
enum ProbeLedState {
    PROBELED_NONE,     // nothing to show
    PROBELED_ROUGH,    // a far probe (drawn like any other since 2026-09-27; the state is for the stream and the screen)
    PROBELED_COASTING, // carried on through a gap: as tracking, a little dimmer
    PROBELED_TRACKING
};

struct ProbeLedInput {
    ProbeLedState state;
    float along, acrossMm;          // the cursor, breadboard terms
    float sigmaRows, sigmaAcrossMm; // its 1-sigma bar
    float confidence;               // 0..1 that the nearest row is right
    float heightMm;                 // the point's height above the surface (0 = touching)
    bool haveUnder;                 // the point is somewhere else than the cursor (pointed mode, lifted)
    float underAlong, underAcrossMm;
    float tiltDeg;  // the probe's lean from vertical (0 = straight up)...
    float aimDeg;   // ...and which way it leans, degrees round the board (0 = +x, along the rows)
    float speedMmS; // how fast the magnet is moving
    float presence; // 0..1: how far the strongest reading is above the presence level - 0 at half the lever, 1 at one and a half times it; the peak scales by it, so the far edge fades rather than switches (2026-09-28)
};

enum ProbeLedScheme {
    PROBELED_SCHEME_CLASSIC, // white on the board, blue lifted (touch/lift colours below)
    PROBELED_SCHEME_HEIGHT,  // hue by height: hueTurns of the wheel from the board to liftFullMm
    PROBELED_SCHEME_SURE,    // hue by how sure the row is: the wheel from a toss-up to certain
    PROBELED_SCHEME_AIM,     // hue by which way the probe leans (one turn of the wheel is the compass, whatever hueTurns says), white standing straight, all colour from PROBELED_AIM_FULL_DEG of lean
    PROBELED_SCHEME_RAINBOW, // hue runs along the board (hueTurns over its 30 rows) and round with time
    PROBELED_SCHEME_COUNT
};
extern const char* const probeLedSchemeNames[ PROBELED_SCHEME_COUNT ];

// The data a look can follow (brightBy, sparkleBy), each 0 at its near end
// and 1 at its far end: how unsure the row is (1 = a toss-up), the height
// (1 at liftFullMm), the tilt (1 at PROBELED_TILT_FULL_DEG), the speed (1 at
// PROBELED_SPEED_FULL_MM_S), or nothing (always 0).
enum ProbeLedData {
    PROBELED_DATA_NONE,
    PROBELED_DATA_SURE,
    PROBELED_DATA_HEIGHT,
    PROBELED_DATA_TILT,
    PROBELED_DATA_SPEED,
    PROBELED_DATA_COUNT
};
extern const char* const probeLedDataNames[ PROBELED_DATA_COUNT ];

// How a wheel scheme's colour meets the board: WHITE - white below
// PROBELED_WHITE_BELOW_MM, blended to the scheme's colour by colourByMm (a
// flat white section at the bottom of the scale); FADE - the scheme's
// colour at colourByMm fading smoothly to white AT the board, no flat
// section (Kevin, 2026-09-28: "smoothly go toward white on the board");
// COLOR - the scheme's colour at every height, no white. The touch ring is
// white whichever.
enum ProbeLedOnBoard {
    PROBELED_ONBOARD_WHITE,
    PROBELED_ONBOARD_FADE,
    PROBELED_ONBOARD_COLOR,
    PROBELED_ONBOARD_COUNT
};
extern const char* const probeLedOnBoardNames[ PROBELED_ONBOARD_COUNT ];
#define PROBELED_TILT_FULL_DEG 60.0f
#define PROBELED_SPEED_FULL_MM_S 200.0f
#define PROBELED_AIM_FULL_DEG 30.0f // the aim scheme's colour is all in at this much lean

#define PROBELED_WHITE_BELOW_MM 1.5f // the cursor is white with the point this close to the board, in every scheme (all the scheme's colour from the style's colourByMm up; classic keeps its own ramp to liftFullMm)
#define PROBELED_SPARKLE_TINT 0.25f  // a sparkle is white with this much of a random hue in it
#define PROBELED_SPARKLE_FLOOR 0.1f  // the sparkle density on the board, as a fraction of the lever; all of it at liftFullMm
#define PROBELED_PULSE_PERIOD_S 1.5f
#define PROBELED_RING_S 0.14f          // a touch ring's life: a flash, not a ripple (2026-09-19: "super fast", "faster", "a bit smaller", "slightly slower")
#define PROBELED_RING_ROWS_PER_S 36.0f // ...and how fast it spreads: 5 rows out in its life, 7 frames at 50 Hz
#define PROBELED_RING_DECAY_S 0.02f    // the LEDs it passed go dark this fast (the cursor's own decay would fill the ring in)
#define PROBELED_RING_WIDTH_ROWS 1.5f  // the ring's half-width: the LED on the ring at full, the next ones out fading to nothing
// Moving on to another hole while down rings again - but only once the point
// is PROBELED_RING_STEP_ROWS from where the last ring started (a point on the
// line between two holes flips between them every frame otherwise) and the
// last ring is PROBELED_RING_REARM_S old.
#define PROBELED_RING_STEP_ROWS 0.8f
#define PROBELED_RING_REARM_S 0.1f
// The bell is never wider than this (a far or lost probe's error bar is the
// whole board): held at it, a far probe is a broad patch - at the peak with
// fullPeak - that the chain's current budget dims as a whole. (Until
// 2026-09-27 the floor faded out with the width and a far fix had a dim
// purple glow of its own; Kevin: "make the rough above brighter ... a
// single colours / height mapping for everything".) The bloom's halo
// widens the same limits.
#define PROBELED_MAX_SIGMA_ROWS 3.0f
#define PROBELED_MAX_SIGMA_ACROSS_MM 7.0f
#define PROBELED_TOUCH_MM 2.0f     // the point is "on the board" below this (as the paint's touch, 2026-09-19)
#define PROBELED_SPOT_HEIGHT_MM 10.0f // "spot by height" counts the point's lift in these (up to PROBELED_SPOT_HEIGHT_MAX of them)
#define PROBELED_ONE_PIXEL_ROWS 1.0f  // the LED nearest the cursor (within this) is always lit at the bell's peak: a pin between two holes lights the nearer one, never nothing (2026-09-28)
#define PROBELED_SPOT_HEIGHT_MAX 6.0f
#define PROBELED_SWEEP_S 0.25f     // a cursor seen this recently is swept to the new one (a fast hand's refused frames are a few; a cold start across the board is a jump)
#define PROBELED_SWEEP_TAIL 0.6f   // the sweep's old end, as a fraction of the head (it was lit last frame, and has decayed since: a comet)
#define PROBELED_LIFTED_MM 5.0f    // ...and was "off it" above this (a ring needs the one after the other)

// Colours, linear 0..255 per channel (gamma is applied on output).
struct ProbeLedStyle {
    uint8_t touchR, touchG, touchB; // cursor colour with the point on the board (classic scheme; white)
    uint8_t liftR, liftG, liftB;    // ...and lifted well above it (blended by height)
    bool tail;                      // the pointed-mode tail at all (off by default, 2026-09-27: at full length and brightness it read as a bar along the lean - "a line showing the north and south poles")
    float tailHueDeg;               // the tail's colour at its far end, in pointed mode (its near end is the cursor's)
    float tailLength;               // how far back toward the point the tail runs, as a fraction of the way (1 = to the point)
    float tailBright;               // the tail's near end, as a fraction of the cursor's peak
    float peak;                     // brightest any LED gets, 0..1
    float minSigmaRows;             // the bell is never narrower than this (one hole lights, neighbours faint)...
    float minSigmaAcrossMm;
    float spot;         // ...times this (the menu's "spot": the floor's size, 1 = one hole lit, its neighbours faint; 0.3 a pin)...
    float errorWidth;   // the fix's error bar times this is the other floor of the width: 0 = the bar is not shown, the spot is "spot" wide whatever the fit knows; 1 = the bar as it is; 2 = twice (the menu's "error width", 2026-09-28)
    float falloff;      // the bell's shape: 1 a Gaussian; higher a flatter top and a sharper edge (3 near a disc); lower a peaked centre with a wide skirt (the menu's "falloff", 2026-09-28)
    float spotByHeight; // ...and this much wider again per PROBELED_SPOT_HEIGHT_MM of the point's lift (the menu's "spot by height": 0 = the same size at any height, 1 = twice as wide 10 mm up and three times at 20 - a flashlight's cone, the peak kept; over the colours page's height scale until 2026-09-28, 60 mm on the bench: nothing to see at working heights)
    float liftFullMm;      // the height scale: where the height's wheel ends (and classic's colour is all "lift"), mm
    float attackS, decayS; // per-LED smoothing time constants
    // The looks.
    int scheme;         // ProbeLedScheme
    float hueTurns;     // turns of the wheel over the scheme's scale - the height (held at the scale's end above it), the sureness, the board's rows (under 1 a chunk of the spectrum, over 1 several rainbows; not the aim's compass, not classic)
    float hueStartDeg;  // where the wheel starts (0 red, 120 green, 240 blue)
    float colourByMm;   // the scheme's colour is all in from this height (white on the board below PROBELED_WHITE_BELOW_MM)...
    int onBoard;        // ProbeLedOnBoard: how a wheel scheme meets the board (the menu's "on board", 2026-09-28; classic keeps its own white-to-blue ramp)
    int brightBy;       // ProbeLedData: what dims the cursor...
    float brightAmount; // ...by this much at the data's far end, SIGNED: +1 doubles the peak there (never past the full peak), -1 takes it to nothing (-0.5 by unsure: a toss-up row at half)
    int sparkleBy;      // ProbeLedData: what raises the sparkle's density from PROBELED_SPARKLE_FLOOR of the lever to all of it (nothing: all of it always)
    bool fullPeak;  // the brightest LED is always `peak`; else the bell keeps its total light and widens dimmer
    float bloom;    // 0..1, a wide soft halo round the cursor
    float sparkle;  // 0..1, white twinkles in the glow
    float pulse;    // 0..1, how deeply the cursor breathes
    bool touchRing;  // a ring spreads from the cursor when the point lands...
    bool ringRepeat; // ...and, if asked, again at every new hole it slides to while down (2026-09-27: it always did; Kevin: "make the touch ring thing not repeat")
};

void probeLedDefaultStyle( ProbeLedStyle* style );

struct ProbeLedFrame {
    int count;
    float level[ PROBELED_MAX ];                                     // linear intensity 0..1, smoothed
    uint8_t r[ PROBELED_MAX ], g[ PROBELED_MAX ], b[ PROBELED_MAX ]; // linear colour of that light
    float target[ PROBELED_MAX ];                                    // this frame's unsmoothed intensity
    // For the looks: the clock, the sparkle's dice, the touch ring.
    float timeS;
    uint32_t rng;
    float lastHeightMm;
    bool wasLifted;
    float ringAgeS; // < 0: no ring running
    float ringAlong, ringAcrossMm;
    int ringHole; // the LED the point was over at the last ring (-1 = none): moving on to another, still down, rings again
    float ring[ PROBELED_MAX ]; // the ring's own layer OVER the LEDs, at the ring's own decay (PROBELED_RING_DECAY_S): what it passes over is left as it was (2026-09-28: it lit the LEDs themselves and cut a comet where it passed)
    uint8_t ringR, ringG, ringB; // its colour (the cursor's when it started)
    bool haveLast; // the cursor last frame (a tracked or coasting one), for the sweep
    float lastAlong, lastAcrossMm, lastTimeS;
};

// Hue (0..360) at full saturation to linear RGB.
void probeLedHue( float hueDeg, float brightness, uint8_t* r, uint8_t* g, uint8_t* b );
// The cursor's colour under the style's scheme for this input (its height,
// confidence, lean; timeS for the rainbow) - white on the board, the
// scheme's colour as it lifts. What the renderer paints the cursor with;
// the colours page's preview runs it over the data.
void probeLedCursorColour( const ProbeLedStyle* style, const ProbeLedInput* in, float timeS, uint8_t* r, uint8_t* g, uint8_t* b );

void probeLedClear( ProbeLedFrame* frame, int count );

// Paint: a layer of lit LEDs that stays (play modes put it there - what the
// tip drew, a target); the cursor is drawn over it, the brighter of the two
// showing on each LED.
struct ProbeLedPaint {
    float level[ PROBELED_MAX ]; // 0 = nothing painted here
    uint8_t r[ PROBELED_MAX ], g[ PROBELED_MAX ], b[ PROBELED_MAX ];
};
void probeLedPaintClear( ProbeLedPaint* paint );

// The brush as the cursor (paint mode): a ring of LEDs just outside what the
// brush would paint - the four neighbours of the one LED for radius 0, a +
// with the centre dark; the outline a row beyond the edge for a wider one -
// in the paint's colour at its level, with nothing else of the cursor's look
// (no glow, bloom, sparkle or height colour) and no fade, so the LEDs inside
// the ring are the ones a touch would paint. Erasing shows a dim white ring.
struct ProbeLedBrush {
    bool active;
    int radiusRows;
    float level;
    uint8_t r, g, b;
};
// The LED nearest to a breadboard place, if one is within `withinRows` of it
// (across in rows too, at the hole pitch); -1 if none.
int ledLayoutNearest( const LedLayout* layout, float along, float acrossMm, float withinRows );

// Render one frame: the bell(s) into target[], then level[] follows with the
// style's attack/decay over dtS. Colours are set where target > 0 and kept
// elsewhere (so a fading LED keeps its colour). `paint` (may be null) goes
// under the cursor.
void probeLedRender( const LedLayout* layout, const ProbeLedInput* in, const ProbeLedStyle* style, float dtS, ProbeLedFrame* frame, const ProbeLedPaint* paint = nullptr,
                     const ProbeLedBrush* brush = nullptr );

// LED i as shown: its level and colour, or the ring's where the ring is on top.
void probeLedShown( const ProbeLedFrame* frame, int i, float* level, uint8_t* r, uint8_t* g, uint8_t* b );
// The 8-bit RGB to send for LED i (gamma 2.2 on the level as shown).
void probeLedRgb( const ProbeLedFrame* frame, int i, uint8_t* r, uint8_t* g, uint8_t* b );

#endif // PROBELEDS_H
