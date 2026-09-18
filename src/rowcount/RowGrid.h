// SPDX-License-Identifier: MIT
#ifndef ROWGRID_H
#define ROWGRID_H
// ---------------------------------------------------------------------------
// A breadboard over the sensor array: which row is a position over, how far
// into it, and how sure can we be.
//
// The breadboard is the Jumperless one: two halves either side of a centre
// channel, rows 1-30 along one half and 31-60 along the other, row 31 facing
// row 1 and row 60 facing row 30. Rows are 2.54 mm apart, each is five holes
// running away from the channel, and the two innermost holes are 7.62 mm apart
// across it. Seen with the numbers going up left to right, rows 1-30 are the
// far half - so the half with rows 1-30 is always to the LEFT of the direction
// the numbers go up in, however the breadboard lies.
//
// A position is turned into a RowPlace: how far ALONG, in rows (14.0 is level
// with the centres of rows 14 and 44, 14.5 the boundary with 15 and 45), and
// how far ACROSS from the channel's centre line in mm (positive = the 1-30
// half).
//
// The grid that does it is a straight-line map from the magnet fit's board
// frame to the breadboard's own millimetres. It starts as "rows along +x, 2.54
// apart" and is then FITTED to anchors - fixes taken with the probe in known
// holes (rowGridFit). One anchor slides the breadboard into place. Two or more
// spread along it also give its angle and the scale: the magnet fit reads a
// percent or two long or short, which is half a row over the board's length.
// Anchors spread both along and across (rowGridCalibrationTargets: six rows,
// inner and outer hole of each) allow a different scale each way and a skew as
// well. What is left over after the fit is reported per anchor, and is the
// honest measure of how well rows can be told apart: it is the part of the
// magnet fit's error that no straight-line map removes.
//
// No Arduino in here: `pio test -e native` runs it on the host.
// ---------------------------------------------------------------------------
#include "Vec3.h"

#define ROWGRID_PITCH_MM 2.54f
#define ROWGRID_ROWS_PER_HALF 30
#define ROWGRID_HOLES_PER_ROW 5
#define ROWGRID_INNER_HOLE_MM 3.81f // channel centre line to the first hole: half of 7.62

struct RowGrid {
    // breadboard mm from board-frame mm:
    float ax, ay, a0; // along  = ax x + ay y + a0: mm past the centres of rows 1 and 31, the way the numbers go up
    float cx, cy, c0; // across = cx x + cy y + c0: mm from the channel's centre line, positive = the rows 1-30 half
};

struct RowPlace {
    float along;    // in rows, see above
    float acrossMm; // from the channel's centre line, positive = the half with rows 1-30
};

// A fix taken with the probe in a known hole. hole: 1 = next to the channel ... 5 = outermost.
struct RowAnchor {
    int row;       // 1-60
    int hole;      // 1-5
    Vec3 position; // what the magnet fit said, mm, board frame
    float sigmaMm; // how good that fix was (its error bar across the board); worse anchors count for less
};

enum RowFitKind {
    ROWFIT_NONE,   // nothing usable: grid untouched
    ROWFIT_SHIFT,  // moved into place, angle and scale kept
    ROWFIT_SCALED, // place, angle and one scale
    ROWFIT_FULL    // place, angle, a scale each way and skew
};

struct RowFitReport {
    RowFitKind kind;
    float rmsMm;   // what the anchors still miss by after the fit, mm on the breadboard
    float worstMm; // ...the worst one
    int worst;     // ...and which (index into the anchors)
};

// Rows counting up along +x at 2.54 mm, rows 1 and 31 level with x = row1X,
// the channel's centre line at y = channelY.
void rowGridDefault( RowGrid* grid, float row1X, float channelY );

RowPlace rowGridPlace( const RowGrid* grid, Vec3 position );

// The other way: where in the board frame (z = 0) a place on the breadboard is.
Vec3 rowGridToBoard( const RowGrid* grid, float along, float acrossMm );

// Where a hole is on the breadboard itself.
RowPlace rowGridHolePlace( int row, int hole );

// The row number, 1-60, from how far along and which half. Up to one row past
// either end still counts as the end row (a probe touching the board cannot be
// anywhere else, and the fit reads a little long out there); further is 0 =
// off the breadboard.
int rowGridRow( float along, bool bottomHalf );

// Which hole of the row: 1 = next to the channel ... 5 = outermost. 0 = over
// the channel itself or out past the fifth hole (the rails).
int rowGridHole( float acrossMm );

// A per-axis position error bar (mm) as error bars along (in rows) and across (mm).
float rowGridSigmaAlong( const RowGrid* grid, Vec3 sigmaMm );
float rowGridSigmaAcross( const RowGrid* grid, Vec3 sigmaMm );

// The chance that the nearest row is the true one: the part of a bell curve
// centred on `along`, sigmaRows wide, that lies inside the row's +/-0.5, times
// the chance that the half is right. Dead centre with sigma 0.25 rows gives
// 95 %; on a boundary it is 50 % at best. It is only as honest as the sigmas
// are, and they know about noise, not about a fit that is wrong the same way
// every time - so treat it as an upper limit.
float rowGridConfidence( RowPlace place, float sigmaRows, float sigmaAcrossMm );

// Fit the grid to anchors (weighted least squares, each anchor weighted by
// 1 / sigma^2). How much is fitted depends on what the anchors can pin down:
// spread of 10 mm or more both along and across, and at least four of them:
// the full map. Spread along or across only: place, angle and one scale.
// Otherwise (one anchor, or all in a huddle): the grid is only shifted.
// What the anchors miss by afterwards is in the report; residualMm (may be
// null) gets each anchor's own miss.
RowFitReport rowGridFit( RowGrid* grid, const RowAnchor* anchors, int count, float* residualMm );

// How much longer than life the magnet fit reads along and across the
// breadboard (1.02 = 2 % long), and the angle the rows count up at from board
// +x, degrees - for telling the user what a fit found.
float rowGridScaleAlong( const RowGrid* grid );
float rowGridScaleAcross( const RowGrid* grid );
float rowGridAngleDeg( const RowGrid* grid );

// The holes a guided calibration asks for: six rows - both ends and the middle
// of both halves - each at the hole next to the channel and at the outermost.
#define ROWGRID_CALIBRATION_TARGETS 12
void rowGridCalibrationTarget( int index, int* row, int* hole );

#endif // ROWGRID_H
