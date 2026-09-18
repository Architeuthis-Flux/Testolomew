// SPDX-License-Identifier: MIT
#ifndef MAGPROBEFIT_H
#define MAGPROBEFIT_H
// ---------------------------------------------------------------------------
// A probe with TWO magnets on its shaft. Where is the tip, and which way does
// the probe point?
//
// Why two. The thing wanted is the TIP, to a fraction of a breadboard row. A
// magnet right at the tip is a few millimetres from the board when it matters
// and its field is sharp; a second magnet up the shaft says which way the
// shaft points, so the tip can be carried from the magnet to the point.
//
// The model is ONE RIGID THING, not two free magnets: both magnets sit on the
// shaft, a fixed distance apart, each magnetised in a fixed direction relative
// to it - at some ANGLE to the shaft (0 = along it, a magnet on the end of a
// rod; 90 = across it, a disc stuck to the side of a pencil), and, for the
// back magnet, TURNED some way round the shaft from the tip one (two discs on
// different sides of the pencil). The unknowns of a fix are the tip magnet's
// position (3), the shaft's direction (2) and, unless both magnets are along
// the shaft, how far the probe is turned about its own shaft (1). As in MagFit
// the field is LINEAR in the two strengths, so for any trial pose they are one
// 2x2 solve (variable projection). Strengths come out signed, so either magnet
// may face either way and no setting says which.
//
// The shape - spacing, both angles, the turn - is LEARNED: with learnShape the
// four shape numbers are searched along with the pose, frame by frame, and
// the caller takes the middle of many frames (MagLocator does). The learned
// shape is then held while tracking.
//
// The shaft direction points from the tip magnet to the back magnet and is
// kept in the upper half-space (a probe does not come up through the board);
// it is carried as (ax, ay) with az = +sqrt(1 - ax^2 - ay^2), which is smooth
// at vertical - the usual way to hold a probe - and only gives out near
// horizontal. The roll is referred to the shaft's x-most perpendicular.
//
// Pure math, no Arduino: `pio test -e native`.
// ---------------------------------------------------------------------------
#include "MagFit.h"

struct MagProbeShape {
    float spacingMm;    // tip magnet centre to back magnet centre, along the shaft
    float tipStrength;  // mT*mm^3, signed; 0 = not known, fitted freely
    float backStrength; // ditto. Known strengths are held softly, as in magFitSolveKnownStrength
    float tipAngleDeg;  // angle between the tip magnet's north pole and the shaft (toward the back): 0 along, 90 across, 180 along the other way
    float backAngleDeg; // ditto for the back magnet
    float backTurnDeg;  // how far round the shaft the back magnet's across part is from the tip magnet's
};

struct MagProbeFitResult {
    bool valid;
    Vec3 tipMagnet;                  // centre of the tip magnet, mm, board frame
    Vec3 backMagnet;                 // centre of the back magnet
    Vec3 axis;                       // unit vector, tip magnet -> back magnet (the shaft)
    Vec3 tipPole;                    // unit vector, the tip magnet's S -> N direction as fitted
    Vec3 backPole;                   // ditto for the back magnet
    float rollDeg;                   // how far the probe is turned about its shaft
    float tipStrength, backStrength; // as fitted, signed, mT*mm^3
    MagProbeShape shape;             // as fitted: the given one, unless learnShape (then strengths positive, angles 0..180, the turn -180..180)
    float residual;                  // RMS misfit per axis, mT
    float signal;                    // RMS reading per axis, mT
    Vec3 sigma;                      // 1-sigma error bar on tipMagnet, mm, per axis (as MagFitResult.sigma)
    float sigmaTiltDeg;              // ...and on the shaft direction, degrees
    int iterations;
};

// The forward model: field at `sensor` from a probe with its tip magnet at
// `tipMagnet`, shaft along `axis` (unit), turned `rollDeg` about it.
Vec3 magProbeField( Vec3 sensor, Vec3 tipMagnet, Vec3 axis, float rollDeg, const MagProbeShape* shape );

// Fit the probe to `count` readings (arguments as magFitSolve). If
// result->valid on entry, its pose (and, with learnShape, its shape) is the
// starting guess; otherwise a single dipole is fitted first (two magnets
// look from a distance like one magnet somewhere between them) and the probe
// is tried along, around and, with learnShape, in every mounting about that.
//
// learnShape also searches the spacing, both angles and the turn, starting
// from the shape's: for measuring a new probe, not for tracking - four more
// unknowns and the fix is noisier for it.
bool magProbeFitSolve( const Vec3* sensors, const Vec3* fields, const bool* use, int count, float maxMisfit,
                       const MagProbeShape* shape, bool learnShape, MagProbeFitResult* result );

#endif // MAGPROBEFIT_H
