// SPDX-License-Identifier: MIT
#include "MagProbeFit.h"

#include <math.h>

// The pose: q = { x, y, z, ax, ay, roll, spacing, tipAngle, backAngle,
// backTurn } (angles in radians). Which of them are searched depends on the
// call: the first five always, roll unless both magnets are along the shaft,
// the last four only when the shape is being learned. `active` lists the
// searched entries.
#define POSE_SIZE 10
#define Q_AX 3
#define Q_AY 4
#define Q_ROLL 5
#define Q_SPACING 6
#define Q_TIP_ANGLE 7
#define Q_BACK_ANGLE 8
#define Q_BACK_TURN 9

#define LM_MAX_ITERATIONS 30
#define LM_MAX_ITERATIONS_TRACKING 4 // from a warm start: the next frame carries on from wherever this one got to
#define LM_MAX_TRIES 8
#define AXIS_LIMIT 0.95f // ax^2 + ay^2 may not pass this: 77 degrees from vertical
#define SPACING_MIN_MM 3.0f
#define SPACING_MAX_MM 80.0f
#define ROLL_MATTERS_RAD 0.09f // an angle to the shaft under 5 degrees: as good as along it

#define DEG ( (float)M_PI / 180.0f )

// Per pose entry: the step the numeric Jacobian takes, the longest step one
// iteration may take, and the step below which it has converged (a fiftieth
// of a millimetre, a thirtieth of a degree: far under the noise).
static const float jacobianStep[ POSE_SIZE ] = { 0.05f, 0.05f, 0.05f, 0.002f, 0.002f, 0.002f, 0.05f, 0.002f, 0.002f, 0.002f };
static const float stepLimit[ POSE_SIZE ] = { 20.0f, 20.0f, 20.0f, 0.3f, 0.3f, 0.5f, 5.0f, 0.3f, 0.3f, 0.5f };
static const float doneStep[ POSE_SIZE ] = { 0.02f, 0.02f, 0.02f, 0.0005f, 0.0005f, 0.002f, 0.02f, 0.001f, 0.001f, 0.002f };

// The strengths are solved in units of this many mT*mm^3, so that the two
// unit fields below are numbers near the readings' size and their squares do
// not crowd single precision (the same reason as MagFit's KERNEL_SCALE).
#define STRENGTH_UNIT 1000.0f

#define KNOWN_STRENGTH_WEIGHT 5.0f // a 1 % strength error costs as much as a 5 % field misfit

struct ProbeProblem {
    const Vec3* sensors;
    const Vec3* fields;
    const bool* use;
    int count;
    int used;
    float xMin, xMax, yMin, yMax;
    float fieldNorm; // sqrt( sum of squares of the readings used )
    float knownTip;  // strengths to hold (0 = free), in STRENGTH_UNITs
    float knownBack;
    int active[ POSE_SIZE ]; // the pose entries searched
    int params;
};

static bool sensorUsed( const ProbeProblem* pp, int i ) {
    return pp->use == nullptr || pp->use[ i ];
}

static Vec3 axisOf( const float* q ) {
    float s = q[ Q_AX ] * q[ Q_AX ] + q[ Q_AY ] * q[ Q_AY ];
    Vec3 a = { q[ Q_AX ], q[ Q_AY ], sqrtf( s < 1.0f ? 1.0f - s : 0.0f ) };
    return a;
}

// A direction perpendicular to the shaft, turned `roll` about it. The
// reference for roll is the shaft's x-most perpendicular, which is well
// defined for any shaft short of horizontal.
static Vec3 acrossOf( Vec3 axis, float roll ) {
    Vec3 e1 = { 1.0f - axis.x * axis.x, -axis.x * axis.y, -axis.x * axis.z }; // x-hat with its shaft part removed
    float n = sqrtf( e1.x * e1.x + e1.y * e1.y + e1.z * e1.z );
    if ( n < 1e-6f ) {
        e1 = { 0, 1, 0 };
        n = 1.0f;
    }
    e1 = { e1.x / n, e1.y / n, e1.z / n };
    Vec3 e2 = { axis.y * e1.z - axis.z * e1.y, axis.z * e1.x - axis.x * e1.z, axis.x * e1.y - axis.y * e1.x }; // shaft x e1
    float c = cosf( roll ), s = sinf( roll );
    Vec3 across = { c * e1.x + s * e2.x, c * e1.y + s * e2.y, c * e1.z + s * e2.z };
    return across;
}

// The way a magnet points: `angle` from the shaft, its across part turned `roll` about it.
static Vec3 poleOf( Vec3 axis, float angle, float roll ) {
    Vec3 across = acrossOf( axis, roll );
    float c = cosf( angle ), s = sinf( angle );
    Vec3 pole = { c * axis.x + s * across.x, c * axis.y + s * across.y, c * axis.z + s * across.z };
    return pole;
}

static void poseOf( const MagProbeShape* shape, float* q ) {
    q[ Q_SPACING ] = shape->spacingMm;
    q[ Q_TIP_ANGLE ] = shape->tipAngleDeg * DEG;
    q[ Q_BACK_ANGLE ] = shape->backAngleDeg * DEG;
    q[ Q_BACK_TURN ] = shape->backTurnDeg * DEG;
}

Vec3 magProbeField( Vec3 sensor, Vec3 tipMagnet, Vec3 axis, float rollDeg, const MagProbeShape* shape ) {
    float roll = rollDeg * DEG;
    Vec3 tipDir = poleOf( axis, shape->tipAngleDeg * DEG, roll );
    Vec3 backDir = poleOf( axis, shape->backAngleDeg * DEG, roll + shape->backTurnDeg * DEG );
    Vec3 back = { tipMagnet.x + shape->spacingMm * axis.x, tipMagnet.y + shape->spacingMm * axis.y, tipMagnet.z + shape->spacingMm * axis.z };
    Vec3 a = magFitDipoleField( sensor, tipMagnet, { shape->tipStrength * tipDir.x, shape->tipStrength * tipDir.y, shape->tipStrength * tipDir.z } );
    Vec3 b = magFitDipoleField( sensor, back, { shape->backStrength * backDir.x, shape->backStrength * backDir.y, shape->backStrength * backDir.z } );
    Vec3 sum = { a.x + b.x, a.y + b.y, a.z + b.z };
    return sum;
}

// For a trial pose: the two best-fitting strengths (one 2x2 solve), the
// residuals they leave (3 per sensor, zeros for unused ones, then one row per
// held strength; may be null) and the sum of squares of all rows. *fieldCost
// (may be null) gets the field rows' share: that is the misfit.
static float evaluate( const ProbeProblem* pp, const float* q, float strengths[ 2 ], float* residuals, float* fieldCost ) {
    static Vec3 tipField[ MAGFIT_MAX_SENSORS ];  // per unit of tip strength
    static Vec3 backField[ MAGFIT_MAX_SENSORS ]; // per unit of back strength

    Vec3 tip = { q[ 0 ], q[ 1 ], q[ 2 ] };
    Vec3 axis = axisOf( q );
    Vec3 tipDir = poleOf( axis, q[ Q_TIP_ANGLE ], q[ Q_ROLL ] );
    Vec3 backDir = poleOf( axis, q[ Q_BACK_ANGLE ], q[ Q_ROLL ] + q[ Q_BACK_TURN ] );
    Vec3 tipMoment = { STRENGTH_UNIT * tipDir.x, STRENGTH_UNIT * tipDir.y, STRENGTH_UNIT * tipDir.z };
    Vec3 backMoment = { STRENGTH_UNIT * backDir.x, STRENGTH_UNIT * backDir.y, STRENGTH_UNIT * backDir.z };
    Vec3 back = { tip.x + q[ Q_SPACING ] * axis.x, tip.y + q[ Q_SPACING ] * axis.y, tip.z + q[ Q_SPACING ] * axis.z };

    float tt = 0, tb = 0, bb = 0, ty = 0, by = 0;
    for ( int i = 0; i < pp->count; i++ ) {
        if ( !sensorUsed( pp, i ) ) {
            continue;
        }
        Vec3 t = tipField[ i ] = magFitDipoleField( pp->sensors[ i ], tip, tipMoment );
        Vec3 b = backField[ i ] = magFitDipoleField( pp->sensors[ i ], back, backMoment );
        const Vec3& y = pp->fields[ i ];
        tt += t.x * t.x + t.y * t.y + t.z * t.z;
        tb += t.x * b.x + t.y * b.y + t.z * b.z;
        bb += b.x * b.x + b.y * b.y + b.z * b.z;
        ty += t.x * y.x + t.y * y.y + t.z * y.z;
        by += b.x * y.x + b.y * y.y + b.z * y.z;
    }

    // A held strength is one more equation, w ( s - known ) / known = 0.
    float wTip = pp->knownTip != 0.0f ? KNOWN_STRENGTH_WEIGHT * pp->fieldNorm / fabsf( pp->knownTip ) : 0.0f;
    float wBack = pp->knownBack != 0.0f ? KNOWN_STRENGTH_WEIGHT * pp->fieldNorm / fabsf( pp->knownBack ) : 0.0f;
    tt += wTip * wTip;
    ty += wTip * wTip * pp->knownTip;
    bb += wBack * wBack;
    by += wBack * wBack * pp->knownBack;

    // A magnet no sensor can see has no say (its strength would be pure noise,
    // and huge): a whisker on the diagonal keeps it small instead.
    float whisker = 1e-6f * ( tt + bb ) + 1e-20f;
    tt += whisker;
    bb += whisker;
    float det = tt * bb - tb * tb;
    if ( det <= 0.0f ) {
        return 1e30f;
    }
    float sTip = ( ty * bb - by * tb ) / det;
    float sBack = ( by * tt - ty * tb ) / det;

    float cost = 0.0f;
    for ( int i = 0; i < pp->count; i++ ) {
        float e[ 3 ] = { 0, 0, 0 };
        if ( sensorUsed( pp, i ) ) {
            e[ 0 ] = pp->fields[ i ].x - sTip * tipField[ i ].x - sBack * backField[ i ].x;
            e[ 1 ] = pp->fields[ i ].y - sTip * tipField[ i ].y - sBack * backField[ i ].y;
            e[ 2 ] = pp->fields[ i ].z - sTip * tipField[ i ].z - sBack * backField[ i ].z;
            cost += e[ 0 ] * e[ 0 ] + e[ 1 ] * e[ 1 ] + e[ 2 ] * e[ 2 ];
        }
        if ( residuals != nullptr ) {
            residuals[ 3 * i + 0 ] = e[ 0 ];
            residuals[ 3 * i + 1 ] = e[ 1 ];
            residuals[ 3 * i + 2 ] = e[ 2 ];
        }
    }
    if ( fieldCost != nullptr ) {
        *fieldCost = cost;
    }
    float heldTip = wTip * ( sTip - pp->knownTip ), heldBack = wBack * ( sBack - pp->knownBack );
    if ( residuals != nullptr ) {
        residuals[ 3 * pp->count + 0 ] = heldTip;
        residuals[ 3 * pp->count + 1 ] = heldBack;
    }
    if ( strengths != nullptr ) {
        strengths[ 0 ] = sTip;
        strengths[ 1 ] = sBack;
    }
    return cost + heldTip * heldTip + heldBack * heldBack;
}

static float clampf( float v, float lo, float hi ) {
    return v < lo ? lo : ( v > hi ? hi : v );
}

static void clampPose( const ProbeProblem* pp, float* q ) {
    q[ 0 ] = clampf( q[ 0 ], pp->xMin, pp->xMax );
    q[ 1 ] = clampf( q[ 1 ], pp->yMin, pp->yMax );
    q[ 2 ] = clampf( q[ 2 ], MAGFIT_Z_MIN, MAGFIT_Z_MAX );
    float s = q[ Q_AX ] * q[ Q_AX ] + q[ Q_AY ] * q[ Q_AY ];
    if ( s > AXIS_LIMIT ) {
        float k = sqrtf( AXIS_LIMIT / s );
        q[ Q_AX ] *= k;
        q[ Q_AY ] *= k;
    }
    q[ Q_SPACING ] = clampf( q[ Q_SPACING ], SPACING_MIN_MM, SPACING_MAX_MM );
}

#define ROWS ( 3 * MAGFIT_MAX_SENSORS + 2 )

// The Jacobian of the residuals at q, numerically, one column per searched
// entry. Because evaluate() re-solves the strengths at every nudged pose, this
// is the variable-projection Jacobian.
static void jacobianAt( const ProbeProblem* pp, const float* q, const float* e0, float jac[ ROWS ][ MAGFIT_MAX_PARAMS ] ) {
    static float e1[ ROWS ];
    int rows = 3 * pp->count + 2;
    for ( int k = 0; k < pp->params; k++ ) {
        int entry = pp->active[ k ];
        float nudged[ POSE_SIZE ];
        for ( int j = 0; j < POSE_SIZE; j++ ) {
            nudged[ j ] = q[ j ];
        }
        nudged[ entry ] += jacobianStep[ entry ];
        evaluate( pp, nudged, nullptr, e1, nullptr );
        for ( int r = 0; r < rows; r++ ) {
            jac[ r ][ k ] = ( e1[ r ] - e0[ r ] ) / jacobianStep[ entry ];
        }
    }
}

// Levenberg-Marquardt over the searched pose entries, from q (updated in
// place). Returns the cost; jtjOut is the last J^T J, for the error bar.
static float refine( const ProbeProblem* pp, float* q, int maxIterations, int* iterationsOut, float jtjOut[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ] ) {
    static float e0[ ROWS ];
    static float e1[ ROWS ];
    static float jac[ ROWS ][ MAGFIT_MAX_PARAMS ];

    int rows = 3 * pp->count + 2;
    int params = pp->params;
    clampPose( pp, q );
    float cost = evaluate( pp, q, nullptr, e0, nullptr );
    float lambda = 1e-2f;
    int iter = 0;

    for ( ; iter < maxIterations; iter++ ) {
        jacobianAt( pp, q, e0, jac );
        float jtj[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ] = { { 0 } };
        float jte[ MAGFIT_MAX_PARAMS ] = { 0 };
        for ( int r = 0; r < rows; r++ ) {
            for ( int a = 0; a < params; a++ ) {
                jte[ a ] += jac[ r ][ a ] * e0[ r ];
                for ( int c = 0; c < params; c++ ) {
                    jtj[ a ][ c ] += jac[ r ][ a ] * jac[ r ][ c ];
                }
            }
        }
        for ( int a = 0; a < params; a++ ) { // kept for the error bar: at convergence it is the answer's own
            for ( int c = 0; c < params; c++ ) {
                jtjOut[ a ][ c ] = jtj[ a ][ c ];
            }
        }

        bool accepted = false;
        bool done = false;
        for ( int tries = 0; tries < LM_MAX_TRIES && !accepted; tries++ ) {
            float a[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ];
            float d[ MAGFIT_MAX_PARAMS ];
            for ( int i = 0; i < params; i++ ) {
                for ( int j = 0; j < params; j++ ) {
                    a[ i ][ j ] = jtj[ i ][ j ];
                }
                // Marquardt scaling, plus a floor so an entry the readings do
                // not care about (roll with both magnets along) does not make
                // the system singular.
                a[ i ][ i ] += lambda * jtj[ i ][ i ] + 1e-9f;
                d[ i ] = -jte[ i ];
            }
            if ( !magFitSolveLinear( a, d, params ) ) {
                lambda *= 4.0f;
                continue;
            }
            // Millimetres, direction cosines and radians are different animals:
            // the step is shrunk until every entry is within its own limit.
            float k = 1.0f;
            for ( int i = 0; i < params; i++ ) {
                float limit = stepLimit[ pp->active[ i ] ];
                if ( fabsf( d[ i ] ) * k > limit ) {
                    k = limit / fabsf( d[ i ] );
                }
            }
            float trial[ POSE_SIZE ];
            for ( int j = 0; j < POSE_SIZE; j++ ) {
                trial[ j ] = q[ j ];
            }
            done = true;
            for ( int i = 0; i < params; i++ ) {
                trial[ pp->active[ i ] ] += k * d[ i ];
                if ( fabsf( k * d[ i ] ) >= doneStep[ pp->active[ i ] ] ) {
                    done = false;
                }
            }
            clampPose( pp, trial );
            float newCost = evaluate( pp, trial, nullptr, e1, nullptr );
            if ( newCost < cost ) {
                for ( int j = 0; j < POSE_SIZE; j++ ) {
                    q[ j ] = trial[ j ];
                }
                for ( int r = 0; r < rows; r++ ) {
                    e0[ r ] = e1[ r ];
                }
                cost = newCost;
                lambda = lambda > 3e-6f ? lambda / 3.0f : 1e-6f;
                accepted = true;
            } else {
                lambda *= 4.0f;
            }
        }
        if ( !accepted || done ) {
            break;
        }
    }
    *iterationsOut += iter;
    return cost;
}

// 1-sigma error bars at the answer, from s^2 ( J^T J )^-1 (see MagFit.h): out[k] for each searched entry.
static void poseSigma( const ProbeProblem* pp, float jtj[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ], float cost, float* out ) {
    int held = ( pp->knownTip != 0.0f ? 1 : 0 ) + ( pp->knownBack != 0.0f ? 1 : 0 );
    int freedom = 3 * pp->used + held - pp->params - 2;
    float variance = freedom > 0 ? cost / freedom : 0.0f;
    if ( variance < MAGFIT_NOISE_FLOOR_MT * MAGFIT_NOISE_FLOOR_MT ) {
        variance = MAGFIT_NOISE_FLOOR_MT * MAGFIT_NOISE_FLOOR_MT;
    }
    for ( int k = 0; k < pp->params; k++ ) { // the diagonal of the inverse, one unit vector at a time
        float a[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ];
        float unit[ MAGFIT_MAX_PARAMS ] = { 0 };
        for ( int i = 0; i < pp->params; i++ ) {
            for ( int j = 0; j < pp->params; j++ ) {
                a[ i ][ j ] = jtj[ i ][ j ];
            }
            a[ i ][ i ] += 1e-9f;
        }
        unit[ k ] = 1.0f;
        out[ k ] = 1e3f; // "could be anywhere" if the matrix will not invert
        if ( magFitSolveLinear( a, unit, pp->params ) && unit[ k ] > 0.0f ) {
            out[ k ] = sqrtf( variance * unit[ k ] );
        }
    }
}

static float wrapAngle( float a ) { // into ( -pi, pi ]
    while ( a > (float)M_PI )
        a -= 2.0f * (float)M_PI;
    while ( a <= -(float)M_PI )
        a += 2.0f * (float)M_PI;
    return a;
}

// A learned shape has more than one description: a magnet of strength s at
// angle t is one of strength -s at 180 - t with its across part turned half
// round. Frame after frame has to agree, so the learned form is: strengths
// POSITIVE, angles in 0..180 (0 = north pole toward the back of the probe,
// 180 = toward the tip, 90 = across), roll and turn adjusted to keep every
// pole where it was.
static void canonical( float* q, float strengths[ 2 ] ) {
    float& tip = q[ Q_TIP_ANGLE ];
    float& back = q[ Q_BACK_ANGLE ];
    tip = wrapAngle( tip );
    if ( tip < 0.0f ) { // cos(-t) a + sin(-t) n(roll) = cos t a + sin t n(roll + pi)
        tip = -tip;
        q[ Q_ROLL ] += (float)M_PI;
        q[ Q_BACK_TURN ] -= (float)M_PI;
    }
    if ( strengths[ 0 ] < 0.0f ) { // -s (cos t a + sin t n(roll)) = s (cos(pi - t) a + sin t n(roll + pi))
        strengths[ 0 ] = -strengths[ 0 ];
        tip = (float)M_PI - tip;
        q[ Q_ROLL ] += (float)M_PI;
        q[ Q_BACK_TURN ] -= (float)M_PI;
    }
    back = wrapAngle( back );
    if ( back < 0.0f ) {
        back = -back;
        q[ Q_BACK_TURN ] += (float)M_PI;
    }
    if ( strengths[ 1 ] < 0.0f ) {
        strengths[ 1 ] = -strengths[ 1 ];
        back = (float)M_PI - back;
        q[ Q_BACK_TURN ] += (float)M_PI;
    }
    q[ Q_ROLL ] = wrapAngle( q[ Q_ROLL ] );
    q[ Q_BACK_TURN ] = wrapAngle( q[ Q_BACK_TURN ] );
}

bool magProbeFitSolve( const Vec3* sensors, const Vec3* fields, const bool* use, int count, float maxMisfit,
                       const MagProbeShape* shape, bool learnShape, MagProbeFitResult* result ) {
    bool warm = result->valid;
    result->valid = false;
    result->iterations = 0;
    if ( count > MAGFIT_MAX_SENSORS ) {
        count = MAGFIT_MAX_SENSORS;
    }

    ProbeProblem pp = { };
    pp.sensors = sensors;
    pp.fields = fields;
    pp.use = use;
    pp.count = count;
    pp.xMin = pp.yMin = 1e9f;
    pp.xMax = pp.yMax = -1e9f;
    pp.knownTip = learnShape ? 0.0f : shape->tipStrength / STRENGTH_UNIT;
    pp.knownBack = learnShape ? 0.0f : shape->backStrength / STRENGTH_UNIT;
    bool rollMatters = learnShape || shape->tipAngleDeg * DEG > ROLL_MATTERS_RAD || shape->backAngleDeg * DEG > ROLL_MATTERS_RAD;
    pp.params = 0;
    for ( int i = 0; i < 5; i++ ) {
        pp.active[ pp.params++ ] = i;
    }
    if ( rollMatters ) {
        pp.active[ pp.params++ ] = Q_ROLL;
    }
    if ( learnShape ) {
        pp.active[ pp.params++ ] = Q_SPACING;
        pp.active[ pp.params++ ] = Q_TIP_ANGLE;
        pp.active[ pp.params++ ] = Q_BACK_ANGLE;
        pp.active[ pp.params++ ] = Q_BACK_TURN;
    }

    float sumSquares = 0.0f;
    for ( int i = 0; i < count; i++ ) {
        if ( !sensorUsed( &pp, i ) ) {
            continue;
        }
        pp.used++;
        sumSquares += fields[ i ].x * fields[ i ].x + fields[ i ].y * fields[ i ].y + fields[ i ].z * fields[ i ].z;
        pp.xMin = sensors[ i ].x < pp.xMin ? sensors[ i ].x : pp.xMin;
        pp.xMax = sensors[ i ].x > pp.xMax ? sensors[ i ].x : pp.xMax;
        pp.yMin = sensors[ i ].y < pp.yMin ? sensors[ i ].y : pp.yMin;
        pp.yMax = sensors[ i ].y > pp.yMax ? sensors[ i ].y : pp.yMax;
    }
    if ( pp.used < 3 || sumSquares <= 0.0f ) {
        return false;
    }
    pp.xMin -= MAGFIT_XY_MARGIN;
    pp.xMax += MAGFIT_XY_MARGIN;
    pp.yMin -= MAGFIT_XY_MARGIN;
    pp.yMax += MAGFIT_XY_MARGIN;
    pp.fieldNorm = sqrtf( sumSquares );
    result->signal = sqrtf( sumSquares / ( 3.0f * pp.used ) );

    float misfitLimit = maxMisfit * maxMisfit * sumSquares;
    float best[ POSE_SIZE ];
    float bestCost = 1e30f;
    float bestJtj[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ] = { { 0 } };
    float jtj[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ] = { { 0 } };

    if ( warm ) {
        best[ 0 ] = result->tipMagnet.x;
        best[ 1 ] = result->tipMagnet.y;
        best[ 2 ] = result->tipMagnet.z;
        best[ Q_AX ] = result->axis.x;
        best[ Q_AY ] = result->axis.y;
        best[ Q_ROLL ] = result->rollDeg * DEG;
        poseOf( learnShape ? &result->shape : shape, best );
        bestCost = refine( &pp, best, LM_MAX_ITERATIONS_TRACKING, &result->iterations, bestJtj );
    }

    // Cold start, and the fallback when the warm start no longer explains the
    // readings. Fit one magnet first: two magnets look from a distance like one
    // magnet somewhere between them, pointing along the shaft if they are along
    // it, across if they are across. Then try the probe with the one magnet
    // taken for the tip one, the back one and half way, along the fitted
    // direction and upright, with the roll set from the pole and a quarter
    // turn on; learning the shape, also in every mounting (each magnet along
    // or across, the back one turned or not).
    if ( !warm || bestCost > misfitLimit ) {
        MagFitResult single = { };
        magFitSolve( sensors, fields, use, count, 1e6f, &single );
        result->iterations += single.iterations;
        float inv = single.strength > 0.0f ? 1.0f / single.strength : 0.0f;
        Vec3 pole = { single.moment.x * inv, single.moment.y * inv, single.moment.z * inv };
        if ( pole.z < 0.0f ) {
            pole = { -pole.x, -pole.y, -pole.z };
        }
        Vec3 upright = { 0, 0, 1 };
        int rolls = rollMatters ? 2 : 1;
        // Learning: the five mountings that differ - each magnet along or
        // across, and two across ones on the same side or a quarter turn
        // apart - with the shaft seeded upright where an across magnet makes
        // the pole useless for it. Tracking: the shape as given, and the shaft
        // seeded along the pole as well as upright.
        static const float mountingAngles[ 5 ][ 3 ] = { { 0, 0, 0 }, { 90, 0, 0 }, { 0, 90, 0 }, { 90, 90, 0 }, { 90, 90, 90 } };
        int mountings = learnShape ? 5 : 1;
        for ( int m = 0; m < mountings; m++ ) {
            float tipAngle = shape->tipAngleDeg * DEG, backAngle = shape->backAngleDeg * DEG, backTurn = shape->backTurnDeg * DEG;
            if ( learnShape ) {
                tipAngle = mountingAngles[ m ][ 0 ] * DEG;
                backAngle = mountingAngles[ m ][ 1 ] * DEG;
                backTurn = mountingAngles[ m ][ 2 ] * DEG;
            }
            for ( int s = 0; s < 6; s++ ) {
                Vec3 a = s < 3 ? pole : upright;
                if ( learnShape && ( s < 3 ) == ( m > 0 ) ) {
                    continue; // along-along: pole seeds only; any across: upright seeds only
                }
                float back = shape->spacingMm * 0.5f * ( s % 3 );
                for ( int r = 0; r < rolls; r++ ) {
                    float roll = 0.0f;
                    if ( rollMatters ) {
                        Vec3 e1 = acrossOf( a, 0.0f ), e2 = acrossOf( a, 0.5f * (float)M_PI );
                        roll = atan2f( pole.x * e2.x + pole.y * e2.y + pole.z * e2.z, pole.x * e1.x + pole.y * e1.y + pole.z * e1.z ) + r * 0.5f * (float)M_PI;
                    }
                    float q[ POSE_SIZE ] = { single.position.x - back * a.x, single.position.y - back * a.y, single.position.z - back * a.z,
                                             a.x, a.y, roll, shape->spacingMm, tipAngle, backAngle, backTurn };
                    float cost = refine( &pp, q, LM_MAX_ITERATIONS, &result->iterations, jtj );
                    if ( cost < bestCost ) {
                        bestCost = cost;
                        for ( int i = 0; i < POSE_SIZE; i++ ) {
                            best[ i ] = q[ i ];
                        }
                        for ( int i = 0; i < MAGFIT_MAX_PARAMS; i++ ) {
                            for ( int j = 0; j < MAGFIT_MAX_PARAMS; j++ ) {
                                bestJtj[ i ][ j ] = jtj[ i ][ j ];
                            }
                        }
                    }
                }
            }
        }
    }
    if ( bestCost >= 1e29f ) {
        return false;
    }

    float strengths[ 2 ], fieldCost;
    evaluate( &pp, best, strengths, nullptr, &fieldCost );
    if ( learnShape ) {
        canonical( best, strengths ); // (the pose it describes is unchanged, so the cost stands)
    }
    result->tipMagnet = { best[ 0 ], best[ 1 ], best[ 2 ] };
    result->axis = axisOf( best );
    result->rollDeg = best[ Q_ROLL ] / DEG;
    result->shape = *shape;
    result->shape.spacingMm = best[ Q_SPACING ];
    result->shape.tipAngleDeg = best[ Q_TIP_ANGLE ] / DEG;
    result->shape.backAngleDeg = best[ Q_BACK_ANGLE ] / DEG;
    result->shape.backTurnDeg = best[ Q_BACK_TURN ] / DEG;
    result->shape.tipStrength = result->tipStrength = strengths[ 0 ] * STRENGTH_UNIT;
    result->shape.backStrength = result->backStrength = strengths[ 1 ] * STRENGTH_UNIT;
    result->backMagnet = { best[ 0 ] + best[ Q_SPACING ] * result->axis.x, best[ 1 ] + best[ Q_SPACING ] * result->axis.y, best[ 2 ] + best[ Q_SPACING ] * result->axis.z };
    Vec3 tipDir = poleOf( result->axis, best[ Q_TIP_ANGLE ], best[ Q_ROLL ] );
    Vec3 backDir = poleOf( result->axis, best[ Q_BACK_ANGLE ], best[ Q_ROLL ] + best[ Q_BACK_TURN ] );
    float tipSign = strengths[ 0 ] < 0.0f ? -1.0f : 1.0f, backSign = strengths[ 1 ] < 0.0f ? -1.0f : 1.0f;
    result->tipPole = { tipSign * tipDir.x, tipSign * tipDir.y, tipSign * tipDir.z };
    result->backPole = { backSign * backDir.x, backSign * backDir.y, backSign * backDir.z };
    result->residual = sqrtf( fieldCost / ( 3.0f * pp.used ) );

    float sigma[ MAGFIT_MAX_PARAMS ];
    poseSigma( &pp, bestJtj, bestCost, sigma );
    result->sigma = { sigma[ 0 ], sigma[ 1 ], sigma[ 2 ] };
    // (ax, ay) are direction cosines: a small error in them is that many radians at vertical, more when tilted.
    float cosTilt = result->axis.z > 0.2f ? result->axis.z : 0.2f;
    result->sigmaTiltDeg = sqrtf( sigma[ 3 ] * sigma[ 3 ] + sigma[ 4 ] * sigma[ 4 ] ) / cosTilt / DEG;

    result->valid = fieldCost <= misfitLimit;
    return result->valid;
}
