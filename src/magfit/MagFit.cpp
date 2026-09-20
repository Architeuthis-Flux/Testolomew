// SPDX-License-Identifier: MIT
#include "MagFit.h"

#include <math.h>

// The dipole kernel falls off as 1/r^3, so at tens of mm its entries are
// 1e-4..1e-6 and their squares crowd single precision. The kernel is carried
// multiplied by this, and the solved moment is scaled back.
#define KERNEL_SCALE 1000.0f

#define LM_MAX_ITERATIONS 30
#define LM_SEED_ITERATIONS 12 // a cold start's seeds: one that leads somewhere converges in far fewer; one that does not is cut short (it was most of a 12 ms slice)
#define LM_MAX_TRIES 8
#define LM_STEP_LIMIT_MM 20.0f // longest move in one iteration
#define LM_DONE_MM 0.005f      // converged when a step is shorter than this
#define JACOBIAN_STEP_MM 0.05f

struct FitProblem {
    const Vec3* sensors;
    const Vec3* fields;
    const bool* use;
    int count;
    float xMin, xMax, yMin, yMax;
};

static bool sensorUsed( const FitProblem* fp, int i ) {
    return fp->use == nullptr || fp->use[ i ];
}

// The symmetric 3x3 kernel G with B = G m, times KERNEL_SCALE.
static void dipoleKernel( Vec3 sensor, Vec3 magnet, float g[ 3 ][ 3 ] ) {
    float r[ 3 ] = { sensor.x - magnet.x, sensor.y - magnet.y, sensor.z - magnet.z };
    float r2 = r[ 0 ] * r[ 0 ] + r[ 1 ] * r[ 1 ] + r[ 2 ] * r[ 2 ];
    if ( r2 < 0.01f ) {
        r2 = 0.01f; // the magnet is ON the sensor; the model is meaningless but must stay finite
    }
    float r1 = sqrtf( r2 );
    float inv3 = KERNEL_SCALE / ( r2 * r1 );
    float inv5 = 3.0f * inv3 / r2;
    for ( int a = 0; a < 3; a++ ) {
        for ( int b = 0; b < 3; b++ ) {
            g[ a ][ b ] = inv5 * r[ a ] * r[ b ] - ( a == b ? inv3 : 0.0f );
        }
    }
}

Vec3 magFitDipoleField( Vec3 sensor, Vec3 magnet, Vec3 moment ) {
    float g[ 3 ][ 3 ];
    dipoleKernel( sensor, magnet, g );
    float m[ 3 ] = { moment.x / KERNEL_SCALE, moment.y / KERNEL_SCALE, moment.z / KERNEL_SCALE };
    Vec3 b;
    b.x = g[ 0 ][ 0 ] * m[ 0 ] + g[ 0 ][ 1 ] * m[ 1 ] + g[ 0 ][ 2 ] * m[ 2 ];
    b.y = g[ 1 ][ 0 ] * m[ 0 ] + g[ 1 ][ 1 ] * m[ 1 ] + g[ 1 ][ 2 ] * m[ 2 ];
    b.z = g[ 2 ][ 0 ] * m[ 0 ] + g[ 2 ][ 1 ] * m[ 1 ] + g[ 2 ][ 2 ] * m[ 2 ];
    return b;
}

// Solve the 3x3 system a x = b (Cramer's rule). false if a is singular.
static bool solve3( const float a[ 3 ][ 3 ], const float b[ 3 ], float x[ 3 ] ) {
    float c00 = a[ 1 ][ 1 ] * a[ 2 ][ 2 ] - a[ 1 ][ 2 ] * a[ 2 ][ 1 ];
    float c01 = a[ 1 ][ 2 ] * a[ 2 ][ 0 ] - a[ 1 ][ 0 ] * a[ 2 ][ 2 ];
    float c02 = a[ 1 ][ 0 ] * a[ 2 ][ 1 ] - a[ 1 ][ 1 ] * a[ 2 ][ 0 ];
    float det = a[ 0 ][ 0 ] * c00 + a[ 0 ][ 1 ] * c01 + a[ 0 ][ 2 ] * c02;
    if ( fabsf( det ) < 1e-30f ) {
        return false;
    }
    float inv = 1.0f / det;
    x[ 0 ] = inv * ( b[ 0 ] * c00 + a[ 0 ][ 1 ] * ( a[ 1 ][ 2 ] * b[ 2 ] - b[ 1 ] * a[ 2 ][ 2 ] ) + a[ 0 ][ 2 ] * ( b[ 1 ] * a[ 2 ][ 1 ] - a[ 1 ][ 1 ] * b[ 2 ] ) );
    x[ 1 ] = inv * ( a[ 0 ][ 0 ] * ( b[ 1 ] * a[ 2 ][ 2 ] - a[ 1 ][ 2 ] * b[ 2 ] ) + b[ 0 ] * c01 + a[ 0 ][ 2 ] * ( a[ 1 ][ 0 ] * b[ 2 ] - b[ 1 ] * a[ 2 ][ 0 ] ) );
    x[ 2 ] = inv * ( a[ 0 ][ 0 ] * ( a[ 1 ][ 1 ] * b[ 2 ] - b[ 1 ] * a[ 2 ][ 1 ] ) + a[ 0 ][ 1 ] * ( b[ 1 ] * a[ 2 ][ 0 ] - a[ 1 ][ 0 ] * b[ 2 ] ) + b[ 0 ] * c02 );
    return true;
}

// For a trial magnet position: the best-fitting moment (one linear solve), the
// residuals it leaves (3 per sensor, zeros for unused sensors; may be null)
// and their sum of squares. Returns a huge cost if the solve is singular.
static float evaluate( const FitProblem* fp, Vec3 p, Vec3* momentOut, float* residuals ) {
    float gtg[ 3 ][ 3 ] = { { 0 } };
    float gtb[ 3 ] = { 0 };
    float g[ 3 ][ 3 ];

    for ( int i = 0; i < fp->count; i++ ) {
        if ( !sensorUsed( fp, i ) ) {
            continue;
        }
        dipoleKernel( fp->sensors[ i ], p, g );
        float b[ 3 ] = { fp->fields[ i ].x, fp->fields[ i ].y, fp->fields[ i ].z };
        for ( int a = 0; a < 3; a++ ) {
            for ( int c = 0; c < 3; c++ ) {
                // G is symmetric, so G^T G = G G.
                gtg[ a ][ c ] += g[ a ][ 0 ] * g[ 0 ][ c ] + g[ a ][ 1 ] * g[ 1 ][ c ] + g[ a ][ 2 ] * g[ 2 ][ c ];
            }
            gtb[ a ] += g[ a ][ 0 ] * b[ 0 ] + g[ a ][ 1 ] * b[ 1 ] + g[ a ][ 2 ] * b[ 2 ];
        }
    }

    float m[ 3 ];
    if ( !solve3( gtg, gtb, m ) ) {
        return 1e30f;
    }

    float cost = 0.0f;
    for ( int i = 0; i < fp->count; i++ ) {
        float e[ 3 ] = { 0, 0, 0 };
        if ( sensorUsed( fp, i ) ) {
            dipoleKernel( fp->sensors[ i ], p, g );
            float b[ 3 ] = { fp->fields[ i ].x, fp->fields[ i ].y, fp->fields[ i ].z };
            for ( int a = 0; a < 3; a++ ) {
                e[ a ] = b[ a ] - ( g[ a ][ 0 ] * m[ 0 ] + g[ a ][ 1 ] * m[ 1 ] + g[ a ][ 2 ] * m[ 2 ] );
                cost += e[ a ] * e[ a ];
            }
        }
        if ( residuals != nullptr ) {
            residuals[ 3 * i + 0 ] = e[ 0 ];
            residuals[ 3 * i + 1 ] = e[ 1 ];
            residuals[ 3 * i + 2 ] = e[ 2 ];
        }
    }

    if ( momentOut != nullptr ) {
        momentOut->x = m[ 0 ] * KERNEL_SCALE;
        momentOut->y = m[ 1 ] * KERNEL_SCALE;
        momentOut->z = m[ 2 ] * KERNEL_SCALE;
    }
    return cost;
}

// The cost alone, without the residuals: with the moment solved by least
// squares the misfit is |b|^2 - (G^T b) . m, so the second pass over the
// sensors is not needed. Half the work of evaluate(); used by the lattice
// search, where the cost is all that is compared. sumSquares is |b|^2 over
// the used sensors.
static float costAt( const FitProblem* fp, Vec3 p, float sumSquares ) {
    float gtg[ 3 ][ 3 ] = { { 0 } };
    float gtb[ 3 ] = { 0 };
    float g[ 3 ][ 3 ];
    for ( int i = 0; i < fp->count; i++ ) {
        if ( !sensorUsed( fp, i ) ) {
            continue;
        }
        dipoleKernel( fp->sensors[ i ], p, g );
        float b[ 3 ] = { fp->fields[ i ].x, fp->fields[ i ].y, fp->fields[ i ].z };
        for ( int a = 0; a < 3; a++ ) {
            for ( int c = 0; c < 3; c++ ) {
                gtg[ a ][ c ] += g[ a ][ 0 ] * g[ 0 ][ c ] + g[ a ][ 1 ] * g[ 1 ][ c ] + g[ a ][ 2 ] * g[ 2 ][ c ];
            }
            gtb[ a ] += g[ a ][ 0 ] * b[ 0 ] + g[ a ][ 1 ] * b[ 1 ] + g[ a ][ 2 ] * b[ 2 ];
        }
    }
    float m[ 3 ];
    if ( !solve3( gtg, gtb, m ) ) {
        return 1e30f;
    }
    float cost = sumSquares - ( gtb[ 0 ] * m[ 0 ] + gtb[ 1 ] * m[ 1 ] + gtb[ 2 ] * m[ 2 ] );
    return cost < 0.0f ? 0.0f : cost;
}

static float clampf( float v, float lo, float hi ) {
    return v < lo ? lo : ( v > hi ? hi : v );
}

static Vec3 clampToBox( const FitProblem* fp, Vec3 p ) {
    p.x = clampf( p.x, fp->xMin, fp->xMax );
    p.y = clampf( p.y, fp->yMin, fp->yMax );
    p.z = clampf( p.z, MAGFIT_Z_MIN, MAGFIT_Z_MAX );
    return p;
}

// Levenberg-Marquardt over the magnet position, from `start`. Returns the
// final cost; *pOut is where it ended up.
// Levenberg-Marquardt from `start`: at most maxIterations, and (for a
// steady load) at least minIterations even once converged; *convergedOut
// (may be null) says it stopped on its own.
static float refine( const FitProblem* fp, Vec3 start, Vec3* pOut, int* iterationsOut, int maxIterations = LM_MAX_ITERATIONS, int minIterations = 0, bool* convergedOut = nullptr ) {
    static float e0[ 3 * MAGFIT_MAX_SENSORS ];
    static float e1[ 3 * MAGFIT_MAX_SENSORS ];
    static float jac[ 3 * MAGFIT_MAX_SENSORS ][ 3 ];

    int rows = 3 * fp->count;
    Vec3 p = clampToBox( fp, start );
    float cost = evaluate( fp, p, nullptr, e0 );
    float lambda = 1e-2f;
    int iter = 0;
    bool converged = false;

    for ( ; iter < maxIterations; iter++ ) {
        // Numeric Jacobian of the residuals. Because evaluate() re-solves the
        // moment at the nudged position, this is the variable-projection
        // Jacobian with no extra algebra.
        for ( int k = 0; k < 3; k++ ) {
            Vec3 q = p;
            if ( k == 0 )
                q.x += JACOBIAN_STEP_MM;
            if ( k == 1 )
                q.y += JACOBIAN_STEP_MM;
            if ( k == 2 )
                q.z += JACOBIAN_STEP_MM;
            evaluate( fp, q, nullptr, e1 );
            for ( int r = 0; r < rows; r++ ) {
                jac[ r ][ k ] = ( e1[ r ] - e0[ r ] ) / JACOBIAN_STEP_MM;
            }
        }

        float jtj[ 3 ][ 3 ] = { { 0 } };
        float jte[ 3 ] = { 0 };
        for ( int r = 0; r < rows; r++ ) {
            for ( int a = 0; a < 3; a++ ) {
                jte[ a ] += jac[ r ][ a ] * e0[ r ];
                for ( int c = 0; c < 3; c++ ) {
                    jtj[ a ][ c ] += jac[ r ][ a ] * jac[ r ][ c ];
                }
            }
        }

        bool accepted = false;
        float stepLength = 0.0f;
        for ( int tries = 0; tries < LM_MAX_TRIES && !accepted; tries++ ) {
            float a[ 3 ][ 3 ];
            float rhs[ 3 ];
            float d[ 3 ];
            for ( int i = 0; i < 3; i++ ) {
                for ( int j = 0; j < 3; j++ ) {
                    a[ i ][ j ] = jtj[ i ][ j ];
                }
                a[ i ][ i ] += lambda * jtj[ i ][ i ] + 1e-12f;
                rhs[ i ] = -jte[ i ];
            }
            if ( !solve3( a, rhs, d ) ) {
                lambda *= 4.0f;
                continue;
            }
            stepLength = sqrtf( d[ 0 ] * d[ 0 ] + d[ 1 ] * d[ 1 ] + d[ 2 ] * d[ 2 ] );
            if ( stepLength > LM_STEP_LIMIT_MM ) {
                float k = LM_STEP_LIMIT_MM / stepLength;
                d[ 0 ] *= k;
                d[ 1 ] *= k;
                d[ 2 ] *= k;
                stepLength = LM_STEP_LIMIT_MM;
            }
            Vec3 q = { p.x + d[ 0 ], p.y + d[ 1 ], p.z + d[ 2 ] };
            q = clampToBox( fp, q );
            float newCost = evaluate( fp, q, nullptr, e1 );
            if ( newCost < cost ) {
                p = q;
                cost = newCost;
                for ( int r = 0; r < rows; r++ ) {
                    e0[ r ] = e1[ r ];
                }
                lambda = lambda > 3e-6f ? lambda / 3.0f : 1e-6f;
                accepted = true;
            } else {
                lambda *= 4.0f;
            }
        }

        if ( !accepted || stepLength < LM_DONE_MM ) {
            converged = true;
            if ( iter + 1 >= minIterations ) {
                break;
            }
        }
    }

    *pOut = p;
    *iterationsOut += iter;
    if ( convergedOut != nullptr )
        *convergedOut = converged;
    return cost;
}

// 1-sigma position error at p, from the residuals' Jacobian there. Because
// evaluate() re-solves the moment at each nudged position, this Jacobian has
// the moment's freedom already taken out of it, which is exactly what makes
// the 3x3 below the position block of the full six-parameter covariance.
static Vec3 positionSigma( const FitProblem* fp, Vec3 p, float cost, int used ) {
    static float e0[ 3 * MAGFIT_MAX_SENSORS ];
    static float e1[ 3 * MAGFIT_MAX_SENSORS ];
    float jtj[ 3 ][ 3 ] = { { 0 } };
    static float jac[ 3 * MAGFIT_MAX_SENSORS ][ 3 ];
    int rows = 3 * fp->count;

    evaluate( fp, p, nullptr, e0 );
    for ( int k = 0; k < 3; k++ ) {
        Vec3 q = p;
        if ( k == 0 )
            q.x += JACOBIAN_STEP_MM;
        if ( k == 1 )
            q.y += JACOBIAN_STEP_MM;
        if ( k == 2 )
            q.z += JACOBIAN_STEP_MM;
        evaluate( fp, q, nullptr, e1 );
        for ( int r = 0; r < rows; r++ ) {
            jac[ r ][ k ] = ( e1[ r ] - e0[ r ] ) / JACOBIAN_STEP_MM;
        }
    }
    for ( int r = 0; r < rows; r++ ) {
        for ( int a = 0; a < 3; a++ ) {
            for ( int c = 0; c < 3; c++ ) {
                jtj[ a ][ c ] += jac[ r ][ a ] * jac[ r ][ c ];
            }
        }
    }

    int freedom = 3 * used - 6; // readings less unknowns
    float variance = cost / ( freedom > 0 ? freedom : 1 );
    if ( variance < MAGFIT_NOISE_FLOOR_MT * MAGFIT_NOISE_FLOOR_MT ) {
        variance = MAGFIT_NOISE_FLOOR_MT * MAGFIT_NOISE_FLOOR_MT;
    }

    // The diagonal of the inverse, one column at a time.
    Vec3 sigma = { 999.0f, 999.0f, 999.0f };
    float unit[ 3 ][ 3 ] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    float column[ 3 ];
    if ( solve3( jtj, unit[ 0 ], column ) && column[ 0 ] > 0 )
        sigma.x = sqrtf( variance * column[ 0 ] );
    if ( solve3( jtj, unit[ 1 ], column ) && column[ 1 ] > 0 )
        sigma.y = sqrtf( variance * column[ 1 ] );
    if ( solve3( jtj, unit[ 2 ], column ) && column[ 2 ] > 0 )
        sigma.z = sqrtf( variance * column[ 2 ] );
    return sigma;
}

// The search box round the sensors, and the signal in the readings.
static int describeProblem( FitProblem* fp, const Vec3* sensors, const Vec3* fields, float margin, float* sumSquares ) {
    int used = 0;
    fp->xMin = 1e9f;
    fp->xMax = -1e9f;
    fp->yMin = 1e9f;
    fp->yMax = -1e9f;
    *sumSquares = 0.0f;
    for ( int i = 0; i < fp->count; i++ ) {
        if ( !sensorUsed( fp, i ) ) {
            continue;
        }
        used++;
        if ( sensors[ i ].x < fp->xMin )
            fp->xMin = sensors[ i ].x;
        if ( sensors[ i ].x > fp->xMax )
            fp->xMax = sensors[ i ].x;
        if ( sensors[ i ].y < fp->yMin )
            fp->yMin = sensors[ i ].y;
        if ( sensors[ i ].y > fp->yMax )
            fp->yMax = sensors[ i ].y;
        *sumSquares += fields[ i ].x * fields[ i ].x + fields[ i ].y * fields[ i ].y + fields[ i ].z * fields[ i ].z;
    }
    fp->xMin -= margin;
    fp->xMax += margin;
    fp->yMin -= margin;
    fp->yMax += margin;
    return used;
}

// The lattice search: every point of a coarse lattice over the box, then a
// 3x3x3 lattice of half the step round the best, and again, down to
// MAGFIT_COARSE_FINAL_MM. Returns the best point and its cost.
static float latticeSearch( const FitProblem* fp, float sumSquares, Vec3* best ) {
    float bestCost = 1e30f;
    Vec3 b = { 0, 0, 0 };
    float step = MAGFIT_COARSE_STEP_MM;

    // A magnet low down between two sensors has a cost basin only a few mm
    // wide, which a 25 mm lattice can miss entirely. The |B|^2-weighted
    // centroid of the sensors is within a sensor pitch of it, so that point
    // at a few low heights goes in first.
    float cx = 0.0f, cy = 0.0f, cw = 0.0f;
    for ( int i = 0; i < fp->count; i++ ) {
        if ( !sensorUsed( fp, i ) )
            continue;
        float w = fp->fields[ i ].x * fp->fields[ i ].x + fp->fields[ i ].y * fp->fields[ i ].y + fp->fields[ i ].z * fp->fields[ i ].z;
        cx += w * fp->sensors[ i ].x;
        cy += w * fp->sensors[ i ].y;
        cw += w;
    }
    if ( cw > 0.0f ) {
        static const float lowHeights[ 4 ] = { 3.0f, 6.0f, 10.0f, 15.0f };
        for ( int k = 0; k < 4; k++ ) {
            Vec3 p = { cx / cw, cy / cw, lowHeights[ k ] };
            float cost = costAt( fp, p, sumSquares );
            if ( cost < bestCost ) {
                bestCost = cost;
                b = p;
            }
        }
    }
    float xLo = fp->xMin + MAGFIT_XY_MARGIN - MAGFIT_COARSE_MARGIN_MM, xHi = fp->xMax - MAGFIT_XY_MARGIN + MAGFIT_COARSE_MARGIN_MM;
    float yLo = fp->yMin + MAGFIT_XY_MARGIN - MAGFIT_COARSE_MARGIN_MM, yHi = fp->yMax - MAGFIT_XY_MARGIN + MAGFIT_COARSE_MARGIN_MM;
    for ( float z = MAGFIT_COARSE_Z_MIN_MM; z <= MAGFIT_COARSE_Z_MAX_MM; z += step ) {
        for ( float y = yLo; y <= yHi + 0.01f; y += step ) {
            for ( float x = xLo; x <= xHi + 0.01f; x += step ) {
                Vec3 p = { x, y, z };
                float cost = costAt( fp, p, sumSquares );
                if ( cost < bestCost ) {
                    bestCost = cost;
                    b = p;
                }
            }
        }
    }
    for ( step *= 0.5f; step >= MAGFIT_COARSE_FINAL_MM; step *= 0.5f ) {
        Vec3 centre = b;
        for ( int dz = -1; dz <= 1; dz++ ) {
            for ( int dy = -1; dy <= 1; dy++ ) {
                for ( int dx = -1; dx <= 1; dx++ ) {
                    if ( dx == 0 && dy == 0 && dz == 0 )
                        continue;
                    Vec3 p = clampToBox( fp, { centre.x + dx * step, centre.y + dy * step, centre.z + dz * step } );
                    float cost = costAt( fp, p, sumSquares );
                    if ( cost < bestCost ) {
                        bestCost = cost;
                        b = p;
                    }
                }
            }
        }
    }
    *best = b;
    return bestCost;
}

bool magFitCoarse( const Vec3* sensors, const Vec3* fields, const bool* use, int count, MagFitResult* result ) {
    result->valid = false;
    result->iterations = 0;
    result->sigma = { 999.0f, 999.0f, 999.0f };
    if ( count > MAGFIT_MAX_SENSORS ) {
        count = MAGFIT_MAX_SENSORS;
    }
    FitProblem fp = { sensors, fields, use, count, 0, 0, 0, 0 };
    float sumSquares;
    int used = describeProblem( &fp, sensors, fields, MAGFIT_XY_MARGIN, &sumSquares );
    if ( used < 3 || sumSquares <= 0.0f ) {
        return false;
    }
    result->signal = sqrtf( sumSquares / ( 3.0f * used ) );
    Vec3 best;
    float cost = latticeSearch( &fp, sumSquares, &best );
    if ( cost >= 1e29f ) {
        return false;
    }
    result->position = best;
    evaluate( &fp, best, &result->moment, nullptr );
    result->strength = sqrtf( result->moment.x * result->moment.x + result->moment.y * result->moment.y + result->moment.z * result->moment.z );
    result->residual = sqrtf( cost / ( 3.0f * used ) );
    result->sigma = positionSigma( &fp, best, cost, used );
    result->valid = true;
    return true;
}

// Seeds for a cold start whose lattice answer came out poor or low: over
// the strongest readings at three heights, then a ring around that point
// low down.
static const float coldSeeds[][ 3 ] = {
    { 0, 0, 5 },
    { 0, 0, 15 },
    { 0, 0, 40 },
    { 7, 7, 6 },
    { -7, 7, 6 },
    { 7, -7, 6 },
    { -7, -7, 6 },
};
#define COLD_SEED_COUNT ( (int)( sizeof( coldSeeds ) / sizeof( coldSeeds[ 0 ] ) ) )

bool magFitSolve( const Vec3* sensors, const Vec3* fields, const bool* use, int count,
                  float maxMisfit, MagFitResult* result ) {
    result->coldStage = 0; // a whole solve never continues an earlier cold start (nor trusts an uninitialised result)
    bool warm = result->valid;
    bool ok = magFitSolveStep( sensors, fields, use, count, maxMisfit, result );
    if ( !ok && warm && result->coldStage == 0 ) {
        ok = magFitSolveStep( sensors, fields, use, count, maxMisfit, result ); // the warm start failed: a cold one, whole
    }
    while ( result->coldStage > 0 ) {
        ok = magFitSolveStep( sensors, fields, use, count, maxMisfit, result );
    }
    return ok;
}

bool magFitSolveStep( const Vec3* sensors, const Vec3* fields, const bool* use, int count,
                      float maxMisfit, MagFitResult* result, int budget ) {
    int maxIt = budget > 0 ? budget : LM_MAX_ITERATIONS;
    int seedIt = budget > 0 ? budget : LM_SEED_ITERATIONS;
    int minIt = budget > 0 ? budget : 0;
    if ( result->coldStage < 0 || result->coldStage > COLD_SEED_COUNT + 1 ) {
        result->coldStage = 0; // not a stage of ours
    }
    bool continuing = result->coldStage > 0;
    bool warm = result->valid && !continuing;
    Vec3 warmStart = result->position;
    result->valid = false;
    result->iterations = 0;
    result->sigma = { 999.0f, 999.0f, 999.0f };

    if ( count > MAGFIT_MAX_SENSORS ) {
        count = MAGFIT_MAX_SENSORS;
    }

    FitProblem fp = { sensors, fields, use, count, 1e9f, -1e9f, 1e9f, -1e9f };

    // The search box, the signal level, and a |B|^2-weighted centroid of the
    // sensors as the cold-start guess for x and y.
    int used = 0;
    float sumSquares = 0.0f;
    float cx = 0.0f, cy = 0.0f, cw = 0.0f;
    for ( int i = 0; i < count; i++ ) {
        if ( !sensorUsed( &fp, i ) ) {
            continue;
        }
        used++;
        if ( sensors[ i ].x < fp.xMin )
            fp.xMin = sensors[ i ].x;
        if ( sensors[ i ].x > fp.xMax )
            fp.xMax = sensors[ i ].x;
        if ( sensors[ i ].y < fp.yMin )
            fp.yMin = sensors[ i ].y;
        if ( sensors[ i ].y > fp.yMax )
            fp.yMax = sensors[ i ].y;
        float w = fields[ i ].x * fields[ i ].x + fields[ i ].y * fields[ i ].y + fields[ i ].z * fields[ i ].z;
        sumSquares += w;
        cx += w * sensors[ i ].x;
        cy += w * sensors[ i ].y;
        cw += w;
    }
    if ( used < 3 || cw <= 0.0f ) {
        return false; // 6 unknowns want at least 3 sensors, and some field
    }
    fp.xMin -= MAGFIT_XY_MARGIN;
    fp.xMax += MAGFIT_XY_MARGIN;
    fp.yMin -= MAGFIT_XY_MARGIN;
    fp.yMax += MAGFIT_XY_MARGIN;

    result->signal = sqrtf( sumSquares / ( 3.0f * used ) );

    Vec3 best = { 0, 0, 0 };
    float bestCost = 1e30f;
    float misfitLimit = maxMisfit * maxMisfit * sumSquares;

    if ( warm ) {
        // A warm start: one refinement from the last answer. If it lands
        // somewhere that does not explain the readings (the magnet jumped,
        // or was swapped) that is simply no fix; the caller starts cold on
        // a later call (magFitSolve does so at once).
        bestCost = refine( &fp, warmStart, &best, &result->iterations, maxIt, minIt );
    } else if ( !continuing ) {
        // A cold start, stage 0: the lattice. Its best point is within a few
        // mm of the answer wherever the magnet is, and one refinement from
        // there (the next call) is usually the whole cold start.
        Vec3 coarse;
        float coarseCost = latticeSearch( &fp, sumSquares, &coarse );
        if ( coarseCost >= 1e29f ) {
            return false; // nothing to refine
        }
        result->coldStage = 1;
        result->coldBest = coarse;
        result->coldCost = 1e30f; // the lattice's cost is not a refined one: the refinement decides
        result->coldIterations = 0;
        return false;
    } else if ( result->coldStage == 1 ) {
        // Stage 1: refine the lattice's point - on a budget, resumed next
        // call until it converges or the cap is spent.
        best = result->coldBest;
        bool converged = true;
        int before = result->iterations;
        bestCost = refine( &fp, best, &best, &result->iterations, maxIt, minIt, &converged );
        result->coldIterations += result->iterations - before;
        if ( budget > 0 && !converged && result->coldIterations < LM_MAX_ITERATIONS ) {
            result->coldBest = best;
            result->coldCost = bestCost;
            return false; // more of the same next call
        }
        // A magnet close to the board and off to one side of a sensor has a
        // false minimum pinned under that sensor, and it can fit to a few
        // percent (a 6x3 mm N52 3 mm up beside the last sensor: 4.6 % misfit
        // 9 mm away). The lattice cannot tell; only a start on the right side
        // of it escapes. So a cold start that came out low, or that fits
        // poorly, tries the seeds too - one per call from here.
        if ( bestCost > misfitLimit * MAGFIT_COARSE_GOOD_ENOUGH || best.z < MAGFIT_COARSE_LOW_MM ) {
            result->coldStage = 2;
            result->coldBest = best;
            result->coldCost = bestCost;
            return false;
        }
        result->coldStage = 0;
    } else {
        // Stage k >= 2: seed k-2, on a short budget.
        best = result->coldBest;
        bestCost = result->coldCost;
        int seed = result->coldStage - 2;
        Vec3 start = { cx / cw + coldSeeds[ seed ][ 0 ], cy / cw + coldSeeds[ seed ][ 1 ], coldSeeds[ seed ][ 2 ] };
        Vec3 p;
        float cost = refine( &fp, start, &p, &result->iterations, seedIt, minIt );
        if ( cost < bestCost ) {
            bestCost = cost;
            best = p;
        }
        if ( seed + 1 < COLD_SEED_COUNT ) {
            result->coldStage = seed + 3;
            result->coldBest = best;
            result->coldCost = bestCost;
            return false; // more seeds next call
        }
        result->coldStage = 0;
        // The best of the seeds was found against an earlier frame's
        // readings: settle it on this frame's.
        if ( bestCost < 1e29f ) {
            bestCost = refine( &fp, best, &best, &result->iterations, maxIt, minIt );
        }
    }

    if ( bestCost >= 1e29f ) {
        return false;
    }

    result->position = best;
    evaluate( &fp, best, &result->moment, nullptr );
    result->strength = sqrtf( result->moment.x * result->moment.x + result->moment.y * result->moment.y + result->moment.z * result->moment.z );
    result->residual = sqrtf( bestCost / ( 3.0f * used ) );
    result->sigma = positionSigma( &fp, best, bestCost, used );
    result->valid = bestCost <= misfitLimit;
    return result->valid;
}

// ---- known strength ----------------------------------------------------------

#define KNOWN_PARAMS 6 // x, y, z and the moment as strength * (vx, vy, vz)
#define KNOWN_ROWS ( 3 * MAGFIT_MAX_SENSORS + 1 )
#define KNOWN_STRENGTH_WEIGHT 5.0f // a 1 % strength error costs as much as a 5 % field misfit

// Solve the n x n system a x = b in place (Gaussian elimination, partial
// pivoting). false if singular.
bool magFitSolveLinear( float a[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ], float b[ MAGFIT_MAX_PARAMS ], int n ) {
    for ( int col = 0; col < n; col++ ) {
        int pivot = col;
        for ( int r = col + 1; r < n; r++ ) {
            if ( fabsf( a[ r ][ col ] ) > fabsf( a[ pivot ][ col ] ) )
                pivot = r;
        }
        if ( fabsf( a[ pivot ][ col ] ) < 1e-20f ) {
            return false;
        }
        for ( int c = 0; c < n; c++ ) {
            float t = a[ col ][ c ];
            a[ col ][ c ] = a[ pivot ][ c ];
            a[ pivot ][ c ] = t;
        }
        float t = b[ col ];
        b[ col ] = b[ pivot ];
        b[ pivot ] = t;
        for ( int r = col + 1; r < n; r++ ) {
            float k = a[ r ][ col ] / a[ col ][ col ];
            for ( int c = col; c < n; c++ )
                a[ r ][ c ] -= k * a[ col ][ c ];
            b[ r ] -= k * b[ col ];
        }
    }
    for ( int r = n - 1; r >= 0; r-- ) {
        for ( int c = r + 1; c < n; c++ )
            b[ r ] -= a[ r ][ c ] * b[ c ];
        b[ r ] /= a[ r ][ r ];
    }
    return true;
}

// Residuals for q = { x, y, z, vx, vy, vz }: the field rows, then one row that
// holds |v| at 1 (the moment is strength * v). Returns the FIELD rows' sum of
// squares; *total includes the strength row.
static float knownResiduals( const FitProblem* fp, const float q[ KNOWN_PARAMS ], float strength, float strengthRowWeight,
                             float* rows, float* total ) {
    Vec3 p = { q[ 0 ], q[ 1 ], q[ 2 ] };
    float m[ 3 ] = { strength * q[ 3 ] / KERNEL_SCALE, strength * q[ 4 ] / KERNEL_SCALE, strength * q[ 5 ] / KERNEL_SCALE };
    float g[ 3 ][ 3 ];
    float cost = 0.0f;
    for ( int i = 0; i < fp->count; i++ ) {
        float e[ 3 ] = { 0, 0, 0 };
        if ( sensorUsed( fp, i ) ) {
            dipoleKernel( fp->sensors[ i ], p, g );
            float b[ 3 ] = { fp->fields[ i ].x, fp->fields[ i ].y, fp->fields[ i ].z };
            for ( int a = 0; a < 3; a++ ) {
                e[ a ] = b[ a ] - ( g[ a ][ 0 ] * m[ 0 ] + g[ a ][ 1 ] * m[ 1 ] + g[ a ][ 2 ] * m[ 2 ] );
                cost += e[ a ] * e[ a ];
            }
        }
        rows[ 3 * i + 0 ] = e[ 0 ];
        rows[ 3 * i + 1 ] = e[ 1 ];
        rows[ 3 * i + 2 ] = e[ 2 ];
    }
    float length = sqrtf( q[ 3 ] * q[ 3 ] + q[ 4 ] * q[ 4 ] + q[ 5 ] * q[ 5 ] );
    float strengthRow = strengthRowWeight * ( length - 1.0f );
    rows[ 3 * fp->count ] = strengthRow;
    *total = cost + strengthRow * strengthRow;
    return cost;
}

bool magFitSolveKnownStrength( const Vec3* sensors, const Vec3* fields, const bool* use, int count,
                               float maxMisfit, float strength, MagFitResult* result ) {
    // The free fit first. If it cannot explain the readings, nothing below will.
    bool freeOk = magFitSolve( sensors, fields, use, count, maxMisfit, result );
    if ( strength <= 0.0f || result->strength <= 0.0f || result->signal <= 0.0f ) {
        return freeOk;
    }
    if ( count > MAGFIT_MAX_SENSORS ) {
        count = MAGFIT_MAX_SENSORS;
    }

    FitProblem fp = { sensors, fields, use, count, 1e9f, -1e9f, 1e9f, -1e9f };
    int used = 0;
    for ( int i = 0; i < count; i++ ) {
        if ( !sensorUsed( &fp, i ) )
            continue;
        used++;
        if ( sensors[ i ].x < fp.xMin )
            fp.xMin = sensors[ i ].x;
        if ( sensors[ i ].x > fp.xMax )
            fp.xMax = sensors[ i ].x;
        if ( sensors[ i ].y < fp.yMin )
            fp.yMin = sensors[ i ].y;
        if ( sensors[ i ].y > fp.yMax )
            fp.yMax = sensors[ i ].y;
    }
    fp.xMin -= MAGFIT_XY_MARGIN;
    fp.xMax += MAGFIT_XY_MARGIN;
    fp.yMin -= MAGFIT_XY_MARGIN;
    fp.yMax += MAGFIT_XY_MARGIN;

    // The strength row is weighed against the whole field vector, so the pull
    // toward the known strength is the same for a strong reading and a weak one.
    float fieldNorm = result->signal * sqrtf( 3.0f * used );
    float rowWeight = KNOWN_STRENGTH_WEIGHT * fieldNorm;

    static float e0[ KNOWN_ROWS ], e1[ KNOWN_ROWS ];
    static float jac[ KNOWN_ROWS ][ KNOWN_PARAMS ];
    static const float step[ KNOWN_PARAMS ] = { JACOBIAN_STEP_MM, JACOBIAN_STEP_MM, JACOBIAN_STEP_MM, 0.002f, 0.002f, 0.002f };
    int rows = 3 * count + 1;

    float inv = 1.0f / result->strength;
    float q[ KNOWN_PARAMS ] = { result->position.x, result->position.y, result->position.z,
                                result->moment.x * inv, result->moment.y * inv, result->moment.z * inv };
    float total;
    float fieldCost = knownResiduals( &fp, q, strength, rowWeight, e0, &total );
    float lambda = 1e-2f;

    for ( int iter = 0; iter < LM_MAX_ITERATIONS; iter++ ) {
        for ( int k = 0; k < KNOWN_PARAMS; k++ ) {
            float nudged[ KNOWN_PARAMS ];
            for ( int j = 0; j < KNOWN_PARAMS; j++ )
                nudged[ j ] = q[ j ];
            nudged[ k ] += step[ k ];
            float unused;
            knownResiduals( &fp, nudged, strength, rowWeight, e1, &unused );
            for ( int r = 0; r < rows; r++ )
                jac[ r ][ k ] = ( e1[ r ] - e0[ r ] ) / step[ k ];
        }
        float jtj[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ] = { { 0 } };
        float jte[ MAGFIT_MAX_PARAMS ] = { 0 };
        for ( int r = 0; r < rows; r++ ) {
            for ( int a = 0; a < KNOWN_PARAMS; a++ ) {
                jte[ a ] += jac[ r ][ a ] * e0[ r ];
                for ( int c = 0; c < KNOWN_PARAMS; c++ )
                    jtj[ a ][ c ] += jac[ r ][ a ] * jac[ r ][ c ];
            }
        }

        bool accepted = false;
        float moved = 0.0f;
        for ( int tries = 0; tries < LM_MAX_TRIES && !accepted; tries++ ) {
            float a[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ];
            float d[ MAGFIT_MAX_PARAMS ];
            for ( int i = 0; i < KNOWN_PARAMS; i++ ) {
                for ( int j = 0; j < KNOWN_PARAMS; j++ )
                    a[ i ][ j ] = jtj[ i ][ j ];
                a[ i ][ i ] += lambda * jtj[ i ][ i ] + 1e-12f;
                d[ i ] = -jte[ i ];
            }
            if ( !magFitSolveLinear( a, d, KNOWN_PARAMS ) ) {
                lambda *= 4.0f;
                continue;
            }
            moved = sqrtf( d[ 0 ] * d[ 0 ] + d[ 1 ] * d[ 1 ] + d[ 2 ] * d[ 2 ] );
            if ( moved > LM_STEP_LIMIT_MM ) {
                for ( int i = 0; i < KNOWN_PARAMS; i++ )
                    d[ i ] *= LM_STEP_LIMIT_MM / moved;
                moved = LM_STEP_LIMIT_MM;
            }
            float trial[ KNOWN_PARAMS ];
            for ( int i = 0; i < KNOWN_PARAMS; i++ )
                trial[ i ] = q[ i ] + d[ i ];
            Vec3 boxed = clampToBox( &fp, { trial[ 0 ], trial[ 1 ], trial[ 2 ] } );
            trial[ 0 ] = boxed.x;
            trial[ 1 ] = boxed.y;
            trial[ 2 ] = boxed.z;
            float trialTotal;
            float trialFieldCost = knownResiduals( &fp, trial, strength, rowWeight, e1, &trialTotal );
            if ( trialTotal < total ) {
                for ( int i = 0; i < KNOWN_PARAMS; i++ )
                    q[ i ] = trial[ i ];
                for ( int r = 0; r < rows; r++ )
                    e0[ r ] = e1[ r ];
                total = trialTotal;
                fieldCost = trialFieldCost;
                lambda = lambda > 3e-6f ? lambda / 3.0f : 1e-6f;
                accepted = true;
            } else {
                lambda *= 4.0f;
            }
        }
        result->iterations++;
        if ( !accepted || moved < LM_DONE_MM ) {
            break;
        }
    }

    // The error bar, from the Jacobian at the answer. The strength row stays in:
    // knowing the strength is information, and it is what shrinks sigma.z.
    {
        for ( int k = 0; k < KNOWN_PARAMS; k++ ) {
            float nudged[ KNOWN_PARAMS ];
            for ( int j = 0; j < KNOWN_PARAMS; j++ )
                nudged[ j ] = q[ j ];
            nudged[ k ] += step[ k ];
            float unused;
            knownResiduals( &fp, nudged, strength, rowWeight, e1, &unused );
            for ( int r = 0; r < rows; r++ )
                jac[ r ][ k ] = ( e1[ r ] - e0[ r ] ) / step[ k ];
        }
        int freedom = 3 * used - 5;
        float variance = fieldCost / ( freedom > 0 ? freedom : 1 );
        if ( variance < MAGFIT_NOISE_FLOOR_MT * MAGFIT_NOISE_FLOOR_MT ) {
            variance = MAGFIT_NOISE_FLOOR_MT * MAGFIT_NOISE_FLOOR_MT;
        }
        float* out[ 3 ] = { &result->sigma.x, &result->sigma.y, &result->sigma.z };
        for ( int axis = 0; axis < 3; axis++ ) {
            float a[ MAGFIT_MAX_PARAMS ][ MAGFIT_MAX_PARAMS ] = { { 0 } };
            float unit[ MAGFIT_MAX_PARAMS ] = { 0 };
            for ( int r = 0; r < rows; r++ ) {
                for ( int i = 0; i < KNOWN_PARAMS; i++ ) {
                    for ( int j = 0; j < KNOWN_PARAMS; j++ )
                        a[ i ][ j ] += jac[ r ][ i ] * jac[ r ][ j ];
                }
            }
            unit[ axis ] = 1.0f;
            *out[ axis ] = ( magFitSolveLinear( a, unit, KNOWN_PARAMS ) && unit[ axis ] > 0 ) ? sqrtf( variance * unit[ axis ] ) : 999.0f;
        }
    }

    result->position = { q[ 0 ], q[ 1 ], q[ 2 ] };
    result->moment = { strength * q[ 3 ], strength * q[ 4 ], strength * q[ 5 ] };
    result->strength = strength * sqrtf( q[ 3 ] * q[ 3 ] + q[ 4 ] * q[ 4 ] + q[ 5 ] * q[ 5 ] );
    result->residual = sqrtf( fieldCost / ( 3.0f * used ) );
    result->valid = result->residual <= maxMisfit * result->signal;
    return result->valid;
}
