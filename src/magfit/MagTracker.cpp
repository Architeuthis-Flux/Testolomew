// SPDX-License-Identifier: MIT
#include "MagTracker.h"

#include <math.h>

#define TRACK_INITIAL_SPEED_SIGMA 500.0f // mm/s: how fast a freshly seen probe might already be moving
#define TRACK_SHAFT_CONFIRM 3            // frames a big swing must persist to be believed
#define TRACK_ANGLE_SIGMA 0.05f          // radians (3 deg) of shaft error, for the cursor's bar

static float maxf( float a, float b ) { return a > b ? a : b; }

static float lengthOf( Vec3 v ) { return sqrtf( v.x * v.x + v.y * v.y + v.z * v.z ); }

static Vec3 normalised( Vec3 v ) {
    float l = lengthOf( v );
    if ( l < 1e-6f ) {
        Vec3 up = { 0, 0, 1 };
        return up;
    }
    Vec3 n = { v.x / l, v.y / l, v.z / l };
    return n;
}

void magTrackInit( MagTrack* t, float surfaceZ, float tipOffsetMm ) {
    MagTrack empty = { };
    *t = empty;
    t->enabled = true;
    t->smooth = true;
    t->cursorMode = MAGCURSOR_POINTED;
    t->surfaceZ = surfaceZ;
    t->tipOffsetMm = tipOffsetMm;
    t->accelSigma = MAGTRACK_ACCEL_SIGMA;
    t->sigmaFloorMm = MAGTRACK_SIGMA_FLOOR_MM;
    t->gate = MAGTRACK_GATE;
    t->maxReachMm = MAGTRACK_MAX_REACH_MM;
    t->oneEuroMinCutoff = MAGTRACK_ONE_EURO_MIN_CUTOFF;
    t->oneEuroBeta = MAGTRACK_ONE_EURO_BETA;
    t->viewMinCutoff = MAGTRACK_VIEW_MIN_CUTOFF;
    t->viewBeta = MAGTRACK_VIEW_BETA;
    t->shaftMinCutoff = MAGTRACK_SHAFT_MIN_CUTOFF;
    t->shaftBeta = MAGTRACK_SHAFT_BETA;
    t->hzHalfMm = MAGTRACK_HZ_HALF_MM;
    t->betaHalfMm = MAGTRACK_BETA_HALF_MM;
    t->roughHoldS = MAGTRACK_ROUGH_HOLD_S;
    magTrackReset( t );
}

void magTrackReset( MagTrack* t ) {
    t->state = MAGTRACK_NONE;
    t->hadProperFix = false;
    t->ageMs = 0;
    t->droppedRun = 0;
    t->shaftSwings = 0;
    t->haveShaft = false;
    t->lastDropped = false;
    t->lastGate = 0.0f;
    for ( int a = 0; a < 3; a++ ) {
        OneEuroAxis e = { false, 0.0f, 0.0f };
        t->euro[ a ] = e;
        t->viewEuro[ a ] = e;
        t->shaftEuro[ a ] = e;
        MagTrackAxis x = { 0, 0, 0, 0, 0 };
        t->axis[ a ] = x;
    }
    t->accepted = t->dropped = t->reinits = t->coasted = 0;
}

// ---- the cursor ------------------------------------------------------------

Vec3 magTrackPointer( float surfaceZ, float maxReachMm, Vec3 tip, Vec3 shaft, float* reachMm ) {
    Vec3 cursor = tip;
    *reachMm = 0.0f;
    float drop = tip.z - surfaceZ;
    if ( drop <= MAGTRACK_CONTACT_MM ) {
        return cursor; // touching (in a hole, on the surface, or within a hair of it): the point is the cursor
    }
    cursor.z = surfaceZ;
    // Down the shaft to the plane: a horizontal reach of drop * tan(tilt),
    // capped, in the direction the shaft leans away from vertical (the point
    // is DOWN the shaft from the magnet, so the cursor lies on the -shaft side).
    float lean = sqrtf( shaft.x * shaft.x + shaft.y * shaft.y );
    if ( lean < 1e-4f ) {
        return cursor;
    }
    // tan(tilt), the tilt no flatter than MAGTRACK_MAX_POINT_TILT_DEG.
    float maxTan = tanf( MAGTRACK_MAX_POINT_TILT_DEG * (float)M_PI / 180.0f );
    float tanTilt = shaft.z > 1e-3f ? lean / shaft.z : maxTan;
    if ( tanTilt > maxTan ) {
        tanTilt = maxTan;
    }
    float reach = drop * tanTilt;
    if ( reach > maxReachMm ) {
        reach = maxReachMm;
    }
    cursor.x = tip.x - reach * shaft.x / lean;
    cursor.y = tip.y - reach * shaft.y / lean;
    *reachMm = reach;
    return cursor;
}

Vec3 magTrackCursorOf( const MagTrack* t, Vec3 tip, Vec3 shaft, float* reachMm ) {
    if ( t->cursorMode == MAGCURSOR_UNDER ) {
        *reachMm = 0.0f;
        Vec3 cursor = tip;
        if ( tip.z > t->surfaceZ ) {
            cursor.z = t->surfaceZ; // straight under the tip; in a hole, the point itself
        }
        return cursor;
    }
    return magTrackPointer( t->surfaceZ, t->maxReachMm, tip, shaft, reachMm );
}

// The 1-Euro filter: low-pass with a cutoff that rises with the (filtered) speed.
static float oneEuroAlpha( float cutoffHz, float dtS ) {
    float tau = 1.0f / ( 2.0f * (float)M_PI * cutoffHz );
    return 1.0f / ( 1.0f + tau / dtS );
}

static float oneEuro( OneEuroAxis* e, float x, float dtS, float minCutoff, float beta ) {
    if ( !e->started ) {
        e->started = true;
        e->x = x;
        e->dx = 0.0f;
        return x;
    }
    float dx = ( x - e->x ) / dtS;
    float ad = oneEuroAlpha( MAGTRACK_ONE_EURO_D_CUTOFF, dtS );
    e->dx += ad * ( dx - e->dx );
    float cutoff = minCutoff + beta * fabsf( e->dx );
    float a = oneEuroAlpha( cutoff, dtS );
    e->x += a * ( x - e->x );
    return e->x;
}

// ---- the filter ------------------------------------------------------------

static void predictAxis( MagTrackAxis* a, float dt, float q ) {
    a->p += a->v * dt;
    float dt2 = dt * dt;
    a->p00 += 2.0f * dt * a->p01 + dt2 * a->p11 + q * dt2 * dt2 * 0.25f;
    a->p01 += dt * a->p11 + q * dt2 * dt * 0.5f;
    a->p11 += q * dt2;
}

static void updateAxis( MagTrackAxis* a, float z, float r ) {
    float s = a->p00 + r;
    float k0 = a->p00 / s, k1 = a->p01 / s;
    float nu = z - a->p;
    a->p += k0 * nu;
    a->v += k1 * nu;
    float p00 = a->p00, p01 = a->p01;
    a->p00 = p00 - k0 * p00;
    a->p01 = p01 - k0 * p01;
    a->p11 = a->p11 - k1 * p01;
    if ( a->p00 < 0.0f )
        a->p00 = 0.0f;
    if ( a->p11 < 0.0f )
        a->p11 = 0.0f;
}

static void startAxis( MagTrackAxis* a, float z, float r ) {
    a->p = z;
    a->v = 0.0f;
    a->p00 = r;
    a->p01 = 0.0f;
    a->p11 = TRACK_INITIAL_SPEED_SIGMA * TRACK_INITIAL_SPEED_SIGMA;
}

static void startTrack( MagTrack* t, const MagTrackInput* in, const float r[ 3 ] ) {
    startAxis( &t->axis[ 0 ], in->position.x, r[ 0 ] );
    startAxis( &t->axis[ 1 ], in->position.y, r[ 1 ] );
    startAxis( &t->axis[ 2 ], in->position.z, r[ 2 ] );
    t->hadProperFix = in->valid;
    t->droppedRun = 0;
    t->haveShaft = false;
    t->shaftSwings = 0;
    for ( int a = 0; a < 3; a++ ) {
        t->euro[ a ].started = false;
        t->viewEuro[ a ].started = false;
    }
}

static void finishFrame( MagTrack* t, float dtS, bool roughHeld ) {
    t->position = { t->axis[ 0 ].p, t->axis[ 1 ].p, t->axis[ 2 ].p };
    t->velocity = { t->axis[ 0 ].v, t->axis[ 1 ].v, t->axis[ 2 ].v };
    t->sigma = { maxf( sqrtf( t->axis[ 0 ].p00 ), t->sigmaFloorMm ), maxf( sqrtf( t->axis[ 1 ].p00 ), t->sigmaFloorMm ), maxf( sqrtf( t->axis[ 2 ].p00 ), t->sigmaFloorMm ) };
    // The far glow, held between rough fixes: the filter's covariance would
    // run away in a second (its process noise is a hand's acceleration), and
    // the glow with it; instead the last rough fix's bar spreads slowly.
    if ( t->state == MAGTRACK_ROUGH ) {
        if ( roughHeld ) {
            float spread = MAGTRACK_ROUGH_SPREAD_MM_S * dtS;
            t->roughSigma.x += spread;
            t->roughSigma.y += spread;
            t->roughSigma.z += spread;
            t->sigma = t->roughSigma;
        } else {
            t->roughSigma = t->sigma;
        }
    }
    // The filter's covariance was widened for the fixes' correlation (the
    // field smoothing), which keeps it honest between fixes but makes it
    // claim less than a single fix does. While fixes are coming in, the
    // track is at least as good as the last one.
    if ( t->state == MAGTRACK_TRACKING ) {
        if ( t->lastFixSigma.x < t->sigma.x )
            t->sigma.x = t->lastFixSigma.x;
        if ( t->lastFixSigma.y < t->sigma.y )
            t->sigma.y = t->lastFixSigma.y;
        if ( t->lastFixSigma.z < t->sigma.z )
            t->sigma.z = t->lastFixSigma.z;
    }
    if ( !t->haveShaft ) {
        Vec3 up = { 0, 0, 1 };
        t->shaft = up;
    }
    t->tiltDeg = acosf( t->shaft.z > 1.0f ? 1.0f : ( t->shaft.z < -1.0f ? -1.0f : t->shaft.z ) ) * 180.0f / (float)M_PI;
    t->tip = { t->position.x - t->tipOffsetMm * t->shaft.x, t->position.y - t->tipOffsetMm * t->shaft.y, t->position.z - t->tipOffsetMm * t->shaft.z };
    t->rawCursor = magTrackCursorOf( t, t->tip, t->shaft, &t->reachMm );
    // The filters slow with the point's height (MAGTRACK_HZ_HALF_MM, the
    // Hz; MAGTRACK_BETA_HALF_MM, the betas): a far fix is a noisy one, and
    // calm "about here" is what it should show.
    float height = t->tip.z - t->surfaceZ;
    if ( height < 0.0f )
        height = 0.0f;
    float kHz = t->hzHalfMm > 0.0f ? 1.0f / ( 1.0f + height / t->hzHalfMm ) : 1.0f;
    float kBeta = t->betaHalfMm > 0.0f ? 1.0f / ( 1.0f + height / t->betaHalfMm ) : 1.0f;
    float cursorCutoff = t->oneEuroMinCutoff * kHz, cursorBeta = t->oneEuroBeta * kBeta;
    float viewCutoff = t->viewMinCutoff * kHz, viewBeta = t->viewBeta * kBeta;
    if ( !t->smooth ) {
        // No smoothing: the track's own output as it is (the bare fix with
        // the tracker off), and the filters start afresh when switched on.
        t->cursor = t->rawCursor;
        t->viewPosition = t->position;
        for ( int a = 0; a < 3; a++ ) {
            OneEuroAxis e = { false, 0.0f, 0.0f };
            t->euro[ a ] = e;
            t->viewEuro[ a ] = e;
        }
    } else {
        t->cursor.x = oneEuro( &t->euro[ 0 ], t->rawCursor.x, dtS, cursorCutoff, cursorBeta );
        t->cursor.y = oneEuro( &t->euro[ 1 ], t->rawCursor.y, dtS, cursorCutoff, cursorBeta );
        t->cursor.z = t->rawCursor.z;
        // What the scene draws: the same magnet through a filter of its own, so
        // the picture can be calmer than the track without slowing the cursor.
        t->viewPosition.x = oneEuro( &t->viewEuro[ 0 ], t->position.x, dtS, viewCutoff, viewBeta );
        t->viewPosition.y = oneEuro( &t->viewEuro[ 1 ], t->position.y, dtS, viewCutoff, viewBeta );
        t->viewPosition.z = oneEuro( &t->viewEuro[ 2 ], t->position.z, dtS, viewCutoff, viewBeta );
    }
    t->viewTip = { t->viewPosition.x - t->tipOffsetMm * t->shaft.x, t->viewPosition.y - t->tipOffsetMm * t->shaft.y, t->viewPosition.z - t->tipOffsetMm * t->shaft.z };
    // The cursor's bar: the track's, plus what a few degrees of shaft error do
    // over the reach, plus the tip offset's share of the same.
    float xy = sqrtf( t->sigma.x * t->sigma.x + t->sigma.y * t->sigma.y );
    t->cursorSigmaMm = xy + TRACK_ANGLE_SIGMA * ( t->reachMm + t->tipOffsetMm );
}

void magTrackUpdate( MagTrack* t, float dtS, const MagTrackInput* in ) {
    if ( dtS < 1e-4f )
        dtS = 1e-4f;
    if ( dtS > 1.0f )
        dtS = 1.0f;

    if ( !t->enabled ) {
        // Straight through, for comparing: this frame's fix is the track.
        if ( in->valid || in->rough ) {
            t->state = in->valid ? MAGTRACK_TRACKING : MAGTRACK_ROUGH;
            for ( int a = 0; a < 3; a++ ) {
                float z = a == 0 ? in->position.x : ( a == 1 ? in->position.y : in->position.z );
                float s = a == 0 ? in->sigma.x : ( a == 1 ? in->sigma.y : in->sigma.z );
                t->axis[ a ].p = z;
                t->axis[ a ].v = 0.0f;
                t->axis[ a ].p00 = s * s;
            }
            t->lastFixSigma = { maxf( in->sigma.x, t->sigmaFloorMm ), maxf( in->sigma.y, t->sigmaFloorMm ), maxf( in->sigma.z, t->sigmaFloorMm ) };
            if ( in->haveShaft ) {
                t->shaft = in->shaft;
                t->haveShaft = true;
            }
            t->accepted++;
            t->ageMs = 0;
            finishFrame( t, dtS, false );
        } else {
            t->state = MAGTRACK_NONE;
        }
        return;
    }

    // Predict: the track moves on at its velocity, its uncertainty grows. A
    // rough track (12 mm fixes from far away) is asked for "about here",
    // not for following a hand: it averages more, with its own smaller
    // process noise.
    float accel = t->state == MAGTRACK_ROUGH ? MAGTRACK_ROUGH_ACCEL_SIGMA : t->accelSigma;
    float q = accel * accel;
    if ( t->state != MAGTRACK_NONE ) {
        for ( int a = 0; a < 3; a++ ) {
            predictAxis( &t->axis[ a ], dtS, q );
        }
        t->ageMs += (uint32_t)( dtS * 1000.0f + 0.5f );
    }

    bool measured = in->valid || in->rough;
    t->lastDropped = false;
    if ( measured ) {
        // Its weight. The bar is floored for the array's systematic error, and
        // widened for the field smoothing that made it (a fix made of fields
        // smoothed over 1/alpha frames repeats most of the previous fix's noise).
        float alpha = in->alpha;
        if ( alpha < 0.05f )
            alpha = 0.05f;
        if ( alpha > 1.0f )
            alpha = 1.0f;
        float inflate = ( 2.0f - alpha ) / alpha;
        float r[ 3 ];
        float s[ 3 ] = { in->sigma.x, in->sigma.y, in->sigma.z };
        float z[ 3 ] = { in->position.x, in->position.y, in->position.z };
        for ( int a = 0; a < 3; a++ ) {
            float sd = maxf( s[ a ], t->sigmaFloorMm );
            r[ a ] = sd * sd * inflate;
        }

        if ( t->state == MAGTRACK_NONE ) {
            startTrack( t, in, r );
            t->accepted++;
        } else {
            // The gate: how far is this fix from where the track expects it,
            // in units of their combined uncertainty; and the speed it implies
            // since the track was last confirmed.
            float d2 = 0.0f, jump2 = 0.0f;
            for ( int a = 0; a < 3; a++ ) {
                float nu = z[ a ] - t->axis[ a ].p;
                d2 += nu * nu / ( t->axis[ a ].p00 + r[ a ] );
                jump2 += nu * nu;
            }
            t->lastGate = sqrtf( d2 );
            float sinceS = maxf( dtS, t->ageMs / 1000.0f );
            bool tooFast = sqrtf( jump2 ) / sinceS > MAGTRACK_MAX_SPEED_MM_S;
            float g2 = t->gate * t->gate;
            if ( d2 <= MAGTRACK_GATE_DROP * MAGTRACK_GATE_DROP && !tooFast ) {
                // Soft (Huber): inside the gate a fix counts in full; outside
                // it counts for less, its variance grown by how far out it is
                // (g^2/d^2), never for nothing until the drop line - a hard
                // gate turned a fast onset into a staircase of drops and
                // restarts (docs/magnetometer-fusion-prior-art.md 5.2).
                float w = d2 <= g2 ? 1.0f : g2 / d2;
                for ( int a = 0; a < 3; a++ ) {
                    updateAxis( &t->axis[ a ], z[ a ], r[ a ] / w );
                }
                t->accepted++;
                t->droppedRun = 0;
            } else {
                t->dropped++;
                t->lastDropped = true;
                int slot = t->droppedRun < MAGTRACK_REINIT_AFTER ? t->droppedRun : MAGTRACK_REINIT_AFTER - 1;
                if ( t->droppedRun >= MAGTRACK_REINIT_AFTER ) {
                    for ( int k = 1; k < MAGTRACK_REINIT_AFTER; k++ )
                        t->droppedAt[ k - 1 ] = t->droppedAt[ k ];
                }
                t->droppedAt[ slot ] = in->position;
                t->droppedRun++;
                // Enough dropped fixes that agree with each other: the track is
                // what is wrong. "Agree" allows for a moving hand: the middle
                // one lies on the line between the first and the last (random
                // rubbish is not collinear), and the whole run is not faster
                // than a hand. The restarted track takes its velocity from them
                // too, so it is already moving.
                if ( t->droppedRun >= MAGTRACK_REINIT_AFTER ) {
                    const Vec3& p0 = t->droppedAt[ 0 ];
                    const Vec3& p1 = t->droppedAt[ MAGTRACK_REINIT_AFTER / 2 ];
                    const Vec3& p2 = t->droppedAt[ MAGTRACK_REINIT_AFTER - 1 ];
                    Vec3 mid = { 0.5f * ( p0.x + p2.x ), 0.5f * ( p0.y + p2.y ), 0.5f * ( p0.z + p2.z ) };
                    Vec3 off = { p1.x - mid.x, p1.y - mid.y, p1.z - mid.z };
                    Vec3 span = { p2.x - p0.x, p2.y - p0.y, p2.z - p0.z };
                    float spanS = dtS * ( MAGTRACK_REINIT_AFTER - 1 );
                    bool agree = lengthOf( off ) <= MAGTRACK_REINIT_AGREE_MM && lengthOf( span ) <= MAGTRACK_MAX_SPEED_MM_S * spanS;
                    if ( agree ) {
                        startTrack( t, in, r );
                        t->axis[ 0 ].v = span.x / spanS;
                        t->axis[ 1 ].v = span.y / spanS;
                        t->axis[ 2 ].v = span.z / spanS;
                        t->reinits++;
                        t->accepted++;
                        t->lastDropped = false;
                    }
                }
            }
        }
        if ( !t->lastDropped ) {
            if ( in->valid ) {
                t->ageMs = 0;
                t->state = MAGTRACK_TRACKING;
                t->hadProperFix = true;
                t->lastFixSigma = { maxf( s[ 0 ], t->sigmaFloorMm ), maxf( s[ 1 ], t->sigmaFloorMm ), maxf( s[ 2 ], t->sigmaFloorMm ) };
            } else if ( !t->hadProperFix || t->ageMs > MAGTRACK_COAST_MS ) {
                t->state = MAGTRACK_ROUGH;
            } else {
                // A rough fix that agrees with a fresh track: the filter took it
                // at its own (wide) weight, and the track goes on as a track.
                // Calling this "coasting" dimmed the LEDs and swapped the error
                // bar on 30 % of frames at a hover, where the fit's bar
                // straddles the rough line (2026-09-23, the far pencil).
                t->state = MAGTRACK_TRACKING;
            }
            t->sinceAnyMs = 0;
        }
    }

    if ( t->state != MAGTRACK_NONE && ( !measured || t->lastDropped ) ) {
        // Nothing usable this frame: coast. With NO fix at all the velocity
        // dies away (a hand that vanished is not still moving at the same
        // speed); with fixes arriving but dropped it is kept, else the track
        // would fall further behind a fast hand with every fix it refused.
        if ( !measured ) {
            // The far glow holds still: the velocity a rough track carries is
            // the rough fixes' jitter (12 mm bars at 100 Hz), not motion.
            float decay = t->state == MAGTRACK_ROUGH ? 0.0f : expf( -dtS / MAGTRACK_COAST_TAU_S );
            for ( int a = 0; a < 3; a++ ) {
                t->axis[ a ].v *= decay;
            }
        }
        t->sinceAnyMs += (uint32_t)( dtS * 1000.0f + 0.5f );
        if ( t->state == MAGTRACK_TRACKING ) {
            t->state = MAGTRACK_COASTING;
            t->coasted++;
        }
        // A real track gives up after MAGTRACK_COAST_MS; the far glow is
        // held much longer (roughHoldS): rough fixes come at the cold-start
        // pace and not every attempt lands, and "about here" stays true for
        // seconds while the probe hovers at the edge of the array's reach.
        uint32_t limit = t->state == MAGTRACK_ROUGH ? (uint32_t)( t->roughHoldS * 1000.0f ) : MAGTRACK_COAST_MS;
        if ( t->sinceAnyMs > limit ) {
            t->state = MAGTRACK_NONE;
        }
    }
    bool roughHeld = t->state == MAGTRACK_ROUGH && ( !measured || t->lastDropped );

    if ( t->state == MAGTRACK_NONE ) {
        t->haveShaft = false;
        return;
    }

    // The shaft, through a 1-Euro filter of its own (each component, then
    // renormalised): heavy smoothing at rest, none to speak of while the
    // pencil turns, which a fixed time constant could not give - 150 ms of
    // lag on every turn, or a shaft that trembled. A big swing has to
    // persist to be believed (one frame's wild direction is a glitch).
    if ( in->valid && in->haveShaft && !t->lastDropped ) {
        Vec3 s = normalised( in->shaft );
        if ( !t->haveShaft ) {
            t->shaft = s;
            t->haveShaft = true;
            t->shaftSwings = 0;
            for ( int a = 0; a < 3; a++ ) {
                OneEuroAxis e = { false, 0.0f, 0.0f };
                t->shaftEuro[ a ] = e;
            }
        } else {
            float dot = s.x * t->shaft.x + s.y * t->shaft.y + s.z * t->shaft.z;
            float swingDeg = acosf( dot > 1.0f ? 1.0f : ( dot < -1.0f ? -1.0f : dot ) ) * 180.0f / (float)M_PI;
            if ( swingDeg > MAGTRACK_SHAFT_FLIP_DEG && ++t->shaftSwings < TRACK_SHAFT_CONFIRM ) {
                // wait
            } else {
                if ( t->shaftSwings >= TRACK_SHAFT_CONFIRM || !t->smooth ) {
                    t->shaft = s; // it meant it
                    for ( int a = 0; a < 3; a++ ) {
                        OneEuroAxis e = { false, 0.0f, 0.0f };
                        t->shaftEuro[ a ] = e;
                    }
                } else {
                    Vec3 m = { oneEuro( &t->shaftEuro[ 0 ], s.x, dtS, t->shaftMinCutoff, t->shaftBeta ), oneEuro( &t->shaftEuro[ 1 ], s.y, dtS, t->shaftMinCutoff, t->shaftBeta ),
                               oneEuro( &t->shaftEuro[ 2 ], s.z, dtS, t->shaftMinCutoff, t->shaftBeta ) };
                    t->shaft = normalised( m );
                }
                t->shaftSwings = 0;
            }
        }
    }

    finishFrame( t, dtS, roughHeld );
}
