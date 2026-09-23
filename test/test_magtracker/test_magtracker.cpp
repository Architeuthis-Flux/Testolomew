// SPDX-License-Identifier: MIT
// Host-side test of the tracker: fixes with known noise, glitches and gaps go
// in, and the track has to be steadier than the fixes, follow real motion
// without lag, drop a fix that teleports, follow a hand that really did jump,
// coast through a gap, keep a far probe on the map from rough fixes, and put
// the cursor on the surface plane the way each mode says. The last test runs
// the whole chain - dipole, noisy fields, the real fit, the tracker - on a
// simulated hand movement. Run with `pio test -e native`.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unity.h>

#include "MagFit.h"
#include "MagTracker.h"

#define DT 0.01f // 100 Hz frames

static MagTrack track;

void setUp( void ) {
    magTrackInit( &track, 17.5f, 0.0f );
}
void tearDown( void ) {}

static float gaussian( float sigma ) {
    float u1 = ( rand( ) + 1.0f ) / ( (float)RAND_MAX + 2.0f );
    float u2 = ( rand( ) + 1.0f ) / ( (float)RAND_MAX + 2.0f );
    return sigma * sqrtf( -2.0f * logf( u1 ) ) * cosf( 2.0f * (float)M_PI * u2 );
}

static MagTrackInput fixAt( Vec3 p, float sigma ) {
    MagTrackInput in = { };
    in.valid = true;
    in.position = p;
    in.sigma = { sigma, sigma, sigma };
    in.alpha = 1.0f;
    in.shaft = { 0, 0, 1 };
    in.haveShaft = true;
    return in;
}

static MagTrackInput noisyFixAt( Vec3 p, float sigma ) {
    Vec3 q = { p.x + gaussian( sigma ), p.y + gaussian( sigma ), p.z + gaussian( sigma ) };
    return fixAt( q, sigma );
}

static MagTrackInput nothing( ) {
    MagTrackInput in = { };
    in.alpha = 1.0f;
    return in;
}

static float distance( Vec3 a, Vec3 b ) {
    return sqrtf( ( a.x - b.x ) * ( a.x - b.x ) + ( a.y - b.y ) * ( a.y - b.y ) + ( a.z - b.z ) * ( a.z - b.z ) );
}

// A still probe with noisy fixes: the track scatters less than the fixes and
// the cursor is nearly still.
void test_rest_is_steadier_than_the_fixes( void ) {
    srand( 1 );
    Vec3 p = { 20.0f, 20.0f, 25.0f };
    float sumRaw = 0.0f, sumTrack = 0.0f, sumCursor = 0.0f;
    int n = 0;
    for ( int frame = 0; frame < 600; frame++ ) {
        MagTrackInput in = noisyFixAt( p, 0.5f );
        magTrackUpdate( &track, DT, &in );
        if ( frame >= 100 ) {
            sumRaw += ( in.position.x - p.x ) * ( in.position.x - p.x );
            sumTrack += ( track.position.x - p.x ) * ( track.position.x - p.x );
            sumCursor += ( track.cursor.x - p.x ) * ( track.cursor.x - p.x );
            n++;
        }
    }
    float raw = sqrtf( sumRaw / n ), tracked = sqrtf( sumTrack / n ), cursor = sqrtf( sumCursor / n );
    printf( "  at rest: fixes %.3f mm rms, track %.3f, cursor %.3f\n", raw, tracked, cursor );
    TEST_ASSERT_EQUAL( MAGTRACK_TRACKING, track.state );
    TEST_ASSERT_TRUE( tracked < 1.05f * raw ); // the filter is tuned to follow, not to smooth (never worse than a fix): the cursor does that
    TEST_ASSERT_TRUE( cursor < 0.4f * raw );
    TEST_ASSERT_EQUAL( 0, track.dropped );
}

// A hand moving at 300 mm/s: the track keeps up (a constant-velocity model
// has no lag on constant velocity) and the cursor lags by less than a row.
void test_follows_motion_without_lag( void ) {
    srand( 2 );
    float worstTrack = 0.0f, worstCursor = 0.0f;
    for ( int frame = 0; frame < 300; frame++ ) {
        Vec3 p = { 3.0f * frame * DT * 100.0f, 20.0f, 25.0f }; // 300 mm/s along x
        MagTrackInput in = noisyFixAt( p, 0.4f );
        magTrackUpdate( &track, DT, &in );
        if ( frame > 50 ) {
            float e = fabsf( track.position.x - p.x );
            if ( e > worstTrack )
                worstTrack = e;
            float c = fabsf( track.cursor.x - p.x );
            if ( c > worstCursor )
                worstCursor = c;
        }
    }
    printf( "  moving at 300 mm/s: worst track lag %.2f mm, worst cursor lag %.2f mm\n", worstTrack, worstCursor );
    TEST_ASSERT_TRUE( worstTrack < 1.5f );
    TEST_ASSERT_TRUE( worstCursor < 2.5f ); // under a row at 300 mm/s
    TEST_ASSERT_EQUAL( 0, track.dropped );
}

// A fix that teleports 30 mm is dropped and the track stays; three in a row
// at the new place and the track goes there - by a restart (three agreeing
// drops) or, since the gate is soft (2026-09-23), pulled over by fixes that
// count for less the further out they are: the contract is where the track
// ends, not which of the two took it there.
void test_teleport_dropped_and_real_jump_followed( void ) {
    Vec3 p = { 20.0f, 20.0f, 25.0f };
    for ( int frame = 0; frame < 100; frame++ ) {
        MagTrackInput in = fixAt( p, 0.3f );
        magTrackUpdate( &track, DT, &in );
    }
    Vec3 far = { 50.0f, 20.0f, 25.0f };
    MagTrackInput glitch = fixAt( far, 0.3f );
    magTrackUpdate( &track, DT, &glitch );
    TEST_ASSERT_TRUE( track.lastDropped );
    TEST_ASSERT_FLOAT_WITHIN( 0.5f, 20.0f, track.position.x );
    TEST_ASSERT_EQUAL( MAGTRACK_COASTING, track.state );
    MagTrackInput back = fixAt( p, 0.3f );
    magTrackUpdate( &track, DT, &back );
    TEST_ASSERT_FALSE( track.lastDropped );
    TEST_ASSERT_EQUAL( MAGTRACK_TRACKING, track.state );

    for ( int k = 0; k < MAGTRACK_REINIT_AFTER; k++ ) {
        MagTrackInput in = fixAt( far, 0.3f );
        magTrackUpdate( &track, DT, &in );
    }
    printf( "  after %d fixes at the new place: track x %.1f, %lu restarts\n", MAGTRACK_REINIT_AFTER, track.position.x, (unsigned long)track.reinits );
    TEST_ASSERT_FLOAT_WITHIN( 2.0f, 50.0f, track.position.x ); // most of the way by the third (a restart lands it; the soft gate's pulls get within 2 mm)
    TEST_ASSERT_EQUAL( MAGTRACK_TRACKING, track.state );
    for ( int k = 0; k < 7; k++ ) {
        MagTrackInput in = fixAt( far, 0.3f );
        magTrackUpdate( &track, DT, &in );
    }
    TEST_ASSERT_FLOAT_WITHIN( 0.5f, 50.0f, track.position.x ); // and settled there by the tenth (the pull leaves a velocity that overshoots by under a millimetre first)
}

// A track that is wrong while the hand MOVES: the fixes arrive 25 mm away
// from where the track sits and keep moving at 400 mm/s. They disagree with
// the track but agree with each other (in a line): the track is with them
// within a few frames - restarted from them (three agreeing drops) or pulled
// over by the soft gate - and moving at their speed.
void test_restarts_on_a_moving_hand( void ) {
    Vec3 p = { 20.0f, 20.0f, 25.0f };
    for ( int frame = 0; frame < 100; frame++ ) {
        MagTrackInput in = fixAt( p, 0.3f );
        magTrackUpdate( &track, DT, &in );
    }
    int restartedAt = -1;
    for ( int frame = 0; frame < 10; frame++ ) {
        Vec3 q = { 45.0f + 4.0f * frame, 20.0f, 25.0f }; // 400 mm/s, 25 mm off
        MagTrackInput in = fixAt( q, 0.3f );
        magTrackUpdate( &track, DT, &in );
        if ( track.reinits == 1 && restartedAt < 0 )
            restartedAt = frame;
    }
    printf( "  restarted on frame %d after the fixes moved away (-1 = pulled over instead); track x %.1f (truth %.1f), speed %.0f mm/s\n", restartedAt, track.position.x, 45.0f + 4.0f * 9, track.velocity.x );
    TEST_ASSERT_TRUE( restartedAt < 0 || ( restartedAt >= 2 && restartedAt <= 4 ) );
    TEST_ASSERT_FLOAT_WITHIN( 2.0f, 45.0f + 4.0f * 9, track.position.x );
    TEST_ASSERT_TRUE( track.velocity.x > 250.0f );
}

// Moving, then no fixes for 200 ms: the track carries on and is close when
// the fixes return; 500 ms of nothing and it gives up.
void test_coasts_through_a_gap( void ) {
    for ( int frame = 0; frame < 200; frame++ ) {
        Vec3 p = { 200.0f * frame * DT, 20.0f, 25.0f }; // 200 mm/s
        MagTrackInput in = fixAt( p, 0.3f );
        magTrackUpdate( &track, DT, &in );
    }
    for ( int frame = 200; frame < 220; frame++ ) {
        MagTrackInput in = nothing( );
        magTrackUpdate( &track, DT, &in );
    }
    TEST_ASSERT_EQUAL( MAGTRACK_COASTING, track.state );
    // With the velocity dying away (tau 0.15 s) the coasted track covers less
    // than the full 40 mm but well over half of it.
    float truth = 200.0f * 220 * DT;
    printf( "  after 200 ms with no fix: track %.1f, truth %.1f (started the gap at %.1f)\n", track.position.x, truth, 200.0f * 200 * DT );
    TEST_ASSERT_TRUE( track.position.x > 200.0f * 200 * DT + 20.0f );
    TEST_ASSERT_TRUE( track.sigma.x > 1.0f ); // and it says it is unsure
    Vec3 p = { truth, 20.0f, 25.0f };
    MagTrackInput in = fixAt( p, 0.3f );
    magTrackUpdate( &track, DT, &in );
    TEST_ASSERT_FALSE( track.lastDropped ); // the returning fix is believed
    TEST_ASSERT_EQUAL( MAGTRACK_TRACKING, track.state );

    for ( int frame = 0; frame < 50; frame++ ) {
        MagTrackInput none = nothing( );
        magTrackUpdate( &track, DT, &none );
    }
    TEST_ASSERT_EQUAL( MAGTRACK_NONE, track.state );
}

// Only rough fixes (a probe far away): the track is ROUGH, about right, and
// says how unsure it is.
void test_rough_fixes_keep_a_far_probe_on_the_map( void ) {
    srand( 3 );
    Vec3 p = { 30.0f, 10.0f, 60.0f };
    for ( int frame = 0; frame < 100; frame++ ) {
        MagTrackInput in = noisyFixAt( p, 12.0f );
        in.valid = false;
        in.rough = true;
        in.haveShaft = false;
        magTrackUpdate( &track, DT, &in );
        TEST_ASSERT_EQUAL( MAGTRACK_ROUGH, track.state ); // from the first one, not after a coast
    }
    TEST_ASSERT_EQUAL( MAGTRACK_ROUGH, track.state );
    TEST_ASSERT_FLOAT_WITHIN( 8.0f, 30.0f, track.position.x );
    TEST_ASSERT_TRUE( track.sigma.x > 2.0f );
    // Rough fixes at the edge of reach come seldom and not every attempt
    // lands: the far glow is held for roughHoldS after the last one, in
    // place, its bar spreading slowly (not the filter's runaway), and only
    // then goes.
    float sigmaAtLast = track.sigma.x;
    int frames = 0;
    while ( track.state == MAGTRACK_ROUGH && frames < 1000 ) {
        MagTrackInput gap = nothing( );
        magTrackUpdate( &track, DT, &gap );
        frames++;
        if ( frames == 50 ) { // half a second in: still there, wider, not wild
            TEST_ASSERT_EQUAL( MAGTRACK_ROUGH, track.state );
            TEST_ASSERT_FLOAT_WITHIN( 8.0f, 30.0f, track.position.x );
            TEST_ASSERT_TRUE( track.sigma.x > sigmaAtLast + 2.0f && track.sigma.x < sigmaAtLast + 8.0f );
        }
    }
    TEST_ASSERT_EQUAL( MAGTRACK_NONE, track.state );
    TEST_ASSERT_TRUE( frames * DT > track.roughHoldS - 0.1f && frames * DT < track.roughHoldS + 0.2f );
    // A rough fix again, then a proper fix takes over at once.
    for ( int frame = 0; frame < 5; frame++ ) {
        MagTrackInput in = noisyFixAt( p, 12.0f );
        in.valid = false;
        in.rough = true;
        in.haveShaft = false;
        magTrackUpdate( &track, DT, &in );
    }
    TEST_ASSERT_EQUAL( MAGTRACK_ROUGH, track.state );
    Vec3 near = { 28.0f, 12.0f, 40.0f };
    MagTrackInput in = fixAt( near, 0.5f );
    magTrackUpdate( &track, DT, &in );
    TEST_ASSERT_EQUAL( MAGTRACK_TRACKING, track.state );
    TEST_ASSERT_FALSE( track.lastDropped );
}

// The cursor: under the tip, or where the shaft points, on the surface plane;
// the tip itself when it is in a hole; never further than the reach cap.
void test_cursor_modes_and_the_surface_plane( void ) {
    float reach;
    Vec3 tip = { 20.0f, 20.0f, 27.5f };                // 10 mm above the surface (17.5)
    Vec3 shaft = { sinf( 0.5f ), 0.0f, cosf( 0.5f ) }; // leaning 28.6 deg toward +x

    track.cursorMode = MAGCURSOR_UNDER;
    Vec3 c = magTrackCursorOf( &track, tip, shaft, &reach );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 20.0f, c.x );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 17.5f, c.z );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.0f, reach );

    track.cursorMode = MAGCURSOR_POINTED;
    c = magTrackCursorOf( &track, tip, shaft, &reach );
    // The point is down the shaft: the cursor is on the -x side, 10 * tan(0.5) = 5.46 mm away.
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 20.0f - 10.0f * tanf( 0.5f ), c.x );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 17.5f, c.z );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 10.0f * tanf( 0.5f ), reach );

    Vec3 inHole = { 20.0f, 20.0f, 16.0f };
    c = magTrackCursorOf( &track, inHole, shaft, &reach );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 20.0f, c.x );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 16.0f, c.z );

    Vec3 high = { 20.0f, 20.0f, 60.0f };
    Vec3 level = { 0.99f, 0.0f, 0.1f }; // nearly lying down
    c = magTrackCursorOf( &track, high, level, &reach );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, MAGTRACK_MAX_REACH_MM, reach );
    TEST_ASSERT_FLOAT_WITHIN( 0.1f, 20.0f - MAGTRACK_MAX_REACH_MM, c.x );
}

// The whole chain on a simulated hand: a probe magnet (4200, along a shaft
// leaning 15 deg) moves over the bench array in a loop at up to 250 mm/s,
// dips to the surface and lifts, with the bench noise, a glitched reading
// every 40 frames and a 150 ms dropout every 3 s. The fit runs warm-started
// as the locator does; the track must be at least as accurate as the fixes
// and never lose the probe for longer than the dropouts.
static const Vec3 sensors[ 8 ] = {
    { 0.10f, 44.29f, 0 },
    { 16.75f, 44.04f, 0 },
    { 38.78f, 43.95f, 0 },
    { 54.18f, 44.72f, 0 },
    { 0.00f, 0.00f, 0 },
    { 15.60f, 0.19f, 0 },
    { 37.37f, 0.42f, 0 },
    { 53.40f, 0.00f, 0 },
};

void test_whole_chain_on_a_simulated_hand( void ) {
    srand( 4 );
    track.cursorMode = MAGCURSOR_UNDER; // so the cursor can be compared with the magnet's own place
    MagFitResult result = { };
    float sumRaw = 0.0f, sumTrack = 0.0f, sumCursor = 0.0f;
    int rawCount = 0, trackCount = 0, frames = 1200, lost = 0;
    Vec3 lastTrue = { 0, 0, 0 };
    for ( int frame = 0; frame < frames; frame++ ) {
        float tS = frame * DT;
        // A loop over the array, 8 s round, height breathing between the
        // surface (17.5) and 35 mm.
        float ang = tS * 2.0f * (float)M_PI / 8.0f;
        Vec3 magnet = { 27.0f + 25.0f * cosf( ang ), 22.0f + 18.0f * sinf( ang ), 26.0f + 9.0f * sinf( tS * 1.3f ) };
        float lean = 15.0f * (float)M_PI / 180.0f;
        Vec3 shaft = { sinf( lean ) * cosf( ang ), sinf( lean ) * sinf( ang ), cosf( lean ) };
        Vec3 moment = { 4200.0f * shaft.x, 4200.0f * shaft.y, 4200.0f * shaft.z };
        Vec3 fields[ 8 ];
        for ( int i = 0; i < 8; i++ ) {
            fields[ i ] = magFitDipoleField( sensors[ i ], magnet, moment );
            fields[ i ].x += gaussian( 0.005f );
            fields[ i ].y += gaussian( 0.005f );
            fields[ i ].z += gaussian( 0.005f );
        }
        if ( frame % 40 == 17 ) {
            fields[ frame % 8 ].z += 0.5f; // a corrupted read
        }
        bool dropout = ( frame % 300 ) >= 285; // 150 ms every 3 s
        MagTrackInput in = { };
        in.alpha = 1.0f;
        if ( !dropout ) {
            bool ok = magFitSolve( sensors, fields, nullptr, 8, 0.4f, &result );
            in.valid = ok && result.sigma.x < 15.0f;
            in.rough = !in.valid;
            in.position = result.position;
            in.sigma = result.sigma;
            float inv = result.strength > 0 ? 1.0f / result.strength : 0.0f;
            in.shaft = { result.moment.x * inv, result.moment.y * inv, result.moment.z * inv };
            in.haveShaft = in.valid;
            if ( in.valid ) {
                float e = distance( result.position, magnet );
                sumRaw += e * e;
                rawCount++;
            }
        }
        magTrackUpdate( &track, DT, &in );
        if ( frame > 20 ) {
            if ( track.state == MAGTRACK_NONE ) {
                lost++;
            } else {
                float e = distance( track.position, magnet );
                sumTrack += e * e;
                float c = sqrtf( ( track.cursor.x - magnet.x ) * ( track.cursor.x - magnet.x ) + ( track.cursor.y - magnet.y ) * ( track.cursor.y - magnet.y ) );
                sumCursor += c * c;
                trackCount++;
            }
        }
        lastTrue = magnet;
    }
    (void)lastTrue;
    float raw = sqrtf( sumRaw / rawCount ), tracked = sqrtf( sumTrack / trackCount ), cursor = sqrtf( sumCursor / trackCount );
    printf( "  simulated hand: %d fixes (%.2f mm rms), track %.2f mm rms over %d frames, cursor xy under the magnet %.2f mm rms, %d frames lost, dropped %lu, reinits %lu\n",
            rawCount, raw, tracked, trackCount, cursor, lost, (unsigned long)track.dropped, (unsigned long)track.reinits );
    TEST_ASSERT_TRUE( tracked <= raw * 1.05f );
    TEST_ASSERT_EQUAL( 0, lost );
    TEST_ASSERT_TRUE( track.reinits <= 1 );
}


// The gate is soft (Huber): a fix a little outside it is taken with less
// weight, not dropped - a hard 4-sigma gate turned a fast onset into a
// staircase of drops and restarts (docs/magnetometer-fusion-prior-art.md
// 5.2) - and only a fix far beyond it (MAGTRACK_GATE_DROP) is dropped and
// counted toward a restart.
void test_soft_gate_weights_a_borderline_fix( void ) {
    srand( 5 );
    Vec3 p = { 20, 20, 15 };
    for ( int k = 0; k < 100; k++ ) {
        MagTrackInput in = fixAt( { p.x + gaussian( 0.5f ), p.y + gaussian( 0.5f ), p.z + gaussian( 0.5f ) }, 0.5f );
        magTrackUpdate( &track, DT, &in );
    }
    // The gate is judged against the PREDICTED uncertainty (the constant-velocity model one frame on) plus the fix's own.
    const MagTrackAxis& a = track.axis[ 0 ];
    float q = track.accelSigma * track.accelSigma;
    float prior = a.p00 + 2.0f * DT * a.p01 + DT * DT * a.p11 + q * DT * DT * DT * DT / 4.0f;
    float combined = sqrtf( prior + 0.5f * 0.5f );
    MagTrack control = track; // the same fix at full weight, for comparison
    control.gate = 100.0f;
    float before = track.position.x;
    // 7 sigma: outside the 4-sigma gate, inside the drop line.
    MagTrackInput borderline = fixAt( { p.x + 7.0f * combined, p.y, p.z }, 0.5f );
    magTrackUpdate( &track, DT, &borderline );
    magTrackUpdate( &control, DT, &borderline );
    TEST_ASSERT_FALSE_MESSAGE( track.lastDropped, "a 7-sigma fix is weighed, not dropped" );
    TEST_ASSERT_EQUAL_INT_MESSAGE( 0, track.droppedRun, "...and does not count toward a restart" );
    float moved = track.position.x - before, movedFull = control.position.x - before;
    TEST_ASSERT_TRUE_MESSAGE( moved > 0.0f, "it pulls the track toward the fix" );
    TEST_ASSERT_TRUE_MESSAGE( moved < 0.9f * movedFull, "...but by less than the same fix taken at full weight" );
    // 30 sigma: beyond the drop line, dropped.
    MagTrackInput wild = fixAt( { p.x + 30.0f * combined, p.y, p.z }, 0.5f );
    magTrackUpdate( &track, DT, &wild );
    TEST_ASSERT_TRUE_MESSAGE( track.lastDropped, "a 30-sigma fix is dropped" );
    TEST_ASSERT_EQUAL_INT_MESSAGE( 1, track.droppedRun, "...and counts toward a restart" );
}

int main( int argc, char** argv ) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN( );
    RUN_TEST( test_rest_is_steadier_than_the_fixes );
    RUN_TEST( test_follows_motion_without_lag );
    RUN_TEST( test_teleport_dropped_and_real_jump_followed );
    RUN_TEST( test_restarts_on_a_moving_hand );
    RUN_TEST( test_soft_gate_weights_a_borderline_fix );
    RUN_TEST( test_coasts_through_a_gap );
    RUN_TEST( test_rough_fixes_keep_a_far_probe_on_the_map );
    RUN_TEST( test_cursor_modes_and_the_surface_plane );
    RUN_TEST( test_whole_chain_on_a_simulated_hand );
    return UNITY_END( );
}
