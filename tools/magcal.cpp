// SPDX-License-Identifier: MIT
// ---------------------------------------------------------------------------
// magcal - self-calibration of the magnetometer array from a recording.
//
// Wave one magnet around over the array while the firmware streams frames (the
// `f` console command), save the stream to a file, and this works out where
// each sensor really is and how it is turned - the way a camera rig calibrates
// itself from pictures of a moving target. It alternates two steps until they
// stop improving:
//   1. with the sensors as currently believed, fit the magnet (MagFit, the
//      firmware's own solver) in every frame;
//   2. with those magnet poses, move / turn / scale each sensor to best
//      explain its own readings.
//
// What fields cannot tell is fixed by convention: sensor ORIGIN sits at (0,0),
// sensor XAXIS lies on the +x axis, and the distance between them is given on
// the command line (the fields fix the array's shape but not its size).
//
// Build and run (host):
//   c++ -std=c++11 -O2 -I../src/magfit -I../src/common magcal.cpp ../src/magfit/MagFit.cpp -o magcal
//   ./magcal recording.txt <distance sensor4-sensor7 in mm>
// The recording must have been made with the table printed under "START".
// ---------------------------------------------------------------------------
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <vector>

#include "MagFit.h"

#define SENSORS 8
#define ORIGIN 4 // gauge: this sensor is (0, 0)...
#define XAXIS 7  // ...and this one is on the +x axis

struct Sensor {
    float x, y;  // mm
    float angle; // package rotation, degrees (the table's rotationDeg)
    float gain;  // what its readings are multiplied by
    // The full answer: board-frame field = m * (the sensor's own reading). To
    // start with it is just the rotation and gain above; the calibration then
    // frees all nine numbers, which takes in different gains on the three axes
    // (X/Y and Z are different Hall structures), axes that are not quite
    // square, and a package that is not quite flat.
    float m[ 3 ][ 3 ];
};

struct Frame {
    Vec3 reading[ SENSORS ]; // sensor frame, mT
    float signal;            // rms reading
    MagFitResult pose;
    bool used;
};

// START: the table the recording was made with. The `f` stream is in the board
// frame, so each reading has to be turned back into its sensor's own frame
// with the rotation and gain that were in MagArrayConfig.h AT THE TIME. Keep
// this equal to the firmware's table; the old one is here only so that the
// first recording (2026-09-17-bench-array-calibration.txt) still replays:
//   ./magcal recording.txt 53.4 100 first
static Sensor firstTable[ SENSORS ] = {
    { 0, 45, 90, 1 }, { 15, 45, 90, 1 }, { 30, 45, 90, 1 }, { 45, 45, 90, 1 },
    { 0, 0, 270, 1 }, { 15, 0, 270, 1 }, { 30, 0, 270, 1 }, { 45, 0, 270, 1 },
};
static Sensor sensors[ SENSORS ] = {
    { 0.10f, 44.29f, 85.0f, 0.987f }, { 16.75f, 44.04f, 89.3f, 0.985f }, { 38.78f, 43.95f, 89.7f, 0.977f }, { 54.18f, 44.72f, 91.7f, 0.968f },
    { 0.00f, 0.00f, 271.5f, 1.019f }, { 15.60f, 0.19f, 271.9f, 1.015f }, { 37.37f, 0.42f, 271.6f, 1.022f }, { 53.40f, 0.00f, 271.8f, 1.028f },
};

// Same conventions as MagArray.cpp (top-side parts).
static Vec3 sensorToBoard( Vec3 r, float angleDeg, float gain ) {
    float a = angleDeg * (float)M_PI / 180.0f, c = cosf( a ), s = sinf( a );
    float x = r.x, y = -r.y, z = -r.z;
    Vec3 b = { gain * ( x * c - y * s ), gain * ( x * s + y * c ), gain * z };
    return b;
}
static Vec3 boardToSensor( Vec3 f, float angleDeg, float gain ) {
    float a = angleDeg * (float)M_PI / 180.0f, c = cosf( a ), s = sinf( a ), k = 1.0f / gain;
    Vec3 r = { k * ( f.x * c + f.y * s ), -k * ( -f.x * s + f.y * c ), -k * f.z };
    return r;
}

static void matrixFromAngle( Sensor& sn ) {
    Vec3 ex = sensorToBoard( { 1, 0, 0 }, sn.angle, sn.gain ), ey = sensorToBoard( { 0, 1, 0 }, sn.angle, sn.gain ), ez = sensorToBoard( { 0, 0, 1 }, sn.angle, sn.gain );
    float m[ 3 ][ 3 ] = { { ex.x, ey.x, ez.x }, { ex.y, ey.y, ez.y }, { ex.z, ey.z, ez.z } };
    memcpy( sn.m, m, sizeof( m ) );
}
static Vec3 apply( const float m[ 3 ][ 3 ], Vec3 r ) {
    Vec3 b = { m[ 0 ][ 0 ] * r.x + m[ 0 ][ 1 ] * r.y + m[ 0 ][ 2 ] * r.z, m[ 1 ][ 0 ] * r.x + m[ 1 ][ 1 ] * r.y + m[ 1 ][ 2 ] * r.z,
               m[ 2 ][ 0 ] * r.x + m[ 2 ][ 1 ] * r.y + m[ 2 ][ 2 ] * r.z };
    return b;
}

static std::vector<Frame> frames;
static float magnetStrength = 0; // 0 = fit each frame's strength freely

static void fitPoses( float* meanMisfit, float* worstMisfit, int* fitted ) {
    Vec3 place[ SENSORS ], field[ SENSORS ];
    for ( int i = 0; i < SENSORS; i++ ) place[ i ] = { sensors[ i ].x, sensors[ i ].y, 0 };
    double sum = 0; float worst = 0; int n = 0;
    MagFitResult last = { };
    for ( size_t f = 0; f < frames.size( ); f++ ) {
        Frame& fr = frames[ f ];
        for ( int i = 0; i < SENSORS; i++ ) field[ i ] = apply( sensors[ i ].m, fr.reading[ i ] );
        MagFitResult r = last; // warm start from the frame before
        magFitSolveKnownStrength( place, field, nullptr, SENSORS, 1e6f, magnetStrength, &r );
        if ( r.signal <= 0 ) { fr.used = false; continue; }
        float misfit = r.residual / r.signal;
        if ( misfit > 0.5f ) { // a bad warm start: try cold
            MagFitResult cold = { };
            magFitSolveKnownStrength( place, field, nullptr, SENSORS, 1e6f, magnetStrength, &cold );
            if ( cold.signal > 0 && cold.residual / cold.signal < misfit ) { r = cold; misfit = cold.residual / cold.signal; }
        }
        fr.pose = r; fr.used = true; last = r; last.valid = true;
        sum += misfit; if ( misfit > worst ) worst = misfit; n++;
    }
    *meanMisfit = n ? (float)( sum / n ) : 1; *worstMisfit = worst; *fitted = n;
}

// Weighted squared misfit of sensor i with trial parameters { x, y, rotation, gain }, over all frames.
static double sensorCost( int i, const float p[ 4 ] ) {
    double cost = 0;
    Vec3 place = { p[ 0 ], p[ 1 ], 0 };
    for ( size_t f = 0; f < frames.size( ); f++ ) {
        const Frame& fr = frames[ f ];
        if ( !fr.used ) continue;
        Vec3 model = magFitDipoleField( place, fr.pose.position, fr.pose.moment );
        Vec3 meas = sensorToBoard( fr.reading[ i ], p[ 2 ], p[ 3 ] );
        float w = 1.0f / ( fr.signal * fr.signal ); // every frame counts by its misfit RATIO
        cost += w * ( ( meas.x - model.x ) * ( meas.x - model.x ) + ( meas.y - model.y ) * ( meas.y - model.y ) + ( meas.z - model.z ) * ( meas.z - model.z ) );
    }
    return cost;
}

// Coordinate-wise pattern search: plain, derivative-free, good enough for 4 numbers.
//
// (A free 3x3 response matrix per sensor was tried here, solved in closed form.
// It does not work: regressing the model on noisy readings always comes out a
// little small, the next round of magnet fits answers smaller fields by moving
// the magnets further away, and the two run off together toward nothing. Four
// bounded numbers per sensor cannot do that.)
static void refineSensor( int i, bool movable ) {
    float p[ 4 ] = { sensors[ i ].x, sensors[ i ].y, sensors[ i ].angle, sensors[ i ].gain };
    float step[ 4 ] = { movable ? 1.0f : 0.0f, movable ? 1.0f : 0.0f, 2.0f, 0.02f };
    double best = sensorCost( i, p );
    for ( int round = 0; round < 40; round++ ) {
        bool improved = false;
        for ( int k = 0; k < 4; k++ ) {
            if ( step[ k ] == 0 ) continue;
            for ( int dir = -1; dir <= 1; dir += 2 ) {
                float q[ 4 ] = { p[ 0 ], p[ 1 ], p[ 2 ], p[ 3 ] };
                q[ k ] += dir * step[ k ];
                if ( q[ 3 ] < 0.8f || q[ 3 ] > 1.2f ) continue; // a TMAG5273's gain error is a few percent
                double c = sensorCost( i, q );
                if ( c < best ) { best = c; memcpy( p, q, sizeof( p ) ); improved = true; break; }
            }
        }
        if ( !improved ) for ( int k = 0; k < 4; k++ ) step[ k ] *= 0.5f;
    }
    sensors[ i ].x = p[ 0 ];
    sensors[ i ].y = p[ 1 ];
    sensors[ i ].angle = p[ 2 ];
    sensors[ i ].gain = p[ 3 ];
    matrixFromAngle( sensors[ i ] );
}

// Put the array back in the agreed frame: ORIGIN at (0,0), XAXIS on +x at `span`, mean gain 1.
static void fixGauge( float span ) {
    float dx = sensors[ XAXIS ].x - sensors[ ORIGIN ].x, dy = sensors[ XAXIS ].y - sensors[ ORIGIN ].y;
    float length = sqrtf( dx * dx + dy * dy ), turn = atan2f( dy, dx ), scale = span / length;
    float ox = sensors[ ORIGIN ].x, oy = sensors[ ORIGIN ].y, c = cosf( -turn ), s = sinf( -turn ), gainSum = 0;
    for ( int i = 0; i < SENSORS; i++ ) {
        float x = sensors[ i ].x - ox, y = sensors[ i ].y - oy;
        sensors[ i ].x = scale * ( x * c - y * s );
        sensors[ i ].y = scale * ( x * s + y * c );
        sensors[ i ].angle -= turn * 180.0f / (float)M_PI;
        gainSum += sensors[ i ].gain;
    }
    for ( int i = 0; i < SENSORS; i++ ) {
        sensors[ i ].gain *= SENSORS / gainSum; // overall gain is not observable: a stronger magnet looks the same
        matrixFromAngle( sensors[ i ] );
    }
}

int main( int argc, char** argv ) {
    if ( argc < 3 ) { printf( "usage: magcal recording.txt <mm between sensor %d and sensor %d> [test-from-seconds] [first]\n", ORIGIN, XAXIS ); return 1; }
    float span = (float)atof( argv[ 2 ] );
    float testFrom = argc > 3 ? (float)atof( argv[ 3 ] ) : 1e9f;
    if ( argc > 4 && strcmp( argv[ 4 ], "first" ) == 0 ) memcpy( sensors, firstTable, sizeof( sensors ) );

    for ( int i = 0; i < SENSORS; i++ ) matrixFromAngle( sensors[ i ] );

    FILE* in = fopen( argv[ 1 ], "r" );
    if ( !in ) { printf( "cannot open %s\n", argv[ 1 ] ); return 1; }
    char line[ 1024 ];
    std::vector<Frame> all; std::vector<float> times; float t0 = -1;
    while ( fgets( line, sizeof( line ), in ) ) {
        if ( strncmp( line, "mag,", 4 ) != 0 ) continue;
        float v[ 25 ]; int n = 0;
        for ( char* tok = strtok( line + 4, "," ); tok && n < 25; tok = strtok( nullptr, "," ) ) v[ n++ ] = (float)atof( tok );
        if ( n != 25 ) continue;
        if ( t0 < 0 ) t0 = v[ 0 ];
        Frame fr = { }; double sumSq = 0; float peak = 0; int seen = 0;
        for ( int i = 0; i < SENSORS; i++ ) {
            Vec3 b = { v[ 1 + 3 * i ], v[ 2 + 3 * i ], v[ 3 + 3 * i ] };
            float m = sqrtf( b.x * b.x + b.y * b.y + b.z * b.z );
            sumSq += m * m; if ( m > peak ) peak = m; if ( m > 0.10f ) seen++;
            fr.reading[ i ] = boardToSensor( b, sensors[ i ].angle, sensors[ i ].gain );
        }
        fr.signal = sqrtf( (float)sumSq / ( 3 * SENSORS ) );
        // Keep frames every sensor sees clearly, and none with the magnet so close that it is no dipole.
        if ( seen < 5 || peak > 30.0f ) continue;
        if ( all.empty( ) ) t0 = v[ 0 ]; // seconds count from the first frame that is kept
        all.push_back( fr ); times.push_back( ( v[ 0 ] - t0 ) / 1000.0f );
    }
    fclose( in );

    std::vector<Frame> test;
    for ( size_t f = 0; f < all.size( ); f++ ) ( times[ f ] < testFrom ? frames : test ).push_back( all[ f ] );
    printf( "%zu frames to calibrate on, %zu held back to test on\n", frames.size( ), test.size( ) );

    float mean, worst; int fitted;
    fitPoses( &mean, &worst, &fitted );
    printf( "round  0: mean misfit %5.2f%%  worst %5.1f%%  (the table as it was)\n", mean * 100, worst * 100 );
    float startMean = mean;

    for ( int round = 1; round <= 40; round++ ) {
        for ( int i = 0; i < SENSORS; i++ ) refineSensor( i, true );
        fixGauge( span );
        fitPoses( &mean, &worst, &fitted );
        if ( round % 5 == 0 || round == 1 ) printf( "round %2d: mean misfit %5.2f%%  worst %5.1f%%\n", round, mean * 100, worst * 100 );
    }

    printf( "\n #     x_mm    y_mm   rotation   gain\n" );
    for ( int i = 0; i < SENSORS; i++ ) printf( " %d  %7.2f %7.2f   %7.1f   %5.3f\n", i, sensors[ i ].x, sensors[ i ].y, sensors[ i ].angle, sensors[ i ].gain );

    // The magnet's strength should be one number all along - a check on the whole thing.
    double s1 = 0, s2 = 0; int n = 0;
    for ( size_t f = 0; f < frames.size( ); f++ ) if ( frames[ f ].used ) { s1 += frames[ f ].pose.strength; s2 += (double)frames[ f ].pose.strength * frames[ f ].pose.strength; n++; }
    double m = s1 / n, sd = sqrt( s2 / n - m * m );
    printf( "\nmagnet strength over the recording: %.0f +/- %.0f mT*mm^3 (%.1f%%)\n", m, sd, 100 * sd / m );

    if ( !test.empty( ) ) {
        frames = test;
        fitPoses( &mean, &worst, &fitted );
        printf( "held-back frames: mean misfit %.2f%%, worst %.1f%%  (it was %.2f%% on the calibration frames before calibrating)\n", mean * 100, worst * 100, startMean * 100 );
    }
    return 0;
}
