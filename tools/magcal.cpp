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

#define SENSORS 9 // MAG_SENSOR_COUNT: 8 TMAG5273 + 1 MMC56x3 (2026-09-21). A recording with fewer columns leaves the rest out
#define ORIGIN 4 // gauge: this sensor is (0, 0)...
#define XAXIS 7  // ...and this one is on the +x axis

struct Sensor {
    float x, y;  // mm
    float z;     // mm above the TMAG plane (the MMC on the back sits at -2; not refined)
    float angle; // package rotation, degrees (the table's rotationDeg)
    float gain;  // what its X and Y readings are multiplied by
    float gainZ; // ...and its Z: a Hall structure of its own (the datasheet: Z-vs-X/Y mismatch 1 %, its drift up to 15 %; 2026-09-23)
    bool zUp;    // the part's own +z points out of its top (an MMC56x3; false = into it, a TMAG5273: MagArray's magSensorToBoard)
    float ox, oy, oz; // an offset in the sensor's own frame, subtracted from its readings: the error of its zero at recording time (fitted for only= sensors)
    // (rangeMt is by type: MMC_RANGE_MT for zUp parts, TMAG_RANGE_MT else - a reading past
    // CLIP_FRACTION of it is clipped and that sensor sits out of the frame's fit)
    // The full answer: board-frame field = m * (the sensor's own reading). To
    // start with it is just the rotation and gain above; the calibration then
    // frees all nine numbers, which takes in different gains on the three axes
    // (X/Y and Z are different Hall structures), axes that are not quite
    // square, and a package that is not quite flat.
    float m[ 3 ][ 3 ];
};

struct Frame {
    Vec3 reading[ SENSORS ]; // sensor frame, mT
    bool clipped[ SENSORS ]; // this sensor read past its range in this frame: left out of it
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
    { 0, 45, 0, 90, 1, 1.0f, false, 0, 0, 0 }, { 15, 45, 0, 90, 1, 1.0f, false, 0, 0, 0 }, { 30, 45, 0, 90, 1, 1.0f, false, 0, 0, 0 }, { 45, 45, 0, 90, 1, 1.0f, false, 0, 0, 0 },
    { 0, 0, 0, 270, 1, 1.0f, false, 0, 0, 0 }, { 15, 0, 0, 270, 1, 1.0f, false, 0, 0, 0 }, { 30, 0, 0, 270, 1, 1.0f, false, 0, 0, 0 }, { 45, 0, 0, 270, 1, 1.0f, false, 0, 0, 0 },
    { 26.7f, 22.1f, -2.0f, 0, 1, 1.0f, false, 0, 0, 0 }, // placeholder: the first recording had no MMC (8 columns), so this row is never used with it
};
static Sensor sensors[ SENSORS ] = {
    { 0.10f, 44.29f, 0, 85.0f, 0.987f, 1.0f, false, 0, 0, 0 }, { 16.75f, 44.04f, 0, 89.3f, 0.985f, 1.0f, false, 0, 0, 0 }, { 38.78f, 43.95f, 0, 89.7f, 0.977f, 1.0f, false, 0, 0, 0 }, { 54.18f, 44.72f, 0, 91.7f, 0.968f, 1.0f, false, 0, 0, 0 },
    { 0.00f, 0.00f, 0, 271.5f, 1.019f, 1.0f, false, 0, 0, 0 }, { 15.60f, 0.19f, 0, 271.9f, 1.015f, 1.0f, false, 0, 0, 0 }, { 37.37f, 0.42f, 0, 271.6f, 1.022f, 1.0f, false, 0, 0, 0 }, { 53.40f, 0.00f, 0, 271.8f, 1.028f, 1.0f, false, 0, 0, 0 },
    // The MMC56x3: the centre of the board, hanging under it, 2 mm below the TMAG plane. Measured 2026-09-21 (only=8 on
    // tools/recordings/2026-09-21-mmc-centre-calibration.txt): zUp = true, its +z reads up in the board frame (MagArrayConfig.h:
    // underside = false); the other face cost 40x more. The gain soaks up a height error (z is not refined).
    { 27.52f, 25.19f, -3.77f, 260.6f, 0.936f, 0.947f, true, 0, 0, 0 }, // its height searched (2026-09-23; with z held at -2 the gain was 1.129)
};
static int sensorCount = SENSORS; // how many the recording carries (8 in the ones before 2026-09-20)
// The rows the FIRMWARE had when the recording was made: the stream is in
// the board frame through that table (MagArray's magSensorToBoard), and it
// is undone through the same rows, whatever the table here says now. A copy
// of `sensors` unless the recording says otherwise with a line
//   # streamed-with <i> <x> <y> <z> <rotation> <gain> <zUp 0|1> [<gainZ>]
// (2026-09-21: the MMC was streamed at rotation 0, gain 1, flipped; once
// its measured row went into the table, decoding the stream through the
// new row put the misfit at 34 % - the recording has to carry its rows).
static Sensor streamTable[ SENSORS ];
static bool streamTableGiven[ SENSORS ] = { };
#define TMAG_RANGE_MT 40.0f
#define MMC_RANGE_MT 3.0f
#define CLIP_FRACTION 0.9f
// A frame is used when at least this many sensors read above SEEN_MT (the
// third argument, "seen", overrides the count: the probe's small magnet
// lights fewer sensors than the calibration magnet did).
static int seenNeeded = 5;
#define SEEN_MT 0.10f
// only=<list>: refine only these sensors (comma-separated), the rest held as
// the table has them - for adding sensors to an array already calibrated
// (the MMCs, 2026-09-21): fewer unknowns, and the gauge (origin, x axis,
// mean gain) stays the calibrated sensors'. Each listed sensor is also
// tried with its z axis the other way round (a breakout placed face down)
// and keeps whichever explains its readings better.
static bool onlyMode = false;
static bool onlyList[ SENSORS ] = { };
static float rangeOf( const Sensor& sn ) { return sn.zUp ? MMC_RANGE_MT : TMAG_RANGE_MT; }

// Same conventions as MagArray.cpp (top-side parts): a part whose own z
// points into its top (the TMAG5273) is a half turn about x from the board.
static Vec3 sensorToBoard( Vec3 r, float angleDeg, float gain, float gainZ, bool zUp ) {
    float a = angleDeg * (float)M_PI / 180.0f, c = cosf( a ), s = sinf( a );
    float x = r.x, y = zUp ? r.y : -r.y, z = zUp ? r.z : -r.z;
    Vec3 b = { gain * ( x * c - y * s ), gain * ( x * s + y * c ), gainZ * z };
    return b;
}
static Vec3 boardToSensor( Vec3 f, float angleDeg, float gain, float gainZ, bool zUp ) {
    float a = angleDeg * (float)M_PI / 180.0f, c = cosf( a ), s = sinf( a ), k = 1.0f / gain;
    float y = k * ( -f.x * s + f.y * c ), z = f.z / gainZ;
    Vec3 r = { k * ( f.x * c + f.y * s ), zUp ? y : -y, zUp ? z : -z };
    return r;
}

static void matrixFromAngle( Sensor& sn ) {
    Vec3 ex = sensorToBoard( { 1, 0, 0 }, sn.angle, sn.gain, sn.gainZ, sn.zUp ), ey = sensorToBoard( { 0, 1, 0 }, sn.angle, sn.gain, sn.gainZ, sn.zUp ), ez = sensorToBoard( { 0, 0, 1 }, sn.angle, sn.gain, sn.gainZ, sn.zUp );
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
    bool use[ SENSORS ];
    for ( int i = 0; i < SENSORS; i++ ) place[ i ] = { sensors[ i ].x, sensors[ i ].y, sensors[ i ].z };
    double sum = 0; float worst = 0; int n = 0;
    MagFitResult last = { };
    for ( size_t f = 0; f < frames.size( ); f++ ) {
        Frame& fr = frames[ f ];
        for ( int i = 0; i < SENSORS; i++ ) {
            Vec3 r = { fr.reading[ i ].x - sensors[ i ].ox, fr.reading[ i ].y - sensors[ i ].oy, fr.reading[ i ].z - sensors[ i ].oz };
            field[ i ] = apply( sensors[ i ].m, r );
            use[ i ] = i < sensorCount && !fr.clipped[ i ] && !( onlyMode && onlyList[ i ] );
        }
        MagFitResult r = last; // warm start from the frame before
        magFitSolveKnownStrength( place, field, use, SENSORS, 1e6f, magnetStrength, &r );
        if ( r.signal <= 0 ) { fr.used = false; continue; }
        float misfit = r.residual / r.signal;
        if ( misfit > 0.5f ) { // a bad warm start: try cold
            MagFitResult cold = { };
            magFitSolveKnownStrength( place, field, use, SENSORS, 1e6f, magnetStrength, &cold );
            if ( cold.signal > 0 && cold.residual / cold.signal < misfit ) { r = cold; misfit = cold.residual / cold.signal; }
        }
        fr.pose = r; fr.used = true; last = r; last.valid = true;
        sum += misfit; if ( misfit > worst ) worst = misfit; n++;
    }
    *meanMisfit = n ? (float)( sum / n ) : 1; *worstMisfit = worst; *fitted = n;
}

// Weighted squared misfit of sensor i with trial parameters { x, y, rotation, gain, ox, oy, oz, gainZ, z }, over all frames
// (the offset, in the sensor's own frame, is what its zero was off by while the recording was made).
static double sensorCost( int i, const float p[ 9 ] ) {
    double cost = 0;
    Vec3 place = { p[ 0 ], p[ 1 ], p[ 8 ] };
    for ( size_t f = 0; f < frames.size( ); f++ ) {
        const Frame& fr = frames[ f ];
        if ( !fr.used || fr.clipped[ i ] ) continue;
        Vec3 model = magFitDipoleField( place, fr.pose.position, fr.pose.moment );
        Vec3 r = { fr.reading[ i ].x - p[ 4 ], fr.reading[ i ].y - p[ 5 ], fr.reading[ i ].z - p[ 6 ] };
        Vec3 meas = sensorToBoard( r, p[ 2 ], p[ 3 ], p[ 7 ], sensors[ i ].zUp );
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
// The pattern search from p: the first four numbers (place, rotation, gain)
// and, if withOffset, the sensor's zero error too.
// A sensor being added (only=: withOffset) has its HEIGHT searched too (the MMC hangs under the board 'about 2 mm' - a
// height error was soaked up by its gain, 1.129, until 2026-09-23); the TMAGs' z is the plane, 0, by definition.
static double patternSearch( int i, float p[ 9 ], bool movable, bool withOffset ) {
    float step[ 9 ] = { movable ? 1.0f : 0.0f, movable ? 1.0f : 0.0f, 2.0f, 0.02f, withOffset ? 0.01f : 0.0f, withOffset ? 0.01f : 0.0f, withOffset ? 0.01f : 0.0f, 0.02f, withOffset && movable ? 0.5f : 0.0f };
    double best = sensorCost( i, p );
    for ( int round = 0; round < 60; round++ ) {
        bool improved = false;
        for ( int k = 0; k < 9; k++ ) {
            if ( step[ k ] == 0 ) continue;
            for ( int dir = -1; dir <= 1; dir += 2 ) {
                float q[ 9 ];
                memcpy( q, p, sizeof( q ) );
                q[ k ] += dir * step[ k ];
                if ( q[ 3 ] < 0.8f || q[ 3 ] > 1.2f || q[ 7 ] < 0.8f || q[ 7 ] > 1.2f ) continue; // a TMAG5273's (or MMC56x3's) gain error is a few percent, on either axis
                if ( q[ 8 ] < -10.0f || q[ 8 ] > 5.0f ) continue;                                     // a part hangs under the board or sits on it, within a centimetre
                double c = sensorCost( i, q );
                if ( c < best ) { best = c; memcpy( p, q, sizeof( float ) * 9 ); improved = true; break; }
            }
        }
        if ( !improved ) for ( int k = 0; k < 9; k++ ) step[ k ] *= 0.5f;
    }
    return best;
}

static void refineSensor( int i, bool movable ) {
    // The offset is fitted only for a sensor being added (only=): for the
    // TMAGs it is degenerate with the magnet fits that made their table.
    // Such a sensor's rotation is unknown from the start, and a pattern
    // search from one angle can settle in a wrong basin (with the offset
    // free it did, on a synthetic recording): the rotation, gain and place
    // are searched from four starting angles first, the best kept, and only
    // then the offset joins the search.
    bool withOffset = onlyMode && onlyList[ i ];
    float p[ 9 ] = { sensors[ i ].x, sensors[ i ].y, sensors[ i ].angle, sensors[ i ].gain, sensors[ i ].ox, sensors[ i ].oy, sensors[ i ].oz, sensors[ i ].gainZ, sensors[ i ].z };
    if ( withOffset ) {
        float bestP[ 9 ];
        double bestCost = 1e300;
        for ( int start = 0; start < 4; start++ ) {
            float q[ 9 ] = { sensors[ i ].x, sensors[ i ].y, sensors[ i ].angle + 90.0f * start, 1.0f, 0, 0, 0, 1.0f, sensors[ i ].z };
            double c = patternSearch( i, q, movable, false );
            if ( c < bestCost ) { bestCost = c; memcpy( bestP, q, sizeof( bestP ) ); }
        }
        memcpy( p, bestP, sizeof( p ) );
    }
    patternSearch( i, p, movable, withOffset );
    sensors[ i ].x = p[ 0 ];
    sensors[ i ].y = p[ 1 ];
    sensors[ i ].angle = p[ 2 ];
    sensors[ i ].gain = p[ 3 ];
    sensors[ i ].ox = p[ 4 ];
    sensors[ i ].oy = p[ 5 ];
    sensors[ i ].oz = p[ 6 ];
    sensors[ i ].gainZ = p[ 7 ];
    sensors[ i ].z = p[ 8 ];
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
        if ( i < sensorCount ) gainSum += sensors[ i ].gain;
    }
    for ( int i = 0; i < SENSORS; i++ ) {
        sensors[ i ].gain *= sensorCount / gainSum; // overall gain is not observable: a stronger magnet looks the same
        sensors[ i ].gainZ *= sensorCount / gainSum; // ...and Z keeps its ratio to X/Y, which is
        matrixFromAngle( sensors[ i ] );
    }
}

int main( int argc, char** argv ) {
    if ( argc < 3 ) { printf( "usage: magcal recording.txt <mm between sensor %d and sensor %d> [test-from-seconds] [first] [seen=<n>] [only=<i,j,...>]\n  seen=<n>: frames need n sensors above %.2f mT (default %d; the probe's small magnet lights fewer than the calibration magnet did)\n  only=<list>: refine only these sensors (both z senses tried), the rest kept as the table has them (adding sensors to a calibrated array)\n", ORIGIN, XAXIS, SEEN_MT, seenNeeded ); return 1; }
    float span = (float)atof( argv[ 2 ] );
    float testFrom = argc > 3 ? (float)atof( argv[ 3 ] ) : 1e9f;
    for ( int a = 4; a < argc; a++ ) {
        if ( strcmp( argv[ a ], "first" ) == 0 ) memcpy( sensors, firstTable, sizeof( sensors ) );
        else if ( strncmp( argv[ a ], "seen=", 5 ) == 0 ) seenNeeded = atoi( argv[ a ] + 5 );
        else if ( strncmp( argv[ a ], "only=", 5 ) == 0 ) {
            onlyMode = true;
            for ( char* tok = strtok( argv[ a ] + 5, "," ); tok; tok = strtok( nullptr, "," ) ) { int i = atoi( tok ); if ( i >= 0 && i < SENSORS ) onlyList[ i ] = true; }
        }
    }

    for ( int i = 0; i < SENSORS; i++ ) matrixFromAngle( sensors[ i ] );

    FILE* in = fopen( argv[ 1 ], "r" );
    if ( !in ) { printf( "cannot open %s\n", argv[ 1 ] ); return 1; }
    char line[ 1024 ];
    std::vector<Frame> all; std::vector<float> times; float t0 = -1;
    memcpy( streamTable, sensors, sizeof( streamTable ) );
    while ( fgets( line, sizeof( line ), in ) ) {
        if ( strncmp( line, "# streamed-with ", 16 ) == 0 ) {
            int i = -1, zUp = 0; float x, y, z, rot, gain, gainZ = 1.0f;
            int got = sscanf( line + 16, "%d %f %f %f %f %f %d %f", &i, &x, &y, &z, &rot, &gain, &zUp, &gainZ ); // the Z gain is the eighth number, 1 in recordings before 2026-09-23
            if ( got >= 7 && i >= 0 && i < SENSORS ) {
                if ( got < 8 ) gainZ = 1.0f;
                streamTable[ i ].x = x; streamTable[ i ].y = y; streamTable[ i ].z = z; streamTable[ i ].angle = rot; streamTable[ i ].gain = gain; streamTable[ i ].gainZ = gainZ; streamTable[ i ].zUp = zUp != 0;
                streamTableGiven[ i ] = true;
                printf( "sensor %d was streamed through x %.2f y %.2f rotation %.1f gain %.3f (z %.3f) z %s (the recording says)\n", i, x, y, rot, gain, gainZ, zUp ? "up" : "down" );
            } else printf( "cannot read: %s", line );
            continue;
        }
        if ( strncmp( line, "mag,", 4 ) != 0 ) continue;
        float v[ 1 + 3 * SENSORS ]; int n = 0;
        for ( char* tok = strtok( line + 4, "," ); tok && n < 1 + 3 * SENSORS; tok = strtok( nullptr, "," ) ) v[ n++ ] = (float)atof( tok );
        if ( n < 1 + 3 * 3 || ( n - 1 ) % 3 != 0 ) continue;
        int columns = ( n - 1 ) / 3;
        if ( all.empty( ) ) { sensorCount = columns; printf( "the recording carries %d sensors\n", sensorCount ); }
        if ( columns != sensorCount ) continue;
        if ( t0 < 0 ) t0 = v[ 0 ];
        Frame fr = { }; double sumSq = 0; float peak = 0; int seen = 0;
        int clipped = 0;
        for ( int i = 0; i < sensorCount; i++ ) {
            Vec3 b = { v[ 1 + 3 * i ], v[ 2 + 3 * i ], v[ 3 + 3 * i ] };
            float m = sqrtf( b.x * b.x + b.y * b.y + b.z * b.z );
            float axisMax = fabsf( b.x ) > fabsf( b.y ) ? fabsf( b.x ) : fabsf( b.y ); if ( fabsf( b.z ) > axisMax ) axisMax = fabsf( b.z );
            fr.clipped[ i ] = axisMax > CLIP_FRACTION * rangeOf( streamTable[ i ] ) / streamTable[ i ].gain; // (the stream is gained; compare in the part's own scale)
            if ( fr.clipped[ i ] ) { clipped++; continue; }
            sumSq += m * m; if ( m > peak ) peak = m; if ( m > SEEN_MT ) seen++;
            fr.reading[ i ] = boardToSensor( b, streamTable[ i ].angle, streamTable[ i ].gain, streamTable[ i ].gainZ, streamTable[ i ].zUp ); // the firmware's rows, not this table's
        }
        fr.signal = sqrtf( (float)sumSq / ( 3 * ( sensorCount - clipped ) ) );
        // Keep frames enough sensors see clearly, and none with the magnet so close that it is no dipole.
        if ( seen < seenNeeded || peak > 30.0f ) continue;
        if ( all.empty( ) ) t0 = v[ 0 ]; // seconds count from the first frame that is kept
        all.push_back( fr ); times.push_back( ( v[ 0 ] - t0 ) / 1000.0f );
    }
    fclose( in );

    // A glitched read (a corrupted byte on the bus: an MMC's z jumped to
    // -1 mT for one frame of a static night, 3 frames in 3716) would pull a
    // sensor's calibration by more than a night of good ones push it back:
    // each sensor's reading is the median of its three consecutive frames,
    // as the firmware's locator filters them before the fit.
    if ( all.size( ) >= 3 ) {
        std::vector<Frame> raw = all;
        for ( size_t f = 1; f + 1 < raw.size( ); f++ ) {
            for ( int i = 0; i < sensorCount; i++ ) {
                float* out[ 3 ] = { &all[ f ].reading[ i ].x, &all[ f ].reading[ i ].y, &all[ f ].reading[ i ].z };
                for ( int k = 0; k < 3; k++ ) {
                    float a = k == 0 ? raw[ f - 1 ].reading[ i ].x : k == 1 ? raw[ f - 1 ].reading[ i ].y : raw[ f - 1 ].reading[ i ].z;
                    float b = k == 0 ? raw[ f ].reading[ i ].x : k == 1 ? raw[ f ].reading[ i ].y : raw[ f ].reading[ i ].z;
                    float c = k == 0 ? raw[ f + 1 ].reading[ i ].x : k == 1 ? raw[ f + 1 ].reading[ i ].y : raw[ f + 1 ].reading[ i ].z;
                    float lo = a < b ? a : b, hi = a < b ? b : a;
                    *out[ k ] = c < lo ? lo : ( c > hi ? hi : c );
                }
            }
        }
    }

    std::vector<Frame> test;
    for ( size_t f = 0; f < all.size( ); f++ ) ( times[ f ] < testFrom ? frames : test ).push_back( all[ f ] );
    printf( "%zu frames to calibrate on, %zu held back to test on\n", frames.size( ), test.size( ) );

    float mean, worst; int fitted;
    fitPoses( &mean, &worst, &fitted );
    printf( "round  0: mean misfit %5.2f%%  worst %5.1f%%  (the table as it was)\n", mean * 100, worst * 100 );
    float startMean = mean;

    if ( onlyMode ) {
        // Each listed sensor, each z sense: refined from the table's start,
        // and the sense whose own readings fit better wins (the poses come
        // from the trusted sensors either way, so the two are comparable).
        for ( int i = 0; i < sensorCount; i++ ) {
            if ( !onlyList[ i ] ) continue;
            Sensor asTable = sensors[ i ];
            Sensor best = asTable; double bestCost = 1e300;
            for ( int sense = 0; sense < 2; sense++ ) {
                sensors[ i ] = asTable;
                sensors[ i ].zUp = sense == 0 ? asTable.zUp : !asTable.zUp;
                matrixFromAngle( sensors[ i ] );
                for ( int round = 0; round < 2; round++ ) refineSensor( i, true );
                float p[ 9 ] = { sensors[ i ].x, sensors[ i ].y, sensors[ i ].angle, sensors[ i ].gain, sensors[ i ].ox, sensors[ i ].oy, sensors[ i ].oz, sensors[ i ].gainZ, sensors[ i ].z };
                double c = sensorCost( i, p );
                printf( "sensor %d with z %s: x %.2f y %.2f height %.2f rotation %.1f gain %.3f (z %.3f), its zero off by (%+.4f %+.4f %+.4f) mT in its own frame, cost %.4g\n", i,
                        sensors[ i ].zUp ? "up" : "down", p[ 0 ], p[ 1 ], p[ 8 ], p[ 2 ], p[ 3 ], p[ 7 ], p[ 4 ], p[ 5 ], p[ 6 ], c );
                if ( c < bestCost ) { bestCost = c; best = sensors[ i ]; }
            }
            sensors[ i ] = best;
            matrixFromAngle( sensors[ i ] );
        }
        fitPoses( &mean, &worst, &fitted );
        printf( "with the listed sensors refined: mean misfit %5.2f%%  worst %5.1f%%\n", mean * 100, worst * 100 );
    } else {
        for ( int round = 1; round <= 40; round++ ) {
            for ( int i = 0; i < sensorCount; i++ ) refineSensor( i, true );
            fixGauge( span );
            fitPoses( &mean, &worst, &fitted );
            if ( round % 5 == 0 || round == 1 ) printf( "round %2d: mean misfit %5.2f%%  worst %5.1f%%\n", round, mean * 100, worst * 100 );
        }
    }

    printf( "\n #     x_mm    y_mm   rotation   gain   gainZ\n" );
    for ( int i = 0; i < sensorCount; i++ ) {
        bool mmc = i >= 8; // (the MMC slots; a TMAG's z sense is not searched)
        printf( " %d  %7.2f %7.2f   %7.1f   %5.3f  %5.3f%s%s\n", i, sensors[ i ].x, sensors[ i ].y, sensors[ i ].angle, sensors[ i ].gain, sensors[ i ].gainZ, mmc ? "" : "",
                !mmc ? "" : ( sensors[ i ].zUp ? "   (MMC56x3, z up: underside = false in the table)" : "   (MMC56x3, z DOWN: underside = true in the table)" ) );
    }

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
