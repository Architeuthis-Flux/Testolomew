// SPDX-License-Identifier: MIT
// Fit a bench capture of the `f` stream (a static probe: the frames are averaged, then fitted one by one) six ways:
// strength free or held, weights flat (the old cap of 2), by noise, or the TMAGs alone - and print each sensor's
// reading against the model. This is what showed, on 2026-09-21, that the far probe needs the strength held AND the
// quiet sensor trusted (docs/morning-report-2026-09-21.md). The stream's lines carry the firmware's board-frame fields;
// strip any host-time prefix first. Build from the repo root:
//   c++ -std=c++11 -O2 -I tools/hostsim -I src/board -I src/magarray -I src/magfit -I src/common tools/farfit.cpp src/magfit/MagFit.cpp -o farfit
//   ./farfit capture.txt
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "Arduino.h"
#include "MagFit.h"
#include "MagArrayConfig.h"
int main( int argc, char** argv ) {
    FILE* in = fopen( argv[ 1 ], "r" ); char line[ 2048 ];
    static double sum[ 9 ][ 3 ]; int n = 0;
    while ( fgets( line, sizeof line, in ) ) {
        if ( strncmp( line, "mag,", 4 ) ) continue;
        float v[ 28 ]; int k = 0;
        for ( char* t = strtok( line + 4, "," ); t && k < 28; t = strtok( 0, "," ) ) v[ k++ ] = atof( t );
        if ( k != 28 ) continue;
        for ( int i = 0; i < 9; i++ ) for ( int a = 0; a < 3; a++ ) sum[ i ][ a ] += v[ 1 + 3 * i + a ];
        n++;
    }
    Vec3 pos[ 9 ], fld[ 9 ]; float wFlat[ 9 ], wNoise[ 9 ];
    for ( int i = 0; i < 9; i++ ) {
        pos[ i ] = { magSensorPlaces[ i ].x, magSensorPlaces[ i ].y, magSensorPlaces[ i ].z };
        fld[ i ] = { (float)( sum[ i ][ 0 ] / n ), (float)( sum[ i ][ 1 ] / n ), (float)( sum[ i ][ 2 ] / n ) };
        wFlat[ i ] = magSensorPlaces[ i ].type == MAG_MMC56X3 ? 2.0f : 1.0f;
        wNoise[ i ] = MAG_WEIGHT_REFERENCE_MT / magSensorTypeNoiseMt( magSensorPlaces[ i ].type );
    }
    printf( "%d frames averaged; MMC mean %.4f %.4f %.4f mT\n", n, fld[ 8 ].x, fld[ 8 ].y, fld[ 8 ].z );
    struct { const char* name; const float* w; float strength; } runs[] = {
        { "free strength, cap 2", wFlat, 0 }, { "held 4232, cap 2", wFlat, 4232 }, { "free strength, noise weights", wNoise, 0 }, { "held 4232, noise weights", wNoise, 4232 },
        { "free, TMAGs only (weights 1)", nullptr, 0 }, { "held 4232, TMAGs only", nullptr, 4232 } };
    for ( auto& r : runs ) {
        bool use[ 9 ]; for ( int i = 0; i < 9; i++ ) use[ i ] = !( r.w == nullptr && i == 8 );
        MagFitResult res = { };
        bool ok = r.strength > 0 ? magFitSolveKnownStrength( pos, fld, use, 9, 0.4f, r.strength, &res, r.w ) : magFitSolve( pos, fld, use, 9, 0.4f, &res, r.w );
        printf( "%-32s ok %d  x %6.1f y %6.1f z %6.1f  sigma %5.1f %5.1f %5.1f  strength %6.0f  misfit %4.1f %%  axis %.2f %.2f %.2f\n", r.name, ok, res.position.x, res.position.y, res.position.z,
                res.sigma.x, res.sigma.y, res.sigma.z, res.strength, 100 * res.residual / res.signal, res.moment.x / res.strength, res.moment.y / res.strength, res.moment.z / res.strength );
        printf( "    per sensor |reading| / |residual| (mT, unweighted):" );
        for ( int i = 0; i < 9; i++ ) {
            if ( !use[ i ] ) continue;
            Vec3 m = magFitDipoleField( pos[ i ], res.position, res.moment );
            float rr = sqrtf( fld[ i ].x * fld[ i ].x + fld[ i ].y * fld[ i ].y + fld[ i ].z * fld[ i ].z );
            float e = sqrtf( ( fld[ i ].x - m.x ) * ( fld[ i ].x - m.x ) + ( fld[ i ].y - m.y ) * ( fld[ i ].y - m.y ) + ( fld[ i ].z - m.z ) * ( fld[ i ].z - m.z ) );
            printf( " %d:%.3f/%.3f", i, rr, e );
        }
        printf( "\n" );
    }
    // Per-frame with the strength held and noise weights: the scatter of the answer
    rewind( in ); double sx = 0, sy = 0, sz = 0, sxx = 0, syy = 0, szz = 0; int m = 0; MagFitResult res = { };
    while ( fgets( line, sizeof line, in ) ) {
        if ( strncmp( line, "mag,", 4 ) ) continue;
        float v[ 28 ]; int k = 0;
        for ( char* t = strtok( line + 4, "," ); t && k < 28; t = strtok( 0, "," ) ) v[ k++ ] = atof( t );
        if ( k != 28 ) continue;
        for ( int i = 0; i < 9; i++ ) fld[ i ] = { v[ 1 + 3 * i ], v[ 2 + 3 * i ], v[ 3 + 3 * i ] };
        if ( !magFitSolveKnownStrength( pos, fld, nullptr, 9, 0.4f, 4232, &res, wNoise ) ) { res.valid = false; continue; }
        sx += res.position.x; sy += res.position.y; sz += res.position.z; sxx += res.position.x * res.position.x; syy += res.position.y * res.position.y; szz += res.position.z * res.position.z; m++;
    }
    if ( m ) printf( "per frame (no smoothing), held 4232, noise weights: %d/%d fits, mean x %.1f y %.1f z %.1f, scatter %.1f %.1f %.1f mm\n", m, n, sx / m, sy / m, sz / m,
                     sqrt( sxx / m - sx / m * sx / m ), sqrt( syy / m - sy / m * sy / m ), sqrt( szz / m - sz / m * sz / m ) );

    // Leave one out: the fit of the other eight (strength held, noise weights) predicts each sensor's field;
    // reading minus prediction is that sensor's own error - its zero, plus its gain's share of the field.
    // (The zero audit's arithmetic, 2026-09-21 night.)
    printf( "leave one out (held 4232, noise weights, the averaged frame): reading -> prediction, residual (mT, board frame)\n" );
    for ( int i = 0; i < 9; i++ ) {
        bool useNot[ 9 ];
        for ( int k = 0; k < 9; k++ ) useNot[ k ] = k != i;
        MagFitResult r = { };
        bool ok = magFitSolveKnownStrength( pos, fld, useNot, 9, 0.5f, 4232.0f, &r, wNoise );
        Vec3 pred = magFitDipoleField( pos[ i ], r.position, r.moment );
        Vec3 res = { fld[ i ].x - pred.x, fld[ i ].y - pred.y, fld[ i ].z - pred.z };
        float pm = sqrtf( pred.x * pred.x + pred.y * pred.y + pred.z * pred.z ), rm = sqrtf( res.x * res.x + res.y * res.y + res.z * res.z );
        printf( "  %d: ok %d misfit %4.1f %%  reading %+.4f %+.4f %+.4f -> pred %+.4f %+.4f %+.4f  residual %+.4f %+.4f %+.4f  |res| %.4f = %.1f %% of |pred| %.3f\n", i, ok,
                r.signal > 0 ? 100.0f * r.residual / r.signal : 0.0f, fld[ i ].x, fld[ i ].y, fld[ i ].z, pred.x, pred.y, pred.z, res.x, res.y, res.z, rm, pm > 0 ? 100.0f * rm / pm : 0.0f, pm );
    }
}
