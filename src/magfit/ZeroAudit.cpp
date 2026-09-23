// SPDX-License-Identifier: MIT
#include "ZeroAudit.h"

#include <math.h>

void ZeroAudit::reset( int i ) {
    if ( i < 0 || i >= ZERO_AUDIT_MAX )
        return;
    ZeroAuditSensor keep = s[ i ];
    s[ i ] = ZeroAuditSensor( );
    s[ i ].applied = keep.applied;
    s[ i ].lastApplied = keep.lastApplied;
    s[ i ].excluded = keep.excluded;
}

void ZeroAudit::resetAll( ) {
    for ( int i = 0; i < ZERO_AUDIT_MAX; i++ )
        s[ i ] = ZeroAuditSensor( );
}

void ZeroAudit::addSample( int i, Vec3 prediction, Vec3 reading, uint32_t nowMs ) {
    if ( i < 0 || i >= ZERO_AUDIT_MAX )
        return;
    ZeroAuditSensor& a = s[ i ];
    if ( a.n == 0 )
        a.firstMs = nowMs;
    float pv[ 3 ] = { prediction.x, prediction.y, prediction.z }, rv[ 3 ] = { reading.x, reading.y, reading.z };
    for ( int k = 0; k < 3; k++ ) {
        a.r[ k ] += rv[ k ];
        a.p[ k ] += pv[ k ];
        a.pp[ k ] += (double)pv[ k ] * pv[ k ];
        a.pr[ k ] += (double)pv[ k ] * rv[ k ];
        a.rr[ k ] += (double)rv[ k ] * rv[ k ];
    }
    a.n++;
}

bool ZeroAudit::solve( int i, int minSamples, float minSpread ) {
    if ( i < 0 || i >= ZERO_AUDIT_MAX )
        return false;
    ZeroAuditSensor& a = s[ i ];
    a.solved = false;
    if ( a.n < 2 )
        return false;
    double n = a.n;
    double num = 0, den = 0, meanSq = 0;
    for ( int k = 0; k < 3; k++ ) {
        num += a.pr[ k ] - a.r[ k ] * a.p[ k ] / n;
        den += a.pp[ k ] - a.p[ k ] * a.p[ k ] / n;
        meanSq += ( a.p[ k ] / n ) * ( a.p[ k ] / n );
    }
    // The spread of the prediction over the samples, against its mean size.
    a.spread = meanSq > 0 ? (float)sqrt( ( den / n ) / meanSq ) : 0.0f;
    if ( den <= 0 )
        return false;
    double g = num / den;
    double z[ 3 ];
    for ( int k = 0; k < 3; k++ )
        z[ k ] = ( a.r[ k ] - g * a.p[ k ] ) / n;
    // The residual's rms before (reading - pred) and after (reading - z - g pred).
    double before = 0, after = 0;
    for ( int k = 0; k < 3; k++ ) {
        before += a.rr[ k ] - 2 * a.pr[ k ] + a.pp[ k ];
        // sum (r - z - g p)^2 = rr - 2 z r - 2 g pr + n z^2 + 2 z g p + g^2 pp
        after += a.rr[ k ] - 2 * z[ k ] * a.r[ k ] - 2 * g * a.pr[ k ] + n * z[ k ] * z[ k ] + 2 * z[ k ] * g * a.p[ k ] + g * g * a.pp[ k ];
    }
    a.rmsBefore = (float)sqrt( before > 0 ? before / n : 0 );
    a.rmsAfter = (float)sqrt( after > 0 ? after / n : 0 );
    a.gain = (float)g;
    a.zero = { (float)z[ 0 ], (float)z[ 1 ], (float)z[ 2 ] };
    a.solved = a.n >= minSamples && a.spread >= minSpread;
    return a.solved;
}
