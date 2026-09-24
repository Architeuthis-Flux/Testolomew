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
    s[ i ].lastGainApplied = keep.lastGainApplied;
    s[ i ].previousGain = keep.previousGain;
    s[ i ].previousZero = keep.previousZero;
    s[ i ].hasPrevious = keep.hasPrevious;
    s[ i ].excluded = keep.excluded;
    s[ i ].correctable = keep.correctable;
}

void ZeroAudit::resetAll( ) {
    for ( int i = 0; i < ZERO_AUDIT_MAX; i++ )
        s[ i ] = ZeroAuditSensor( );
}

void ZeroAudit::addSample( int i, Vec3 prediction, Vec3 reading, uint32_t nowMs ) {
    if ( i < 0 || i >= ZERO_AUDIT_MAX )
        return;
    ZeroAuditSensor& a = s[ i ];
    if ( a.n == 0 ) {
        a.firstMs = nowMs;
        a.firstPred = prediction;
        a.firstRead = reading;
    }
    float pv[ 3 ] = { prediction.x, prediction.y, prediction.z }, rv[ 3 ] = { reading.x, reading.y, reading.z };
    float p0[ 3 ] = { a.firstPred.x, a.firstPred.y, a.firstPred.z }, r0[ 3 ] = { a.firstRead.x, a.firstRead.y, a.firstRead.z };
    for ( int k = 0; k < 3; k++ ) {
        float u = pv[ k ] - p0[ k ], v = rv[ k ] - r0[ k ];
        a.su[ k ] += u;
        a.sv[ k ] += v;
        a.suu[ k ] += u * u;
        a.suv[ k ] += u * v;
        a.svv[ k ] += v * v;
        a.spp[ k ] += pv[ k ] * pv[ k ];
    }
    a.n++;
}

// Per axis, the line reading = zero + gain * prediction through the samples
// (centred on the first sample: the sums are small and do not cancel in
// float). An axis whose predictions hardly vary (spreadAxis under
// minSpread) cannot tell its gain from its zero: it keeps gain 1 and the
// offset takes the mean difference. The solve's own 1-sigma per axis comes
// with it: the locator applies a correction only past three of them.
bool ZeroAudit::solve( int i, int minSamples, float minSpread, float minPredMt ) {
    if ( i < 0 || i >= ZERO_AUDIT_MAX )
        return false;
    ZeroAuditSensor& a = s[ i ];
    a.solved = false;
    if ( a.n < 2 )
        return false;
    float n = (float)a.n;
    float p0[ 3 ] = { a.firstPred.x, a.firstPred.y, a.firstPred.z }, r0[ 3 ] = { a.firstRead.x, a.firstRead.y, a.firstRead.z };
    float g[ 3 ], z[ 3 ], gs[ 3 ], zs[ 3 ];
    float meanSq = 0.0f, denAll = 0.0f, before = 0.0f, after = 0.0f;
    for ( int k = 0; k < 3; k++ ) {
        float mu = a.su[ k ] / n, mv = a.sv[ k ] / n;
        float sxx = a.suu[ k ] - a.su[ k ] * a.su[ k ] / n;
        float sxy = a.suv[ k ] - a.su[ k ] * a.sv[ k ] / n;
        float syy = a.svv[ k ] - a.sv[ k ] * a.sv[ k ] / n;
        float meanP = p0[ k ] + mu;
        a.spreadAxis[ k ] = sxx > 0.0f && meanP != 0.0f ? sqrtf( ( sxx / n ) / ( meanP * meanP ) ) : 0.0f;
        meanSq += meanP * meanP;
        denAll += sxx > 0.0f ? sxx : 0.0f;
        float predRms = sqrtf( a.spp[ k ] / n );
        bool axisOk = sxx > 0.0f && a.spreadAxis[ k ] >= minSpread && predRms >= minPredMt;
        g[ k ] = axisOk ? sxy / sxx : 1.0f;
        z[ k ] = r0[ k ] - g[ k ] * p0[ k ] + ( mv - g[ k ] * mu );
        float residual = axisOk ? syy - g[ k ] * sxy : syy - 2.0f * sxy + sxx; // the sum of squares left, about the line
        if ( residual < 0.0f )
            residual = 0.0f;
        after += residual;
        float d0 = r0[ k ] - p0[ k ];
        before += n * d0 * d0 + 2.0f * d0 * ( a.sv[ k ] - a.su[ k ] ) + ( a.svv[ k ] - 2.0f * a.suv[ k ] + a.suu[ k ] );
        float var = residual / ( n > 2.0f ? n - 2.0f : 1.0f );
        gs[ k ] = axisOk ? sqrtf( var / sxx ) : 1e9f;
        zs[ k ] = sqrtf( var / n );
    }
    a.spread = meanSq > 0.0f ? sqrtf( ( denAll / n ) / meanSq ) : 0.0f;
    a.rmsBefore = sqrtf( before > 0.0f ? before / n : 0.0f );
    a.rmsAfter = sqrtf( after > 0.0f ? after / n : 0.0f );
    a.gain = { g[ 0 ], g[ 1 ], g[ 2 ] };
    a.zero = { z[ 0 ], z[ 1 ], z[ 2 ] };
    a.gainSigma = { gs[ 0 ], gs[ 1 ], gs[ 2 ] };
    a.zeroSigma = { zs[ 0 ], zs[ 1 ], zs[ 2 ] };
    a.solved = a.n >= minSamples && a.spread >= minSpread;
    return a.solved;
}
