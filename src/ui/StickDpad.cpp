// SPDX-License-Identifier: MIT
#include "StickDpad.h"

void stickDpadInit( StickDpad* d, float onAt, float offAt, uint32_t holdMs, uint32_t rearmMs ) {
    d->onAt = onAt;
    d->offAt = offAt < onAt ? offAt : onAt;
    d->holdMs = holdMs;
    d->rearmMs = rearmMs;
    d->axis = -1;
    d->sign = 0;
    d->pendingAxis = -1;
    d->pendingSign = 0;
    d->pendingSinceMs = 0;
    for ( int a = 0; a < 2; a++ ) {
        d->inside[ a ] = true;
        d->insideSinceMs[ a ] = 0;
        d->armed[ a ] = true;
    }
    for ( int k = 0; k < 4; k++ )
        d->down[ k ] = false;
}

static void setDown( StickDpad* d ) {
    for ( int k = 0; k < 4; k++ )
        d->down[ k ] = false;
    if ( d->axis == 1 )
        d->down[ d->sign > 0 ? STICK_UP : STICK_DOWN ] = true;
    else if ( d->axis == 0 )
        d->down[ d->sign > 0 ? STICK_RIGHT : STICK_LEFT ] = true;
}

void stickDpadFeed( StickDpad* d, float x, float y, bool pressDown, uint32_t nowMs ) {
    float v[ 2 ] = { x, y };
    float mag[ 2 ] = { x < 0 ? -x : x, y < 0 ? -y : y };
    // The off band and the re-arm, per axis.
    for ( int a = 0; a < 2; a++ ) {
        if ( mag[ a ] < d->offAt ) {
            if ( !d->inside[ a ] ) {
                d->inside[ a ] = true;
                d->insideSinceMs[ a ] = nowMs;
            }
            if ( !d->armed[ a ] && nowMs - d->insideSinceMs[ a ] >= d->rearmMs )
                d->armed[ a ] = true;
        } else {
            d->inside[ a ] = false;
        }
    }
    // A direction on: it lasts until its axis is back inside the off band.
    if ( d->axis >= 0 ) {
        if ( mag[ d->axis ] < d->offAt ) {
            // The spring-back is not a direction - and nor is the OTHER axis's
            // lean, still past 'on' as the pushed axis lets go: both have to
            // rest in the off band first (2026-09-26: a push down with a lean
            // left fired left as it was released, and left changes a value).
            d->armed[ 0 ] = d->armed[ 1 ] = false;
            d->axis = -1;
            d->sign = 0;
        }
        setDown( d );
        return;
    }
    // None on: the dominant axis, past the on threshold and armed, is a
    // candidate; it has to hold holdMs, and the stick's button cancels it.
    int candidate = -1, candidateSign = 0;
    if ( !pressDown ) {
        int a = mag[ 0 ] >= mag[ 1 ] ? 0 : 1;
        if ( mag[ a ] >= d->onAt && d->armed[ a ] ) {
            candidate = a;
            candidateSign = v[ a ] > 0 ? 1 : -1;
        }
    }
    if ( candidate != d->pendingAxis || candidateSign != d->pendingSign ) {
        d->pendingAxis = candidate;
        d->pendingSign = candidateSign;
        d->pendingSinceMs = nowMs;
    }
    if ( d->pendingAxis >= 0 && nowMs - d->pendingSinceMs >= d->holdMs ) {
        d->axis = d->pendingAxis;
        d->sign = d->pendingSign;
        d->pendingAxis = -1;
        d->pendingSign = 0;
    }
    setDown( d );
}
