// SPDX-License-Identifier: MIT
#include "Play.h"

#include <math.h>

#include "Console.h"
#include "ProbeLedService.h"
#include "config.h"

PlayService& play = PlayService::getInstance( );

PlayService& PlayService::getInstance( ) {
    static PlayService instance;
    return instance;
}

const char* const playModeNames[ PLAY_MODE_COUNT ] = { "off", "paint", "target" };

void playHsvToRgb( float hueDeg, float sat, uint8_t* r, uint8_t* g, uint8_t* b ) {
    if ( sat < 0.0f )
        sat = 0.0f;
    if ( sat > 1.0f )
        sat = 1.0f;
    float h = fmodf( hueDeg, 360.0f );
    if ( h < 0.0f )
        h += 360.0f;
    float sector = h / 60.0f;
    int i = (int)sector;
    float f = sector - i;
    float p = 1.0f - sat, q = 1.0f - sat * f, t = 1.0f - sat * ( 1.0f - f );
    float fr, fg, fb;
    switch ( i % 6 ) {
    case 0:
        fr = 1, fg = t, fb = p;
        break;
    case 1:
        fr = q, fg = 1, fb = p;
        break;
    case 2:
        fr = p, fg = 1, fb = t;
        break;
    case 3:
        fr = p, fg = q, fb = 1;
        break;
    case 4:
        fr = t, fg = p, fb = 1;
        break;
    default:
        fr = 1, fg = p, fb = q;
        break;
    }
    *r = (uint8_t)( fr * 255.0f + 0.5f );
    *g = (uint8_t)( fg * 255.0f + 0.5f );
    *b = (uint8_t)( fb * 255.0f + 0.5f );
}

static void onScore( Stream* out ) {
    play.printScore( out );
}

static void onMode( Stream* out ) {
    long which = consoleReadNumber( out, "0 = off, 1 = paint (the point paints the LEDs it touches), 2 = target (touch the green LED), then Enter: ", 8000 );
    if ( which < 0 || which >= PLAY_MODE_COUNT ) {
        out->println( "no number - nothing changed" );
        return;
    }
    play.mode = (int)which;
    out->print( "play: " );
    out->println( playModeNames[ play.mode ] );
}

static void onClear( Stream* out ) {
    play.clearPaint( );
    out->println( "paint cleared" );
}

void PlayService::begin( ) {
    clearPaint( );
    consoleAddCommand( 'y', "play mode <number>: 0 off, 1 paint, 2 target (y1<Enter>)", onMode );
    consoleAddCommand( 'w', "play: the target game's score and the paint's state (W clears the paint)", onScore );
    consoleAddCommand( 'W', "play: clear the paint", onClear );
}

void PlayService::clearPaint( ) {
    paintClear( &paint, &stroke );
    targetLed = -1;
}

void PlayService::movePicker( float dx, float dy ) {
    paintPickerMove( &paintHue, &paintSat, &pickerCycling, dx, dy ); // (Paint.cpp: against the rim the stick's tilt cycles the hue until it is let go)
}

// The target LED back to the paint that was under it.
void PlayService::releaseTarget( ) {
    if ( targetLed < 0 )
        return;
    paint.level[ targetLed ] = targetWasLevel;
    paint.r[ targetLed ] = targetWasR;
    paint.g[ targetLed ] = targetWasG;
    paint.b[ targetLed ] = targetWasB;
    targetLed = -1;
}

void PlayService::setPaintBright( float level ) {
    // The brush's brightness only: what is painted keeps the level it was
    // painted at (a dimmer stroke next to a brighter one is the point).
    if ( level < PLAY_BRIGHT_STEP )
        level = PLAY_BRIGHT_STEP;
    if ( level > 1.0f )
        level = 1.0f;
    paintBright = level;
}

static uint32_t xorshift( uint32_t* s ) {
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

// The LED screen's mapping (the LEDs app): 7 px a row along the
// panel, rows 1-30 above the channel line at yMid, the 31-60 half below.
void PlayService::tracePlace( float along, float acrossMm, int* x, int* y ) {
    const int cell = PLAY_TRACE_W / ( PROBELED_ROWS + 2 );
    const float pxPerMm = cell / 2.54f;
    const int x0 = ( PLAY_TRACE_W - PROBELED_ROWS * cell ) / 2;
    const int yMid = PLAY_TRACE_MID_Y;
    *x = x0 + (int)( ( along - 1.0f ) * cell + 0.5f );
    *y = yMid - (int)( acrossMm * pxPerMm + 0.5f );
}

// A dab of the brush at a place on the board (Paint.h does the rule).
void PlayService::paintAt( float along, float acrossMm ) {
    const LedLayout& layout = probeLeds.layout;
    int i = ledLayoutNearest( &layout, along, acrossMm, PLAY_WITHIN_ROWS );
    if ( i < 0 )
        return;
    PaintBrush brush;
    brush.size = (int)( brushSize + 0.5f );
    brush.erase = erase;
    brush.bright = paintBright;
    paintColour( &brush.r, &brush.g, &brush.b );
    paintDab( &paint, &stroke, &layout, i, &brush, targetLed );
}

void PlayService::newTarget( ) {
    const LedLayout& layout = probeLeds.layout;
    if ( layout.count == 0 )
        return;
    // A hole LED, not the last one, not on the outermost holes (a target at
    // the very edge is a test of the array's reach, not of the hand).
    int outermost = 0;
    for ( int k = 0; k < layout.count; k++ ) {
        if ( layout.kind[ k ] == PROBELED_HOLE && layout.hole[ k ] > outermost )
            outermost = layout.hole[ k ];
    }
    for ( int tries = 0; tries < 50; tries++ ) {
        int i = (int)( xorshift( &rng ) % (uint32_t)layout.count );
        if ( layout.kind[ i ] != PROBELED_HOLE || i == targetLed || layout.hole[ i ] >= outermost )
            continue;
        releaseTarget( );
        targetLed = i;
        targetWasLevel = paint.level[ i ];
        targetWasR = paint.r[ i ];
        targetWasG = paint.g[ i ];
        targetWasB = paint.b[ i ];
        paint.level[ i ] = 1.0f;
        paint.r[ i ] = 40;
        paint.g[ i ] = 255;
        paint.b[ i ] = 60;
        targetSinceMs = millis( );
        return;
    }
}

void PlayService::printScore( Stream* out ) const {
    char line[ 200 ];
    int painted = 0;
    for ( int i = 0; i < PROBELED_MAX; i++ )
        painted += paint.level[ i ] > 0.0f;
    snprintf( line, sizeof( line ), "play: mode %s, %d LEDs painted (hue %.0f sat %.2f bright %.2f brush %d touch %.1f mm%s); target: %lu hit, %lu missed; mean %.2f s to reach, mean miss %.2f mm; the last %.2f s, %.2f mm",
              playModeNames[ mode ], painted, paintHue, paintSat, paintBright, (int)( brushSize + 0.5f ), touchMm, erase ? ", erasing" : "", (unsigned long)hits, (unsigned long)misses, meanMs * 1e-3f,
              meanMissMm, lastMs * 1e-3f, lastMissMm );
    out->println( line );
}

ServiceStatus PlayService::service( ) {
    if ( layoutSeen != probeLeds.layoutGeneration ) {
        // The layout was rebuilt (B): the LED numbers mean other LEDs now.
        layoutSeen = probeLeds.layoutGeneration;
        clearPaint( );
    }
    if ( mode != PLAY_TARGET ) {
        releaseTarget( ); // whichever mode it left for (the menu steps target to paint directly)
    }
    // In paint mode the LED cursor is the brush itself (ProbeLedBrush).
    probeLeds.brush.active = mode == PLAY_PAINT;
    if ( probeLeds.brush.active ) {
        probeLeds.brush.radiusRows = (int)( brushSize + 0.5f );
        if ( erase ) {
            probeLeds.brush.level = 0.3f;
            probeLeds.brush.r = probeLeds.brush.g = probeLeds.brush.b = 255;
        } else {
            probeLeds.brush.level = paintBright > 0.15f ? paintBright : 0.15f;
            paintColour( &probeLeds.brush.r, &probeLeds.brush.g, &probeLeds.brush.b );
        }
        // The ring fades out as the point comes down (PLAY_RING_FADE_MM above
        // the touch height to nothing at it), so what is under it - the paint
        // there, what the stroke is about to lay over - is seen (2026-09-28,
        // Kevin: "make the outline of the brush dim to 0 as we get closer").
        float lift = probeLeds.input.heightMm - touchMm;
        float k = lift / PLAY_RING_FADE_MM;
        k = k < 0.0f ? 0.0f : ( k > 1.0f ? 1.0f : k );
        probeLeds.brush.level *= k;
    }
    if ( mode == PLAY_OFF ) {
        wasTouching = false;
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    const ProbeLedInput& in = probeLeds.input;
    // Where the point is on the board: under the tip when the tracker says
    // so (pointed mode, lifted), else the cursor itself.
    bool have = in.state == PROBELED_TRACKING || in.state == PROBELED_COASTING;
    float along = in.haveUnder ? in.underAlong : in.along;
    float across = in.haveUnder ? in.underAcrossMm : in.acrossMm;
    bool touching = have && in.heightMm < touchMm + ( wasTouching ? PLAY_TOUCH_RELEASE_MM : 0.0f ); // with hysteresis

    if ( mode == PLAY_PAINT ) {
        if ( touching && !wasTouching ) {
            paintStrokeBegin( &stroke ); // the point came down: a new stroke, which paints over what is there
        } else if ( !touching && wasTouching ) {
            paintStrokeEnd( &stroke );
        }
        if ( touching ) {
            paintAt( along, across );
        }
    } else if ( mode == PLAY_TARGET ) {
        if ( targetLed < 0 ) {
            newTarget( );
        } else if ( touching && !wasTouching ) {
            // A touch-down: was it the target?
            const LedLayout& layout = probeLeds.layout;
            float da = along - layout.along[ targetLed ];
            float dc = across - layout.acrossMm[ targetLed ];
            float missMm = sqrtf( da * 2.54f * da * 2.54f + dc * dc );
            lastMissMm = missMm;
            lastMs = (float)( millis( ) - targetSinceMs );
            if ( missMm < 2.54f * PLAY_WITHIN_ROWS ) {
                hits++;
                meanMs += ( lastMs - meanMs ) / hits;
                meanMissMm += ( missMm - meanMissMm ) / hits;
                Stream* out = console.port( );
                if ( out != nullptr ) {
                    char line[ 120 ];
                    snprintf( line, sizeof( line ), "target hit in %.2f s, %.1f mm off (row %d hole %d)", lastMs * 1e-3f, missMm, layout.row[ targetLed ], layout.hole[ targetLed ] );
                    out->println( line );
                }
                newTarget( );
            } else {
                misses++;
            }
        }
    }
    wasTouching = touching;
    lastStatus = ServiceStatus::BUSY;
    return lastStatus;
}
