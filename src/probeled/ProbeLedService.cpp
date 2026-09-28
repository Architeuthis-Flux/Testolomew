// SPDX-License-Identifier: MIT
#include "ProbeLedService.h"

#include "BoardPins.h"
#include "Console.h"
#include "LedStrip.h"
#include "MagLocator.h"
#include "config.h"
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif
#if MODULE_PLAY
#include "Play.h"
#endif

ProbeLedService& probeLeds = ProbeLedService::getInstance( );

ProbeLedService& ProbeLedService::getInstance( ) {
    static ProbeLedService instance;
    return instance;
}

static void onStream( Stream* out ) {
    probeLeds.streaming = !probeLeds.streaming;
    if ( probeLeds.streaming ) {
        out->println( "cursor,state,along_rows,across_mm,sigma_rows,sigma_across_mm,confidence,height_mm,have_under,under_along,under_across,tilt_deg,aim_deg,speed_mm_s" );
    }
}

static void onStrip( Stream* out ) {
    probeLeds.setStrip( !probeLeds.strip, out );
}

void ProbeLedService::setStrip( bool on, Stream* out ) {
    if ( on == strip ) {
        return;
    }
    strip = on;
    ledStripTouch( &chain );
    ledStripTouch( &top );
    if ( !strip ) {
        ledStripWait( &chain, 50 );
        ledStripClear( &chain );
        ledStripShow( &chain );
        if ( topStrip ) {
            ledStripWait( &top, 50 );
            ledStripClear( &top );
            ledStripShow( &top );
        }
    }
    if ( out != nullptr ) {
        out->println( strip ? "LED strip on" : "LED strip off (cleared)" );
    }
}

static void onStripTest( Stream* out ) {
    char line[ 240 ];
    if ( !probeLeds.strip ) {
        snprintf( line, sizeof( line ), "LED strip is not set up: no SPI peripheral has the pin as its MOSI (peripheral %d)", ledStripPeripheral( &probeLeds.chain ) );
        out->println( line );
        return;
    }
    // How the frames have been going out: the statistics since the last
    // report. Then one frame sent by hand, watched: proof the SPI is clocking
    // bytes out, whatever the LEDs make of them.
    LedStrip* s = &probeLeds.chain;
    ledStripReport( s, line, sizeof( line ) );
    out->print( "chain on " );
    out->println( line );
    snprintf( line, sizeof( line ), "a run: render %lu us, strip %lu us (fill %lu, budget+set %lu, show %lu)", (unsigned long)probeLeds.renderUs, (unsigned long)probeLeds.stripUs,
              (unsigned long)probeLeds.stripFillUs, (unsigned long)probeLeds.stripBudgetUs, (unsigned long)probeLeds.stripShowUs );
    out->println( line );
    snprintf( line, sizeof( line ), "power: the last frame %.0f mA by the model (%.0f mA a channel at full - an assumption), budget %.0f mA (the menu's %.0f, the ceiling %.0f); %lu frames scaled down to it, the worst to x%.2f",
              probeLeds.stripLastMa, PROBELED_MA_PER_CHANNEL, probeLeds.stripBudgetMa( ), probeLeds.stripMaxMa, PROBELED_STRIP_HARD_MAX_MA, (unsigned long)probeLeds.stripScaledFrames, probeLeds.stripWorstScale );
    out->println( line );
    probeLeds.stripWorstScale = 1.0f;
    if ( probeLeds.topStrip ) {
        ledStripReport( &probeLeds.top, line, sizeof( line ) );
        out->print( "rails on " );
        out->println( line );
    }
    ledStripWait( s, 50 ); // the service's frame, if one is on the wire
    ledStripClear( s );
    ledStripSet( s, 0, 40, 40, 40 );
    uint32_t started = micros( );
    bool began = ledStripShow( s );
    uint32_t left = ledStripBytesLeft( s );
    delay( 2 );
    uint32_t left2 = ledStripBytesLeft( s );
    bool done = ledStripWait( s, 30 );
    snprintf( line, sizeof( line ), "one frame of %d bytes %s: %lu bytes left after 0 ms, %lu after 2 ms, %s after %lu us", s->count * LEDSTRIP_BYTES_PER_LED,
              began ? "started" : "NOT started", (unsigned long)left, (unsigned long)left2, done ? "finished" : "STILL GOING", (unsigned long)( micros( ) - started ) );
    out->println( line );
    probeLeds.stripTestStartMs = millis( );
    probeLeds.stripTestUntilMs = probeLeds.stripTestStartMs + PROBELED_CHASE_MS;
    snprintf( line, sizeof( line ),
              "now a dot runs from pixel 0 (row 1, outer hole) up the chain, %d pixels a second: rows 1-60 five LEDs each, then the rails from pixel 300 (top outer, top inner, bottom inner, bottom outer, each from the row-1 end)%s",
              PROBELED_CHASE_PIXELS_PER_S, probeLeds.topStrip ? " - on the rails' own strip too" : "" );
    out->println( line );
}

static void onBoard( Stream* out ) {
    probeLeds.useV5( !probeLeds.v5 );
    out->println( probeLeds.v5 ? ( probeLeds.strip ? "LED layout: V5 (a V5 chain is wired, so it stays V5)" : "LED layout: V5 (5+5 holes across a 7.62 mm channel, 4 x 25 rail LEDs)" )
                               : "LED layout: V6 (6+6 holes meeting at the centre, 4 x 30 rail LEDs)" );
}

void ProbeLedService::useV5( bool on ) {
    layoutGeneration++;
    if ( strip && !on ) {
        on = true; // a real V5 chain is wired: its layout is not a matter of choice
    }
    v5 = on;
    if ( v5 ) {
        ledLayoutV5( &layout );
    } else {
        ledLayoutV6( &layout );
    }
    probeLedClear( &frame, layout.count );
}

void ProbeLedService::begin( ) {
    probeLedDefaultStyle( &style );
    useV5( false );
    consoleAddCommand( 'L', "stream the LED cursor for a V5 (toggle)", onStream );
    consoleAddCommand( 'B', "LED layout: V6 / V5 (toggle)", onBoard );
    if ( PIN_LED_STRIP >= 0 ) {
        // A real chain: the V5 layout, and drive it.
        strip = ledStripBegin( &chain, PIN_LED_STRIP, PIN_LED_STRIP_SCK, PIN_LED_STRIP_MISO, LED_STRIP_COUNT, PROBELED_CHAIN_DMA );
        if ( strip ) {
            useV5( true );
            ledStripShow( &chain ); // all off
        }
        Stream* out = console.port( );
        if ( out != nullptr ) {
            char line[ 120 ];
            snprintf( line, sizeof( line ), strip ? "LED strip: %d LEDs on SPI%d (data on the MOSI pin), DMA1 channel %d" : "LED strip: NOT set up, no SPI has PIN_LED_STRIP as its MOSI (%d)",
                      LED_STRIP_COUNT, ledStripPeripheral( &chain ), PROBELED_CHAIN_DMA );
            out->println( line );
        }
        if ( strip && PIN_LED_STRIP_TOP >= 0 ) {
            topStrip = ledStripBegin( &top, PIN_LED_STRIP_TOP, PIN_LED_STRIP_TOP_SCK, PIN_LED_STRIP_TOP_MISO, LED_STRIP_TOP_COUNT, PROBELED_TOP_DMA );
            if ( topStrip ) {
                ledStripShow( &top );
            }
            if ( out != nullptr ) {
                char line[ 120 ];
                snprintf( line, sizeof( line ), topStrip ? "rail strip: %d LEDs on SPI%d, DMA1 channel %d" : "rail strip: NOT set up, no SPI has PIN_LED_STRIP_TOP as its MOSI (%d)",
                          LED_STRIP_TOP_COUNT, ledStripPeripheral( &top ), PROBELED_TOP_DMA );
                out->println( line );
            }
        }
        consoleAddCommand( 'N', "the V5 LED strip on/off (toggle)", onStrip );
        consoleAddCommand( 'n', "LED strip: frame statistics, then a dot runs up the whole chain (rows, then rails)", onStripTest );
    }
}

// The V5 layout's LEDs to the chain, in JumperlOS's pixel order: the holes
// 0-299, the rails 300-399 after them - and the rails again, as 0-99, to
// the rail strip if there is one. A strip still busy with the last frame is
// left alone this tick (the frame is not worth a wait; the next tick's is
// fresher).
void ProbeLedService::sendStrip( ) {
    bool chainFree = !ledStripBusy( &chain );
    bool topFree = topStrip && !ledStripBusy( &top );
    if ( !chainFree && !topFree ) {
        return;
    }
    float scale = stripBrightness;
    for ( int p = 0; p < LED_STRIP_COUNT; p++ ) {
        stripRgb[ p ][ 0 ] = stripRgb[ p ][ 1 ] = stripRgb[ p ][ 2 ] = 0;
    }
    if ( stripTestUntilMs != 0 ) {
        // The chase: one white dot walking up the chain, with the pixels
        // behind it in the same row (or rail group of five) left faintly lit.
        uint32_t now = millis( );
        if ( (int32_t)( now - stripTestUntilMs ) >= 0 ) {
            stripTestUntilMs = 0;
        } else {
            int dot = (int)( ( now - stripTestStartMs ) * PROBELED_CHASE_PIXELS_PER_S / 1000 ) % LED_STRIP_COUNT;
            for ( int p = 0; p < LED_STRIP_COUNT; p++ ) {
                uint8_t v = p == dot ? (uint8_t)( 255 * scale ) : ( p < dot && p / 5 == dot / 5 ? (uint8_t)( 40 * scale ) : 0 );
                stripRgb[ p ][ 0 ] = stripRgb[ p ][ 1 ] = stripRgb[ p ][ 2 ] = v;
            }
            sendFrame( chainFree, topFree, LED_STRIP_COUNT );
            return;
        }
    }
    uint32_t t0 = micros( );
    for ( int i = 0; i < layout.count; i++ ) {
        int pixel = ledLayoutV5Pixel( &layout, i );
        if ( pixel < 0 || pixel >= LED_STRIP_COUNT )
            continue;
        uint8_t r, g, b;
        probeLedRgb( &frame, i, &r, &g, &b );
        stripRgb[ pixel ][ 0 ] = (uint8_t)( r * scale + 0.5f );
        stripRgb[ pixel ][ 1 ] = (uint8_t)( g * scale + 0.5f );
        stripRgb[ pixel ][ 2 ] = (uint8_t)( b * scale + 0.5f );
    }
    stripFillUs = micros( ) - t0;
    sendFrame( chainFree, topFree, LED_STRIP_COUNT );
}

// stripRgb[] to the chain(s), scaled down whole if it would draw more than
// the budget (ProbeLedService.h): the picture keeps its shape, the rail
// keeps its volts.
void ProbeLedService::sendFrame( bool chainFree, bool topFree, int count ) {
    uint32_t tb = micros( );
    float ma = 0.0f;
    for ( int p = 0; p < count; p++ ) {
        ma += ( stripRgb[ p ][ 0 ] + stripRgb[ p ][ 1 ] + stripRgb[ p ][ 2 ] ) * ( PROBELED_MA_PER_CHANNEL / 255.0f );
    }
    float k = 1.0f;
    float budget = stripBudgetMa( );
    if ( budget > 0.0f && ma > budget ) {
        k = budget / ma;
        stripScaledFrames++;
        if ( k < stripWorstScale ) {
            stripWorstScale = k;
        }
    }
    // The slew: no more than PROBELED_MAX_STEP_MA above the last frame sent.
    float allowed = stripLastMa + PROBELED_MAX_STEP_MA;
    if ( ma * k > allowed && allowed > 0.0f ) {
        k = allowed / ma;
        stripSlewedFrames++;
    }
    // Scaled channels are truncated, not rounded: rounding up by half a
    // count on each of 1200 channels is 28 mA past the budget (8 % of it).
    // What is sent is what is summed for the report.
    float wasMa = stripLastMa;
    stripLastMa = 0.0f;
    for ( int p = 0; p < count; p++ ) {
        uint8_t r = stripRgb[ p ][ 0 ], g = stripRgb[ p ][ 1 ], b = stripRgb[ p ][ 2 ];
        if ( k < 1.0f ) {
            r = (uint8_t)( r * k );
            g = (uint8_t)( g * k );
            b = (uint8_t)( b * k );
        }
        stripLastMa += ( r + g + b ) * ( PROBELED_MA_PER_CHANNEL / 255.0f );
        if ( chainFree )
            ledStripSet( &chain, p, r, g, b );
        if ( topFree && p >= 300 )
            ledStripSet( &top, p - 300, r, g, b ); // the rails' own numbering
    }
    stripBudgetUs = micros( ) - tb;
    uint32_t ts = micros( );
    bool sent = false;
    if ( chainFree )
        sent = ledStripShow( &chain ) || sent;
    if ( topFree )
        sent = ledStripShow( &top ) || sent;
    stripShowUs = micros( ) - ts;
    if ( sent ) {
        // The chain's current changes only when a frame goes out.
        float step = stripLastMa - wasMa;
        if ( step < 0.0f )
            step = -step;
        if ( step > stripMaxStepMa )
            stripMaxStepMa = step;
        if ( stripFramesSent == 0 || stripLastMa < stripLeastMa )
            stripLeastMa = stripLastMa;
        if ( stripFramesSent == 0 || stripLastMa > stripMostMa )
            stripMostMa = stripLastMa;
        stripFramesSent++;
    }
}

void ProbeLedService::printCursorLine( Stream* out ) const {
    char line[ 160 ];
    snprintf( line, sizeof( line ), "cursor,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%.2f,%.0f,%.0f,%.0f", (int)input.state, input.along, input.acrossMm, input.sigmaRows, input.sigmaAcrossMm,
              input.confidence, input.heightMm, input.haveUnder ? 1 : 0, input.underAlong, input.underAcrossMm, input.tiltDeg, input.aimDeg, input.speedMmS ); // (the last three since 2026-09-25; a reader of the first ten is unchanged)
    out->println( line );
}

ServiceStatus ProbeLedService::service( ) {
    uint32_t now = micros( );
    float dtS = lastUs == 0 ? 0.02f : ( now - lastUs ) * 1e-6f;
    lastUs = now;

    const MagTrack& track = magLocator.track;
    ProbeLedInput in = { };
    // The track, in both modes: with the tracker off it is this frame's fix
    // passed straight through, and the cursor's 1-Euro filter (cursor Hz /
    // beta, slowing with the height) applies either way (2026-09-27: raw
    // mode read fix.pointer, and the levers did nothing there).
    bool live = track.enabled ? track.state != MAGTRACK_NONE : magLocator.fix.valid;
    if ( live ) {
        Vec3 cursor = track.cursor;
        Vec3 sigma = track.sigma;
        float bar = track.cursorSigmaMm;
        in.state = track.state == MAGTRACK_ROUGH ? PROBELED_ROUGH : ( track.state == MAGTRACK_COASTING ? PROBELED_COASTING : PROBELED_TRACKING );
        Vec3 barVec = { bar * 0.7071f, bar * 0.7071f, sigma.z };
#if MODULE_ROW_COUNT
        const RowGrid& grid = rowCounter.grid;
        RowPlace place = rowGridPlace( &grid, cursor );
        in.along = place.along;
        in.acrossMm = place.acrossMm;
        in.sigmaRows = rowGridSigmaAlong( &grid, barVec );
        in.sigmaAcrossMm = rowGridSigmaAcross( &grid, barVec );
        in.confidence = rowGridConfidence( place, in.sigmaRows, in.sigmaAcrossMm );
        Vec3 tip = track.tip;
        in.heightMm = track.heightMm; // through the height's own filter (2026-09-28); negative below the believed surface - the renderer clamps its own uses
        // The lean (for the aim scheme and the tilt data) and the speed.
        Vec3 shaft = track.shaft;
        in.tiltDeg = track.tiltDeg;
        in.aimDeg = atan2f( shaft.y, shaft.x ) * ( 180.0f / (float)M_PI );
        if ( in.aimDeg < 0.0f )
            in.aimDeg += 360.0f;
        in.speedMmS = sqrtf( track.velocity.x * track.velocity.x + track.velocity.y * track.velocity.y + track.velocity.z * track.velocity.z ); // 0 with the tracker off
        if ( track.reachMm > 0.5f ) {
            Vec3 under = { tip.x, tip.y, magLocator.boardZ };
            RowPlace u = rowGridPlace( &grid, under );
            in.haveUnder = true;
            in.underAlong = u.along;
            in.underAcrossMm = u.acrossMm;
        }
#else
        // Without the row counter there is no breadboard frame: x along (a row per 2.54 mm), y across.
        in.along = cursor.x / 2.54f;
        in.acrossMm = cursor.y;
        in.sigmaRows = barVec.x / 2.54f;
        in.sigmaAcrossMm = barVec.y;
        in.confidence = 0.5f;
#endif
    }
    if ( !track.enabled ) {
        // Raw fixes: the tracker's coast is not there to carry a refused
        // frame, so the last cursor is held a moment (PROBELED_RAW_HOLD_MS),
        // shown as coasting, rather than the LEDs going dark on it.
        uint32_t nowMs = millis( );
        if ( live ) {
            rawHeld = in;
            rawHeldMs = nowMs;
        } else if ( rawHeld.state != PROBELED_NONE && nowMs - rawHeldMs < PROBELED_RAW_HOLD_MS ) {
            in = rawHeld;
            if ( in.state != PROBELED_ROUGH )
                in.state = PROBELED_COASTING;
        }
    }
    input = in;
    uint32_t t0 = micros( );
#if MODULE_PLAY
    probeLedRender( &layout, &in, &style, dtS, &frame, play.mode != PLAY_OFF ? &play.paint : nullptr, &brush ); // the drawing is kept while play is off, just not shown
#else
    probeLedRender( &layout, &in, &style, dtS, &frame );
#endif
    renderUs = micros( ) - t0;

    Stream* out = console.port( );
    tick++;
    if ( streaming && out != nullptr && tick % PROBELED_STREAM_EVERY == 0 ) {
        printCursorLine( out );
    }
    if ( strip && v5 ) {
        if ( tick == 1 ) {
            // The gap from begin()'s first frame to here is the rest of
            // setup(), not a running gap: start the statistics now.
            chain.frameCount = top.frameCount = 0;
            chain.gapMaxUs = top.gapMaxUs = 0;
            chain.gapSumUs = top.gapSumUs = 0;
            chain.gapCount = top.gapCount = 0;
        }
        uint32_t t1 = micros( );
        sendStrip( );
        stripUs = micros( ) - t1;
    }
    lastStatus = ServiceStatus::BUSY;
    return lastStatus;
}
