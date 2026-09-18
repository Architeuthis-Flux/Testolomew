// SPDX-License-Identifier: MIT
#include "MagLocator.h"

#include <math.h>

#include "Console.h"
#include "MagArray.h"

#define STREAM_EVERY_N_FIXES 5 // 20 Hz of CSV

MagLocator& magLocator = MagLocator::getInstance( );

static float middleOf( float a, float b, float c ) {
    if ( ( a <= b && b <= c ) || ( c <= b && b <= a ) )
        return b;
    if ( ( b <= a && a <= c ) || ( c <= a && a <= b ) )
        return a;
    return c;
}

MagLocator& MagLocator::getInstance( ) {
    static MagLocator instance;
    return instance;
}

static void onStream( Stream* out ) {
    magLocator.streaming = !magLocator.streaming;
    if ( magLocator.streaming ) {
        out->println( "fix,t_ms,present,valid,x,y,z,tip_x,tip_y,tip_z,axis_x,axis_y,axis_z,tilt_deg,strength,residual_mT,fit_us,misfit,seen_by,sigma_x,sigma_y,sigma_z,faint_by,raw_x,raw_y,raw_z" );
    }
}

static void onLatest( Stream* out ) { magLocator.printFix( out ); }

static void onOrientation( Stream* out ) { magLocator.printOrientationCheck( out ); }

static void onLearn( Stream* out ) {
    magLocator.startLearningStrength( );
    out->println( "learning this magnet: move it slowly over the array, 1.5-3 cm up, for about ten seconds" );
}

static void onForget( Stream* out ) {
    magLocator.forgetStrength( );
    out->println( "magnet strength forgotten: fitting it freely again" );
}

static void onTipOffset( Stream* out ) {
    long mm = consoleReadNumber( out, "mm from the magnet's centre down the shaft to the probe's point (0 = the magnet is at the point), then Enter: ", 8000 );
    if ( mm < 0 || mm > 100 ) {
        out->println( "no number (0-100) - nothing changed" );
        return;
    }
    magLocator.tipOffsetMm = (float)mm;
    char line[ 120 ];
    snprintf( line, sizeof( line ), "the point is %ld mm down the shaft from the magnet; MAGLOC_TIP_OFFSET_MM makes it permanent", mm );
    out->println( line );
}

static void onMagnetAngle( Stream* out ) {
    long deg = consoleReadNumber( out, "the magnet's angle to the shaft, degrees (0 = magnetised along it, 90 = a disc lying flat on it), then Enter: ", 8000 );
    if ( deg < 0 || deg > 90 ) {
        out->println( "no number (0-90) - nothing changed" );
        return;
    }
    magLocator.magnetAngleDeg = (float)deg;
    char line[ 160 ];
    snprintf( line, sizeof( line ), "magnet at %ld deg to the shaft; MAGLOC_MAGNET_ANGLE_DEG makes it permanent.%s", deg,
              deg > 45 ? " Across the shaft the pole cannot show a lean sideways to itself: hold the probe upright." : "" );
    out->println( line );
}

void MagLocator::begin( ) {
    consoleAddCommand( 'd', "stream probe fixes as CSV (toggle)", onStream );
    consoleAddCommand( 'l', "latest probe fix", onLatest );
    consoleAddCommand( 'o', "orientation check: hold a magnet 1-2 cm over the array first", onOrientation );
    consoleAddCommand( 'k', "learn this magnet's strength and hold the fit to it", onLearn );
    consoleAddCommand( 'K', "forget the magnet strength (fit it freely)", onForget );
    consoleAddCommand( 't', "the magnet's centre is <number> mm up the shaft from the probe's point (t12<Enter>)", onTipOffset );
    consoleAddCommand( 'T', "the magnet's angle to the shaft: T0 = along it, T90 = a disc lying flat on it", onMagnetAngle );
}

// The shaft carried on from the point down to the board's surface. A point
// already at or below the surface, or a shaft too near level to meet it
// anywhere sensible, stays where it is.
Vec3 MagLocator::pointerOf( Vec3 tip, Vec3 shaft ) const {
    float drop = tip.z - boardZ;
    if ( drop <= 0.0f || shaft.z < 0.3f ) {
        return tip;
    }
    float k = drop / shaft.z;
    Vec3 pointer = { tip.x - k * shaft.x, tip.y - k * shaft.y, tip.z - k * shaft.z };
    return pointer;
}

ServiceStatus MagLocator::service( ) {
    if ( magArray.frameCount == lastFrame || !magArray.baselineReady( ) ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    lastFrame = magArray.frameCount;

    // Glitch filter: each axis of each sensor goes through a median of its last
    // three frames. The bus has no checksum, and one corrupted read would
    // otherwise sit in the smoothing below for ten frames, none of which fit.
    // It costs one frame (10 ms) of delay and passes any real movement.
    Vec3 filtered[ MAGFIT_MAX_SENSORS ];
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        const Vec3& b = magArray.field[ i ];
        filtered[ i ] = { middleOf( b.x, recent[ i ][ 0 ].x, recent[ i ][ 1 ].x ), middleOf( b.y, recent[ i ][ 0 ].y, recent[ i ][ 1 ].y ), middleOf( b.z, recent[ i ][ 0 ].z, recent[ i ][ 1 ].z ) };
        if ( magArray.fresh[ i ] ) {
            recent[ i ][ 1 ] = recent[ i ][ 0 ];
            recent[ i ][ 0 ] = b;
        }
    }

    // How strong is the strongest reading? That sets the smoothing.
    float rawPeakSq = 0.0f;
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        const Vec3& b = filtered[ i ];
        float sq = b.x * b.x + b.y * b.y + b.z * b.z;
        if ( magArray.fresh[ i ] && sq > rawPeakSq ) {
            rawPeakSq = sq;
        }
    }
    // A frame in which no sensor was read at all (the loop was held up, or the
    // bus was being reset) says nothing about the magnet: wait for the next one.
    int freshCount = 0;
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        freshCount += magArray.fresh[ i ] ? 1 : 0;
    }
    if ( freshCount == 0 ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }

    float alpha = sqrtf( rawPeakSq ) / MAGLOC_FAST_MT;
    if ( alpha < MAGLOC_SLOWEST_ALPHA )
        alpha = MAGLOC_SLOWEST_ALPHA;
    if ( alpha > 1.0f )
        alpha = 1.0f;

    float peakSq = 0.0f;
    fix.seenBy = 0;
    fix.faintBy = 0;
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        if ( !magArray.fresh[ i ] ) {
            continue;
        }
        Vec3& b = smooth[ i ];
        b.x += alpha * ( filtered[ i ].x - b.x );
        b.y += alpha * ( filtered[ i ].y - b.y );
        b.z += alpha * ( filtered[ i ].z - b.z );
        float sq = b.x * b.x + b.y * b.y + b.z * b.z;
        if ( sq > peakSq ) {
            peakSq = sq;
        }
        if ( sq > MAGLOC_SEEN_MT * MAGLOC_SEEN_MT ) {
            fix.seenBy++;
        } else if ( sq > MAGLOC_FAINT_MT * MAGLOC_FAINT_MT ) {
            fix.faintBy++;
        }
    }
    fix.peakMt = sqrtf( peakSq );
    // A little stickiness, so a magnet right at the threshold does not flicker in and out.
    fix.present = fix.peakMt > ( fix.present ? 0.75f * MAGLOC_PRESENT_MT : MAGLOC_PRESENT_MT );

    if ( !fix.present || fix.seenBy + fix.faintBy < MAGLOC_MIN_SENSORS ) {
        fix.valid = false;
        result.valid = false; // next time is a cold start
        haveSmoothed = false;
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }

    bool wasTracking = result.valid;
    if ( !wasTracking ) {
        uint32_t now = millis( );
        if ( now < nextColdStartMs ) {
            lastStatus = ServiceStatus::IDLE;
            return lastStatus;
        }
        nextColdStartMs = now + MAGLOC_COLD_START_PERIOD_MS;
    }
    Vec3 lastGood = result.position;

    uint32_t start = micros( );
    bool good;
    if ( knownStrength > 0.0f && !learning( ) ) {
        good = magFitSolveKnownStrength( magArray.position, smooth, magArray.fresh, magArray.sensorCount( ), MAGLOC_MAX_MISFIT, knownStrength, &result );
    } else {
        good = magFitSolve( magArray.position, smooth, magArray.fresh, magArray.sensorCount( ), MAGLOC_MAX_MISFIT, &result );
    }
    fix.fitUs = micros( ) - start;
    Vec3 position = result.position, sigma = result.sigma;
    float residual = result.residual, signal = result.signal;

    fix.residual = residual;
    fix.misfit = signal > 0.0f ? residual / signal : 1.0f;
    fix.sigma = sigma;
    fix.errorXyMm = sqrtf( sigma.x * sigma.x + sigma.y * sigma.y );
    fix.errorMm = sqrtf( fix.errorXyMm * fix.errorXyMm + sigma.z * sigma.z );
    if ( fix.errorMm > MAGLOC_MAX_ERROR_MM ) {
        good = false; // it "fits", but it could be anywhere
    }
    fix.valid = good;
    fix.rawMagnet = position;

    bool keepTracking = good;
    if ( good ) {
        misses = 0;
    } else if ( wasTracking && misses < MAGLOC_MAX_MISSES ) {
        // One frame that will not fit (the magnet moved fast enough to smear
        // the smoothed fields, a glitched reading) is not a lost magnet: no fix
        // for this frame, but the next one starts from the last good place
        // instead of waiting out the cold-start period.
        misses++;
        result.position = lastGood;
        keepTracking = true;
    }
    result.valid = keepTracking;

    if ( good && learning( ) && fix.misfit < MAGLOC_LEARN_MAX_MISFIT && fix.seenBy >= MAGLOC_LEARN_MIN_SENSORS ) {
        learnFrom( result.strength );
    }
    if ( !fix.valid ) {
        // Streamed too (valid = 0), so that a spot where nothing fits can be studied.
        Stream* out = console.port( );
        if ( streaming && out != nullptr && ++streamTick % STREAM_EVERY_N_FIXES == 0 ) {
            fix.magnet = fix.rawMagnet;
            printFixCsv( out );
        }
        haveSmoothed = false;
        lastStatus = ServiceStatus::BUSY;
        return lastStatus;
    }

    // Smooth what is shown; the solver keeps warm-starting from its own raw answer.
    float k = haveSmoothed ? MAGLOC_SMOOTHING : 1.0f;
    fix.magnet.x += k * ( position.x - fix.magnet.x );
    fix.magnet.y += k * ( position.y - fix.magnet.y );
    fix.magnet.z += k * ( position.z - fix.magnet.z );
    haveSmoothed = true;

    // The probe's shaft from the magnet's pole (see MAGLOC_MAGNET_ANGLE_DEG):
    // the direction at the magnet's angle from the pole that is nearest to
    // straight up. `up` is the perpendicular to the pole in the pole's own
    // vertical plane; the shaft is cos(angle) along the pole (either way) plus
    // sin(angle) along `up`, whichever way is higher.
    fix.strength = result.strength;
    float inv = result.strength > 0.0f ? 1.0f / result.strength : 0.0f;
    Vec3 pole = { result.moment.x * inv, result.moment.y * inv, result.moment.z * inv };
    fix.axis = pole;
    Vec3 up = { -pole.z * pole.x, -pole.z * pole.y, 1.0f - pole.z * pole.z }; // z-hat with its pole part removed
    float upLength = sqrtf( up.x * up.x + up.y * up.y + up.z * up.z );
    if ( upLength > 1e-3f ) {
        up = { up.x / upLength, up.y / upLength, up.z / upLength };
    } else {
        up = { 1, 0, 0 }; // the pole is vertical: every perpendicular is level, take one
    }
    float c = cosf( magnetAngleDeg * (float)M_PI / 180.0f ), s = sinf( magnetAngleDeg * (float)M_PI / 180.0f );
    Vec3 one = { c * pole.x + s * up.x, c * pole.y + s * up.y, c * pole.z + s * up.z };
    Vec3 other = { -c * pole.x + s * up.x, -c * pole.y + s * up.y, -c * pole.z + s * up.z };
    fix.shaft = one.z >= other.z ? one : other;
    fix.tiltDeg = acosf( fix.shaft.z > 1.0f ? 1.0f : fix.shaft.z ) * 180.0f / (float)M_PI;
    fix.tip = { fix.magnet.x - tipOffsetMm * fix.shaft.x, fix.magnet.y - tipOffsetMm * fix.shaft.y, fix.magnet.z - tipOffsetMm * fix.shaft.z };
    fix.rawTip = { fix.rawMagnet.x - tipOffsetMm * fix.shaft.x, fix.rawMagnet.y - tipOffsetMm * fix.shaft.y, fix.rawMagnet.z - tipOffsetMm * fix.shaft.z };
    fix.pointer = pointerOf( fix.tip, fix.shaft );
    fix.rawPointer = pointerOf( fix.rawTip, fix.shaft );
    fix.count++;

    Stream* out = console.port( );
    if ( streaming && out != nullptr && ++streamTick % STREAM_EVERY_N_FIXES == 0 ) {
        printFixCsv( out );
    }

    lastStatus = ServiceStatus::BUSY;
    return lastStatus;
}

void MagLocator::startLearningStrength( ) {
    learnCount = 0;
}

void MagLocator::forgetStrength( ) {
    learnCount = -1;
    knownStrength = 0.0f;
}

// Collect good free fixes; when there are enough, take the median (a few wild
// ones do not matter) and hold the fit to it from then on.
void MagLocator::learnFrom( float strength ) {
    int at = learnCount++;
    while ( at > 0 && learned[ at - 1 ] > strength ) { // keep the list sorted
        learned[ at ] = learned[ at - 1 ];
        at--;
    }
    learned[ at ] = strength;
    if ( learnCount < MAGLOC_LEARN_FIXES ) {
        return;
    }
    knownStrength = learned[ MAGLOC_LEARN_FIXES / 2 ];
    float spread = ( learned[ MAGLOC_LEARN_FIXES * 3 / 4 ] - learned[ MAGLOC_LEARN_FIXES / 4 ] ) / knownStrength;
    learnCount = -1;

    Stream* out = console.port( );
    if ( out != nullptr ) {
        char line[ 160 ];
        snprintf( line, sizeof( line ), "magnet strength %.0f mT*mm^3 (middle half of the fixes within %.1f %%) - held from now on. MAGLOC_MAGNET_STRENGTH makes it permanent.",
                  knownStrength, spread * 100.0f );
        out->println( line );
    }
}

void MagLocator::printFixCsv( Stream* out ) const {
    char line[ 340 ];
    snprintf( line, sizeof( line ), "fix,%lu,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,%.0f,%.3f,%lu,%.3f,%d,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f",
              (unsigned long)millis( ), fix.present, fix.valid, fix.magnet.x, fix.magnet.y, fix.magnet.z,
              fix.tip.x, fix.tip.y, fix.tip.z, fix.axis.x, fix.axis.y, fix.axis.z,
              fix.tiltDeg, fix.strength, fix.residual, (unsigned long)fix.fitUs, fix.misfit, fix.seenBy, fix.sigma.x, fix.sigma.y, fix.sigma.z,
              fix.faintBy, fix.rawMagnet.x, fix.rawMagnet.y, fix.rawMagnet.z );
    out->println( line );
}

void MagLocator::printFix( Stream* out ) const {
    char line[ 300 ];
    if ( !fix.present ) {
        snprintf( line, sizeof( line ), "no magnet (strongest reading %.3f mT, threshold %.2f)", fix.peakMt, MAGLOC_PRESENT_MT );
    } else if ( fix.seenBy + fix.faintBy < MAGLOC_MIN_SENSORS ) {
        snprintf( line, sizeof( line ), "magnet near (%.2f mT) but only %d sensor%s notice it (%.3f mT or more) - a fix needs %d. Closer, stronger magnet, or tighter sensor pitch.",
                  fix.peakMt, fix.seenBy + fix.faintBy, fix.seenBy + fix.faintBy == 1 ? "" : "s", MAGLOC_FAINT_MT, MAGLOC_MIN_SENSORS );
    } else if ( !fix.valid ) {
        snprintf( line, sizeof( line ), "magnet seen by %d sensors, faintly by %d (strongest %.2f mT) but no usable fix: misfit %.0f %% (limit %.0f), error bar %.1f mm (limit %.0f)",
                  fix.seenBy, fix.faintBy, fix.peakMt, fix.misfit * 100.0f, MAGLOC_MAX_MISFIT * 100.0f, fix.errorMm, MAGLOC_MAX_ERROR_MM );
    } else {
        snprintf( line, sizeof( line ), "magnet at x %.1f +/-%.1f  y %.1f +/-%.1f  z %.1f +/-%.1f mm   tilt %.0f deg   strength %.0f%s   misfit %.0f %%   seen by %d+%d faint   fit %lu us",
                  fix.magnet.x, fix.sigma.x, fix.magnet.y, fix.sigma.y, fix.magnet.z, fix.sigma.z, fix.tiltDeg, fix.strength,
                  learning( ) ? " (learning)" : ( knownStrength > 0.0f ? " (held)" : "" ), fix.misfit * 100.0f, fix.seenBy, fix.faintBy, (unsigned long)fix.fitUs );
    }
    out->println( line );
}

// ---- orientation check -------------------------------------------------------

struct OrientationGuess {
    int rotationA, rotationB; // package rotation of set A / set B, degrees
    bool underside;
    bool rowsSwapped;          // relative to the table as it stands
    bool reversedA, reversedB; // that set's order along x turned round, ditto
    float misfit;              // residual / signal of the best dipole (0 = perfect)
    Vec3 magnet;
};

#define ORIENTATION_SHOWN 6

void MagLocator::printOrientationCheck( Stream* out ) const {
    char line[ 160 ];
    int count = magArray.sensorCount( );

    if ( !magArray.baselineReady( ) ) {
        out->println( "the baseline is still being averaged - try again in a second" );
        return;
    }

    // Average a third of a second of readings, as the sensors gave them
    // (whatever the table says now). This runs inside a console command, so
    // the array is sampled by hand here.
    const int frames = 32;
    Vec3 reading[ MAG_SENSOR_COUNT ] = { };
    bool use[ MAG_SENSOR_COUNT ];
    for ( int i = 0; i < count; i++ ) {
        use[ i ] = magArray.sensor( i ).ok;
    }
    float yMin = 1e9f, yMax = -1e9f;
    for ( int f = 0; f < frames; f++ ) {
        magArray.service( );
        for ( int i = 0; i < count; i++ ) {
            Vec3 r = magArray.sensorFrameField( i );
            reading[ i ].x += r.x / frames;
            reading[ i ].y += r.y / frames;
            reading[ i ].z += r.z / frames;
        }
        delay( 10 );
    }

    int seen = 0;
    float strongest = 0.0f;
    out->print( "averaged |B| per sensor (mT):" );
    for ( int i = 0; i < count; i++ ) {
        float magnitude = sqrtf( reading[ i ].x * reading[ i ].x + reading[ i ].y * reading[ i ].y + reading[ i ].z * reading[ i ].z );
        snprintf( line, sizeof( line ), "  %d: %.3f", i, magnitude );
        out->print( line );
        if ( magnitude > MAGLOC_SEEN_MT )
            seen++;
        if ( magnitude > strongest )
            strongest = magnitude;
        if ( magArray.position[ i ].y < yMin )
            yMin = magArray.position[ i ].y;
        if ( magArray.position[ i ].y > yMax )
            yMax = magArray.position[ i ].y;
    }
    out->println( );
    if ( seen < 4 ) {
        snprintf( line, sizeof( line ), "only %d sensors read above %.2f mT. The check compares sensors with each other, so it needs at least four", seen, MAGLOC_SEEN_MT );
        out->println( line );
        out->println( "seeing the magnet - hold it higher and nearer the middle, or use a stronger one - and press o again." );
        return;
    }

    OrientationGuess best[ ORIENTATION_SHOWN ];
    int kept = 0;
    OrientationGuess current = { };
    current.misfit = -1.0f;

    // Each set's extent along x, for turning its order round.
    float xMinSet[ 2 ] = { 1e9f, 1e9f }, xMaxSet[ 2 ] = { -1e9f, -1e9f };
    for ( int i = 0; i < count; i++ ) {
        int set = magSensorPlaces[ i ].gndPin == magSensorPlaces[ 0 ].gndPin ? 0 : 1;
        if ( magArray.position[ i ].x < xMinSet[ set ] )
            xMinSet[ set ] = magArray.position[ i ].x;
        if ( magArray.position[ i ].x > xMaxSet[ set ] )
            xMaxSet[ set ] = magArray.position[ i ].x;
    }

    out->print( "fitting 256 combinations (about 15 s) " );
    for ( int layout = 0; layout < 8; layout++ ) {
        bool swapped = layout & 1;
        bool reversed[ 2 ] = { ( layout & 2 ) != 0, ( layout & 4 ) != 0 };
        out->print( '.' );
        for ( int underside = 0; underside < 2; underside++ ) {
            for ( int rotationA = 0; rotationA < 360; rotationA += 90 ) {
                for ( int rotationB = 0; rotationB < 360; rotationB += 90 ) {
                    Vec3 place[ MAG_SENSOR_COUNT ];
                    Vec3 field[ MAG_SENSOR_COUNT ];
                    bool isCurrent = layout == 0;
                    for ( int i = 0; i < count; i++ ) {
                        int set = magSensorPlaces[ i ].gndPin == magSensorPlaces[ 0 ].gndPin ? 0 : 1;
                        int rotation = set == 0 ? rotationA : rotationB;
                        place[ i ] = magArray.position[ i ];
                        if ( swapped ) {
                            place[ i ].y = yMin + yMax - place[ i ].y;
                        }
                        if ( reversed[ set ] ) {
                            place[ i ].x = xMinSet[ set ] + xMaxSet[ set ] - place[ i ].x;
                        }
                        field[ i ] = magSensorToBoard( reading[ i ], rotation, underside != 0 );
                        // "current" = the table's rotation is within 45 degrees of this one
                        float off = fabsf( fmodf( magSensorPlaces[ i ].rotationDeg - rotation + 540.0f, 360.0f ) - 180.0f );
                        isCurrent &= off < 45.0f && magSensorPlaces[ i ].underside == ( underside != 0 );
                    }

                    MagFitResult trial = { };
                    magFitSolve( place, field, use, count, 1e6f, &trial ); // no misfit limit: we want the number
                    OrientationGuess guess = { rotationA, rotationB, underside != 0, swapped, reversed[ 0 ], reversed[ 1 ],
                                               trial.signal > 0.0f ? trial.residual / trial.signal : 1.0f, trial.position };
                    if ( isCurrent ) {
                        current = guess;
                    }

                    // Keep the best few, best first.
                    int at = kept < ORIENTATION_SHOWN ? kept : ORIENTATION_SHOWN;
                    while ( at > 0 && best[ at - 1 ].misfit > guess.misfit ) {
                        at--;
                    }
                    if ( at < ORIENTATION_SHOWN ) {
                        int last = kept < ORIENTATION_SHOWN ? kept : ORIENTATION_SHOWN - 1;
                        for ( int k = last; k > at; k-- ) {
                            best[ k ] = best[ k - 1 ];
                        }
                        best[ at ] = guess;
                        if ( kept < ORIENTATION_SHOWN ) {
                            kept++;
                        }
                    }
                }
            }
        }
    }
    out->println( );

    snprintf( line, sizeof( line ), "orientation check, %d sensors see the magnet, strongest %.2f mT. Misfit: a few %% = a dipole explains it, tens of %% = it cannot.", seen, strongest );
    out->println( line );
    out->println( "  misfit   rows     side       set A: rot  order      set B: rot  order      magnet would be at (mm)" );
    for ( int k = 0; k < kept; k++ ) {
        const OrientationGuess& g = best[ k ];
        snprintf( line, sizeof( line ), "  %5.1f%%   %-7s  %-9s  %10d  %-9s  %10d  %-9s  x %6.1f  y %6.1f  z %5.1f",
                  g.misfit * 100.0f, g.rowsSwapped ? "SWAPPED" : "as is", g.underside ? "underside" : "top",
                  g.rotationA, g.reversedA ? "REVERSED" : "as is", g.rotationB, g.reversedB ? "REVERSED" : "as is",
                  g.magnet.x, g.magnet.y, g.magnet.z );
        out->println( line );
    }
    out->println( "Every answer shows up four times: itself, its mirror image top-to-bottom and left-to-right (other SIDE -" );
    out->println( "the fields cannot tell a mirrored magnet and array from the real one), and the same array with the frame" );
    out->println( "turned half way round. Take the line whose side and rotation match what is really on the board." );
    if ( current.misfit >= 0.0f ) {
        snprintf( line, sizeof( line ), "the table as it stands (%s, set A %d, set B %d): misfit %.1f%%", current.underside ? "underside" : "top",
                  current.rotationA, current.rotationB, current.misfit * 100.0f );
        out->println( line );
    }
}
