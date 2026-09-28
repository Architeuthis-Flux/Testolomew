// SPDX-License-Identifier: MIT
#include "MagLocator.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Console.h"
#include "MagArray.h"
#include "config.h"
#if MODULE_ROW_COUNT
#include "RowCounter.h" // the :probe row form places the probe on the row grid
#endif

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

static void onUnpollute( Stream* out ) {
    long which = consoleReadNumber( out, "0 = the last good fix, 1 = the reference fix in MagLocator.h (2026-09-18, row 35 hole 3), then Enter: ", 8000 );
    if ( which == 1 ) {
        magLocator.lastGoodPosition = { MAGLOC_REFERENCE_X, MAGLOC_REFERENCE_Y, MAGLOC_REFERENCE_Z };
        magLocator.lastGoodAxis = { MAGLOC_REFERENCE_AX, MAGLOC_REFERENCE_AY, MAGLOC_REFERENCE_AZ };
        magLocator.lastGoodStrength = MAGLOC_REFERENCE_STRENGTH;
        magLocator.haveLastGood = true;
    } else if ( which != 0 ) {
        out->println( "no number - nothing changed" );
        return;
    }
    magLocator.unpolluteBaseline( out );
}

static void onStream( Stream* out ) {
    magLocator.streaming = !magLocator.streaming;
    if ( magLocator.streaming ) {
        out->println( "fix,t_ms,present,valid,x,y,z,tip_x,tip_y,tip_z,axis_x,axis_y,axis_z,tilt_deg,strength,residual_mT,fit_us,misfit,seen_by,sigma_x,sigma_y,sigma_z,faint_by,raw_x,raw_y,raw_z,track_state,track_x,track_y,track_z,track_sx,track_sy,track_sz,cursor_x,cursor_y,cursor_z,gate,dropped,chi" );
    }
}

static void onLatest( Stream* out ) { magLocator.printFix( out ); }

static void onOrientation( Stream* out ) { magLocator.printOrientationCheck( out ); }

static void onLearn( Stream* out ) {
    magLocator.startLearningStrength( );
    char line[ 120 ];
    snprintf( line, sizeof( line ), "holding the magnet's strength at %.0f and measuring it again: rest the probe on the board, or move it slowly 1-3 cm up", magLocator.knownStrength );
    out->println( line );
}

// :strength <n> | learn | free - a known magnet held as measured (a verb, with
// its number on the line: a key that waits for digits holds the whole loop,
// the LED strip included, for up to 8 s on a terminal that sends none).
static void onStrengthVerb( int argc, char** argv, Stream* out ) {
    char line[ 160 ];
    if ( argc >= 2 && strcmp( argv[ 1 ], "learn" ) == 0 ) {
        onLearn( out );
        return;
    }
    if ( argc >= 2 && strcmp( argv[ 1 ], "free" ) == 0 ) {
        magLocator.forgetStrength( );
        out->println( "the magnet's strength is free: fitted every frame, never held (the far probe is then out of reach)" );
        return;
    }
    float n = argc >= 2 ? (float)atof( argv[ 1 ] ) : 0.0f;
    if ( n <= 0.0f ) {
        snprintf( line, sizeof( line ), "strength: held at %.0f%s. :strength <mT*mm^3> holds a known magnet as measured; :strength learn measures it again (k); :strength free never holds it (K)", magLocator.knownStrength,
                  magLocator.strengthIsMeasured ? " (measured)" : " (not measured)" );
        out->println( line );
        return;
    }
    magLocator.holdStrength( n );
    snprintf( line, sizeof( line ), "holding the magnet's strength at %.0f as measured; the ring watches for another magnet", magLocator.knownStrength );
    out->println( line );
}

static void onForget( Stream* out ) {
    magLocator.forgetStrength( );
    out->println( "magnet strength not held: fitting it freely (k holds it again)" );
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

// The tip offset from taps: the point resting in ONE hole, the probe at a
// new angle each time, `q` takes the fix (the magnet's place and the shaft);
// `Q` solves the one t for which the points magnet - t * shaft coincide:
// with c the hole, minimise sum |m_k - t s_k - c|^2 over t and c, so
// t = sum <m_k - mean m, s_k - mean s> / sum |s_k - mean s|^2. It needs
// angles that differ (the shafts spread), and it says how well the points
// then agree (the rms about the hole).
#define TIP_CAL_MAX 12
static Vec3 tipCalMagnet[ TIP_CAL_MAX ], tipCalShaft[ TIP_CAL_MAX ];
static int tipCalCount = 0;

static void onTipSample( Stream* out ) {
    const MagProbeFix& fix = magLocator.fix;
    if ( !fix.valid || fix.rough || fix.seenBy < 4 ) {
        out->println( "no fix to take: rest the point in a hole with the magnet in the array's reach, then q" );
        return;
    }
    if ( tipCalCount >= TIP_CAL_MAX ) {
        out->println( "12 samples is the most: Q solves them (or clears them)" );
        return;
    }
    tipCalMagnet[ tipCalCount ] = fix.rawMagnet;
    tipCalShaft[ tipCalCount ] = fix.shaft;
    tipCalCount++;
    char line[ 160 ];
    snprintf( line, sizeof( line ), "tip sample %d: magnet at %.1f %.1f %.1f, tilt %.0f deg - now another angle in the same hole, then q again; Q solves (3 or more)", tipCalCount, fix.rawMagnet.x,
              fix.rawMagnet.y, fix.rawMagnet.z, fix.tiltDeg );
    out->println( line );
}

static void onTipSolve( Stream* out ) {
    // The samples q took, through the lean calibration's solver (TipModel.h,
    // 2026-09-28): one hole, the model learned when the leans allow it.
    TipSample samples[ TIP_CAL_MAX ];
    for ( int k = 0; k < tipCalCount; k++ ) {
        samples[ k ].magnet = tipCalMagnet[ k ];
        samples[ k ].shaft = tipCalShaft[ k ];
        samples[ k ].hole = 0;
    }
    TipModel m;
    TipFitReport report;
    char line[ 240 ];
    if ( !tipModelFit( samples, tipCalCount, &m, &report ) ) {
        snprintf( line, sizeof( line ), "%d sample%s: %s (the samples are cleared; q takes one at each angle)", tipCalCount, tipCalCount == 1 ? "" : "s", report.refused );
        out->println( line );
        tipCalCount = 0;
        return;
    }
    magLocator.learnTipModel( m, report.tipMm );
    snprintf( line, sizeof( line ), "tip %.1f mm from %d angles (%.0f-%.0f deg; the points agree to %.2f mm rms, worst %.2f): %s - in use and saved as tracking/tip", report.tipMm, tipCalCount,
              report.leanMinDeg, report.leanMaxDeg, report.rmsMm, report.worstMm, report.scalar ? report.note : "the lean model, bias and all" );
    out->println( line );
    tipCalCount = 0;
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

static void onCursorMode( Stream* out ) {
    MagTrack& t = magLocator.track;
    t.cursorMode = t.cursorMode == MAGCURSOR_UNDER ? MAGCURSOR_POINTED : MAGCURSOR_UNDER;
    out->println( t.cursorMode == MAGCURSOR_UNDER ? "cursor: straight under the tip" : "cursor: where the tip points, on the board's surface (never further than the reach cap)" );
}

static void onSurface( Stream* out ) {
    long mm = consoleReadNumber( out, "the breadboard's surface is this many mm above the sensors (0 = not known, use the sensor plane), then Enter: ", 8000 );
    if ( mm < 0 || mm > 100 ) {
        out->println( "no number (0-100) - nothing changed" );
        return;
    }
    magLocator.boardZ = (float)mm;
    char line[ 120 ];
    snprintf( line, sizeof( line ), "surface at %ld mm; MAGLOC_BOARD_Z_MM makes it permanent", mm );
    out->println( line );
}

static void onTracker( Stream* out ) {
    MagTrack& t = magLocator.track;
    t.enabled = !t.enabled;
    magTrackReset( &t );
    out->println( t.enabled ? "tracker on: fixes filtered, gated and carried through gaps" : "tracker off: every fix shown as it comes" );
}

// ---- the simulated probe --------------------------------------------------------

void MagLocator::simProbeSet( Vec3 position, Vec3 shaft, float sigmaMm, bool rough, uint32_t ms ) {
    float n = sqrtf( shaft.x * shaft.x + shaft.y * shaft.y + shaft.z * shaft.z );
    if ( n < 1e-6f ) {
        shaft = { 0, 0, 1 };
    } else {
        shaft = { shaft.x / n, shaft.y / n, shaft.z / n };
    }
    if ( shaft.z < 0.0f ) {
        shaft = { -shaft.x, -shaft.y, -shaft.z }; // a probe is not held upside down
    }
    sim.on = true;
    sim.position = position;
    sim.shaft = shaft;
    sim.sigmaMm = sigmaMm > 0.05f ? sigmaMm : 0.05f;
    sim.rough = rough;
    uint32_t until = millis( ) + ms;
    sim.untilMs = ms == 0 ? 0 : ( until == 0 ? 1 : until );
}

void MagLocator::simProbeOff( ) {
    sim.on = false;
    fix.valid = false;
    fix.present = false;
    result.valid = false; // the real fit starts cold
    haveSmoothed = false;
}

// One frame from the simulated probe: `fix` as a fit would leave it, and
// the tracker's input.
ServiceStatus MagLocator::simFrame( MagTrackInput* in ) {
    fix.present = true;
    fix.rough = sim.rough;
    fix.valid = !sim.rough;
    fix.magnet = fix.rawMagnet = sim.position;
    fix.axis = sim.shaft;
    fix.shaft = sim.shaft;
    fix.tiltDeg = acosf( fix.shaft.z > 1.0f ? 1.0f : fix.shaft.z ) * 180.0f / (float)M_PI;
    fix.strength = lastGoodStrength > 0.0f ? lastGoodStrength : 4200.0f;
    fix.residual = 0.0f;
    fix.misfit = 0.02f;
    fix.sigma = { sim.sigmaMm, sim.sigmaMm, sim.sigmaMm };
    fix.errorXyMm = sqrtf( 2.0f ) * sim.sigmaMm;
    fix.errorMm = sqrtf( 3.0f ) * sim.sigmaMm;
    fix.peakMt = 1.0f;
    fix.seenBy = magArray.sensorCount( );
    fix.faintBy = 0;
    fix.fitUs = 0;
    fix.tip = pointOf( fix.magnet, fix.shaft );
    fix.rawTip = fix.tip;
    fix.pointer = pointerOf( fix.tip, fix.shaft );
    fix.rawPointer = fix.pointer;
    fix.count++;
    in->valid = !sim.rough;
    in->rough = sim.rough;
    in->position = sim.position;
    in->sigma = fix.sigma;
    in->alpha = 1.0f;
    in->shaft = sim.shaft;
    in->haveShaft = !sim.rough;
    result.valid = false; // the real fit starts cold when the simulation ends
    haveSmoothed = false;
    return ServiceStatus::BUSY;
}

// :probe <x> <y> <z> [ms] [sigma s] [shaft dx dy dz] [rough]
// :probe row <r> <h> [up mm] [ms] [lean deg]   (the point in that hole)
// :probe off | :probe
static void onProbeVerb( int argc, char** argv, Stream* out ) {
    char line[ 160 ];
    MagLocator::ProbeSim& sim = magLocator.sim;
    if ( argc == 1 ) {
        if ( sim.on ) {
            snprintf( line, sizeof( line ), "probe sim on: magnet %.1f %.1f %.1f shaft %.2f %.2f %.2f sigma %.2f%s%s", sim.position.x, sim.position.y, sim.position.z, sim.shaft.x, sim.shaft.y,
                      sim.shaft.z, sim.sigmaMm, sim.rough ? " rough" : "", sim.untilMs != 0 ? " timed" : "" );
        } else {
            snprintf( line, sizeof( line ), "probe sim off" );
        }
        consoleOk( out, line );
        return;
    }
    if ( strcmp( argv[ 1 ], "off" ) == 0 ) {
        magLocator.simProbeOff( );
        consoleOk( out, "probe sim off" );
        return;
    }
    Vec3 position = { 0, 0, 0 };
    Vec3 shaft = { 0, 0, 1 };
    float sigma = 0.3f;
    bool rough = false;
    uint32_t ms = 0;
    int next;
    if ( strcmp( argv[ 1 ], "row" ) == 0 ) {
#if MODULE_ROW_COUNT
        if ( argc < 4 ) {
            consoleErr( out, "usage: :probe row <1-60> <hole 1-6> [up mm] [ms] [lean deg]" );
            return;
        }
        int row = atoi( argv[ 2 ] ), hole = atoi( argv[ 3 ] );
        if ( row < 1 || row > 2 * ROWGRID_ROWS_PER_HALF || hole < 1 || hole > 6 ) {
            consoleErr( out, "row 1-60, hole 1-6" );
            return;
        }
        float up = argc >= 5 ? atof( argv[ 4 ] ) : 0.0f;
        ms = argc >= 6 ? (uint32_t)atol( argv[ 5 ] ) : 0;
        float lean = argc >= 7 ? atof( argv[ 6 ] ) : 0.0f;
        RowPlace place = rowGridHolePlace( row, hole );
        Vec3 tip = rowGridToBoard( &rowCounter.grid, place.along, place.acrossMm );
        tip.z = magLocator.boardZ + up;
        float leanRad = lean * (float)M_PI / 180.0f;
        shaft = { sinf( leanRad ), 0.0f, cosf( leanRad ) };
        position = { tip.x + magLocator.tipOffsetMm * shaft.x, tip.y + magLocator.tipOffsetMm * shaft.y, tip.z + magLocator.tipOffsetMm * shaft.z };
        next = argc; // nothing more to parse
#else
        consoleErr( out, "no row counter in this build" );
        return;
#endif
    } else {
        if ( argc < 4 ) {
            consoleErr( out, "usage: :probe <x> <y> <z> [ms] [sigma s] [shaft dx dy dz] [rough] | row <r> <h> [up mm] [ms] | off" );
            return;
        }
        position = { (float)atof( argv[ 1 ] ), (float)atof( argv[ 2 ] ), (float)atof( argv[ 3 ] ) };
        next = 4;
        if ( next < argc && ( ( argv[ next ][ 0 ] >= '0' && argv[ next ][ 0 ] <= '9' ) ) ) {
            ms = (uint32_t)atol( argv[ next++ ] );
        }
    }
    while ( next < argc ) {
        if ( strcmp( argv[ next ], "sigma" ) == 0 && next + 1 < argc ) {
            sigma = atof( argv[ next + 1 ] );
            next += 2;
        } else if ( strcmp( argv[ next ], "shaft" ) == 0 && next + 3 < argc ) {
            shaft = { (float)atof( argv[ next + 1 ] ), (float)atof( argv[ next + 2 ] ), (float)atof( argv[ next + 3 ] ) };
            next += 4;
        } else if ( strcmp( argv[ next ], "rough" ) == 0 ) {
            rough = true;
            next++;
        } else {
            snprintf( line, sizeof( line ), "did not understand '%s' (sigma <s>, shaft <dx dy dz>, rough)", argv[ next ] );
            consoleErr( out, line );
            return;
        }
    }
    magLocator.simProbeSet( position, shaft, sigma, rough, ms );
    snprintf( line, sizeof( line ), "probe sim: magnet %.1f %.1f %.1f shaft %.2f %.2f %.2f sigma %.2f%s%s", magLocator.sim.position.x, magLocator.sim.position.y, magLocator.sim.position.z,
              magLocator.sim.shaft.x, magLocator.sim.shaft.y, magLocator.sim.shaft.z, magLocator.sim.sigmaMm, rough ? " rough" : "", ms != 0 ? " timed" : "" );
    consoleOk( out, line );
}

static void onAuditVerb( int argc, char** argv, Stream* out ); // below, with the audit

void MagLocator::begin( ) {
    magTrackInit( &track, boardZ, tipOffsetMm );
    surfaceMapClear( &surfaceMap );
    tipModelClear( &tipModel );
    consoleAddVerb( "probe", "<x> <y> <z> [ms] [sigma s] [shaft dx dy dz] [rough] | row <r> <h> [up mm] [ms] [lean deg] | off", "a simulated probe fed to the tracker in place of the fit", CONSOLE_CHANGES,
                    onProbeVerb );
    consoleAddCommand( 'd', "stream probe fixes as CSV (toggle)", onStream );
    consoleAddCommand( 'l', "latest probe fix", onLatest );
    consoleAddCommand( 'o', "orientation check: hold a magnet 1-2 cm over the array first", onOrientation );
    consoleAddCommand( 'k', "the magnet's strength (learned from the near fixes, held): hold it and measure it again from scratch (:strength <n> holds a known one)", onLearn );
    consoleAddVerb( "strength", "<mT*mm^3> | learn | free", "the magnet's strength: a known one held as measured; learn = measure it again (k); free = never hold it (K)", CONSOLE_CHANGES, onStrengthVerb );
    consoleAddCommand( 'K', "never hold the magnet's strength (fit it freely; the far probe is then out of reach)", onForget );
    consoleAddCommand( 't', "the magnet's centre is <number> mm up the shaft from the probe's point (t12<Enter>)", onTipOffset );
    consoleAddCommand( 'T', "the magnet's angle to the shaft, <number> degrees: T0 = along it, T90 = a disc lying flat on it", onMagnetAngle );
    consoleAddCommand( 'u', "cursor: under the tip / where the tip points (toggle)", onCursorMode );
    consoleAddCommand( 'S', "the breadboard's surface is <number> mm above the sensors (S17<Enter>)", onSurface );
    consoleAddCommand( 'g', "tracker on/off (off = raw fixes, for comparing)", onTracker );
    consoleAddCommand( 'Y', "the probe lay where it last was while the array zeroed: take that magnet back out of the baseline (<number>: 0 = last fix, 1 = the reference)", onUnpollute );
    consoleAddVerb( "audit", "[reset | forget]", "each sensor's zero and gain against the fit of the others (the zero audit); reset = the samples cleared; forget = the gain trims back to 1 as well", CONSOLE_READS, onAuditVerb );
    consoleAddCommand( 'q', "tip calibration: the point resting in one hole, take the fix at this angle (then another angle, q again...)", onTipSample );
    consoleAddCommand( 'Q', "tip calibration: solve the magnet-to-point distance from the q samples (3 or more angles) and use it", onTipSolve );
}

Vec3 MagLocator::pointerOf( Vec3 tip, Vec3 shaft ) const {
    float reach = 0.0f;
    return magTrackPointer( boardZ, track.maxReachMm, tip, shaft, &reach ); // the one geometry, the tracker's cursor included (the menu's reach; the define until 2026-09-25, so the raw pointer ignored it)
}

// Does the new baseline have a magnet in it? See MAGLOC_BASELINE_* in the header.
// The boot zeroed with the magnet on the board, and the magnet has not moved
// since the last good fix: its field at each sensor, from that fix, comes
// out of the baseline. (Sensor offsets are random and drown a weak magnet in
// the polluted-baseline check, so the check cannot always tell.)
void MagLocator::unpolluteBaseline( Stream* out ) {
    if ( !haveLastGood ) {
        out->println( "no last good fix to take out of the baseline" );
        return;
    }
    if ( magArray.baselineRestored || baselineCorrected ) {
        // Only a zero taken fresh (boot, or z) with the magnet in place has
        // the magnet in it once; a saved one, or one already corrected, has
        // not, and taking the magnet out again would put a phantom in.
        out->println( "the baseline in use is a saved or already corrected one, not a fresh zero: press z first (with the probe where it was), then Y" );
        return;
    }
    baselineCorrected = true;
    Vec3 moment = { lastGoodAxis.x * lastGoodStrength, lastGoodAxis.y * lastGoodStrength, lastGoodAxis.z * lastGoodStrength };
    float biggest = 0.0f;
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        Vec3 b = magFitDipoleField( magArray.position[ i ], lastGoodPosition, moment );
        magArray.shiftBaseline( i, b );
        float m = sqrtf( b.x * b.x + b.y * b.y + b.z * b.z );
        if ( m > biggest )
            biggest = m;
    }
    checkedBaseline = 0; // the pollution check runs again on the corrected one
    char line[ 200 ];
    snprintf( line, sizeof( line ), "baseline corrected: the magnet last fitted at x %.1f y %.1f z %.1f (%.0f mT*mm^3, up to %.3f mT at a sensor) taken out; the fit should find it now",
              lastGoodPosition.x, lastGoodPosition.y, lastGoodPosition.z, lastGoodStrength, biggest );
    out->println( line );
}

bool MagLocator::magnetExplains( const Vec3* fields, const bool* use, int count, MagFitResult* out ) const {
    Vec3 mean = { 0, 0, 0 };
    int used = 0;
    for ( int i = 0; i < count; i++ ) {
        if ( !use[ i ] )
            continue;
        mean = { mean.x + fields[ i ].x, mean.y + fields[ i ].y, mean.z + fields[ i ].z };
        used++;
    }
    if ( used < 4 ) {
        *out = MagFitResult( );
        return false;
    }
    mean = { mean.x / used, mean.y / used, mean.z / used }; // the same at every sensor (the earth's field, a common drift) is not a dipole's
    Vec3 residue[ MAGFIT_MAX_SENSORS ];
    for ( int i = 0; i < count && i < MAGFIT_MAX_SENSORS; i++ ) {
        residue[ i ] = { fields[ i ].x - mean.x, fields[ i ].y - mean.y, fields[ i ].z - mean.z };
    }
    MagFitResult r = { };
    magFitSolve( magArray.position, residue, use, count, 1e6f, &r );
    *out = r;
    float misfit = r.signal > 0.0f ? r.residual / r.signal : 1.0f;
    return misfit < MAGLOC_BASELINE_MAGNET_MISFIT && r.strength > MAGLOC_BASELINE_MAGNET_STRENGTH && r.position.z > 0.0f;
}

// The zero put back at boot, against the live readings (see the header).
void MagLocator::adoptBootZero( ) {
    int count = magArray.sensorCount( );
    Vec3 mean[ MAGFIT_MAX_SENSORS ], diff[ MAGFIT_MAX_SENSORS ];
    bool use[ MAGFIT_MAX_SENSORS ];
    float largest = 0.0f;
    int largestAt = -1;
    for ( int i = 0; i < count && i < MAGFIT_MAX_SENSORS; i++ ) {
        use[ i ] = bootFrames[ i ] >= MAG_BASELINE_FRAMES / 2 && magArray.sensor( i ).ok && !magArray.ignored( i );
        mean[ i ] = bootFrames[ i ] > 0 ? Vec3 { bootSum[ i ].x / bootFrames[ i ], bootSum[ i ].y / bootFrames[ i ], bootSum[ i ].z / bootFrames[ i ] } : Vec3 { 0, 0, 0 };
        Vec3 b = magArray.baselineOf( i );
        diff[ i ] = { mean[ i ].x - b.x, mean[ i ].y - b.y, mean[ i ].z - b.z };
        float size = sqrtf( diff[ i ].x * diff[ i ].x + diff[ i ].y * diff[ i ].y + diff[ i ].z * diff[ i ].z );
        if ( use[ i ] && size > largest ) {
            largest = size;
            largestAt = i;
        }
    }
    MagFitResult r;
    bool magnet = magnetExplains( diff, use, count, &r );
    Stream* out = console.port( );
    char line[ 220 ];
    if ( magnet ) {
        snprintf( line, sizeof( line ), "zero: a magnet was near at boot (about %.0f mT*mm^3 at x %.0f y %.0f z %.0f): the saved zero is kept, and doubted by %.3f mT until the probe is away",
                  r.strength, r.position.x, r.position.y, r.position.z, MAG_ZERO_UNCERTAINTY_TMAG_MT );
    } else {
        for ( int i = 0; i < count && i < MAGFIT_MAX_SENSORS; i++ ) {
            if ( use[ i ] )
                magArray.setBaseline( i, mean[ i ] ); // the live baseline; the saved zero is still z's and the audit's
        }
        snprintf( line, sizeof( line ), "zero: the live readings adopted as the zero at boot - the saved zero was off by up to %.3f mT (sensor %d); no dipole in the difference", largest, largestAt );
    }
    if ( out != nullptr )
        out->println( line );
}

void MagLocator::checkBaseline( ) {
    checkedBaseline = magArray.baselineCount;
    baselineCorrected = false; // a zero just taken: Y may be applied to it once
    if ( magArray.baselineRestored ) {
        baselinePolluted = false; // a saved zero, checked when it was taken; never retaken behind the user's back...
        baselineRetakes = 0;
        retakeBaselineAtMs = 0;
        // ...but checked against the live readings once they are in (adoptBootZero).
        bootAdopting = true;
        bootAdoptFrames = 0;
        for ( int i = 0; i < MAGFIT_MAX_SENSORS; i++ ) {
            bootSum[ i ] = { 0, 0, 0 };
            bootFrames[ i ] = 0;
        }
        return;
    }
    int count = magArray.sensorCount( );
    Vec3 baseline[ MAGFIT_MAX_SENSORS ];
    bool use[ MAGFIT_MAX_SENSORS ];
    for ( int i = 0; i < count && i < MAGFIT_MAX_SENSORS; i++ ) {
        baseline[ i ] = magArray.baselineOf( i );
        use[ i ] = magArray.sensor( i ).ok;
    }
    MagFitResult r;
    baselinePolluted = magnetExplains( baseline, use, count, &r );
    baselineMagnetMisfit = r.signal > 0.0f ? r.residual / r.signal : 1.0f;
    baselineMagnetStrength = r.strength;

    Stream* out = console.port( );
    if ( baselinePolluted ) {
        if ( out != nullptr ) {
            char line[ 200 ];
            snprintf( line, sizeof( line ), "BASELINE POLLUTED: a magnet (about %.0f mT*mm^3 at x %.0f y %.0f z %.0f, misfit %.0f %%) was over the array while it zeroed. Move it away%s",
                      r.strength, r.position.x, r.position.y, r.position.z, baselineMagnetMisfit * 100.0f,
                      baselineRetakes < MAGLOC_BASELINE_RETAKES ? " - zeroing again in 2 s." : " and press z." );
            out->println( line );
        }
        if ( baselineRetakes < MAGLOC_BASELINE_RETAKES ) {
            baselineRetakes++;
            retakeBaselineAtMs = millis( ) + MAGLOC_BASELINE_RETAKE_MS;
        }
    } else {
        baselineRetakes = 0;
        retakeBaselineAtMs = 0; // a clean one is kept, whatever was pending
    }
}

ServiceStatus MagLocator::service( ) {
    if ( magArray.baselineReady( ) && magArray.baselineCount != checkedBaseline ) {
        checkBaseline( );
    }
    if ( retakeBaselineAtMs != 0 && millis( ) >= retakeBaselineAtMs ) {
        retakeBaselineAtMs = 0;
        magArray.startBaseline( );
    }
    if ( magArray.frameCount == lastFrame || !magArray.baselineReady( ) ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    lastFrame = magArray.frameCount;
    if ( bootAdopting ) {
        int done = 0, wanted = 0;
        for ( int i = 0; i < magArray.sensorCount( ) && i < MAGFIT_MAX_SENSORS; i++ ) {
            if ( magArray.fresh[ i ] && bootFrames[ i ] < MAG_BASELINE_FRAMES ) {
                bootSum[ i ] = { bootSum[ i ].x + magArray.raw[ i ].x, bootSum[ i ].y + magArray.raw[ i ].y, bootSum[ i ].z + magArray.raw[ i ].z };
                bootFrames[ i ]++;
            }
            if ( magArray.sensor( i ).ok && !magArray.ignored( i ) ) {
                wanted++;
                done += bootFrames[ i ] >= MAG_BASELINE_FRAMES ? 1 : 0;
            }
        }
        bootAdoptFrames++;
        if ( done == wanted || bootAdoptFrames > 2 * MAG_BASELINE_FRAMES ) { // every sensor has its frames, or some never will (absent): judge what there is
            bootAdopting = false;
            adoptBootZero( );
        }
    }

    // One frame: the fit (or not), then the tracker, which is told about every
    // frame - the ones with no fix are what it coasts through.
    uint32_t nowUs = micros( );
    float dtS = lastTrackUs == 0 ? 0.01f : ( nowUs - lastTrackUs ) * 1e-6f;
    lastTrackUs = nowUs;
    MagTrackInput in = { };
    in.alpha = 1.0f;
    if ( sim.on && sim.untilMs != 0 && (int32_t)( millis( ) - sim.untilMs ) >= 0 ) {
        simProbeOff( );
    }
    if ( fitHeld || bootAdopting ) {
        // Held (:load fit off), or the first MAG_BASELINE_FRAMES after boot
        // while the zero put back is judged against the readings (adoptBootZero):
        // nothing is present and nothing is fitted until the zero is settled.
        fix.valid = false;
        fix.present = false;
        result.valid = false;
        lastStatus = ServiceStatus::IDLE;
    } else {
        lastStatus = sim.on ? simFrame( &in ) : fitFrame( &in );
    }
    keepStrengthRecord( millis( ) );
    // The surface under the point (the map, 2026-09-28): where the track
    // had the point last frame, else this frame's fix, else the centre.
    if ( track.state != MAGTRACK_NONE )
        track.surfaceZ = surfaceAt( track.tip.x, track.tip.y );
    else if ( fix.valid )
        track.surfaceZ = surfaceAt( fix.rawTip.x, fix.rawTip.y );
    else
        track.surfaceZ = boardZ;
    if ( tipModel.valid && fabsf( tipOffsetMm - tipModelD ) > 0.01f )
        forgetTipModel( ); // "tip" edited by hand: the plain scalar is what was asked for
    track.tipOffsetMm = tipOffsetMm;
    track.tipModel = tipModel;
    magTrackUpdate( &track, dtS, &in );
    return lastStatus;
}

// The dipole fit on this frame's readings, filling in `fix` and what the
// tracker needs to know about the frame.
ServiceStatus MagLocator::fitFrame( MagTrackInput* in ) {
    // The frame, in stages: filter and smooth the readings, assess what is
    // there, keep the zeros, then fit and publish (2026-09-21 night: one
    // 450-line function before, the stages below unchanged in what they do).
    FrameScratch f;
    if ( !filterFrame( f ) ) {
        return ServiceStatus::IDLE;
    }
    assessFrame( f );
    in->alpha = f.alpha;
    smoothAlpha = f.alpha;
    // The weights for this frame's fit: each sensor's by what it reads
    // (MagArray::frameWeights - the MMC56x3 counts for ~37 TMAGs where it
    // reads noise, for one close in).
    magArray.frameWeights( smooth, f.weights );
    // The Z axis counts by its quietness, as far as the noise is the error: near
    // the magnet the error is the model's share of the field (MAG_MODEL_ERROR:
    // the table's gains and places, the dipole approximation), on every axis
    // alike, and a Z weighed by its noise alone amplified those (2026-09-23,
    // the bench-like sim: the middle rows a hole off across). So the weight is
    // the X/Y error over the Z error at this frame's strongest reading: about
    // 1 at writing height (0.3-0.7 mT), 1.9 in a far hover (0.04 mT).
    {
        float model = MAG_MODEL_ERROR * fix.peakMt;
        float wz = sqrtf( MAG_WEIGHT_REFERENCE_MT * MAG_WEIGHT_REFERENCE_MT + model * model ) / sqrtf( MAG_NOISE_Z_MT * MAG_NOISE_Z_MT + model * model );
        magFitSetAxisWeights( 1.0f, 1.0f, wz );
    }
    // With the strength held and a direction to lean on, one sensor that
    // sees the magnet plainly is enough to try a fit (the far probe, seen
    // by the MMC alone); else it takes MAGLOC_MIN_SENSORS noticing it.
    f.held = knownStrength;
    // (One plainly-seeing sensor is enough only with the MMC56x3 in: it is
    // the sensor that sees a far probe alone. Without it, one TMAG5273 above
    // the seen level is its stale zero after a boot, and this clause would
    // run the lattice on it every 400 ms until the absorb took it.)
    f.enough = fix.seenBy + fix.faintBy >= MAGLOC_MIN_SENSORS || ( f.held > 0.0f && magArray.useMmc && fix.seenBy >= 1 );
    ServiceStatus status;
    if ( !fix.present || !f.enough ) {
        if ( result.valid ) {
            if ( !fix.present )
                coldWhyAbsent++;
            else
                coldWhyFew++;
        }
        // Nothing to fit: the track (if any) coasts, and the fit starts from
        // it when there is something again - afresh: what ran before is not
        // held against it.
        fix.valid = false;
        result.valid = false;
        haveSmoothed = false;
        missRun = 0;
        coldRejectedRun = 0;
        status = ServiceStatus::IDLE;
    } else {
        status = runFit( in, f );
    }
    keepZeros( f );
    return status;
}

// Stage 1: the glitch filter and which sensors may vote; the smoothing
// strength for this frame. false when no sensor was read at all.
bool MagLocator::filterFrame( FrameScratch& f ) {
    // Glitch filter: each axis of each sensor goes through a median of its last
    // three frames. The bus has no checksum, and one corrupted read would
    // otherwise sit in the smoothing below for ten frames, none of which fit.
    // It costs one frame (10 ms) of delay and passes any real movement.
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        const Vec3& b = magArray.field[ i ];
        // A sensor back after missing a few frames has stale history: its
        // median and smoothing start again from this reading.
        f.resync[ i ] = magArray.fresh[ i ] && missedFrames[ i ] >= 3;
        if ( f.resync[ i ] ) {
            recent[ i ][ 0 ] = recent[ i ][ 1 ] = b;
        }
        f.filtered[ i ] = { middleOf( b.x, recent[ i ][ 0 ].x, recent[ i ][ 1 ].x ), middleOf( b.y, recent[ i ][ 0 ].y, recent[ i ][ 1 ].y ), middleOf( b.z, recent[ i ][ 0 ].z, recent[ i ][ 1 ].z ) };
        if ( magArray.fresh[ i ] ) {
            recent[ i ][ 1 ] = recent[ i ][ 0 ];
            recent[ i ][ 0 ] = b;
            missedFrames[ i ] = 0;
        } else if ( missedFrames[ i ] < 1000 ) {
            missedFrames[ i ]++;
            staleFrames[ i ]++;
        }
    }

    // Which sensors the fit may use: fresh, calibrated, their zero settled
    // (MagArray::usedInFit). The others are still read, shown and counted
    // as seeing the magnet; they just do not vote.
    for ( int i = 0; i < magArray.sensorCount( ) && i < MAGFIT_MAX_SENSORS; i++ ) {
        f.use[ i ] = magArray.usedInFit( i );
    }

    // How strong is the strongest reading? That sets the smoothing.
    float rawPeakSq = 0.0f;
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        const Vec3& b = f.filtered[ i ];
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
        fix.valid = false; // nothing was read: the last fix is not this frame's
        return false;
    }

    f.alpha = sqrtf( rawPeakSq ) / MAGLOC_FAST_MT;
    if ( f.alpha < MAGLOC_SLOWEST_ALPHA )
        f.alpha = MAGLOC_SLOWEST_ALPHA;
    // A moving probe gets less smoothing (see MAGLOC_ALPHA_PER_MM_S). The
    // speed is how far the track has come over the last MAGLOC_SPEED_WINDOW
    // frames (see the define: the filter's own velocity is jitter at rest).
    if ( track.enabled && ( track.state == MAGTRACK_TRACKING || track.state == MAGTRACK_COASTING ) ) {
        float dtS = MAG_FRAME_PERIOD_US * 1e-6f;
        float jitter = speedJitterK * ( fix.sigma.x + fix.sigma.y + fix.sigma.z ) / 3.0f / dtS; // what jitter makes of the one-frame velocity
        float velocity = sqrtf( track.velocity.x * track.velocity.x + track.velocity.y * track.velocity.y + track.velocity.z * track.velocity.z );
        float windowed = 0.0f;
        if ( speedRingCount >= MAGLOC_SPEED_WINDOW ) {
            const Vec3& then = speedRing[ speedRingCount % MAGLOC_SPEED_WINDOW ]; // the oldest: MAGLOC_SPEED_WINDOW frames ago
            float dx = track.position.x - then.x, dy = track.position.y - then.y, dz = track.position.z - then.z;
            windowed = sqrtf( dx * dx + dy * dy + dz * dz ) / ( MAGLOC_SPEED_WINDOW * dtS ) - jitter / MAGLOC_SPEED_WINDOW;
        }
        speedRing[ speedRingCount % MAGLOC_SPEED_WINDOW ] = track.position;
        speedRingCount++;
        smoothSpeed = windowed > velocity - jitter ? windowed : velocity - jitter;
        if ( smoothSpeed < 0.0f )
            smoothSpeed = 0.0f;
        float moving = smoothSpeed * MAGLOC_ALPHA_PER_MM_S;
        if ( moving > f.alpha )
            f.alpha = moving;
    } else {
        smoothSpeed = 0.0f;
        speedRingCount = 0;
    }
    if ( f.alpha > 1.0f )
        f.alpha = 1.0f;

    return true;
}

// Stage 2: the smoothed fields, each sensor's level (seen / faint / quiet,
// plainly), its own clock for the absorb rule, and presence.
float MagLocator::levelScale( ) const {
    if ( knownStrength <= 0.0f ) {
        return 1.0f;
    }
    float scale = knownStrength / MAGLOC_THRESHOLDS_TUNED_AT;
    return scale < 0.25f ? 0.25f : ( scale > 1.0f ? 1.0f : scale );
}

float MagLocator::seenLevelMt( ) const {
    float level = MAGLOC_SEEN_MT * levelScale( );
    return level < MAGLOC_SEEN_FLOOR_MT ? MAGLOC_SEEN_FLOOR_MT : level;
}

void MagLocator::assessFrame( FrameScratch& f ) {
    float seenMt = seenLevelMt( ); // the seen level for the magnet held (MAGLOC_THRESHOLDS_TUNED_AT)
    float peakSq = 0.0f, peakLevelSq = 0.0f;
    fix.seenBy = 0;
    fix.faintBy = 0;
    bool haveMmc = false, mmcPresent = false;
    int tmagPlain = 0; // TMAGs reading plainly (the unscaled level, or the presence lever below it): what corroborates presence
    float corroborateMt = presentMt < MAGLOC_SEEN_MT ? presentMt : MAGLOC_SEEN_MT;
    int tmagPlainLow = 0; // ...and at three quarters of it: what keeps presence on once it is (a zero followed out does not flicker it on its way down)
    int tmagSeen = 0;
    bool peakHeld = false;
    f.plain = 0;
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        f.quiet[ i ] = true;
        if ( magArray.ignored( i ) ) {
            f.quiet[ i ] = false; // the MMC56x3 switched out (sensors / use MMC): not here at all - and not "quiet" either, or its zero followed the probe's field (0.29 mT in two seconds, 2026-09-24)
            continue;
        }
        // Not read this frame: its last smoothed reading still says what is
        // there for a frame or two (MAGLOC_MISSED_HOLD_FRAMES) - presence
        // and the counts, not the fit (usedInFit wants it fresh).
        bool held = !magArray.fresh[ i ];
        if ( held && missedFrames[ i ] > MAGLOC_MISSED_HOLD_FRAMES ) {
            continue;
        }
        Vec3& b = smooth[ i ];
        if ( !held ) {
            float k = f.resync[ i ] ? 1.0f : f.alpha;
            b.x += k * ( f.filtered[ i ].x - b.x );
            b.y += k * ( f.filtered[ i ].y - b.y );
            b.z += k * ( f.filtered[ i ].z - b.z );
        }
        float sq = b.x * b.x + b.y * b.y + b.z * b.z;
        if ( sq > peakSq ) {
            peakSq = sq;
        }
        // A quiet type's levels are lower by its noise ratio (MAGLOC_TYPE_SCALE_FLOOR).
        float scale = magArray.noiseMt[ i ] / MAG_WEIGHT_REFERENCE_MT;
        if ( scale < MAGLOC_TYPE_SCALE_FLOOR ) {
            scale = MAGLOC_TYPE_SCALE_FLOOR;
        }
        float levelSq = sq / ( scale * scale );
        if ( levelSq > peakLevelSq ) {
            peakLevelSq = levelSq;
            peakHeld = held;
        }
        // Plainly: for the MMC56x3 twice its seen level (a probe within 80 mm,
        // which fits); for a TMAG5273 its seen level is already three times
        // its noise, and one there with nothing fitting is its zero (after a
        // boot the restored zeros are 0.02-0.04 mT stale, 2026-09-22).
        float plainLevel = magSensorPlaces[ i ].type == MAG_MMC56X3 ? MAGLOC_SEEN_ABSORB_LEVEL : MAGLOC_SEEN_MT; // plainly is the HIGH bar, not scaled: the strength is learned and the absorb judged only on strong readings (scaled, the learning fired at the array's end where the calibration errors bias it most, 2026-09-23)
        if ( levelSq > plainLevel * plainLevel ) {
            f.plain++;
        }
        if ( magSensorPlaces[ i ].type != MAG_MMC56X3 ) {
            if ( levelSq > corroborateMt * corroborateMt )
                tmagPlain++;
            if ( levelSq > 0.75f * 0.75f * corroborateMt * corroborateMt )
                tmagPlainLow++;
        }
        if ( magSensorPlaces[ i ].type == MAG_MMC56X3 ) {
            haveMmc = true;
            if ( levelSq > presentMt * presentMt )
                mmcPresent = true;
        }
        if ( levelSq > seenMt * seenMt ) {
            fix.seenBy++;
            if ( magSensorPlaces[ i ].type != MAG_MMC56X3 )
                tmagSeen++;
        } else if ( levelSq > MAGLOC_FAINT_MT * MAGLOC_FAINT_MT ) {
            fix.faintBy++;
        }
        f.quiet[ i ] = levelSq < MAGLOC_QUIET_FRACTION * MAGLOC_QUIET_FRACTION * MAGLOC_FAINT_MT * MAGLOC_FAINT_MT;
    }
    fix.peakMt = sqrtf( peakSq );
    fix.peakLevel = sqrtf( peakLevelSq );
    if ( fix.peakLevel > maxPeakLevel ) {
        maxPeakLevel = fix.peakLevel;
    }
    // Stickiness, so a magnet at the threshold does not flicker in and out:
    // once present it stays so down to half the level.
    bool fixedLately = lastFixMs != 0 && millis( ) - lastFixMs < MAGLOC_PRESENT_HOLD_MS;
    // ...and, once present, down to three quarters of the level with no fix
    // lately: a zero followed out crossed the bar back and forth on its way
    // down (six toggles in a stale-zero block, 2026-09-24).
    float level = fix.present ? ( fixedLately ? MAGLOC_PRESENT_LOW : 0.75f ) * presentMt : presentMt;
    bool evidence = fix.peakLevel > level;
    if ( evidence && !fixedLately ) {
        // The MMC's word, or two TMAGs' (MAGLOC_PRESENT_TMAGS): one TMAG
        // alone is its zero (with the MMC out too: a probe near enough to
        // light one TMAG plainly lights its neighbour, 2026-09-22 late).
        // ...reading PLAINLY (0.04), not merely above the scaled seen level: with
        // the level scaled for a weak magnet (0.022) two post-boot stale zeros
        // (0.03-0.045) flapped presence twelve times in half a minute with
        // nothing there (2026-09-23, the review's scene, now scenes/edge.txt).
        // ...and, once present, it stays while ONE TMAG still reads plainly:
        // two stale zeros at the plain level flickered presence at the noise
        // (thirty toggles in forty seconds, 2026-09-24) until they were
        // followed out; a probe leaving takes its last plain sensor with it.
        evidence = ( haveMmc && mmcPresent ) || tmagPlain >= MAGLOC_PRESENT_TMAGS || ( fix.present && tmagPlainLow >= 1 );
    }
    if ( evidence != fix.present ) {
        presenceToggles++;
    }
    fix.present = evidence;
    if ( fix.present ) {
        uint32_t now = millis( );
        lastPresentMs = now == 0 ? 1 : now;
    }
    if ( fix.present && peakHeld ) {
        presenceHeldFrames++;
    }
}

// Stage 3: the zeros. Each sensor's zero is a state with a doubt
// (MagOffsetFilter): the doubt grows every frame; the filter FOLLOWS the
// reading as measured (MAG_OFFSET_FOLLOW_S) whenever the reading is not the
// magnet's - nothing present (and none lately), or this sensor quiet; it HOLDS
// under a fix that two or more sensors make; and a reading no fix explains
// is held for MAG_OFFSET_HOLD_S on this sensor's own clock, then followed
// (a zero error, a screwdriver, the MMC's tail of a zero error that IS a
// probe 90 mm up to three numbers with the strength held - or a steady
// probe at the edge of reach, which goes after the hold as it did). Four
// clocks, two rates, a steadiness
// test and the plain/faint distinction did this until 2026-09-23; what
// they could not express (the doubt, and a restored zero's) the filter
// does. The live baseline is set from it every frame; the saved zero
// (zeroed[]) is still only z's and the audit's. A baseline moved by anyone
// else (z, Y, the audit, a restore) restarts the filter from it. Runs after
// the fit: it needs this frame's verdict.
void MagLocator::keepZeros( const FrameScratch& f ) {
    float dtS = MAG_FRAME_PERIOD_US * 1e-6f;
    uint32_t now = millis( );
    for ( int i = 0; i < magArray.sensorCount( ) && i < MAGFIT_MAX_SENSORS; i++ ) {
        MagSensorType type = magSensorPlaces[ i ].type;
        float n = magArray.noiseMt[ i ];
        Vec3 noise = { n, n, n }; // the type's noise on every axis: the follow rate is a design time, the same on Z (its quieter noise would halve it)
        Vec3 b = magArray.baselineOf( i );
        if ( !offsetsBegun || b.x != offsetWritten[ i ].x || b.y != offsetWritten[ i ].y || b.z != offsetWritten[ i ].z ) {
            // A zero put back at boot is doubted by the boot uncertainty; one
            // z, Y or the audit just wrote is as good as the floor.
            zeroDoubt[ i ] = offsetsBegun ? magSensorTypeZeroFloorMt( type ) : magSensorTypeZeroMt( type );
            // The walk that follows a reading of nothing with the follow time
            // constant (the steady gain is sqrt(q/R) a frame: q = (noise dt / tau)^2),
            // and the doubt it settles to (sqrt(q R)) as the doubt to start from:
            // larger, the filter would follow at once for its first seconds.
            float walk = n * sqrtf( dtS ) / MAG_OFFSET_FOLLOW_S;
            float sigma = sqrtf( n * walk * sqrtf( dtS ) );
            magOffsetInit( &offsets[ i ], b, sigma, walk );
            offsetWritten[ i ] = b;
            unexplainedSinceMs[ i ] = 0;
        }
        if ( !magArray.fresh[ i ] ) {
            zeroDoubt[ i ] += MAG_ZERO_DRIFT_MT_PER_S * dtS; // not read this frame: nothing to say (its clock, like its history, is left alone); the doubt drifts on
            continue;
        }
        bool follow;
        bool presentLately = lastPresentMs != 0 && now - lastPresentMs < (uint32_t)( MAG_OFFSET_HOLDOFF_S * 1000.0f );
        bool explained = fix.present && f.explained;
        if ( ( f.quiet[ i ] && !explained ) || ( !fix.present && !presentLately ) ) {
            // Not the magnet's reading: the drift. (A quiet sensor under a fix
            // the array explains holds too: its small share of the magnet is
            // real, and followed it went into its zero and was put back by
            // the audit every window, a flash write each - 2026-09-24.)
            follow = true;
            unexplainedSinceMs[ i ] = 0;
        } else if ( magArray.ignored( i ) || explained ) {
            follow = false; // a magnet, not drift (a switched-out sensor: read and shown, its zero follows the drift with nothing there and holds otherwise)
            unexplainedSinceMs[ i ] = 0;
        } else if ( fix.present && f.farOnlyFix ) {
            // A fix one sensor makes: held the far hold on this sensor's clock
            // (the zero error's tail IS a far probe to three numbers), then followed.
            if ( unexplainedSinceMs[ i ] == 0 )
                unexplainedSinceMs[ i ] = now == 0 ? 1 : now;
            follow = now - unexplainedSinceMs[ i ] > (uint32_t)( MAG_OFFSET_FAR_HOLD_S * 1000.0f );
        } else {
            // Not quiet, with no fix explaining it (or in the hold-off after
            // presence, wavering at the edge): held on its own clock, then
            // followed - the two-minute hold when a fit was at least tried
            // (something the array could explain may be there), the hold-off
            // alone when too few sensors notice anything for a fit to try
            // (two stale zeros after boot flickered presence for 2.5 minutes
            // at 997e74b). (A frame the TMAGs disown is not evidence about
            // the zeros: at 90 mm their say is noise, and following on those
            // frames absorbed a real far hover's share, 2026-09-23.)
            if ( unexplainedSinceMs[ i ] == 0 )
                unexplainedSinceMs[ i ] = now == 0 ? 1 : now;
            // ...and a reading that was an accepted fix within the far hold
            // keeps the far hold: a far fix flickering about the chi's limit
            // is not drift on its refused frames (the MMC's 90 mm hover was
            // followed to 96 mm in ninety seconds under the half-minute hold).
            bool fixLately = lastFixMs != 0 && now - lastFixMs < (uint32_t)( MAG_OFFSET_FAR_HOLD_S * 1000.0f );
            // ...and so does a reading refused only by the footprint rule (a
            // dipole of the held strength the array cannot pin: a probe off
            // the end of the board, one sensor plain - the same as a fix one
            // sensor makes), or one that MAGLOC_OUTSIDE_MIN_SEEN TMAGs read
            // plainly (drift is one sensor's; a field three see plainly is
            // a probe the fit is failing on). Followed at the half-minute
            // hold, an off-the-end hover went into the zeros, the return
            // over row 15 was refused at the chi's limit on the 0.04 mT
            // absorbed, and half a minute later the probe on the board was
            // followed too (hover.txt, 2026-09-24). The half-minute hold is
            // for what is left: a pattern few sensors see and no dipole
            // explains - a stale zero.
            bool probeLike = fixLately || fix.rejectedOutside || f.plain >= MAGLOC_OUTSIDE_MIN_SEEN;
            float holdS = !f.enough ? MAG_OFFSET_HOLDOFF_S : ( probeLike ? MAG_OFFSET_FAR_HOLD_S : MAG_OFFSET_HOLD_S );
            follow = now - unexplainedSinceMs[ i ] > (uint32_t)( holdS * 1000.0f );
        }
        if ( follow ) {
            float floor = magSensorTypeZeroFloorMt( type );
            zeroDoubt[ i ] *= expf( -dtS / MAG_OFFSET_FOLLOW_S ); // followed: what was wrong is going
            if ( zeroDoubt[ i ] < floor )
                zeroDoubt[ i ] = floor;
        } else if ( zeroDoubt[ i ] < MAG_ZERO_DOUBT_MAX_MT ) {
            zeroDoubt[ i ] += MAG_ZERO_DRIFT_MT_PER_S * dtS; // held: the drift goes on unseen - to the cap
        }
        if ( follow ) {
            // The doubt grows only while the zero follows: grown while it is
            // held, it made the first frames of a follow catch up faster, and
            // a sensor just above quiet whose noise dips under it now and then
            // ate its share of a far probe in a cascade - each nibble a
            // smaller reading, a smaller reading quiet more often (the sim,
            // 2026-09-23: 0.7 % of frames quiet at 20 s, 97 % at 90 s).
            magOffsetPredict( &offsets[ i ], dtS );
            magOffsetUpdate( &offsets[ i ], magArray.raw[ i ], noise );
            magArray.setBaseline( i, offsets[ i ].offset );
            offsetWritten[ i ] = magArray.baselineOf( i ); // what was actually written (refused during a zeroing)
        }
    }
    offsetsBegun = true;
}

// Stage 4: the fit itself - a cold start's slices (the lattice) or a warm
// start from where the fit, or the track, last put the magnet; the free fit,
// then the refinement with the strength held; and ONE acceptance: the
// residuals against each sensor's own expected error (magFitChi under
// MAGLOC_FIT_MAX_CHI), the bar under MAGLOC_MAX_ERROR_MM, and a fix beyond
// the array's footprint seen plainly by MAGLOC_OUTSIDE_MIN_SEEN. The track is
// the fit's memory: with a track alive and no answer of its own to start
// from (a missed frame, a glitch, a rejected fit) the fit starts from where
// the track predicts the magnet, and the lattice runs only once the track has
// gone - rejected, not again for MAGLOC_COLD_RETRY_MS. The near and far rules,
// the TMAGs' confirmation, the rough offer, the misses ride-through and the
// reacquire window (2026-09-21 to 22, sixteen knobs) did this until
// 2026-09-23; the tracker's coast is the ride-through now.
ServiceStatus MagLocator::runFit( MagTrackInput* in, FrameScratch& f ) {
    bool wasTracking = result.valid;
    bool continuing = result.coldStage > 0;
    if ( !wasTracking && !continuing && track.enabled && track.state != MAGTRACK_NONE ) {
        float dtS = MAG_FRAME_PERIOD_US * 1e-6f;
        result.position = { track.position.x + track.velocity.x * dtS, track.position.y + track.velocity.y * dtS, track.position.z + track.velocity.z * dtS };
        if ( result.strength <= 0.0f ) {
            float strength = knownStrength > 0.0f ? knownStrength : MAGLOC_MAGNET_STRENGTH;
            Vec3 axis = haveLastGood ? lastGoodAxis : Vec3 { 0, 0, -1 };
            result.moment = { axis.x * strength, axis.y * strength, axis.z * strength };
            result.strength = strength;
        }
        result.valid = true;
        wasTracking = true;
    }
    if ( !wasTracking ) {
        // A cold start's slices: one every MAGLOC_COLD_SLICE_MS, or every
        // frame with the steady load - but a rejected one waits in both.
        uint32_t now = millis( );
        if ( now < nextColdStartMs && ( !steadyFit || now < coldRetryUntilMs ) ) {
            return ServiceStatus::IDLE;
        }
        nextColdStartMs = now + MAGLOC_COLD_SLICE_MS;
        if ( !continuing ) {
            coldStarts++;
            coldStartUs = 0;
            coldSliceMaxUs = 0;
            if ( missRun > 0 ) {
                coldWhyCoasted++; // the fit failed frame after frame until the track coasted out
                missRun = 0;
            }
        }
    }

    uint32_t start = micros( );
    // The free fit, in slices (a cold start's lattice, its refinement, the
    // seeds: one a call) with the held strength as the lattice's hint; then,
    // once it has an answer, the refinement with the strength held and the
    // direction leaning on the last good fix's pole (MAGLOC_AXIS_PRIOR_MT).
    // With MAGLOC_LEARN_MIN_SENSORS reading the magnet plainly the readings
    // pin its strength themselves: no hint that frame, and the free fit's
    // answer MEASURES the magnet (measureStrength; the held strength
    // follows). Fewer, and the held strength is the hint - the lattice's
    // too: far out, without it, a weak magnet near fits as well as the real
    // one far.
    f.hintFree = f.held > 0.0f && f.plain >= MAGLOC_LEARN_MIN_SENSORS;
    f.good = magFitSolveStep( magArray.position, smooth, f.use, magArray.sensorCount( ), MAGLOC_MAX_MISFIT, &result, steadyFit ? (int)( steadyIterations + 0.5f ) : 0, f.weights, f.hintFree ? 0.0f : f.held );
    MagFitResult freeFit = result; // the free fit's answer, kept: if only the refinement with the held strength fails, the next frame starts from it
    bool freeGood = false;
    if ( f.hintFree && f.good && result.coldStage == 0 && result.signal > 0.0f && result.strength > 0.0f ) {
        float freeMisfit = result.residual / result.signal;
        float freeErr = sqrtf( result.sigma.x * result.sigma.x + result.sigma.y * result.sigma.y + result.sigma.z * result.sigma.z );
        if ( freeMisfit < MAGLOC_LEARN_MAX_MISFIT && freeErr < MAGLOC_STRENGTH_MAX_ERROR_MM ) {
            measureStrength( result.strength, result.position );
            f.held = knownStrength; // this frame's refinement holds what was just learned
            freeGood = true;
        }
    }
    if ( f.held > 0.0f && result.coldStage == 0 && result.signal > 0.0f && result.strength > 0.0f ) {
        f.good = magFitRefineKnownStrength( magArray.position, smooth, f.use, magArray.sensorCount( ), MAGLOC_MAX_MISFIT, f.held, &result, f.weights,
                                          haveLastGood ? &lastGoodAxis : nullptr, MAGLOC_AXIS_PRIOR_MT,
                                          steadyFit ? (int)( steadyIterations + 0.5f ) : MAGLOC_REFINE_ITERATIONS );
    }
    fix.fitUs = micros( ) - start;
    if ( !wasTracking ) {
        coldStartUs += fix.fitUs; // a slice of the cold start
        if ( fix.fitUs > coldSliceMaxUs )
            coldSliceMaxUs = fix.fitUs;
    }
    if ( result.coldStage > 0 ) {
        // More of the cold start next frame: nothing to say about this one.
        fix.valid = false;
        return ServiceStatus::BUSY;
    }
    Vec3 position = result.position, sigma = result.sigma;
    float residual = result.residual, signal = result.signal;

    fix.residual = residual;
    fix.misfit = signal > 0.0f ? residual / signal : 1.0f;
    fix.chi = chiSmoothed( result, f.use, f.alpha );
    fix.sigma = sigma;
    fix.errorXyMm = sqrtf( sigma.x * sigma.x + sigma.y * sigma.y );
    fix.errorMm = sqrtf( fix.errorXyMm * fix.errorXyMm + sigma.z * sigma.z );
    // The acceptance. The footprint is the TMAG5273s' bounding box: a fix
    // beyond it by MAGLOC_OUTSIDE_MM is the mirror-basin case unless enough
    // sensors see the magnet plainly.
    Vec3 lo = { 1e9f, 1e9f, 0 }, hi = { -1e9f, -1e9f, 0 };
    for ( int i = 0; i < magArray.sensorCount( ) && i < MAGFIT_MAX_SENSORS; i++ ) {
        if ( magSensorPlaces[ i ].type == MAG_MMC56X3 || !f.use[ i ] )
            continue;
        const Vec3& p = magArray.position[ i ];
        lo.x = p.x < lo.x ? p.x : lo.x;
        lo.y = p.y < lo.y ? p.y : lo.y;
        hi.x = p.x > hi.x ? p.x : hi.x;
        hi.y = p.y > hi.y ? p.y : hi.y;
    }
    f.outside = position.x < lo.x - MAGLOC_OUTSIDE_MM || position.x > hi.x + MAGLOC_OUTSIDE_MM || position.y < lo.y - MAGLOC_OUTSIDE_MM || position.y > hi.y + MAGLOC_OUTSIDE_MM;
    // ...seen PLAINLY (the unscaled 0.04, f.plain: TMAGs): at the scaled seen
    // level a third sensor's noise let the mirror basin through one frame in
    // four (2026-09-24, the review).
    bool solved = f.good;
    fix.rejectedOutside = solved && fix.chi < fitMaxChi && fix.errorMm < MAGLOC_MAX_ERROR_MM && f.outside && f.plain < MAGLOC_OUTSIDE_MIN_SEEN;
    f.good = f.good && fix.chi < fitMaxChi && fix.errorMm < MAGLOC_MAX_ERROR_MM && ( !f.outside || f.plain >= MAGLOC_OUTSIDE_MIN_SEEN );
    if ( !f.good && !wasTracking ) {
        coldWhyRejected++;
        if ( ++coldRejectedRun >= 2 ) {
            // Rejected twice running: not again at once; and after four, five
            // times as long (a phantom that rejects every lattice is a steady
            // V5F load at the slice pace otherwise).
            nextColdStartMs = millis( ) + MAGLOC_COLD_RETRY_MS * ( coldRejectedRun >= 4 ? 5 : 1 );
            coldRetryUntilMs = nextColdStartMs;
        }
    }
    if ( f.good ) {
        coldRejectedRun = 0;
        missRun = 0;
    } else if ( wasTracking ) {
        missFrames++; // ridden through: the track carries on, the fit starts from it next frame
        missRun++;
    }
    fix.valid = f.good;
    f.explained = f.good && fix.seenBy >= 2; // a fix two sensors make holds the zeros; one sensor's (the far regime) holds them for the far hold only - at the array's reach the chi over the others cannot tell a probe from a zero error's tail (a 0.095 mT phantom followed down to a "probe 89 mm up" and froze, 2026-09-24)
    f.farOnlyFix = f.good && !f.explained;
    fix.rough = false;
    fix.rawMagnet = position;
    if ( f.good ) {
        lastFixMs = millis( ) == 0 ? 1 : millis( );
    }
    // The fit's own memory: its answer; without one, the track's (above) -
    // and with no track, a CONVERGED answer the acceptance refused still,
    // for MAGLOC_RAW_WARM_FRAMES (never a solver's unfinished iterate).
    result.valid = f.good || ( solved && !track.enabled && wasTracking && missRun <= MAGLOC_RAW_WARM_FRAMES );
    if ( f.good ) {
        freeWarmRun = 0;
    }
    if ( !f.good && freeGood && ++freeWarmRun <= MAGLOC_STRENGTH_MIN_SAMPLES ) {
        refineRejected++;
        // The readings pinned the magnet's strength and the free fit was good;
        // only the refinement with the held strength failed the chi - the
        // held strength is wrong (a default, a record of another magnet: the
        // scenes' settings reset puts 1839 back under a 4232 magnet). The
        // free fit's answer is the next frame's start, so the strength ring
        // fills at the frame rate (16 frames) and not one free fit per cold
        // start (2026-09-23: ten rejected cold starts and three seconds) - for
        // as many frames as the ring needs; past that the held strength is
        // not the trouble, and the track's start or a cold start (the seeds)
        // gets its turn.
        result = freeFit;
        result.valid = true;
    }

    if ( !fix.valid ) {
        // Streamed too (valid = 0), so that a spot where nothing fits can be studied.
        Stream* out = console.port( );
        if ( streaming && out != nullptr && ++streamTick % STREAM_EVERY_N_FIXES == 0 ) {
            fix.magnet = fix.rawMagnet;
            printFixCsv( out );
        }
        haveSmoothed = false;
        return ServiceStatus::BUSY;
    }

    return publishFix( in, f );
}

// Stage 5: a fix - smoothed for showing, the shaft from the pole, the tip and
// the pointer, what the settings remember, the tracker's input, the stream.
ServiceStatus MagLocator::publishFix( MagTrackInput* in, const FrameScratch& f ) {
    (void)f;
    // Smooth what is shown; the solver keeps warm-starting from its own raw answer.
    float k = haveSmoothed ? MAGLOC_SMOOTHING : 1.0f;
    fix.magnet.x += k * ( fix.rawMagnet.x - fix.magnet.x );
    fix.magnet.y += k * ( fix.rawMagnet.y - fix.magnet.y );
    fix.magnet.z += k * ( fix.rawMagnet.z - fix.magnet.z );
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
    if ( fix.misfit < MAGLOC_LEARN_MAX_MISFIT && fix.seenBy >= MAGLOC_LEARN_MIN_SENSORS ) {
        // Remembered for Y (and the settings): the track's smoothed position
        // when there is one, and only once it has moved a third of a
        // millimetre (or the pole a few degrees) from what is remembered,
        // so a resting probe's jitter leaves the record alone - the
        // settings write it only once it stands still.
        Vec3 p = track.enabled && track.state == MAGTRACK_TRACKING ? track.viewPosition : fix.rawMagnet;
        float dx = p.x - lastGoodPosition.x, dy = p.y - lastGoodPosition.y, dz = p.z - lastGoodPosition.z;
        float ax = pole.x - lastGoodAxis.x, ay = pole.y - lastGoodAxis.y, az = pole.z - lastGoodAxis.z;
        if ( !haveLastGood || dx * dx + dy * dy + dz * dz > 0.3f * 0.3f || ax * ax + ay * ay + az * az > 0.05f * 0.05f ||
             fabsf( result.strength - lastGoodStrength ) > 0.05f * lastGoodStrength ) {
            lastGoodPosition = p;
            lastGoodAxis = pole;
            lastGoodStrength = result.strength;
            haveLastGood = true;
        }
        // A sensor whose zero is provisional (taken from live frames at
        // boot, nobody knowing whether the probe was away): after
        // MAGLOC_SETTLE_GOOD_FIXES good fixes by the others, the magnet's
        // field at it is known. If its reading is missing that field, the
        // field is in its zero (the probe lay there at boot) and comes out;
        // if the reading shows it, the zero was clean. Only a calibrated
        // sensor: the field's direction at an uncalibrated one is not known
        // in its own frame, and it is out of the fit anyway.
        if ( magArray.provisionalCount( ) > 0 && ++provisionalGoodFixes >= MAGLOC_SETTLE_GOOD_FIXES ) {
            for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
                if ( !magArray.zeroProvisional[ i ] || !magSensorPlaces[ i ].calibrated || !magArray.fresh[ i ] ) {
                    continue;
                }
                Vec3 model = magFitDipoleField( magArray.position[ i ], fix.rawMagnet, result.moment );
                const Vec3& r = magArray.field[ i ];
                float modelSq = model.x * model.x + model.y * model.y + model.z * model.z;
                float readingSq = r.x * r.x + r.y * r.y + r.z * r.z;
                if ( modelSq < MAGLOC_SETTLE_MIN_MT * MAGLOC_SETTLE_MIN_MT ) {
                    continue; // the magnet is too far from this sensor to tell either way: another time
                }
                magArray.settleProvisionalZero( i, model, readingSq < 0.25f * modelSq );
            }
        }
    } else {
        provisionalGoodFixes = 0;
    }
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
    fix.tip = pointOf( fix.magnet, fix.shaft );
    fix.rawTip = pointOf( fix.rawMagnet, fix.shaft );
    fix.pointer = pointerOf( fix.tip, fix.shaft );
    fix.rawPointer = pointerOf( fix.rawTip, fix.shaft );
    fix.count++;
    auditZeros( f );

    fix.rough = fix.errorMm > MAGLOC_ROUGH_ABOVE_MM;
    // Every accepted fix is a fix to the tracker, its bar the weight (2026-09-28:
    // a fix wider than MAGLOC_ROUGH_ABOVE_MM went in as ROUGH - no shaft, no
    // velocity, a calmer process noise, held still for the far hold - and the
    // LEDs jumped where the fix sharpened past that line: the shaft snapping
    // in moved the aim cursor by the whole reach). fix.rough is for the
    // screen and the log; the tracker's rough path is the sim's (:probe).
    in->valid = true;
    in->rough = false;
    in->position = fix.rawMagnet;
    in->sigma = fix.sigma;
    in->shaft = fix.shaft;
    in->haveShaft = true; // the shaft as the fit gives it, far or near: its own filter and its slowing with height take a far fix's noise (2026-09-28 afternoon: leaned toward vertical by the bar for three hours, which turned the aim cursor BACK under the tip as the probe rose - the "jump at 40 mm")

    Stream* out = console.port( );
    if ( streaming && out != nullptr && ++streamTick % STREAM_EVERY_N_FIXES == 0 ) {
        printFixCsv( out );
    }

    return ServiceStatus::BUSY;
}

// The zero audit's sample (ZeroAudit.h): once a second while the fit is
// good and the TMAGs see the magnet plainly, the fit redone without one
// sensor predicts that sensor's field; its reading against that goes into
// the audit, and a sensor whose samples solve to a zero error past the
// threshold has it taken out of its baseline (the settings then keep it).
void MagLocator::auditZeros( const FrameScratch& f ) {
    if ( f.held <= 0.0f || !fix.valid || fix.rough || fix.misfit > MAGLOC_AUDIT_MAX_MISFIT || fix.seenBy < MAGLOC_AUDIT_MIN_SEEN || fix.errorMm > MAGLOC_AUDIT_MAX_ERROR_MM ) {
        return;
    }
    uint32_t now = millis( );
    if ( lastAuditMs != 0 && now - lastAuditMs < MAGLOC_AUDIT_PERIOD_MS ) {
        return;
    }
    int count = magArray.sensorCount( );
    // The next sensor that took part in this fit.
    int i = -1;
    for ( int k = 0; k < count; k++ ) {
        int c = ( auditNext + k ) % count;
        if ( f.use[ c ] && magArray.fresh[ c ] ) {
            i = c;
            break;
        }
    }
    if ( i < 0 ) {
        return;
    }
    auditNext = ( i + 1 ) % count;
    lastAuditMs = now == 0 ? 1 : now;
    // The fit again, without sensor i, from this frame's answer.
    // ...and without any sensor the model does not describe (its own
    // audit found a gain far from 1, or a residual the solve cannot
    // explain): its reading would bend the fit and every prediction with it.
    bool without[ MAGFIT_MAX_SENSORS ];
    int voting = 0;
    for ( int k = 0; k < count; k++ ) {
        without[ k ] = f.use[ k ] && k != i && !audit.sensor( k ).excluded;
        voting += without[ k ] ? 1 : 0;
    }
    if ( voting < MAGLOC_AUDIT_MIN_SEEN ) {
        return;
    }
    MagFitResult r = result;
    if ( !magFitRefineKnownStrength( magArray.position, smooth, without, count, MAGLOC_MAX_MISFIT, f.held, &r, f.weights, haveLastGood ? &lastGoodAxis : nullptr, MAGLOC_AXIS_PRIOR_MT,
                                     MAGLOC_AUDIT_ITERATIONS ) ) {
        return;
    }
    float bar = sqrtf( r.sigma.x * r.sigma.x + r.sigma.y * r.sigma.y + r.sigma.z * r.sigma.z );
    if ( bar > MAGLOC_AUDIT_MAX_ERROR_MM ) {
        return;
    }
    Vec3 prediction = magFitDipoleField( magArray.position[ i ], r.position, r.moment );
    audit.addSample( i, prediction, smooth[ i ], now );
    // The windows fill round robin, so they fill within a sample of each
    // other: the round is judged once EVERY sampled sensor's is full (else
    // the first to fill - sensor 0 - was corrected for what an uncorrected
    // MMC error did to its prediction).
    for ( int k = 0; k < count; k++ ) {
        int n = audit.sensor( k ).n;
        if ( n > 0 && n < MAGLOC_AUDIT_MIN_SAMPLES ) {
            return;
        }
    }
    // A round is full. Every sensor is solved; one the model does not
    // describe (a gain far from 1, or a residual the solve cannot explain)
    // is left out of the audit's fits from here and its own zero is not
    // touched (the solve would be attributing a rotation or a place to an
    // offset) - and the round is thrown away, since its samples were taken
    // with that sensor in the fits. Else the ONE sensor whose zero is
    // furthest past its threshold is corrected; then every window starts
    // again, because the others' samples were taken against fits that
    // carried that error (the sim: the MMC's 0.1 mT, uncorrected, made
    // 10-35 uT "zero errors" at three TMAGs).
    // Every window with its samples must also have the spread before any
    // correction is applied: a sensor whose gain is far off pulls the fits of
    // the others, and the first window to solve was a TRUE sensor's, given a
    // false 6.6 % z gain from fits the bad one was still in (2026-09-23, the
    // sim); solved together, the largest correction goes first and the
    // windows start again without that error.
    for ( int k = 0; k < count; k++ ) {
        ZeroAuditSensor& a = audit.sensor( k );
        if ( a.n >= MAGLOC_AUDIT_MIN_SAMPLES && !( a.excluded && !a.correctable ) && !audit.solve( k, MAGLOC_AUDIT_MIN_SAMPLES, MAGLOC_AUDIT_MIN_SPREAD, MAGLOC_AUDIT_GAIN_MIN_MT ) ) {
            return; // too little spread yet on one of them: the samples keep accumulating (a window that never gets it starts again at MAGLOC_AUDIT_MAX_SAMPLES)
        }
    }
    // Zeros first, then gains: a zero error biases the far sensors most (the
    // MMC's 0.005 mT is half of what it reads 90 mm out) and, through their
    // weight, the fits of the others - the first audit scene's world has no
    // gain error, and the solve showed z gains of 1.06 on three TMAGs and
    // 0.91 on a fourth, all the MMC's zero error in the fits (2026-09-23).
    // Only with no zero left to correct is a gain judged.
    int best = -1, bestGain = -1, bestCorrectable = -1;
    float bestRatio = 1.0f, bestGainRatio = 1.0f;
    bool newlyExcluded = false; // a sensor the model does not describe was found in the fits: this round's other solves are not to be trusted
    int newlyCorrectable = -1;  // a sensor found far off-gain this round: the others' windows were taken with it in their fits (polluted); its own are clean
    for ( int k = 0; k < count; k++ ) {
        ZeroAuditSensor& a = audit.sensor( k );
        if ( a.n < MAGLOC_AUDIT_MIN_SAMPLES ) {
            continue;
        }
        if ( !audit.solve( k, MAGLOC_AUDIT_MIN_SAMPLES, MAGLOC_AUDIT_MIN_SPREAD, MAGLOC_AUDIT_GAIN_MIN_MT ) ) {
            continue;
        }
        float meanPred = sqrtf( ( a.spp[ 0 ] + a.spp[ 1 ] + a.spp[ 2 ] ) / a.n );
        float unexplainedLimit = MAGLOC_AUDIT_MAX_UNEXPLAINED * meanPred;
        float noiseFloor = 3.0f * magArray.noiseMt[ k ];
        if ( unexplainedLimit < noiseFloor )
            unexplainedLimit = noiseFloor;
        float gainAxis[ 3 ] = { a.gain.x, a.gain.y, a.gain.z }, gainSigma[ 3 ] = { a.gainSigma.x, a.gainSigma.y, a.gainSigma.z };
        bool wild = false, offGain = false;
        for ( int ax = 0; ax < 3; ax++ ) {
            bool judged = gainSigma[ ax ] < 1e8f && fabsf( gainAxis[ ax ] - 1.0f ) > 3.0f * gainSigma[ ax ]; // a point estimate alone judged nothing (a true far sensor was left out for good)
            wild = wild || ( judged && ( gainAxis[ ax ] > MAGLOC_AUDIT_MAX_GAIN_TRIM || gainAxis[ ax ] < 1.0f / MAGLOC_AUDIT_MAX_GAIN_TRIM ) );
            offGain = offGain || ( judged && fabsf( gainAxis[ ax ] - 1.0f ) > MAGLOC_AUDIT_MAX_GAIN_ERROR );
        }
        bool bad = wild || ( meanPred > 0.0f && a.rmsAfter > unexplainedLimit );
        if ( bad && !( a.excluded && !a.correctable ) ) {
            a.excluded = true;
            a.correctable = false;
            newlyExcluded = true;
            Stream* out = console.port( );
            if ( out != nullptr ) {
                char line[ 200 ];
                snprintf( line, sizeof( line ), "zero audit: sensor %d does not fit the model (gain x %.3f y %.3f z %.3f, %.4f mT unexplained of %.3f): left out of the audit's fits; its row wants a look (magcal)", k,
                          a.gain.x, a.gain.y, a.gain.z, a.rmsAfter, meanPred );
                out->println( line );
            }
        } else if ( offGain && !a.excluded ) {
            // A gain this far off pulls the fits of the others (2026-09-23,
            // the sim: a sensor 36 % low put a false 6.6 % z gain on a true
            // one). Its own samples come from fits WITHOUT it (the audit is
            // leave-one-out), so its own solve is clean: it is corrected
            // this round, zero and gain together, and until then it is out
            // of the others' fits.
            a.excluded = true;
            a.correctable = true;
            newlyCorrectable = k;
        }
        if ( a.excluded && !a.correctable ) {
            continue;
        }
        // Per axis: a zero past the larger of the apply floor (the axis's own
        // noise: Z is quieter) and three of the solve's sigma; a gain past
        // MAGLOC_AUDIT_APPLY_GAIN and three of its sigma. The sensor with the
        // clearest correction goes first; the rest come round after the
        // windows start again without that error in the fits.
        float zeroAxis[ 3 ] = { a.zero.x, a.zero.y, a.zero.z };
        float zeroSig[ 3 ] = { a.zeroSigma.x, a.zeroSigma.y, a.zeroSigma.z }, gainSig[ 3 ] = { a.gainSigma.x, a.gainSigma.y, a.gainSigma.z };
        float prevZ[ 3 ] = { a.previousZero.x, a.previousZero.y, a.previousZero.z };
        float ratio = 0.0f, gainRatio = 0.0f;
        for ( int ax = 0; ax < 3; ax++ ) {
            float noise = magArray.noiseMt[ k ] * ( ax == 2 && magSensorPlaces[ k ].type != MAG_MMC56X3 ? MAG_NOISE_Z_MT / MAG_WEIGHT_REFERENCE_MT : 1.0f );
            float threshold = MAGLOC_AUDIT_APPLY_NOISE * noise;
            if ( threshold < MAGLOC_AUDIT_APPLY_MT )
                threshold = MAGLOC_AUDIT_APPLY_MT;
            if ( threshold < 3.0f * zeroSig[ ax ] )
                threshold = 3.0f * zeroSig[ ax ];
            bool zAgrees = a.hasPrevious && fabsf( zeroAxis[ ax ] - prevZ[ ax ] ) < MAGLOC_AUDIT_ZERO_AGREE;
            float r = zAgrees && fabsf( zeroAxis[ ax ] ) < MAGLOC_AUDIT_MAX_MT ? fabsf( zeroAxis[ ax ] ) / threshold : 0.0f;
            if ( r > ratio )
                ratio = r;
            float gThreshold = MAGLOC_AUDIT_APPLY_GAIN;
            if ( gThreshold < 3.0f * gainSig[ ax ] )
                gThreshold = 3.0f * gainSig[ ax ];
            float prev[ 3 ] = { a.previousGain.x, a.previousGain.y, a.previousGain.z };
            bool agrees = a.hasPrevious && fabsf( gainAxis[ ax ] - prev[ ax ] ) < MAGLOC_AUDIT_GAIN_AGREE;
            float rg = agrees ? fabsf( gainAxis[ ax ] - 1.0f ) / gThreshold : 0.0f;
            if ( rg > gainRatio )
                gainRatio = rg;
        }
        if ( a.excluded && a.correctable && gainRatio > 1.0f && ( bestCorrectable < 0 || gainRatio > bestGainRatio ) ) {
            bestCorrectable = k; // corrected before anything else: its samples are the clean ones, the others' were taken with its error in their fits
        }
        if ( ratio > bestRatio ) {
            bestRatio = ratio;
            best = k;
        }
        if ( gainRatio > bestGainRatio ) {
            bestGainRatio = gainRatio;
            bestGain = k;
        }
    }
    if ( newlyCorrectable >= 0 ) {
        // The others' windows were taken with this sensor's error in their
        // fits: emptied, nothing kept. Its own window is its first word;
        // the next, from fits without it, decides.
        for ( int k = 0; k < count; k++ ) {
            ZeroAuditSensor& w = audit.sensor( k );
            if ( k == newlyCorrectable && w.solved ) {
                w.previousGain = w.gain;
                w.previousZero = w.zero;
                w.hasPrevious = true;
            } else {
                w.hasPrevious = false;
            }
            audit.reset( k );
        }
        Stream* out = console.port( );
        if ( out != nullptr ) {
            const ZeroAuditSensor& w = audit.sensor( newlyCorrectable );
            char line[ 200 ];
            snprintf( line, sizeof( line ), "zero audit: sensor %d's gain is off (x %.3f y %.3f z %.3f): out of the audit's fits of the others; corrected when its next window agrees", newlyCorrectable,
                      w.previousGain.x, w.previousGain.y, w.previousGain.z );
            out->println( line );
        }
        return;
    }
    bool gainRound = false;
    if ( bestCorrectable >= 0 ) {
        best = bestCorrectable; // zero and gain together
        gainRound = true;
    } else if ( best < 0 && bestGain >= 0 ) {
        best = bestGain; // no zero left to correct: the gains' turn
        gainRound = true;
    }
    if ( newlyExcluded ) {
        best = -1; // this round was taken with that sensor in the fits: again without it
    }
    if ( best >= 0 ) {
        ZeroAuditSensor& a = audit.sensor( best );
        // Every axis over its own threshold is corrected, zero and gain alike:
        // the reading was zero + gain * field, so the field is (reading - zero) / gain.
        float zeroAxis[ 3 ] = { a.zero.x, a.zero.y, a.zero.z }, gainAxis[ 3 ] = { a.gain.x, a.gain.y, a.gain.z };
        float zeroSig[ 3 ] = { a.zeroSigma.x, a.zeroSigma.y, a.zeroSigma.z }, gainSig[ 3 ] = { a.gainSigma.x, a.gainSigma.y, a.gainSigma.z };
        float zApply[ 3 ] = { 0, 0, 0 }, gApply[ 3 ] = { 1, 1, 1 };
        bool anyZero = false, anyGain = false;
        for ( int ax = 0; ax < 3; ax++ ) {
            float noise = magArray.noiseMt[ best ] * ( ax == 2 && magSensorPlaces[ best ].type != MAG_MMC56X3 ? MAG_NOISE_Z_MT / MAG_WEIGHT_REFERENCE_MT : 1.0f );
            float threshold = MAGLOC_AUDIT_APPLY_NOISE * noise;
            if ( threshold < MAGLOC_AUDIT_APPLY_MT )
                threshold = MAGLOC_AUDIT_APPLY_MT;
            if ( threshold < 3.0f * zeroSig[ ax ] )
                threshold = 3.0f * zeroSig[ ax ];
            float prevZ[ 3 ] = { a.previousZero.x, a.previousZero.y, a.previousZero.z };
            bool zAgrees = a.hasPrevious && fabsf( zeroAxis[ ax ] - prevZ[ ax ] ) < MAGLOC_AUDIT_ZERO_AGREE;
            // On a sensor flagged for its gain, a zero is applied only on an
            // axis whose gain was judged: on one too weak to judge (gain held
            // at 1) the gain error leaks into the zero (the 36 %-low sensor of
            // check.txt: x 0.012 and z 0.008 of "zero" that were its gain).
            bool judged = gainSig[ ax ] < 1e8f;
            if ( zAgrees && ( !a.correctable || judged ) && fabsf( zeroAxis[ ax ] ) > threshold && fabsf( zeroAxis[ ax ] ) < MAGLOC_AUDIT_MAX_MT ) {
                zApply[ ax ] = zeroAxis[ ax ];
                anyZero = true;
            }
            float gThreshold = MAGLOC_AUDIT_APPLY_GAIN;
            if ( gThreshold < 3.0f * gainSig[ ax ] )
                gThreshold = 3.0f * gainSig[ ax ];
            float prev[ 3 ] = { a.previousGain.x, a.previousGain.y, a.previousGain.z };
            bool agrees = a.hasPrevious && fabsf( gainAxis[ ax ] - prev[ ax ] ) < MAGLOC_AUDIT_GAIN_AGREE;
            if ( gainRound && agrees && fabsf( gainAxis[ ax ] - 1.0f ) > gThreshold ) {
                gApply[ ax ] = gainAxis[ ax ];
                anyGain = true;
            }
        }
        if ( anyZero ) {
            Vec3 by = { -zApply[ 0 ], -zApply[ 1 ], -zApply[ 2 ] };
            magArray.shiftBaseline( best, by ); // (only with a zero to apply: shiftBaseline seals the live baseline into the saved zero)
            smooth[ best ] = { smooth[ best ].x - zApply[ 0 ], smooth[ best ].y - zApply[ 1 ], smooth[ best ].z - zApply[ 2 ] };
        }
        if ( anyGain ) {
            Vec3 g = { gApply[ 0 ], gApply[ 1 ], gApply[ 2 ] };
            magArray.applyGainTrim( best, g );
            smooth[ best ] = { smooth[ best ].x / g.x, smooth[ best ].y / g.y, smooth[ best ].z / g.z };
            for ( int h = 0; h < 2; h++ ) // the glitch filter's two frames of history are readings too
                recent[ best ][ h ] = { recent[ best ][ h ].x / g.x, recent[ best ][ h ].y / g.y, recent[ best ][ h ].z / g.z };
            a.lastGainApplied = g;
        }
        a.applied++;
        a.lastApplied = { zApply[ 0 ], zApply[ 1 ], zApply[ 2 ] };
        if ( a.excluded && a.correctable ) {
            a.excluded = false; // corrected: back into the others' fits
            a.correctable = false;
        }
        Stream* out = console.port( );
        if ( out != nullptr ) {
            char line[ 220 ];
            if ( anyZero ) {
                snprintf( line, sizeof( line ), "zero audit: sensor %d's zero was off by %.4f %.4f %.4f mT (%d samples, spread %.2f): taken out, the settings keep it", best, zApply[ 0 ], zApply[ 1 ], zApply[ 2 ], a.n,
                          a.spread );
                out->println( line );
            }
            if ( anyGain ) {
                snprintf( line, sizeof( line ), "zero audit: sensor %d's gain was off - x %.3f y %.3f z %.3f of what the others predict (%d samples, spread %.2f): corrected, the settings keep it", best, gApply[ 0 ],
                          gApply[ 1 ], gApply[ 2 ], a.n, a.spread );
                out->println( line );
            }
        }
        for ( int k = 0; k < count; k++ ) {
            ZeroAuditSensor& w = audit.sensor( k );
            if ( k != best && w.solved ) {
                w.previousGain = w.gain; // this window's word, for the next to agree with
                w.previousZero = w.zero;
                w.hasPrevious = true;
            } else if ( k == best ) {
                w.hasPrevious = false; // corrected: the next window starts the count again
            }
            audit.reset( k ); // every window again, against fits without that error
        }
    } else if ( newlyExcluded ) {
        for ( int k = 0; k < count; k++ ) {
            audit.reset( k ); // the round was taken with that sensor in the fits: again without it
        }
    } else {
        // Nothing crossed its threshold: the windows go on filling, and the
        // estimates sharpen with root n (a 0.005 mT zero error at the
        // bench's noise takes a hundred samples to tell from nothing) - up
        // to a cap, past which a sensor's window starts again. A window
        // whose gain is over the floor but has no earlier window to agree
        // with (or disagrees with it) ends now, its word kept for the next:
        // a gain needs two windows running, not one long one.
        for ( int k = 0; k < count; k++ ) {
            ZeroAuditSensor& w = audit.sensor( k );
            bool gainOver = w.solved && ( fabsf( w.gain.x - 1.0f ) > MAGLOC_AUDIT_APPLY_GAIN || fabsf( w.gain.y - 1.0f ) > MAGLOC_AUDIT_APPLY_GAIN || fabsf( w.gain.z - 1.0f ) > MAGLOC_AUDIT_APPLY_GAIN );
            float zf = MAGLOC_AUDIT_APPLY_MT;
            bool zeroOver = w.solved && ( fabsf( w.zero.x ) > zf || fabsf( w.zero.y ) > zf || fabsf( w.zero.z ) > zf );
            if ( gainOver || zeroOver || w.n >= MAGLOC_AUDIT_MAX_SAMPLES ) {
                if ( w.solved ) {
                    w.previousGain = w.gain;
                    w.previousZero = w.zero;
                    w.hasPrevious = true;
                }
                audit.reset( k );
            }
        }
    }
}

void MagLocator::printAudit( Stream* out ) const {
    char line[ 220 ];
    snprintf( line, sizeof( line ), "zero audit: each sensor against the fit of the others while the probe is near (a sample a second; per axis, a zero applied past %.3f mT and a gain past 20 %%, each past three of its own sigma and only when two windows of %d samples over spread %.1f agree; zeros before gains)", MAGLOC_AUDIT_APPLY_MT,
              MAGLOC_AUDIT_MIN_SAMPLES, MAGLOC_AUDIT_MIN_SPREAD );
    out->println( line );
    int applied = 0;
    for ( int i = 0; i < magArray.sensorCount( ); i++ )
        applied += audit.sensor( i ).applied;
    snprintf( line, sizeof( line ), "zero audit: corrections applied since boot: %d", applied );
    out->println( line );
    out->println( " #  samples  spread     z_x     z_y     z_z (mT)   gain x y z          rms before/after (mT)  applied  last z applied   (x = left out of the audit's fits: the model does not describe it)" );
    for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
        ZeroAuditSensor a = audit.sensor( i );
        if ( a.n >= 2 ) {
            ZeroAudit copy = audit;
            copy.solve( i, 2, 0.0f, MAGLOC_AUDIT_GAIN_MIN_MT );
            a = copy.sensor( i );
        }
        if ( a.n >= 2 && a.spread < MAGLOC_AUDIT_MIN_SPREAD ) {
            // The probe has not moved enough: z and g cannot be told apart yet.
            snprintf( line, sizeof( line ), "%2d  %7d  %6.2f  (too little spread of position yet to tell a zero from a gain)   %.4f (reading - prediction, rms)   %3d    %+.4f %+.4f %+.4f %s", i, a.n,
                      a.spread, a.rmsBefore, a.applied, a.lastApplied.x, a.lastApplied.y, a.lastApplied.z, a.excluded ? "x" : "" );
        } else {
            snprintf( line, sizeof( line ), "%2d  %7d  %6.2f  %+.4f %+.4f %+.4f   %.3f %.3f %.3f   %.4f / %.4f            %3d    %+.4f %+.4f %+.4f %s", i, a.n, a.spread, a.zero.x, a.zero.y, a.zero.z,
                      a.gain.x, a.gain.y, a.gain.z, a.rmsBefore, a.rmsAfter, a.applied, a.lastApplied.x, a.lastApplied.y, a.lastApplied.z, a.excluded ? "x" : "" );
        }
        out->println( line );
    }
}

static void onAuditVerb( int argc, char** argv, Stream* out ) {
    if ( argc >= 2 && strcmp( argv[ 1 ], "reset" ) == 0 ) {
        magLocator.audit.resetAll( );
        out->println( "zero audit: samples cleared" );
        return;
    }
    if ( argc >= 2 && strcmp( argv[ 1 ], "forget" ) == 0 ) {
        // The gain trims back to 1 (the zeros they scaled follow), the
        // windows cleared, the settings' sensorgain= lines dropped: the undo
        // for a trim the bench's own errors put on a sensor.
        int dropped = 0;
        for ( int i = 0; i < magArray.sensorCount( ); i++ ) {
            const Vec3& t = magArray.gainTrim[ i ];
            if ( t.x != 1.0f || t.y != 1.0f || t.z != 1.0f ) {
                magArray.applyGainTrim( i, { 1.0f / t.x, 1.0f / t.y, 1.0f / t.z } );
                dropped++;
            }
        }
        magLocator.audit.resetAll( );
        char line[ 120 ];
        snprintf( line, sizeof( line ), "zero audit: %d sensor%s gain trim forgotten (back to the table's), samples cleared", dropped, dropped == 1 ? "'s" : "s'" );
        out->println( line );
        return;
    }
    magLocator.printAudit( out );
}

float MagLocator::chiSmoothed( const MagFitResult& r, const bool* use, float alpha ) const {
    float smoothing = sqrtf( alpha / ( 2.0f - alpha ) ); // what an EMA of alpha leaves of white noise
    float sigma[ MAGFIT_MAX_SENSORS ];
    int count = magArray.sensorCount( ) < MAGFIT_MAX_SENSORS ? magArray.sensorCount( ) : MAGFIT_MAX_SENSORS;
    for ( int i = 0; i < count; i++ ) {
        sigma[ i ] = 0.0f;
        if ( !use[ i ] || !magArray.fresh[ i ] )
            continue;
        const Vec3& b = smooth[ i ];
        float size = sqrtf( b.x * b.x + b.y * b.y + b.z * b.z );
        float noise = magArray.noiseMt[ i ] * smoothing;
        float zero = zeroDoubt[ i ]; // the boot uncertainty, decayed by the follow, grown by the drift while held (keepZeros)
        float m = MAG_MODEL_ERROR * size;
        sigma[ i ] = sqrtf( noise * noise + zero * zero + m * m );
    }
    return magFitChi( magArray.position, smooth, use, count, sigma, &r );
}

// k: hold the strength (again), and measure it from scratch.
void MagLocator::learnTipModel( const TipModel& m, float tipMm ) {
    tipModel = m;
    tipOffsetMm = tipMm;
    tipModelD = tipMm;
}

void MagLocator::forgetTipModel( ) {
    tipModelClear( &tipModel );
    tipModelD = tipOffsetMm;
}

void MagLocator::startLearningStrength( ) {
    strengthRingCount = 0;
    strengthRingAt = 0;
    strengthMedian = 0.0f;
    strengthSpread = 0.0f;
    strengthTaken = false;
    strengthAwayFrames = 0;
    strengthPlaceCount = 0;
    strengthPlaceAt = 0;
    strengthPlaceMedian = 0.0f;
    if ( knownStrength <= 0.0f ) {
        knownStrength = strengthSaved > 0.0f ? strengthSaved : MAGLOC_MAGNET_STRENGTH;
    }
}

// k<n>: a known magnet, held as measured (the ring starts afresh and only
// watches for another magnet from here).
void MagLocator::holdStrength( float strength ) {
    startLearningStrength( );
    knownStrength = strength;
    strengthTaken = true;
    strengthIsMeasured = true;
}

// K: never hold it (the free fit's strength is what the readings say, and
// the far probe is out of reach).
void MagLocator::forgetStrength( ) {
    knownStrength = 0.0f;
}

// The settings' strength= record at boot: what was learned last time.
void MagLocator::restoreStrength( float strength, bool measured ) {
    if ( strength < 0.25f * MAGLOC_MAGNET_STRENGTH || strength > 4.0f * MAGLOC_MAGNET_STRENGTH ) {
        return; // not a probe magnet's
    }
    strengthSaved = strength;
    strengthSavedMeasured = measured;
    // A measured record is trusted until the ring says otherwise; a bare one
    // (2026-09-23: the boot default had been saved as if learned, and every
    // boot restored it as "taken") is held, and the first measurement over
    // enough places takes it as on a fresh board.
    strengthTaken = measured;
    strengthIsMeasured = measured;
    if ( knownStrength > 0.0f ) { // K (the strength free) stays free
        knownStrength = strength;
    }
}

// One frame's measurement of the magnet: the free fit's strength from a
// frame the readings pinned on their own (MAGLOC_LEARN_MIN_SENSORS plain,
// no hint, a sharp fit). Into the ring; the ring's median (a few wild
// answers do not matter), once the ring is deep enough and its middle half
// agrees, is TAKEN as the held strength - once - and after that only
// watched: taken again if it is another magnet's (MAGLOC_STRENGTH_JUMP,
// at once) or has sat MAGLOC_STRENGTH_RETAKE_STEP away for
// MAGLOC_STRENGTH_RETAKE_FRAMES (a slow, real change).
static float medianOf( const float* values, int n ) {
    float sorted[ MAGLOC_STRENGTH_RING > MAGLOC_STRENGTH_PLACES ? MAGLOC_STRENGTH_RING : MAGLOC_STRENGTH_PLACES ];
    for ( int i = 0; i < n; i++ ) { // insertion sort: a few dozen numbers, a few microseconds
        float v = values[ i ];
        int at = i;
        while ( at > 0 && sorted[ at - 1 ] > v ) {
            sorted[ at ] = sorted[ at - 1 ];
            at--;
        }
        sorted[ at ] = v;
    }
    return sorted[ n / 2 ];
}

void MagLocator::measureStrength( float strength, Vec3 at ) {
    strengthRing[ strengthRingAt ] = strength;
    strengthRingPlace[ strengthRingAt ] = at;
    strengthRingAt = ( strengthRingAt + 1 ) % MAGLOC_STRENGTH_RING;
    if ( strengthRingCount < MAGLOC_STRENGTH_RING ) {
        strengthRingCount++;
    }
    strengthMeasured++;
    if ( strengthRingCount < MAGLOC_STRENGTH_MIN_SAMPLES ) {
        return;
    }
    int n = strengthRingCount;
    float median = medianOf( strengthRing, n );
    if ( median <= 0.0f ) {
        return;
    }
    // The middle half's spread: the sorted ring's quartiles.
    float sorted[ MAGLOC_STRENGTH_RING ];
    for ( int i = 0; i < n; i++ ) {
        float v = strengthRing[ i ];
        int at = i;
        while ( at > 0 && sorted[ at - 1 ] > v ) {
            sorted[ at ] = sorted[ at - 1 ];
            at--;
        }
        sorted[ at ] = v;
    }
    strengthMedian = median;
    strengthSpread = ( sorted[ n * 3 / 4 ] - sorted[ n / 4 ] ) / median;
    if ( strengthSpread > MAGLOC_STRENGTH_MAX_SPREAD || knownStrength <= 0.0f ) {
        return; // the answers disagree (the probe moving fast, a glitch), or K: nothing to move
    }
    float was = knownStrength;
    Stream* out = console.port( );
    char line[ 200 ];
    // A ring nothing like what is held is another magnet: taken at once,
    // wherever the probe is (at boot, with the compiled default held, the
    // first ring of a different probe's magnet counts as measured).
    if ( fabsf( median - knownStrength ) > MAGLOC_STRENGTH_JUMP * knownStrength ) {
        if ( strengthTaken ) {
            strengthJumps++;
        }
        knownStrength = median;
        strengthTaken = true;
        strengthIsMeasured = true;
        strengthAwayFrames = 0;
        strengthPlaceCount = 0; // the places were the old magnet's
        strengthPlaceAt = 0;
        strengthPlaceMedian = 0.0f;
        if ( out != nullptr ) {
            snprintf( line, sizeof( line ), "magnet strength %.0f mT*mm^3 - %s (was %.0f; the last %d free fits agree to %.1f %%): held from now on%s",
                      knownStrength, strengthJumps > 0 ? "another magnet" : "measured", was, n, strengthSpread * 100.0f,
                      fabsf( knownStrength - strengthSaved ) > MAGLOC_STRENGTH_SAVE_STEP * strengthSaved ? ", and saved in a minute" : "" );
            out->println( line );
        }
        return;
    }
    // This ring's place: its mean position. A new place (MAGLOC_STRENGTH_PLACE_APART_MM
    // from every one remembered) is added with the ring's median; the
    // measurement is the median over the places.
    Vec3 mean = { 0, 0, 0 };
    for ( int i = 0; i < n; i++ ) {
        mean.x += strengthRingPlace[ i ].x;
        mean.y += strengthRingPlace[ i ].y;
        mean.z += strengthRingPlace[ i ].z;
    }
    mean.x /= n;
    mean.y /= n;
    mean.z /= n;
    bool newPlace = true;
    for ( int k = 0; k < strengthPlaceCount; k++ ) {
        float dx = mean.x - strengthPlace[ k ].x, dy = mean.y - strengthPlace[ k ].y, dz = mean.z - strengthPlace[ k ].z;
        if ( dx * dx + dy * dy + dz * dz < MAGLOC_STRENGTH_PLACE_APART_MM * MAGLOC_STRENGTH_PLACE_APART_MM ) {
            newPlace = false;
            strengthPlaceValue[ k ] = median; // the same place again: its latest answer
            break;
        }
    }
    if ( newPlace ) {
        strengthPlace[ strengthPlaceAt ] = mean;
        strengthPlaceValue[ strengthPlaceAt ] = median;
        strengthPlaceAt = ( strengthPlaceAt + 1 ) % MAGLOC_STRENGTH_PLACES;
        if ( strengthPlaceCount < MAGLOC_STRENGTH_PLACES ) {
            strengthPlaceCount++;
        }
    }
    if ( strengthPlaceCount < MAGLOC_STRENGTH_MIN_PLACES ) {
        return;
    }
    strengthPlaceMedian = medianOf( strengthPlaceValue, strengthPlaceCount );
    const char* why = nullptr;
    if ( !strengthTaken ) {
        why = "measured"; // the first measurement over enough places since boot or k
    } else if ( fabsf( strengthPlaceMedian - knownStrength ) > MAGLOC_STRENGTH_RETAKE_STEP * knownStrength ) {
        if ( ++strengthAwayFrames >= MAGLOC_STRENGTH_RETAKE_FRAMES ) {
            why = "it has changed"; // slowly, but for thirty seconds of near fixes
        }
    } else {
        strengthAwayFrames = 0;
    }
    if ( why == nullptr ) {
        return;
    }
    knownStrength = strengthPlaceMedian;
    strengthTaken = true;
    strengthIsMeasured = true;
    strengthAwayFrames = 0;
    if ( out != nullptr ) {
        snprintf( line, sizeof( line ), "magnet strength %.0f mT*mm^3 - %s over %d places (was %.0f): held from now on%s", knownStrength, why, strengthPlaceCount, was,
                  fabsf( knownStrength - strengthSaved ) > MAGLOC_STRENGTH_SAVE_STEP * strengthSaved ? ", and saved in a minute" : "" );
        out->println( line );
    }
}

// The settings' copy follows the held strength once it has stood
// MAGLOC_STRENGTH_SAVE_STEP away from it for MAGLOC_STRENGTH_SAVE_MS: a
// write a minute at most, and a probe waved over the board for a moment
// writes nothing.
void MagLocator::keepStrengthRecord( uint32_t nowMs ) {
    // A measured strength within the save step of an UNCONFIRMED record still
    // seals it as measured (2026-09-24: the bench-like world's magnet learned
    // 3 % from the default, and the record said unconfirmed for good).
    bool sealOnly = strengthIsMeasured && !strengthSavedMeasured;
    if ( knownStrength <= 0.0f || ( !sealOnly && fabsf( knownStrength - strengthSaved ) < MAGLOC_STRENGTH_SAVE_STEP * strengthSaved ) ) {
        strengthAwaySinceMs = 0;
        return;
    }
    if ( strengthAwaySinceMs == 0 ) {
        strengthAwaySinceMs = nowMs == 0 ? 1 : nowMs;
        return;
    }
    if ( nowMs - strengthAwaySinceMs < MAGLOC_STRENGTH_SAVE_MS ) {
        return;
    }
    strengthSaved = roundf( knownStrength / 20.0f ) * 20.0f;
    strengthSavedMeasured = strengthIsMeasured;
    strengthAwaySinceMs = 0;
}

void MagLocator::printFixCsv( Stream* out ) const {
    char line[ 440 ];
    snprintf( line, sizeof( line ), "fix,%lu,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,%.0f,%.3f,%lu,%.3f,%d,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%lu,%.2f",
              (unsigned long)millis( ), fix.present, fix.valid, fix.magnet.x, fix.magnet.y, fix.magnet.z,
              fix.tip.x, fix.tip.y, fix.tip.z, fix.axis.x, fix.axis.y, fix.axis.z,
              fix.tiltDeg, fix.strength, fix.residual, (unsigned long)fix.fitUs, fix.misfit, fix.seenBy, fix.sigma.x, fix.sigma.y, fix.sigma.z,
              fix.faintBy, fix.rawMagnet.x, fix.rawMagnet.y, fix.rawMagnet.z,
              (int)track.state, track.position.x, track.position.y, track.position.z, track.sigma.x, track.sigma.y, track.sigma.z,
              track.cursor.x, track.cursor.y, track.cursor.z, track.lastGate, (unsigned long)track.dropped, fix.chi );
    out->println( line );
}

void MagLocator::printFix( Stream* out ) const {
    char line[ 300 ];
    if ( !fix.present ) {
        snprintf( line, sizeof( line ), "no magnet (strongest reading %.4f mT, %.3f in TMAG terms; threshold %.2f)", fix.peakMt, fix.peakLevel, presentMt );
    } else if ( fix.seenBy + fix.faintBy < MAGLOC_MIN_SENSORS && !fix.valid ) {
        snprintf( line, sizeof( line ), "magnet near (%.2f mT) but only %d sensor%s notice it (%.3f mT or more) - a fix needs %d. Closer, stronger magnet, or tighter sensor pitch.",
                  fix.peakMt, fix.seenBy + fix.faintBy, fix.seenBy + fix.faintBy == 1 ? "" : "s", MAGLOC_FAINT_MT, MAGLOC_MIN_SENSORS );
        if ( knownStrength > 0.0f && fix.seenBy >= 1 ) {
            // The far fit was tried on that one sensor (the strength held) and did not make it.
            out->println( line );
            snprintf( line, sizeof( line ), "  the fit on it: chi %.1f (limit %.1f: residuals over each sensor's expected error), error bar %.0f mm (limit %.0f)",
                      fix.chi, fitMaxChi, fix.errorMm, MAGLOC_MAX_ERROR_MM );
        }
    } else if ( !fix.valid ) {
        snprintf( line, sizeof( line ), "magnet seen by %d sensors, faintly by %d (strongest %.2f mT) but no usable fix: chi %.1f (limit %.1f), misfit %.0f %% (limit %.0f), error bar %.1f mm (limit %.0f)%s",
                  fix.seenBy, fix.faintBy, fix.peakMt, fix.chi, fitMaxChi, fix.misfit * 100.0f, MAGLOC_MAX_MISFIT * 100.0f, fix.errorMm, MAGLOC_MAX_ERROR_MM,
                  fix.rejectedOutside ? " - beyond the array, seen plainly by too few (the mirror basin)" : "" );
    } else {
        snprintf( line, sizeof( line ), "magnet at x %.1f +/-%.1f  y %.1f +/-%.1f  z %.1f +/-%.1f mm   tilt %.0f deg   strength %.0f%s   misfit %.0f %% chi %.1f   seen by %d+%d faint   fit %lu us",
                  fix.magnet.x, fix.sigma.x, fix.magnet.y, fix.sigma.y, fix.magnet.z, fix.sigma.z, fix.tiltDeg, fix.strength,
                  knownStrength > 0.0f ? " (held)" : " (free)", fix.misfit * 100.0f, fix.chi, fix.seenBy, fix.faintBy, (unsigned long)fix.fitUs );
    }
    out->println( line );
    if ( knownStrength > 0.0f || strengthMeasured > 0 ) {
        snprintf( line, sizeof( line ), "  strength: %s %.0f mT*mm^3%s; measured by %lu frames at %d places (the last %d fits agree to %.1f %% about %.0f; over the places %.0f); another magnet taken %lu times; the settings hold %.0f",
                  knownStrength > 0.0f ? "held at" : "not held, last measured about", knownStrength > 0.0f ? knownStrength : strengthMedian,
                  knownStrength > 0.0f && !strengthTaken ? " (not measured yet: the boot default)" : "", (unsigned long)strengthMeasured, strengthPlaceCount, strengthRingCount,
                  strengthSpread * 100.0f, strengthMedian, strengthPlaceMedian, (unsigned long)strengthJumps, strengthSaved );
        out->println( line );
    }
    if ( baselinePolluted ) {
        snprintf( line, sizeof( line ), "BASELINE POLLUTED (a magnet of about %.0f fitted it at %.0f %% misfit): move the probe away and press z", baselineMagnetStrength, baselineMagnetMisfit * 100.0f );
        out->println( line );
    }
    static const char* stateNames[ 4 ] = { "nothing", "rough", "coasting", "tracking" };
    // Two speeds: the filter's velocity (jitter over a frame at rest: 10-20
    // mm/s on a still probe) and what the smoothing takes as the speed
    // (0 at rest; see MAGLOC_SPEED_JITTER_K).
    snprintf( line, sizeof( line ), "track: %s%s  x %.1f y %.1f z %.1f +/-%.1f %.1f %.1f mm  height %.1f mm (~%d; the surface here %.1f)  velocity %.0f mm/s (moving %.0f, smoothing alpha %.2f)  tilt %.0f deg  cursor %s x %.1f y %.1f (+/-%.1f mm, reach %.1f)  %lu fixes taken, %lu dropped, %lu restarts",
              stateNames[ track.state ], track.enabled ? "" : " (tracker off)", track.position.x, track.position.y, track.position.z, track.sigma.x, track.sigma.y, track.sigma.z,
              track.heightMm, (int)lroundf( track.heightMm ), track.surfaceZ,
              sqrtf( track.velocity.x * track.velocity.x + track.velocity.y * track.velocity.y + track.velocity.z * track.velocity.z ), smoothSpeed, smoothAlpha, track.tiltDeg,
              track.cursorMode == MAGCURSOR_UNDER ? "under" : "pointed", track.cursor.x, track.cursor.y, track.cursorSigmaMm, track.reachMm,
              (unsigned long)track.accepted, (unsigned long)track.dropped, (unsigned long)track.reinits );
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
        // The TMAGs only: the check turns whole rows of them; a fixed-address
        // part at its own (as yet uncalibrated) place would only muddy the ranking.
        use[ i ] = magArray.sensor( i ).ok && magSensorPlaces[ i ].type == MAG_TMAG5273;
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
                        field[ i ] = magSensorToBoard( reading[ i ], rotation, underside != 0, magSensorTypeZIntoTop( magSensorPlaces[ i ].type ) );
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
