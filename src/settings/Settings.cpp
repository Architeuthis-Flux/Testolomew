// SPDX-License-Identifier: MIT
#include "Settings.h"

#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Console.h"
#include "MagArray.h"
#include "MagLocator.h"
#include "config.h"
#if MODULE_ROW_COUNT
#include "RowCounter.h"
#endif

SettingsService& settings = SettingsService::getInstance( );

SettingsService& SettingsService::getInstance( ) {
    static SettingsService instance;
    return instance;
}

static void onShow( Stream* out ) {
    settings.print( out );
}

static void onReset( Stream* out ) {
    settings.reset( out );
}

// Items that are modes rather than settings: not saved, so the board boots
// the same way every time (row mode on, the console quiet).
static const char* const unsaved[] = { "rows/row mode", "LEDs/V5 stream", "LEDs/chain on", "play/mode" };

static bool isUnsaved( const char* key ) {
    for ( unsigned k = 0; k < sizeof( unsaved ) / sizeof( unsaved[ 0 ] ); k++ ) {
        if ( strcmp( key, unsaved[ k ] ) == 0 ) {
            return true;
        }
    }
    return false;
}

// The key of a menu item: its page's label, a slash, its own.
static void itemKey( const Menu* m, int index, char* key, int size ) {
    const MenuItem& item = m->items[ index ];
    const char* page = item.parent == MENU_ROOT ? "" : m->items[ item.parent ].label;
    snprintf( key, size, "%s/%s", page, item.label );
}

// One record of the text: the header (step 0), a menu item, a sensor's zero,
// the magnet, an anchor. Returns what it wrote (0 for a record that is left
// out), -1 when there are no more.
int SettingsService::record( int step, char* out, int size ) {
    if ( step == 0 ) {
        return snprintf( out, size, "tuning=%d\n", SETTINGS_TUNING_VERSION );
    }
    step--;
    int items = menu != nullptr ? menu->count : 0;
    if ( step < items ) {
        const MenuItem& item = menu->items[ step ];
        char key[ 48 ];
        char value[ 24 ];
        switch ( item.kind ) {
        case MENU_NUMBER:
            snprintf( value, sizeof( value ), "%.5g", (double)*item.value );
            break;
        case MENU_TOGGLE:
            snprintf( value, sizeof( value ), "%d", menuToggleGet( menu, step ) ? 1 : 0 );
            break;
        case MENU_CHOICE:
            snprintf( value, sizeof( value ), "%d", menuChoiceGet( menu, step ) );
            break;
        default:
            return 0;
        }
        itemKey( menu, step, key, sizeof( key ) );
        if ( isUnsaved( key ) ) {
            return 0;
        }
        return snprintf( out, size, "%s=%s\n", key, value );
    }
    step -= items;
    // The array's zero as last taken (not the drifting live one), and the
    // last good fix - both so a reboot with the probe on the board recovers.
    if ( step < MAG_SENSOR_COUNT ) {
        if ( !magArray.baselineReady( ) || magArray.zeroedAt == 0 || step >= magArray.sensorCount( ) || !magArray.zeroKnown[ step ] || magArray.zeroProvisional[ step ] ) {
            // A sensor with no zero (absent while the others zeroed) gets no
            // line: a saved 0,0,0 would come back as a real zero. Nor does a
            // PROVISIONAL one, taken from live frames at boot with the probe
            // anywhere: saved, it came back at the next boot as known and
            // was never settled (2026-09-21). It is written once the locator
            // has settled it, or z has taken it deliberately.
            return 0;
        }
        // In the sensor's own frame (MagArray::zeroInSensorFrame), so that a
        // table row calibrated after the zero was taken does not turn the
        // zero into a phantom. (zero= lines, in the board frame, are still
        // read; they are dropped at the next write.)
        Vec3 z = magArray.zeroInSensorFrame( step );
        return snprintf( out, size, "sensorzero=%d,%.4f,%.4f,%.4f\n", step, (double)z.x, (double)z.y, (double)z.z );
    }
    step -= MAG_SENSOR_COUNT;
    if ( step == 0 ) {
        if ( !magLocator.haveLastGood ) {
            return 0;
        }
        // A tenth of a millimetre and a hundredth in direction (it comes
        // from the track's smoothed position, whose jitter at rest is well
        // under that), strength to 20, so a resting probe does not keep the
        // record changing - which would keep it from ever being written.
        const Vec3& p = magLocator.lastGoodPosition;
        const Vec3& a = magLocator.lastGoodAxis;
        return snprintf( out, size, "magnet=%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.0f\n", (double)p.x, (double)p.y, (double)p.z, (double)a.x, (double)a.y, (double)a.z,
                         (double)( roundf( magLocator.lastGoodStrength / 20.0f ) * 20.0f ) );
    }
    step--;
    if ( step == 0 ) {
        // The magnet's strength as learned (MagLocator::keepStrengthRecord
        // moves strengthSaved once the held value has stood a minute away
        // from it, so this is written a minute after a change at most).
        return snprintf( out, size, "strength=%.0f\n", (double)magLocator.strengthSaved );
    }
    step--;
#if MODULE_ROW_COUNT
    const RowAnchor* anchors;
    int count = rowCounter.anchorList( &anchors );
    if ( step < count ) {
        return snprintf( out, size, "anchor=%d,%d,%.4g,%.4g,%.4g,%.4g\n", anchors[ step ].row, anchors[ step ].hole, (double)anchors[ step ].position.x,
                         (double)anchors[ step ].position.y, (double)anchors[ step ].position.z, (double)anchors[ step ].sigmaMm );
    }
#endif
    return -1;
}

bool SettingsService::serialiseSome( char* out, int size, int steps ) {
    if ( !walking ) {
        walking = true;
        walkStep = 0;
        walkLength = 0;
    }
    for ( int k = 0; k < steps; k++ ) {
        int wrote = record( walkStep, out + walkLength, size - walkLength );
        if ( wrote < 0 ) {
            out[ walkLength ] = '\0';
            walking = false;
            return true;
        }
        if ( wrote > 0 && walkLength + wrote < size ) {
            walkLength += wrote;
        }
        walkStep++;
    }
    out[ walkLength ] = '\0';
    return false;
}

int SettingsService::serialise( char* out, int size ) {
    walking = false; // start over, whatever the stepped one was doing (it starts over too)
    while ( !serialiseSome( out, size, 64 ) ) {
    }
    return walkLength;
}

// Records into the menu's variables and the row counter. Unknown keys are
// skipped (an item that was renamed or removed); anchors are collected and
// fitted once at the end.
int SettingsService::apply( const char* text ) {
    int applied = 0;
#if MODULE_ROW_COUNT
    RowAnchor anchors[ ROWCOUNT_MAX_ANCHORS ];
    int anchorCount = 0;
#endif
    Vec3 zeros[ MAG_SENSOR_COUNT ];
    bool haveZero[ MAG_SENSOR_COUNT ];
    int zeroCount = 0;
    for ( int i = 0; i < MAG_SENSOR_COUNT; i++ ) {
        haveZero[ i ] = false;
        zeros[ i ] = { 0, 0, 0 };
    }
    bool oldTuning = true; // no tuning= record at all = older than any
    const char* line = text;
    while ( *line != '\0' ) {
        const char* end = strchr( line, '\n' );
        int length = end != nullptr ? (int)( end - line ) : (int)strlen( line );
        const char* eq = (const char*)memchr( line, '=', length );
        if ( eq != nullptr ) {
            int keyLength = (int)( eq - line );
            const char* value = eq + 1;
            if ( keyLength == 6 && strncmp( line, "tuning", 6 ) == 0 ) {
                oldTuning = strtol( value, nullptr, 10 ) < SETTINGS_TUNING_VERSION;
                if ( oldTuning && console.port( ) != nullptr ) {
                    console.port( )->println( "settings: the saved tracker/smoothing values are from an older tuning: the new defaults are used (and saved)" );
                }
            } else if ( ( keyLength == 4 && strncmp( line, "zero", 4 ) == 0 ) || ( keyLength == 10 && strncmp( line, "sensorzero", 10 ) == 0 ) ) {
                // zero= is in the board frame (the older record, right only
                // under the table row it was taken with); sensorzero= is in
                // the sensor's own frame and comes through the row compiled now.
                bool sensorFrame = keyLength == 10;
                char* next = nullptr;
                int i = (int)strtol( value, &next, 10 );
                if ( i >= 0 && i < MAG_SENSOR_COUNT ) {
                    zeros[ i ].x = strtof( next + 1, &next );
                    zeros[ i ].y = strtof( next + 1, &next );
                    zeros[ i ].z = strtof( next + 1, &next );
                    if ( sensorFrame ) {
                        zeros[ i ] = magArray.sensorFrameToBoard( i, zeros[ i ] );
                    }
                    if ( !haveZero[ i ] )
                        zeroCount++;
                    haveZero[ i ] = true;
                    applied++;
                }
            } else if ( keyLength == 6 && strncmp( line, "magnet", 6 ) == 0 ) {
                char* next = nullptr;
                Vec3 p, a;
                p.x = strtof( value, &next );
                p.y = strtof( next + 1, &next );
                p.z = strtof( next + 1, &next );
                a.x = strtof( next + 1, &next );
                a.y = strtof( next + 1, &next );
                a.z = strtof( next + 1, &next );
                float strength = strtof( next + 1, &next );
                if ( strength > 0.0f ) {
                    magLocator.lastGoodPosition = p;
                    magLocator.lastGoodAxis = a;
                    magLocator.lastGoodStrength = strength;
                    magLocator.haveLastGood = true;
                    applied++;
                }
            } else if ( keyLength == 8 && strncmp( line, "strength", 8 ) == 0 ) {
                float strength = strtof( value, nullptr );
                if ( strength > 0.0f ) {
                    magLocator.restoreStrength( strength );
                    applied++;
                }
            } else if ( keyLength == 6 && strncmp( line, "anchor", 6 ) == 0 ) {
#if MODULE_ROW_COUNT
                if ( anchorCount < ROWCOUNT_MAX_ANCHORS ) {
                    RowAnchor& a = anchors[ anchorCount ];
                    char* next = nullptr;
                    a.row = (int)strtol( value, &next, 10 );
                    a.hole = (int)strtol( next + 1, &next, 10 );
                    a.position.x = strtof( next + 1, &next );
                    a.position.y = strtof( next + 1, &next );
                    a.position.z = strtof( next + 1, &next );
                    a.sigmaMm = strtof( next + 1, &next );
                    if ( a.row >= 1 && a.row <= 60 && a.hole >= 1 && a.hole <= 6 ) {
                        anchorCount++;
                        applied++;
                    }
                }
#endif
            } else if ( menu != nullptr ) {
                if ( oldTuning && ( strncmp( line, "tracker/", 8 ) == 0 || strncmp( line, "smoothing/", 10 ) == 0 || strncmp( line, "play/touch", 10 ) == 0 ) ) {
                    if ( end == nullptr )
                        break;
                    line = end + 1;
                    continue; // an older tuning's value: the new default stands
                }
                for ( int i = 0; i < menu->count; i++ ) {
                    char key[ 48 ];
                    itemKey( menu, i, key, sizeof( key ) );
                    if ( (int)strlen( key ) != keyLength || strncmp( key, line, keyLength ) != 0 ) {
                        continue;
                    }
                    const MenuItem& item = menu->items[ i ];
                    if ( item.kind == MENU_NUMBER ) {
                        float v = strtof( value, nullptr );
                        if ( v >= item.min && v <= item.max ) {
                            *item.value = v;
                            applied++;
                        }
                    } else if ( item.kind == MENU_TOGGLE ) {
                        menuToggleSet( menu, i, strtol( value, nullptr, 10 ) != 0 ); // an accessor item's setter carries it into its module
                        applied++;
                    } else if ( item.kind == MENU_CHOICE ) {
                        int v = (int)strtol( value, nullptr, 10 );
                        if ( v >= 0 && v < item.choiceCount ) {
                            menuChoiceSet( menu, i, v );
                            applied++;
                        }
                    }
                    break;
                }
            }
        }
        if ( end == nullptr ) {
            break;
        }
        line = end + 1;
    }
    if ( zeroCount > 0 ) {
        // The sensors the record covers get their saved zero back. Any it
        // does not: the compiled-in zero if there is one for that sensor
        // (a TMAG whose line was lost is never zeroed blind), else a zero
        // from live frames at boot (a sensor added since the record).
        static const Vec3 compiledZero[ MAG_ZERO_AT_BOOT_COUNT ] = MAG_ZERO_AT_BOOT;
        for ( int i = 0; i < MAG_ZERO_AT_BOOT_COUNT && i < MAG_SENSOR_COUNT; i++ ) {
            if ( !haveZero[ i ] ) {
                zeros[ i ] = compiledZero[ i ];
                haveZero[ i ] = true;
            }
        }
        magArray.restoreBaseline( zeros, haveZero, MAG_SENSOR_COUNT, "the saved zero" );
    }
#if MODULE_ROW_COUNT
    if ( anchorCount > 0 ) {
        rowCounter.restoreAnchors( anchors, anchorCount );
    }
#endif
    return applied;
}

// The flash image: magic, length, a 16-bit sum, the text.
static uint16_t sum16( const char* text, int length ) {
    uint16_t s = 0x1357;
    for ( int i = 0; i < length; i++ ) {
        s = (uint16_t)( ( s << 3 ) | ( s >> 13 ) ) ^ (uint8_t)text[ i ];
    }
    return s;
}

bool SettingsService::readFlash( ) {
    uint32_t magic = 0;
    uint16_t length = 0, sum = 0;
    EEPROM.get( 0, magic );
    EEPROM.get( 4, length );
    EEPROM.get( 6, sum );
    if ( magic != SETTINGS_MAGIC || length == 0 || length >= SETTINGS_TEXT_MAX ) {
        return false;
    }
    for ( int i = 0; i < length; i++ ) {
        saved[ i ] = (char)EEPROM.read( 8 + i );
    }
    saved[ length ] = '\0';
    return sum16( saved, length ) == sum;
}

bool SettingsService::writeFlash( const char* text ) {
    uint16_t length = (uint16_t)strlen( text );
    uint16_t sum = sum16( text, length );
    uint32_t magic = SETTINGS_MAGIC;
    EEPROM.put( 0, magic );
    EEPROM.put( 4, length );
    EEPROM.put( 6, sum );
    for ( int i = 0; i < length; i++ ) {
        EEPROM.write( 8 + i, (uint8_t)text[ i ] );
    }
    EEPROM.write( 8 + length, 0 );
    // The writes above go to the RAM mirror; the commit is the erase and
    // the programming, the tens of ms the board spends in the dark.
    if ( aroundWrite != nullptr ) {
        aroundWrite( true );
    }
    bool written = EEPROM.commit( );
    if ( aroundWrite != nullptr ) {
        aroundWrite( false );
    }
    return written;
}

int SettingsService::begin( Menu* m ) {
    menu = m;
    consoleAddCommand( 's', "settings: what is saved for the next boot", onShow );
    consoleAddCommand( 'Z', "settings: reset everything to its default (now, and saved); the row anchors too", onReset );
    saved[ 0 ] = '\0';
    serialise( defaults, sizeof( defaults ) ); // the compile-time values, before anything saved goes in
    if ( !EEPROM.begin( SETTINGS_SIZE ) ) {
        return -1;
    }
    valid = readFlash( );
    if ( !valid ) {
        saved[ 0 ] = '\0';
        loaded = -1;
        return -1;
    }
    loaded = apply( saved );
    // What is running is now what was saved, bar records that were skipped;
    // the next check sees any difference and writes it after it settles.
    return loaded;
}

// Two texts the same but for their magnet= line: that record (the last good
// fix, for Y) is written along with anything else, and when the probe goes
// away, but never wakes a write by itself - a tracked probe would have the
// flash written every few seconds.
static bool sameButForMagnet( const char* a, const char* b ) {
    while ( *a != '\0' || *b != '\0' ) {
        const char* ea = strchr( a, '\n' );
        const char* eb = strchr( b, '\n' );
        int la = ea != nullptr ? (int)( ea - a ) : (int)strlen( a );
        int lb = eb != nullptr ? (int)( eb - b ) : (int)strlen( b );
        bool ma = strncmp( a, "magnet=", 7 ) == 0, mb = strncmp( b, "magnet=", 7 ) == 0;
        if ( ma && mb ) {
            // both have one here: skip both
        } else if ( ma ) {
            a = ea != nullptr ? ea + 1 : a + la;
            continue;
        } else if ( mb ) {
            b = eb != nullptr ? eb + 1 : b + lb;
            continue;
        } else if ( la != lb || strncmp( a, b, la ) != 0 ) {
            return false;
        }
        a = ea != nullptr ? ea + 1 : a + la;
        b = eb != nullptr ? eb + 1 : b + lb;
    }
    return true;
}

ServiceStatus SettingsService::service( ) {
    lastStatus = ServiceStatus::IDLE;
    if ( !serialiseSome( current, sizeof( current ), SETTINGS_STEPS_PER_RUN ) ) {
        return lastStatus; // the text is not complete yet: the next run adds to it
    }
    uint32_t now = millis( );
    bool probeGone = !magLocator.fix.valid && magLocator.track.state == MAGTRACK_NONE;
    if ( probeGone && !wasGone && strcmp( current, saved ) != 0 ) {
        saveNow( ); // the probe has left: its last good fix goes in with everything else
    }
    wasGone = probeGone;
    if ( sameButForMagnet( current, saved ) ) {
        changedSinceMs = 0;
    } else if ( changedSinceMs == 0 || !sameButForMagnet( current, pending ) ) {
        // Changed, or changed again: the settle time starts (over). A value
        // that keeps moving - a lever being nudged, a magnet being tracked -
        // is never written until it stands still.
        strncpy( pending, current, sizeof( pending ) );
        changedSinceMs = now == 0 ? 1 : now;
    } else if ( now - changedSinceMs >= SETTINGS_SETTLE_MS ) {
        if ( !saveNow( ) ) {
            writeFailures++;
            changedSinceMs = now; // try again after another settle time
        }
    }
    return lastStatus;
}

bool SettingsService::saveNow( ) {
    serialise( current, sizeof( current ) );
    if ( !writeFlash( current ) ) {
        return false;
    }
    strncpy( saved, current, sizeof( saved ) );
    valid = true;
    changedSinceMs = 0;
    saves++;
    return true;
}

void SettingsService::reset( Stream* out ) {
    int applied = apply( defaults ); // through the items' setters: an accessor item's module follows
#if MODULE_ROW_COUNT
    consoleRunCommand( 'C', out ); // the anchors: back to the grid the firmware boots with
#endif
    bool written = saveNow( );
    if ( out != nullptr ) {
        char line[ 120 ];
        snprintf( line, sizeof( line ), "settings reset: %d values back to their defaults, the row anchors forgotten, %s", applied, written ? "and saved" : "but NOT saved (flash write failed)" );
        out->println( line );
    }
}

void SettingsService::print( Stream* out ) {
    char line[ 120 ];
    serialise( current, sizeof( current ) );
    if ( !valid ) {
        out->println( "settings: nothing saved (defaults at boot); what is running now will be saved once it has stood for 2 s:" );
    } else {
        const char* state = strcmp( current, saved ) == 0 ? "and what is running is the same" : "what is running differs (it is written once it has stood for 2 s)";
        if ( loaded >= 0 ) {
            snprintf( line, sizeof( line ), "settings: saved in flash (%d records put back at boot, %lu writes since, %lu failed), %s:", loaded, (unsigned long)saves,
                      (unsigned long)writeFailures, state );
        } else {
            snprintf( line, sizeof( line ), "settings: saved in flash since boot (%lu writes, %lu failed; nothing was saved when it booted), %s:", (unsigned long)saves,
                      (unsigned long)writeFailures, state );
        }
        out->println( line );
    }
    if ( valid && strcmp( current, saved ) != 0 ) {
        // What differs, line by line (the running text's lines not in the saved text).
        const char* q = current;
        while ( *q != '\0' ) {
            const char* end = strchr( q, '\n' );
            int length = end != nullptr ? (int)( end - q ) : (int)strlen( q );
            char one[ 100 ];
            snprintf( one, sizeof( one ), "%.*s\n", length, q );
            if ( strstr( saved, one ) == nullptr ) {
                snprintf( line, sizeof( line ), "  running: %.*s", length, q );
                out->println( line );
            }
            if ( end == nullptr )
                break;
            q = end + 1;
        }
    }
    const char* text = valid ? saved : current;
    const char* p = text;
    while ( *p != '\0' ) {
        const char* end = strchr( p, '\n' );
        int length = end != nullptr ? (int)( end - p ) : (int)strlen( p );
        snprintf( line, sizeof( line ), "  %.*s", length, p );
        out->println( line );
        if ( end == nullptr )
            break;
        p = end + 1;
    }
}
