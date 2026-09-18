// SPDX-License-Identifier: MIT
#include "JumperlOS.h"

jOSmanager& jOS = jOSmanager::getInstance( );

jOSmanager& jOSmanager::getInstance( ) {
    static jOSmanager instance;
    return instance;
}

// micros() is 32-bit and wraps every 71.6 minutes; count the wraps. Only
// called from the main loop, so no locking.
uint64_t micros64( void ) {
    static uint32_t lastLow = 0;
    static uint32_t high = 0;
    uint32_t low = micros( );
    if ( low < lastLow ) {
        high++;
    }
    lastLow = low;
    return ( (uint64_t)high << 32 ) | low;
}

bool jOSmanager::registerService( Service* service ) {
    if ( service == nullptr || serviceCount >= MAX_SERVICES ) {
        return false;
    }
    for ( uint8_t i = 0; i < serviceCount; i++ ) {
        if ( services[ i ] == service ) {
            return false; // Already registered
        }
    }
    services[ serviceCount++ ] = service;
    service->nextDueUs = micros64( ); // First run is due now.
    sortServicesByPriority( );
    return true;
}

bool jOSmanager::unregisterService( Service* service ) {
    for ( uint8_t i = 0; i < serviceCount; i++ ) {
        if ( services[ i ] != service ) {
            continue;
        }
        for ( uint8_t j = i; j + 1 < serviceCount; j++ ) {
            services[ j ] = services[ j + 1 ];
        }
        services[ --serviceCount ] = nullptr;
        if ( blockingService == service ) {
            blockingService = nullptr;
        }
        return true;
    }
    return false;
}

// The due-or-pending gate. Captures (and, only if set, clears) the service's
// requestRun() latch, then answers "run it this pass?".
bool jOSmanager::isDue( Service* svc, uint64_t now ) {
    if ( svc->pending ) {
        svc->pending = false;
        return true;
    }
    uint32_t period = svc->periodUs( );
    if ( period == 0 ) {
        return true;
    }
    return now >= svc->nextDueUs;
}

// Run one service and account for it. nextDueUs is stamped from the run START
// ("from now", not from the previous due time - no catch-up bursts after a
// stall).
ServiceStatus jOSmanager::runService( Service* svc, uint64_t now ) {
    uint32_t period = svc->periodUs( );
    if ( period != 0 ) {
        svc->nextDueUs = now + period;
    }
    ServiceStatus status = svc->service( );
    uint32_t took = (uint32_t)( micros64( ) - now );
    svc->runs++;
    svc->lastUs = took;
    if ( took > svc->maxUs ) {
        svc->maxUs = took;
    }
    svc->totalUs += took;
    if ( period != 0 && took > period ) {
        svc->overruns++;
    }
    return status;
}

void jOSmanager::serviceAll( ) {
    loopCounter++;

    for ( uint8_t i = 0; i < serviceCount; i++ ) {
        Service* svc = services[ i ];

        // While a service is BLOCKING, only the inner set and the blocking
        // service itself run. Checked BEFORE the pending capture so a
        // requestRun() against a blocked service survives the skip.
        if ( blockingService != nullptr && blockingService != svc && !svc->inInnerSet( ) ) {
            continue;
        }

        uint64_t now = micros64( );
        if ( !isDue( svc, now ) ) {
            continue;
        }

        ServiceStatus status = runService( svc, now );

        if ( status == ServiceStatus::BLOCKING ) {
            blockingService = svc;
        } else if ( blockingService == svc ) {
            blockingService = nullptr;
        }
    }
}

// Simple bubble sort - small array. Registration order is kept within a
// priority.
void jOSmanager::sortServicesByPriority( ) {
    for ( uint8_t i = 0; i + 1 < serviceCount; i++ ) {
        for ( uint8_t j = 0; j + 1 < serviceCount - i; j++ ) {
            if ( (int)services[ j ]->getPriority( ) > (int)services[ j + 1 ]->getPriority( ) ) {
                Service* tmp = services[ j ];
                services[ j ] = services[ j + 1 ];
                services[ j + 1 ] = tmp;
            }
        }
    }
}

void jOSmanager::printStats( Stream* out ) const {
    out->println( "service          prio  period_us      runs   last_us    max_us    avg_us  overruns" );
    for ( uint8_t i = 0; i < serviceCount; i++ ) {
        Service* svc = services[ i ];
        uint32_t avg = svc->runs ? (uint32_t)( svc->totalUs / svc->runs ) : 0;
        char line[ 120 ];
        snprintf( line, sizeof( line ), "%-16s %4d %10lu %9lu %9lu %9lu %9lu %9lu",
                  svc->getName( ), (int)svc->getPriority( ), (unsigned long)svc->periodUs( ),
                  (unsigned long)svc->runs, (unsigned long)svc->lastUs, (unsigned long)svc->maxUs,
                  (unsigned long)avg, (unsigned long)svc->overruns );
        out->println( line );
    }
    out->print( "passes: " );
    out->println( loopCounter );
}
