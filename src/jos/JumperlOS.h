// SPDX-License-Identifier: MIT
#ifndef JUMPERLOS_H
#define JUMPERLOS_H
// ---------------------------------------------------------------------------
// The JumperlOS service scheduler, cut down to what a test bed needs.
//
// This file has the same name and the same Service / jOSmanager interface as
// JumperlOS's src/JumperlOS.h ON PURPOSE: a module written here says
// `#include "JumperlOS.h"`, derives from Service and registers with `jOS`
// exactly the way a JumperlOS module does, so a module that works out can be
// copied into JumperlOS with no edits to its scheduling code.
//
// Kept from JumperlOS: ServiceStatus, ServicePriority, Service (service(),
// getName(), getPriority(), periodUs(), requestRun() and the run statistics)
// and jOSmanager's registerService() / serviceAll() with the same
// due-or-pending gate and the same BLOCKING rule.
// Left out: the modal "inner set" passes (serviceInner / servicePython), the
// force-by-name paths, the ContextManager and all the V5 system services.
// inInnerSet() is kept on Service so a ported module compiles unchanged.
//
// Two changes for the CH32H417 (RISC-V, not RP2350): time_us_64() becomes
// micros64() below, and __dmb() becomes a plain volatile store.
// ---------------------------------------------------------------------------

#include <Arduino.h>

// Microseconds since boot, 64-bit (micros() wraps every 71.6 min). Call it at
// least once per wrap - serviceAll() does, every pass.
uint64_t micros64( void );

enum class ServiceStatus {
    IDLE,     // Service has nothing to do this cycle
    BUSY,     // Service is actively working but non-blocking
    BLOCKING, // Service needs exclusive control (blocks lower priority services)
    ERROR     // Service encountered an error
};

// Higher priority services run first in each pass.
enum class ServicePriority {
    CRITICAL = 0, // always runs (user input)
    HIGH = 1,     // time-sensitive (sensor sampling)
    NORMAL = 2,   // periodic tasks (display, measurements)
    LOW = 3       // background
};

class Service {
  public:
    virtual ~Service( ) {}

    // Main service execution method - called when the service is due.
    virtual ServiceStatus service( ) = 0;

    // Name for the service table and for logging.
    virtual const char* getName( ) const = 0;

    virtual ServicePriority getPriority( ) const = 0;

    // How often this service wants to run, in microseconds. 0 = every pass.
    virtual uint32_t periodUs( ) const { return 0; }

    // Does this service keep running while a BLOCKING service holds the loop?
    virtual bool inInnerSet( ) const { return getPriority( ) == ServicePriority::CRITICAL; }

    virtual bool isActive( ) const {
        return lastStatus == ServiceStatus::BUSY || lastStatus == ServiceStatus::BLOCKING;
    }

    ServiceStatus getLastStatus( ) const { return lastStatus; }

    // Ask the scheduler to run this service on its very next pass, whatever
    // its period. Safe from an interrupt: one byte store.
    void requestRun( ) { pending = true; }

    // Managed by jOSmanager - read-only for everyone else (the service table).
    uint64_t nextDueUs = 0;        // micros64() at which the period next elapses
    volatile bool pending = false; // requestRun() latch
    uint32_t runs = 0;             // service() calls
    uint32_t lastUs = 0;           // duration of the last call
    uint32_t maxUs = 0;            // longest call since boot
    uint32_t overruns = 0;         // calls that took longer than periodUs()
    uint64_t totalUs = 0;          // sum of all call durations (avg = totalUs / runs)

  protected:
    ServiceStatus lastStatus = ServiceStatus::IDLE;
};

class jOSmanager {
  public:
    static jOSmanager& getInstance( );

    // Register a service (the pointer must stay valid). false if the table is
    // full or the service is already registered.
    bool registerService( Service* service );
    bool unregisterService( Service* service );

    // One scheduler pass: every due (or requestRun()-pending) service, in
    // priority order. While a service returns BLOCKING only it and the inner
    // set run.
    void serviceAll( );

    uint8_t getServiceCount( ) const { return serviceCount; }
    Service* getServiceAt( uint8_t index ) const {
        return ( index < serviceCount ) ? services[ index ] : nullptr;
    }
    unsigned long getLoopCount( ) const { return loopCounter; }
    Service* getBlockingService( ) const { return blockingService; }

    // The service table: name, period, runs, last / max / average duration.
    void printStats( Stream* out ) const;

  private:
    jOSmanager( ) {}

    static const uint8_t MAX_SERVICES = 24;

    Service* services[ MAX_SERVICES ] = { nullptr };
    uint8_t serviceCount = 0;
    Service* blockingService = nullptr;
    unsigned long loopCounter = 0;

    void sortServicesByPriority( );
    static bool isDue( Service* svc, uint64_t now );
    ServiceStatus runService( Service* svc, uint64_t now );
};

extern jOSmanager& jOS;

#endif // JUMPERLOS_H
