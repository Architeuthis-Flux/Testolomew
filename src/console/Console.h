// SPDX-License-Identifier: MIT
#ifndef CONSOLE_H
#define CONSOLE_H
// ---------------------------------------------------------------------------
// The serial console: single-character commands, the way JumperlOS's main menu
// works. The console knows no module. A module that wants a command registers
// one from its own begin():
//
//     consoleAddCommand( 'z', "re-zero the magnetometer baseline", onZero );
//
// so adding or removing a module never touches this file. `?` lists whatever
// is registered.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"

#define CONSOLE_MAX_COMMANDS 32

typedef void ( *ConsoleHandler )( Stream* out );

// false if the key is already taken or the table is full.
bool consoleAddCommand( char key, const char* help, ConsoleHandler handler );

// For a command that takes a number (R30<Enter>, t25<Enter>): called from the
// command's handler, reads digits up to Enter and echoes them. Nothing else
// runs while it waits, so Enter with no digits, or timeoutMs without one,
// gives up. Returns the number, or -1 if none was typed.
long consoleReadNumber( Stream* io, const char* prompt, uint32_t timeoutMs );

class Console : public Service {
  public:
    static Console& getInstance( );

    Console( const Console& ) = delete;
    Console& operator=( const Console& ) = delete;

    void begin( Stream* port );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "Console"; }
    ServicePriority getPriority( ) const override { return ServicePriority::CRITICAL; }
    uint32_t periodUs( ) const override { return 5000; }

    Stream* port( ) const { return io; }
    void printHelp( );

  private:
    Console( ) = default;

    Stream* io = nullptr;
};

extern Console& console;

#endif // CONSOLE_H
