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
//
// Beside the letters there are VERBS: a line that starts with a colon,
// `:screen:ascii colour`, `:key left hold`, `:probe row 14 3`, the shape of
// JumperlOS's Ser3 backchannel. The colon enters line mode (before the key
// sink sees anything, so Enter ends the line rather than pressing the nav
// stick); the line collects across ticks with echo and backspace, Enter runs
// it, and a line left alone for CONSOLE_LINE_IDLE_MS is dropped. A module
// registers a verb the same way:
//
//     consoleAddVerb( "leds", "", "the LEDs as an ASCII grid", CONSOLE_READS, onLeds );
//
// Every verb answers with ONE frame a host can pick out with a regex: a
// single line `ok{...}` or `err{...}`, or a block `name{` ... `}` with the
// closing brace alone on its line. `:help` lists them with R (reads) / W
// (changes something) tags.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"

#define CONSOLE_MAX_COMMANDS 48 // 33 in use on 2026-09-18; a registration past the table is refused, silently but for "unknown command"
#define CONSOLE_MAX_VERBS 24 // 21 in use on 2026-09-23 (the simulator adds two); a registration past the table is refused with a console line
#define CONSOLE_LINE_MAX 96
#define CONSOLE_MAX_ARGS 12
#define CONSOLE_LINE_IDLE_MS 2000

typedef void ( *ConsoleHandler )( Stream* out );

// false if the key is already taken or the table is full.
bool consoleAddCommand( char key, const char* help, ConsoleHandler handler );

// The table, for anything that wants to offer the same commands another way
// (the on-screen menu). A help text containing "<number>" marks a command
// that reads one with consoleReadNumber().
int consoleCommandCount( );
bool consoleCommandAt( int index, char* key, const char** help );
bool consoleRunCommand( char key, Stream* io ); // false if no such command

// Verbs. argv[ 0 ] is the verb's name as typed (without the colon), the
// arguments follow; argc counts them all.
#define CONSOLE_READS 1
#define CONSOLE_CHANGES 2
typedef void ( *ConsoleVerbHandler )( int argc, char** argv, Stream* out );
bool consoleAddVerb( const char* name, const char* usage, const char* help, int flags, ConsoleVerbHandler handler );
// Run one line (a leading colon is allowed). false if the verb is unknown
// (an err{} frame is printed either way).
bool consoleRunLine( const char* line, Stream* io );
// Helpers for handlers: print the one-line frames.
void consoleOk( Stream* out, const char* text );
void consoleErr( Stream* out, const char* text );

// Something that wants first look at every character typed (the UI's key
// emulation: arrow keys and a few punctuation marks stand in for the nav
// switch, joystick and buttons). Returns true when it has taken the character.
typedef bool ( *ConsoleKeySink )( char c );
void consoleSetKeySink( ConsoleKeySink sink );

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
    void printVerbs( Stream* out );
    bool inLineMode( ) const { return lineLength >= 0; }

  private:
    Console( ) = default;

    Stream* io = nullptr;
    char line[ CONSOLE_LINE_MAX + 1 ];
    int lineLength = -1; // -1 = not in line mode
    uint32_t lineLastMs = 0;
    bool swallowLf = false; // a CR ended the line: the LF of a CRLF is not a key
    void takeLineChar( char c, uint32_t now );
};

extern Console& console;

#endif // CONSOLE_H
