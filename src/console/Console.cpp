// SPDX-License-Identifier: MIT
#include "Console.h"

Console& console = Console::getInstance( );

struct ConsoleCommand {
    char key;
    const char* help;
    ConsoleHandler handler;
};

static ConsoleCommand commands[ CONSOLE_MAX_COMMANDS ];
static int commandCount = 0;

bool consoleAddCommand( char key, const char* help, ConsoleHandler handler ) {
    if ( commandCount >= CONSOLE_MAX_COMMANDS || handler == nullptr ) {
        return false;
    }
    for ( int i = 0; i < commandCount; i++ ) {
        if ( commands[ i ].key == key ) {
            return false;
        }
    }
    commands[ commandCount++ ] = { key, help, handler };
    return true;
}

long consoleReadNumber( Stream* io, const char* prompt, uint32_t timeoutMs ) {
    io->print( prompt );
    long number = 0;
    int digits = 0;
    bool ended = false;
    uint32_t start = millis( );
    while ( !ended && millis( ) - start < timeoutMs ) {
        int c = io->read( );
        if ( c >= '0' && c <= '9' && digits < 6 ) {
            number = number * 10 + ( c - '0' );
            digits++;
            io->print( (char)c );
        } else if ( c == '\r' || c == '\n' ) {
            ended = true;
        }
    }
    io->println( );
    return digits > 0 ? number : -1;
}

static void onHelp( Stream* out ) {
    (void)out;
    console.printHelp( );
}

static void onServices( Stream* out ) {
    jOS.printStats( out );
}

Console& Console::getInstance( ) {
    static Console instance;
    return instance;
}

void Console::begin( Stream* port ) {
    io = port;
    consoleAddCommand( '?', "this list", onHelp );
    consoleAddCommand( 'X', "service table (runs, timing)", onServices );
}

void Console::printHelp( ) {
    io->println( );
    io->println( "Testolomew commands:" );
    for ( int i = 0; i < commandCount; i++ ) {
        io->print( "  " );
        io->print( commands[ i ].key );
        io->print( "  " );
        io->println( commands[ i ].help );
    }
}

ServiceStatus Console::service( ) {
    if ( io == nullptr || !io->available( ) ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    while ( io->available( ) ) {
        char c = (char)io->read( );
        if ( c == '\r' || c == '\n' || c == ' ' ) {
            continue;
        }
        bool found = false;
        for ( int i = 0; i < commandCount; i++ ) {
            if ( commands[ i ].key == c ) {
                commands[ i ].handler( io );
                found = true;
                break;
            }
        }
        if ( !found ) {
            io->print( "unknown command '" );
            io->print( c );
            io->println( "' - ? for the list" );
        }
    }
    lastStatus = ServiceStatus::BUSY;
    return lastStatus;
}
