// SPDX-License-Identifier: MIT
#include "Console.h"

#if __has_include( <ch32h4_spi.h> )
#include "SerialDma.h"
#endif

Console& console = Console::getInstance( );

struct ConsoleCommand {
    char key;
    const char* help;
    ConsoleHandler handler;
};

static ConsoleCommand commands[ CONSOLE_MAX_COMMANDS ];
static int commandCount = 0;
static ConsoleKeySink keySink = nullptr;

int consoleCommandCount( ) {
    return commandCount;
}

bool consoleCommandAt( int index, char* key, const char** help ) {
    if ( index < 0 || index >= commandCount ) {
        return false;
    }
    *key = commands[ index ].key;
    *help = commands[ index ].help;
    return true;
}

bool consoleRunCommand( char key, Stream* io ) {
    for ( int i = 0; i < commandCount; i++ ) {
        if ( commands[ i ].key == key ) {
            commands[ i ].handler( io );
            return true;
        }
    }
    return false;
}

void consoleSetKeySink( ConsoleKeySink sink ) {
    keySink = sink;
}

bool consoleAddCommand( char key, const char* help, ConsoleHandler handler ) {
    if ( commandCount >= CONSOLE_MAX_COMMANDS ) {
        // Said out loud: a command that never registered otherwise only
        // shows as "unknown command" (which is how the table was found full
        // at 33 on 2026-09-18).
        Stream* out = console.port( );
        if ( out != nullptr ) {
            out->print( "console: the command table is full, '" );
            out->print( key );
            out->println( "' not registered - raise CONSOLE_MAX_COMMANDS" );
        }
        return false;
    }
    if ( handler == nullptr ) {
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
            if ( c == '\r' && io->peek( ) == '\n' ) {
                io->read( ); // the other half of a CRLF, so it is not read as a key later
            }
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
#if __has_include( <ch32h4_spi.h> )
    serialDmaService( ); // the next queued run of console output, if the last ended between writes
#endif
    if ( io == nullptr || !io->available( ) ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    while ( io->available( ) ) {
        char c = (char)io->read( );
        if ( keySink != nullptr && keySink( c ) ) {
            continue;
        }
        if ( c == '\r' || c == '\n' || c == ' ' ) {
            continue;
        }
        bool found = consoleRunCommand( c, io );
        if ( !found ) {
            io->print( "unknown command '" );
            io->print( c );
            io->println( "' - ? for the list" );
        }
    }
    lastStatus = ServiceStatus::BUSY;
    return lastStatus;
}
