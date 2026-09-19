// SPDX-License-Identifier: MIT
#include "Console.h"

#include <string.h>

#if __has_include( <ch32h4_spi.h> )
#include "SerialDma.h"
#endif

Console& console = Console::getInstance( );

struct ConsoleCommand {
    char key;
    const char* help;
    ConsoleHandler handler;
};

struct ConsoleVerb {
    const char* name;
    const char* usage;
    const char* help;
    int flags;
    ConsoleVerbHandler handler;
};

static ConsoleCommand commands[ CONSOLE_MAX_COMMANDS ];
static int commandCount = 0;
static ConsoleVerb verbs[ CONSOLE_MAX_VERBS ];
static int verbCount = 0;
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

// ---- verbs ----------------------------------------------------------------------

bool consoleAddVerb( const char* name, const char* usage, const char* help, int flags, ConsoleVerbHandler handler ) {
    if ( verbCount >= CONSOLE_MAX_VERBS ) {
        Stream* out = console.port( );
        if ( out != nullptr ) {
            out->print( "console: the verb table is full, ':" );
            out->print( name );
            out->println( "' not registered - raise CONSOLE_MAX_VERBS" );
        }
        return false;
    }
    if ( name == nullptr || handler == nullptr ) {
        return false;
    }
    for ( int i = 0; i < verbCount; i++ ) {
        if ( strcmp( verbs[ i ].name, name ) == 0 ) {
            return false;
        }
    }
    verbs[ verbCount++ ] = { name, usage == nullptr ? "" : usage, help == nullptr ? "" : help, flags, handler };
    return true;
}

void consoleOk( Stream* out, const char* text ) {
    out->print( "ok{" );
    out->print( text );
    out->println( "}" );
}

void consoleErr( Stream* out, const char* text ) {
    out->print( "err{" );
    out->print( text );
    out->println( "}" );
}

bool consoleRunLine( const char* text, Stream* io ) {
    char buffer[ CONSOLE_LINE_MAX + 1 ];
    strncpy( buffer, text, CONSOLE_LINE_MAX );
    buffer[ CONSOLE_LINE_MAX ] = '\0';
    char* p = buffer;
    while ( *p == ' ' || *p == ':' ) {
        p++;
    }
    char* argv[ CONSOLE_MAX_ARGS ];
    int argc = 0;
    while ( *p != '\0' && argc < CONSOLE_MAX_ARGS ) {
        while ( *p == ' ' ) {
            p++;
        }
        if ( *p == '\0' ) {
            break;
        }
        argv[ argc++ ] = p;
        while ( *p != '\0' && *p != ' ' ) {
            p++;
        }
        if ( *p == ' ' ) {
            *p++ = '\0';
        }
    }
    if ( argc == 0 ) {
        consoleErr( io, "empty line - :help lists the verbs" );
        return false;
    }
    for ( int i = 0; i < verbCount; i++ ) {
        if ( strcmp( verbs[ i ].name, argv[ 0 ] ) == 0 ) {
            verbs[ i ].handler( argc, argv, io );
            return true;
        }
    }
    char line[ 80 ];
    snprintf( line, sizeof( line ), "unknown verb ':%s' - :help lists them", argv[ 0 ] );
    consoleErr( io, line );
    return false;
}

void Console::printVerbs( Stream* out ) {
    out->println( "help{" );
    char line[ 160 ];
    for ( int i = 0; i < verbCount; i++ ) {
        char left[ 48 ];
        snprintf( left, sizeof( left ), ":%s %s", verbs[ i ].name, verbs[ i ].usage );
        snprintf( line, sizeof( line ), "  %-40s %c  %s", left, ( verbs[ i ].flags & CONSOLE_CHANGES ) ? 'W' : 'R', verbs[ i ].help );
        out->println( line );
    }
    out->println( "}" );
}

static void onHelpVerb( int argc, char** argv, Stream* out ) {
    (void)argc;
    (void)argv;
    console.printVerbs( out );
}

// ---- the number prompt ----------------------------------------------------------

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
    lineLength = -1;
    consoleAddCommand( '?', "this list", onHelp );
    consoleAddCommand( 'X', "service table (runs, timing)", onServices );
    consoleAddVerb( "help", "", "this list (R reads, W changes something)", CONSOLE_READS, onHelpVerb );
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
    io->println( "  :  a verb line (:help lists them)" );
}

// A character while a line is being collected: echoed, backspace takes one
// off, Enter runs it.
void Console::takeLineChar( char c, uint32_t now ) {
    lineLastMs = now;
    if ( c == '\r' || c == '\n' ) {
        line[ lineLength ] = '\0';
        io->println( );
        lineLength = -1;
        swallowLf = c == '\r';
        consoleRunLine( line, io );
        return;
    }
    if ( c == 8 || c == 127 ) {
        if ( lineLength > 0 ) {
            lineLength--;
            io->print( "\b \b" );
        }
        return;
    }
    if ( c == 27 ) {
        io->println( " (dropped)" ); // Escape: forget the line
        lineLength = -1;
        return;
    }
    if ( c >= 32 && lineLength < CONSOLE_LINE_MAX ) {
        line[ lineLength++ ] = c;
        io->print( c );
    }
}

ServiceStatus Console::service( ) {
#if __has_include( <ch32h4_spi.h> )
    serialDmaService( ); // the next queued run of console output, if the last ended between writes
#endif
    if ( io == nullptr ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    uint32_t now = millis( );
    if ( lineLength >= 0 && now - lineLastMs > CONSOLE_LINE_IDLE_MS ) {
        io->println( " (dropped: no Enter)" );
        lineLength = -1;
    }
    if ( !io->available( ) ) {
        lastStatus = ServiceStatus::IDLE;
        return lastStatus;
    }
    while ( io->available( ) ) {
        char c = (char)io->read( );
        // Line mode first: what is typed after a colon is a verb line, and
        // its Enter ends the line rather than pressing anything.
        if ( lineLength >= 0 ) {
            takeLineChar( c, now );
            continue;
        }
        if ( swallowLf ) {
            swallowLf = false;
            if ( c == '\n' ) {
                continue;
            }
        }
        if ( c == ':' ) {
            lineLength = 0;
            lineLastMs = now;
            io->print( ':' );
            continue;
        }
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
