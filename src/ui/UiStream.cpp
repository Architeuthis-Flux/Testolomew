// SPDX-License-Identifier: MIT
#include "UiStream.h"

#if __has_include( <ch32h4_spi.h> )
#include "SerialDma.h"
#define UISTREAM_SERIAL_DMA 1
#else
#define UISTREAM_SERIAL_DMA 0 // the host
#endif

UiStream uiStream;

void UiStream::begin( Stream* serial ) {
    port = serial;
#if UISTREAM_SERIAL_DMA
    serialDmaBegin( );
#endif
    head = 0;
    count = 1;
    column = 0;
    lines[ 0 ][ 0 ] = 0;
    keyHead = keyCount = 0;
}

void UiStream::newLine( ) {
    head = ( head + 1 ) % UILOG_LINES;
    if ( count < UILOG_LINES ) {
        count++;
    }
    column = 0;
    lines[ head ][ 0 ] = 0;
    generation++;
}

size_t UiStream::write( uint8_t c ) {
    if ( port != nullptr ) {
#if UISTREAM_SERIAL_DMA
        serialDmaWrite( &c, 1 ); // the port is Serial: by DMA, no wait
#else
        port->write( c );
#endif
    }
    logChar( c );
    return 1;
}

// A block: one call into the DMA ring (a dump's kilobyte row byte by byte
// through write(c) was most of the dump service's tick), then the log.
size_t UiStream::write( const uint8_t* buffer, size_t size ) {
    if ( port != nullptr ) {
#if UISTREAM_SERIAL_DMA
        serialDmaWrite( buffer, size );
#else
        port->write( buffer, size );
#endif
    }
    if ( logToScreen ) {
        for ( size_t i = 0; i < size; i++ ) {
            logChar( buffer[ i ] );
        }
    }
    return size;
}

void UiStream::logChar( uint8_t c ) {
    if ( !logToScreen ) {
        return;
    }
    if ( c == '\n' ) {
        newLine( );
    } else if ( c == '\r' ) {
        // nothing
    } else if ( c == '\t' ) {
        write( (uint8_t)' ' );
    } else if ( c >= 32 ) {
        if ( column >= UILOG_WIDTH ) {
            newLine( ); // wrap
        }
        lines[ head ][ column++ ] = (char)c;
        lines[ head ][ column ] = 0;
        generation++;
    }
}

int UiStream::available( ) {
    return keyCount + ( port != nullptr ? port->available( ) : 0 );
}

int UiStream::read( ) {
    if ( keyCount > 0 ) {
        char c = keys[ keyHead ];
        keyHead = ( keyHead + 1 ) % UISTREAM_KEYS;
        keyCount--;
        return c;
    }
    return port != nullptr ? port->read( ) : -1;
}

int UiStream::peek( ) {
    if ( keyCount > 0 ) {
        return keys[ keyHead ];
    }
    return port != nullptr ? port->peek( ) : -1;
}

void UiStream::flush( ) {
    if ( port != nullptr ) {
#if UISTREAM_SERIAL_DMA
        serialDmaFlush( );
#else
        port->flush( );
#endif
    }
}

bool UiStream::inject( const char* text ) {
    int n = 0;
    while ( text[ n ] != 0 )
        n++;
    if ( keyCount + n > UISTREAM_KEYS ) {
        return false;
    }
    for ( int i = 0; i < n; i++ ) {
        keys[ ( keyHead + keyCount ) % UISTREAM_KEYS ] = text[ i ];
        keyCount++;
    }
    return true;
}

const char* UiStream::logLine( int back ) const {
    if ( back < 0 || back >= count ) {
        return nullptr;
    }
    return lines[ ( head - back + UILOG_LINES ) % UILOG_LINES ];
}
