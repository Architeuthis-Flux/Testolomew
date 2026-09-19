// SPDX-License-Identifier: MIT
#include "LedStrip.h"

#include <SPI.h>
#include <ch32h4_gpio.h> // g_pins
#include <ch32h4_rcc.h>  // ch32h4_pin_af
#include <ch32h4_spi.h>

// The request line is the reference manual's 61 + 2 x peripheral for SPI TX.
#define STRIP_DMA_TX_REQ( id ) ( 61 + 2 * ( id ) )
// GPIO CFGxR nibble for the data pin: CNF 10 = alternate-function push-pull,
// MODE 01 = the 10 MHz output class (CH32H4_CFG_AF_PP_50 is 0xB, the 50 MHz
// one, which is what the SPI library sets).
#define LEDSTRIP_MOSI_CFG 0x9u
#define DEV( s ) ( (SPI_TypeDef*)( s )->dev )
#define DMA( s ) ( (DMA_Channel_TypeDef*)( s )->dma )

static DMA_Channel_TypeDef* dmaChannel( int n ) {
    switch ( n ) {
    case 1:
        return DMA1_Channel1;
    case 2:
        return DMA1_Channel2;
    case 3:
        return DMA1_Channel3;
    case 4:
        return DMA1_Channel4;
    case 5:
        return DMA1_Channel5;
    case 6:
        return DMA1_Channel6;
    case 7:
        return DMA1_Channel7;
    case 8:
        return DMA1_Channel8;
    default:
        return nullptr;
    }
}

// One byte of colour becomes four bytes of symbols, two bits per byte: a
// 0 bit is 1000, a 1 bit 1100, so each pair is 0x88 with 0x40 for the high
// bit set and 0x04 for the low. Arithmetic, not a table: a table sits in
// flash, and 4800 flash reads a frame is milliseconds on the main core
// (2026-09-19: the strip send took 3 ms, of which this loop was nearly all).
static inline uint8_t pair( uint8_t twoBits ) {
    return (uint8_t)( 0x88u | ( ( twoBits & 2u ) ? 0x40u : 0u ) | ( ( twoBits & 1u ) ? 0x04u : 0u ) );
}
static inline void encode( uint8_t* out, uint8_t value ) {
    out[ 0 ] = pair( ( value >> 6 ) & 3 );
    out[ 1 ] = pair( ( value >> 4 ) & 3 );
    out[ 2 ] = pair( ( value >> 2 ) & 3 );
    out[ 3 ] = pair( value & 3 );
}

bool ledStripBegin( LedStrip* s, int dataPin, int sckPin, int misoPin, int count, int dmaCh ) {
    if ( count > LEDSTRIP_MAX ) {
        count = LEDSTRIP_MAX;
    }
    s->count = count;
    s->back = s->frames[ 0 ];
    s->front = s->frames[ 1 ];
    s->dma = dmaChannel( dmaCh );
    if ( s->dma == nullptr ) {
        return false;
    }
    s->dmaFlags = 0xFu << ( 4 * ( dmaCh - 1 ) );             // channel n's GIF/TCIF/HTIF/TEIF
    s->bus = new SPIClassCH32H4( sckPin, misoPin, dataPin ); // once, for good: a strip is never torn down
    s->bus->begin( );                                        // resolves the pins to a peripheral (or to none)
    if ( s->bus->peripheral( ) == 0 ) {
        return false; // no SPI has these pins
    }
    s->dev = ch32h4_spi_regs( s->bus->peripheral( ) );
    // The chain only listens to MOSI. The SPI library put SCK on its pin as
    // a 50 MHz-class push-pull output; that is a 3 MHz square wave on a
    // header wire for nothing, and with the sensor bus on the same headers
    // it was enough to wedge the I2C block every second or two (2026-09-18:
    // ~50 bus resets a minute with the strip running, 2 with it off). The
    // peripheral keeps its clock inside; the pad goes back to an analog
    // input. MOSI gets the 10 MHz-class driver: slower edges, same 320 ns
    // symbol pulses.
    ch32h4_pin_af( g_pins[ sckPin ].port, g_pins[ sckPin ].bit, CH32H4_AF_NONE, CH32H4_CFG_IN_ANALOG );
    ch32h4_pin_af( g_pins[ dataPin ].port, g_pins[ dataPin ].bit, CH32H4_AF_NONE, LEDSTRIP_MOSI_CFG );
    ledStripClear( s );
    for ( int i = 0; i < s->count * LEDSTRIP_BYTES_PER_LED; i++ ) {
        s->front[ i ] = 0x88; // all off in the other buffer too
    }
    s->changed = true; // the first frame always goes: the chain may show anything
    // Settings once and for all: the bus is this chain's alone.
    s->bus->beginTransaction( SPISettings( LEDSTRIP_SPI_HZ, MSBFIRST, SPI_MODE0 ) );
    RCC_HBPeriphClockCmd( RCC_HBPeriph_DMA1, ENABLE );
    DMA_MuxChannelConfig( (uint8_t)( dmaCh - 1 ), STRIP_DMA_TX_REQ( s->bus->peripheral( ) ) ); // DMA_MuxChannelN is N - 1
    return s->dev != nullptr;
}

void ledStripSet( LedStrip* s, int i, uint8_t r, uint8_t g, uint8_t b ) {
    if ( i < 0 || i >= s->count ) {
        return;
    }
    uint8_t* at = s->back + i * LEDSTRIP_BYTES_PER_LED;
    uint8_t now[ LEDSTRIP_BYTES_PER_LED ];
    encode( now, g ); // WS2812 order: green, red, blue
    encode( now + 4, r );
    encode( now + 8, b );
    for ( int k = 0; k < LEDSTRIP_BYTES_PER_LED; k++ ) {
        if ( at[ k ] != now[ k ] ) {
            at[ k ] = now[ k ];
            s->changed = true;
        }
    }
}

void ledStripTouch( LedStrip* s ) {
    s->changed = true;
}

void ledStripClear( LedStrip* s ) {
    for ( int i = 0; i < s->count; i++ ) {
        ledStripSet( s, i, 0, 0, 0 );
    }
}

int ledStripPeripheral( const LedStrip* s ) {
    return s->bus != nullptr ? s->bus->peripheral( ) : 0;
}

uint32_t ledStripBytesLeft( const LedStrip* s ) {
    return s->inFlight ? DMA( s )->CNTR : 0;
}

bool ledStripBusy( LedStrip* s ) {
    if ( !s->inFlight ) {
        return false;
    }
    if ( DMA( s )->CNTR != 0 || ( DEV( s )->STATR & SPI_STATR_BSY ) ) {
        return true;
    }
    // Done: tidy up so the next show can start.
    SPI_I2S_DMACmd( DEV( s ), SPI_I2S_DMAReq_Tx, DISABLE );
    DMA_Cmd( DMA( s ), DISABLE );
    DMA1->INTFCR = s->dmaFlags;
    s->inFlight = false;
    return false;
}

bool ledStripWait( LedStrip* s, uint32_t ms ) {
    uint32_t start = millis( );
    while ( ledStripBusy( s ) ) {
        if ( millis( ) - start > ms ) {
            return false;
        }
    }
    return true;
}

bool ledStripShow( LedStrip* s ) {
    if ( s->dev == nullptr || s->count == 0 ) {
        return false;
    }
    if ( ledStripBusy( s ) ) {
        s->refused++;
        return false;
    }
    if ( !s->changed ) {
        s->unchanged++;
        s->lastStartUs = 0; // the next gap is not a running one
        return false;
    }
    // The written buffer goes out; the one that was on the wire is copied
    // into the other, so the next set()s compare against what the chain
    // has and only a difference sends a frame.
    uint8_t* t = s->front;
    s->front = s->back;
    s->back = t;
    memcpy( s->back, s->front, s->count * LEDSTRIP_BYTES_PER_LED );
    s->changed = false;
    uint32_t now = micros( );
    if ( s->frameCount > 0 && s->lastStartUs != 0 ) {
        s->gapLastUs = now - s->lastStartUs;
        s->gapSumUs += s->gapLastUs;
        s->gapCount++;
        if ( s->gapLastUs > s->gapMaxUs ) {
            s->gapMaxUs = s->gapLastUs;
        }
    }
    s->lastStartUs = now;
    s->frameCount++;

    DMA_Cmd( DMA( s ), DISABLE );
    DMA1->INTFCR = s->dmaFlags;
    DMA_InitTypeDef d = { };
    d.DMA_PeripheralBaseAddr = (uint32_t)&DEV( s )->DATAR;
    d.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    d.DMA_BufferSize = (uint16_t)( s->count * LEDSTRIP_BYTES_PER_LED );
    d.DMA_DIR = DMA_DIR_PeripheralDST;
    d.DMA_Memory0BaseAddr = (uint32_t)s->front;
    d.DMA_MemoryInc = DMA_MemoryInc_Enable;
    d.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init( DMA( s ), &d );
    if ( DEV( s )->STATR & SPI_STATR_RXNE ) {
        (void)DEV( s )->DATAR;
    }
    SPI_I2S_DMACmd( DEV( s ), SPI_I2S_DMAReq_Tx, ENABLE );
    DMA_Cmd( DMA( s ), ENABLE );
    s->inFlight = true;
    return true;
}

void ledStripReport( LedStrip* s, char* line, size_t size ) {
    uint32_t mean = s->gapCount > 0 ? (uint32_t)( s->gapSumUs / s->gapCount ) : 0;
    snprintf( line, size,
              "SPI%d: %lu frames of %d LEDs (%d bytes, %.1f ms on the wire), %lu shows refused (wire busy), %lu skipped (nothing changed); gap between frames %.1f ms last, %.1f mean, %.1f longest since the last report",
              ledStripPeripheral( s ), (unsigned long)s->frameCount, s->count, s->count * LEDSTRIP_BYTES_PER_LED, s->count * LEDSTRIP_BYTES_PER_LED * 8.0f / LEDSTRIP_SPI_HZ * 1e3f,
              (unsigned long)s->refused, (unsigned long)s->unchanged, s->gapLastUs * 1e-3f, mean * 1e-3f, s->gapMaxUs * 1e-3f );
    s->gapMaxUs = 0;
    s->gapSumUs = 0;
    s->gapCount = 0;
}
