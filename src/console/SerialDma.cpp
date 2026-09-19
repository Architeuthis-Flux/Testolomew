// SPDX-License-Identifier: MIT
#include "SerialDma.h"

#include <ch32h4_spi.h> // for the SDK's DMA and USART headers, the same way ST7789.cpp gets them

#define SERIALDMA_REQ_USART1_TX 85

static uint8_t ring[ SERIALDMA_RING ];
static volatile size_t head = 0; // written up to here
static volatile size_t tail = 0; // sent (handed to the DMA) from here
static size_t inFlightEnd = 0;   // the run on its way ends here
static bool inFlight = false;
static bool running = false;
static uint32_t maxWaitUs = 0;
static DMA_Channel_TypeDef* dma = nullptr;
static uint32_t dmaFlags = 0;

static DMA_Channel_TypeDef* channelOf( int n ) {
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
    default:
        return DMA1_Channel8;
    }
}

void serialDmaBegin( void ) {
    dma = channelOf( SERIALDMA_CHANNEL );
    dmaFlags = 0xFu << ( 4 * ( SERIALDMA_CHANNEL - 1 ) );
    RCC_HBPeriphClockCmd( RCC_HBPeriph_DMA1, ENABLE );
    DMA_MuxChannelConfig( (uint8_t)( SERIALDMA_CHANNEL - 1 ), SERIALDMA_REQ_USART1_TX );
    USART_DMACmd( USART1, USART_DMAReq_Tx, ENABLE );
    head = tail = inFlightEnd = 0;
    inFlight = false;
    running = true;
}

static bool busy( void ) {
    if ( !inFlight ) {
        return false;
    }
    if ( dma->CNTR != 0 ) {
        return true;
    }
    DMA_Cmd( dma, DISABLE );
    DMA1->INTFCR = dmaFlags;
    tail = inFlightEnd % SERIALDMA_RING;
    inFlight = false;
    return false;
}

static void startRun( void ) {
    size_t h = head, t = tail;
    if ( h == t ) {
        return;
    }
    size_t end = h > t ? h : SERIALDMA_RING; // up to the wrap
    size_t count = end - t;
    DMA_Cmd( dma, DISABLE );
    DMA1->INTFCR = dmaFlags;
    DMA_InitTypeDef d = { };
    d.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR;
    d.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    d.DMA_BufferSize = (uint16_t)count;
    d.DMA_DIR = DMA_DIR_PeripheralDST;
    d.DMA_Memory0BaseAddr = (uint32_t)( ring + t );
    d.DMA_MemoryInc = DMA_MemoryInc_Enable;
    d.DMA_Priority = DMA_Priority_Low;
    DMA_Init( dma, &d );
    USART_ClearFlag( USART1, USART_FLAG_TC );
    DMA_Cmd( dma, ENABLE );
    inFlightEnd = t + count;
    inFlight = true;
}

void serialDmaService( void ) {
    if ( running && !busy( ) ) {
        startRun( );
    }
}

static size_t freeSpace( void ) {
    size_t h = head, t = tail;
    return h >= t ? SERIALDMA_RING - 1 - ( h - t ) : t - h - 1;
}

size_t serialDmaWrite( const uint8_t* data, size_t size ) {
    if ( !running ) {
        return Serial.write( data, size );
    }
    size_t done = 0;
    while ( done < size ) {
        if ( freeSpace( ) == 0 ) {
            // Full: wait for the run on the wire (the only wait in here).
            uint32_t t0 = micros( );
            while ( freeSpace( ) == 0 ) {
                if ( !busy( ) ) {
                    startRun( );
                }
            }
            uint32_t waited = micros( ) - t0;
            if ( waited > maxWaitUs ) {
                maxWaitUs = waited;
            }
        }
        size_t h = head;
        ring[ h ] = data[ done++ ];
        head = ( h + 1 ) % SERIALDMA_RING;
    }
    if ( !busy( ) ) {
        startRun( );
    }
    return size;
}

void serialDmaFlush( void ) {
    if ( !running ) {
        Serial.flush( );
        return;
    }
    while ( head != tail || busy( ) ) {
        if ( !busy( ) ) {
            startRun( );
        }
    }
    while ( USART_GetFlagStatus( USART1, USART_FLAG_TC ) == RESET ) {
    }
}

size_t serialDmaPending( void ) {
    size_t h = head, t = tail;
    return h >= t ? h - t : SERIALDMA_RING - ( t - h );
}

uint32_t serialDmaMaxWaitUs( void ) {
    return maxWaitUs;
}
