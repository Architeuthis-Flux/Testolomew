// SPDX-License-Identifier: MIT
#ifndef SERIALDMA_H
#define SERIALDMA_H
// ---------------------------------------------------------------------------
// The console's transmit without the wait.
//
// The core's Serial.write() waits for the UART to take every byte: 87 us a
// character at 115200, so a 150-character row line held the loop for 13 ms
// twice a second, a fix stream line 22 ms twenty times a second, and the
// service table for 90 ms - holes in the sensor frames, the LED frames and
// the display, all from printing. Here bytes go into a ring, and DMA1
// channel SERIALDMA_CHANNEL hands the ring to USART1 (request 85, reference
// manual table 10-2) a contiguous run at a time; the loop only ever starts
// the next run. Only a full ring waits (4 KB is a second's worth at 115200,
// more than any one print).
//
// The UART's own configuration is Serial's; this only adds the DMA request
// on its transmit side, so Serial.write() still works (before begin(), or
// for anything that must be out before the next instruction, like a fault).
// ---------------------------------------------------------------------------
#include <Arduino.h>

#define SERIALDMA_RING 4096
#define SERIALDMA_CHANNEL 7 // DMA1 channel: 3 is the LCD, 8 the LED chain, 1 the second chain; 7 is ADCInput's, not used here

// Start driving USART1 (Serial, once it has begun) by DMA.
void serialDmaBegin( void );
// Queue bytes; waits only if the ring is full.
size_t serialDmaWrite( const uint8_t* data, size_t size );
// Start the next run if the DMA is idle and there is one: called from write()
// and, in case the last run ended between writes, from the console's service.
void serialDmaService( void );
// Wait for everything queued to be on the wire.
void serialDmaFlush( void );
// Bytes waiting, and the longest wait a full ring cost (us), for the record.
size_t serialDmaPending( void );
uint32_t serialDmaMaxWaitUs( void );

#endif // SERIALDMA_H
