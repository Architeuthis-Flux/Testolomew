// SPDX-License-Identifier: MIT
#ifndef LEDSTRIP_H
#define LEDSTRIP_H
// ---------------------------------------------------------------------------
// A WS2812-class LED chain (the breadboard LEDs of a V5, wired to its data
// in) driven from an SPI MOSI pin with DMA, so the loop is not held up: at
// 3.125 MHz every WS2812 bit is four SPI bits - 1000 for a 0 (320 ns high),
// 1100 for a 1 (640 ns high), 1.28 us a bit, which is inside the WS2812B's
// window (T0H 220-380 / T1H 580-1000 ns on the tighter datasheets, period
// 1.25 us +/- 600 ns) - so a 400-LED frame is 4.8 KB and 12 ms on the wire,
// none of it on the CPU. The line idles low between frames (every symbol
// ends in 0), which is the chain's latch.
//
// The pin has to be a MOSI of some SPI peripheral (PD7 is SPI1's; its SCK
// and MISO pins, PA5 / PA6, are named so the library finds the peripheral,
// and the SCK pad is then switched back to an input: the chain does not
// need the clock, and a 3 MHz square wave on a header wire next to the
// sensor bus was found to glitch it). The data goes out at 3.3 V into a
// chain that is usually fed 5 V; that works on a V5 already (its RP2350
// does the same), and the grounds must be shared.
//
// One LedStrip per chain (a V5 from hardware revision 4 has its rails on a
// second one), each with its own SPI and its own DMA1 channel, given at
// begin(): 8 is nobody's among the core's libraries, 1 is DACAudio's (not
// used here); 2 and 3 are the SPI library's and the LCD's push, 4-5 I2S, 6
// PSRAM, 7 ADCInput. The strip's transfer runs at the DMA's highest priority:
// it is the one with bit timing to keep (the LCD does not mind a pause).
//
// Two frame buffers: set() writes the one that is not on the wire, show()
// swaps them and starts the DMA. The times between frames are kept for the
// `n` report, so a late or missed frame shows as a number and not a guess.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#define LEDSTRIP_MAX 448          // a V5 is 445 in all
#define LEDSTRIP_SPI_HZ 3125000   // 100 MHz HCLK / 32
#define LEDSTRIP_BYTES_PER_LED 12 // 24 bits x 4 SPI bits / 8

class SPIClassCH32H4;

struct LedStrip {
    SPIClassCH32H4* bus = nullptr;
    void* dev = nullptr;   // the SPI_TypeDef (the SDK's anonymous typedefs cannot be forward-declared)
    void* dma = nullptr;   // the DMA_Channel_TypeDef
    uint32_t dmaFlags = 0; // this channel's bits of DMA1's INTFR / INTFCR
    int count = 0;
    bool inFlight = false;
    bool changed = true; // set() wrote something different from what the chain last got
    uint8_t frames[ 2 ][ LEDSTRIP_MAX * LEDSTRIP_BYTES_PER_LED ];
    uint8_t* back = nullptr;  // being written
    uint8_t* front = nullptr; // on the wire, or last sent
    // Statistics for the report.
    uint32_t frameCount = 0;
    uint32_t refused = 0; // shows while the wire was busy
    uint32_t unchanged = 0; // shows with nothing new to send
    uint32_t lastStartUs = 0;
    uint32_t gapLastUs = 0, gapMaxUs = 0;
    uint64_t gapSumUs = 0;
    uint32_t gapCount = 0;
};

// Set up the SPI peripheral whose MOSI is `dataPin` (its SCK and MISO pins
// are taken too) and DMA1 channel `dmaChannel` (1-8). false if no
// peripheral has that pin as MOSI.
bool ledStripBegin( LedStrip* s, int dataPin, int sckPin, int misoPin, int count, int dmaChannel );

// Set one LED (0-255 per channel, as the chain shows them; GRB order is
// handled here). Takes effect on the next show().
void ledStripSet( LedStrip* s, int i, uint8_t r, uint8_t g, uint8_t b );
void ledStripClear( LedStrip* s );

// Start sending the frame (DMA; returns at once). false if the previous one
// is still going out - call again on a later tick (the frame set so far is
// kept and goes out then) - or if nothing has changed since the last frame
// (a WS2812 keeps what it was last given; a frame that says the same is
// 12 ms of a 3 MHz signal on a wire for nothing).
bool ledStripShow( LedStrip* s );
// Send the next frame whatever set() did: after a strip has been off, or to
// be sure it has what it should.
void ledStripTouch( LedStrip* s );
bool ledStripBusy( LedStrip* s );
// Wait (up to `ms`) for the frame on the wire to finish. true if it did.
bool ledStripWait( LedStrip* s, uint32_t ms );

// For bring-up: which SPI peripheral (1-4, 0 = none) the pin resolved to,
// how many bytes of the frame in flight the DMA has still to hand over,
// and a line of frame statistics (frames sent, shows refused because the
// wire was busy, and the gaps between frame starts: last / mean / longest
// since the last report).
int ledStripPeripheral( const LedStrip* s );
uint32_t ledStripBytesLeft( const LedStrip* s );
void ledStripReport( LedStrip* s, char* line, size_t size );

#endif // LEDSTRIP_H
