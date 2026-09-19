// SPDX-License-Identifier: MIT
#ifndef ST7789_H
#define ST7789_H
// ---------------------------------------------------------------------------
// ST7789 LCD on the nanoCH32H417's FPC "SPI-LCD" connector, driven the simple
// way: drawing happens in a RAM framebuffer (an Adafruit GFXcanvas16, so all of
// GFX's lines, shapes and text work and JumperlOS drawing code ports over), and
// st7789PushRows() sends a band of rows in one DMA SPI transfer and waits;
// st7789PushRowsStart() / st7789PushBusy() / st7789PushFinish() send one
// without waiting, so the loop reads sensors while the wire is busy (a whole
// 240x240 frame is 115 KB = 26 ms on the wire). MagView sends the frame in
// two such bands. 240x240x2 bytes = 115 KB of the ~700 KB heap.
//
// The panel here is the one wuxx's demo targets: 1.54" 240x240. Other panels
// that fit the same 12-pin FPC only need the numbers below changed:
//     1.3"  240x240  offset 0,0        1.69" 240x280  offset 0,20
//     1.14" 135x240  offset 52,40      2.0"  240x320  offset 0,0
// ---------------------------------------------------------------------------
#include <Arduino.h>

#define LCD_WIDTH 240
#define LCD_HEIGHT 240
#define LCD_X_OFFSET 0
#define LCD_Y_OFFSET 0
#define LCD_MADCTL 0x00 // orientation; 0x60/0xC0/0xA0 = rotated 90/180/270 (then mind the offsets)
#define LCD_INVERT true // IPS ST7789 panels want inversion ON to show true colours
// The core divides its 100 MHz HCLK by a power of two and picks the largest
// rate not over what is asked, so 36 MHz gave 25 and this gives 50 (the
// ST7789's write cycle allows 66). If the picture tears or shifts, 36 is safe.
#define LCD_SPI_HZ 50000000
#define LCD_SPI_MODE SPI_MODE3

// RGB565 helpers.
#define RGB565( r, g, b ) ( (uint16_t)( ( ( ( r ) & 0xF8 ) << 8 ) | ( ( ( g ) & 0xFC ) << 3 ) | ( ( b ) >> 3 ) ) )

// Reset and initialise the panel, clear it to black. false if SPI could not be
// set up on the LCD pins.
bool st7789Begin( void );

// Send rows y0 .. y0+rows-1 of a full LCD_WIDTH x LCD_HEIGHT frame of
// native-endian RGB565 pixels. Those rows are byte-swapped IN PLACE for the
// wire (the panel is big-endian), so send each row once and then redraw.
void st7789PushRows( uint16_t* frame, int y0, int rows );

// The same without waiting: start the DMA and return (false if one is still
// in flight or the band is over one DMA block, 136 rows), poll
// st7789PushBusy() from the scheduler, and st7789PushFinish() when it is
// clear (it waits if it has to). The band is byte-swapped in place at the
// start and must not be drawn into until finished.
bool st7789PushRowsStart( uint16_t* frame, int y0, int rows );
bool st7789PushBusy( void );
void st7789PushFinish( void );

#endif // ST7789_H
