// SPDX-License-Identifier: MIT
#ifndef ST7789_H
#define ST7789_H
// ---------------------------------------------------------------------------
// ST7789 LCD on the nanoCH32H417's FPC "SPI-LCD" connector, driven the simple
// way: drawing happens in a RAM framebuffer (an Adafruit GFXcanvas16, so all of
// GFX's lines, shapes and text work and JumperlOS drawing code ports over), and
// st7789PushRows() sends a band of rows in one DMA SPI transfer. A whole
// 240x240 frame is 115 KB = 26 ms on the wire, which would stall everything
// else in the loop, so the caller sends it a band per scheduler tick instead
// (see MagView). 240x240x2 bytes = 115 KB of the ~700 KB heap.
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
#define LCD_MADCTL 0x00     // orientation; 0x60/0xC0/0xA0 = rotated 90/180/270 (then mind the offsets)
#define LCD_INVERT true     // IPS ST7789 panels want inversion ON to show true colours
#define LCD_SPI_HZ 36000000 // wuxx's demo runs the panel at 36 MHz
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

#endif // ST7789_H
