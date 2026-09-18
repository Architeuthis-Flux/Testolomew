// SPDX-License-Identifier: MIT
#include "ST7789.h"

#include <SPI.h>

#include "BoardPins.h"

#define LCD_BUS SPI

#define CMD_SWRESET 0x01
#define CMD_SLPOUT 0x11
#define CMD_NORON 0x13
#define CMD_INVOFF 0x20
#define CMD_INVON 0x21
#define CMD_DISPON 0x29
#define CMD_CASET 0x2A
#define CMD_RASET 0x2B
#define CMD_RAMWR 0x2C
#define CMD_MADCTL 0x36
#define CMD_COLMOD 0x3A

static void writeCommand( uint8_t command, const uint8_t* data, size_t count ) {
    digitalWrite( PIN_LCD_DC, LOW );
    LCD_BUS.transfer( command );
    digitalWrite( PIN_LCD_DC, HIGH );
    for ( size_t i = 0; i < count; i++ ) {
        LCD_BUS.transfer( data[ i ] );
    }
}

static void setWindow( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 ) {
    uint8_t cols[ 4 ] = { (uint8_t)( x0 >> 8 ), (uint8_t)x0, (uint8_t)( x1 >> 8 ), (uint8_t)x1 };
    uint8_t rows[ 4 ] = { (uint8_t)( y0 >> 8 ), (uint8_t)y0, (uint8_t)( y1 >> 8 ), (uint8_t)y1 };
    writeCommand( CMD_CASET, cols, 4 );
    writeCommand( CMD_RASET, rows, 4 );
}

bool st7789Begin( void ) {
    pinMode( PIN_LCD_CS, OUTPUT );
    pinMode( PIN_LCD_DC, OUTPUT );
    pinMode( PIN_LCD_RST, OUTPUT );
    digitalWrite( PIN_LCD_CS, HIGH );

    // The panel is write-only. MISO defaults to an SPI1 pin, which would stop
    // SPI2 being found for these pins, so it is marked unused.
    LCD_BUS.setSCK( PIN_LCD_SCK );
    LCD_BUS.setMOSI( PIN_LCD_MOSI );
    LCD_BUS.setMISO( (pin_size_t)-1 );
    LCD_BUS.begin( );
    if ( LCD_BUS.peripheral( ) == 0 ) {
        return false;
    }

    digitalWrite( PIN_LCD_RST, LOW );
    delay( 10 );
    digitalWrite( PIN_LCD_RST, HIGH );
    delay( 120 );

    LCD_BUS.beginTransaction( SPISettings( LCD_SPI_HZ, MSBFIRST, LCD_SPI_MODE ) );
    digitalWrite( PIN_LCD_CS, LOW );

    writeCommand( CMD_SWRESET, nullptr, 0 );
    delay( 150 );
    writeCommand( CMD_SLPOUT, nullptr, 0 );
    delay( 120 );
    uint8_t colmod = 0x55; // 16 bits per pixel
    writeCommand( CMD_COLMOD, &colmod, 1 );
    uint8_t madctl = LCD_MADCTL;
    writeCommand( CMD_MADCTL, &madctl, 1 );
    writeCommand( LCD_INVERT ? CMD_INVON : CMD_INVOFF, nullptr, 0 );
    writeCommand( CMD_NORON, nullptr, 0 );
    delay( 10 );

    // Black before the display turns on, so there is no flash of garbage.
    setWindow( LCD_X_OFFSET, LCD_Y_OFFSET, LCD_X_OFFSET + LCD_WIDTH - 1, LCD_Y_OFFSET + LCD_HEIGHT - 1 );
    writeCommand( CMD_RAMWR, nullptr, 0 );
    static const uint8_t blackRow[ LCD_WIDTH * 2 ] = { 0 };
    for ( int y = 0; y < LCD_HEIGHT; y++ ) {
        LCD_BUS.transfer( blackRow, nullptr, sizeof( blackRow ) );
    }
    writeCommand( CMD_DISPON, nullptr, 0 );

    digitalWrite( PIN_LCD_CS, HIGH );
    LCD_BUS.endTransaction( );
    return true;
}

void st7789PushRows( uint16_t* frame, int y0, int rows ) {
    uint16_t* band = frame + (size_t)y0 * LCD_WIDTH;
    size_t pixels = (size_t)rows * LCD_WIDTH;
    for ( size_t i = 0; i < pixels; i++ ) {
        band[ i ] = (uint16_t)( ( band[ i ] << 8 ) | ( band[ i ] >> 8 ) );
    }

    LCD_BUS.beginTransaction( SPISettings( LCD_SPI_HZ, MSBFIRST, LCD_SPI_MODE ) );
    digitalWrite( PIN_LCD_CS, LOW );
    setWindow( LCD_X_OFFSET, LCD_Y_OFFSET + y0, LCD_X_OFFSET + LCD_WIDTH - 1, LCD_Y_OFFSET + y0 + rows - 1 );
    writeCommand( CMD_RAMWR, nullptr, 0 );
    LCD_BUS.transfer( band, nullptr, pixels * 2 ); // one DMA transfer, send-only
    digitalWrite( PIN_LCD_CS, HIGH );
    LCD_BUS.endTransaction( );
}
