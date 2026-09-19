// SPDX-License-Identifier: MIT
#include "FastDraw.h"

// GFX's font table, a private copy: the library keeps its own `static` in
// Adafruit_GFX.cpp. Five bytes a character, one per column, bit 0 the top
// row. Its PROGMEM is empty on this core (and on a PC).
#include <glcdfont.c>

void fastText( GFXcanvas16* canvas, int x, int y, int size, uint16_t colour, const char* text ) {
    if ( canvas == nullptr || text == nullptr || size < 1 ) {
        return;
    }
    uint16_t* buffer = canvas->getBuffer( );
    const int width = canvas->width( ), height = canvas->height( );
    if ( buffer == nullptr || y >= height || y + 8 * size <= 0 ) {
        return;
    }
    for ( ; *text != '\0'; text++, x += 6 * size ) {
        if ( x >= width ) {
            break; // the rest is off the right edge
        }
        if ( x + 5 * size <= 0 ) {
            continue; // off the left edge
        }
        const unsigned char* glyph = font + (unsigned char)*text * 5;
        for ( int column = 0; column < 5; column++ ) {
            uint8_t bits = glyph[ column ];
            if ( bits == 0 ) {
                continue;
            }
            int x0 = x + column * size, x1 = x0 + size;
            if ( x0 < 0 )
                x0 = 0;
            if ( x1 > width )
                x1 = width;
            if ( x0 >= x1 ) {
                continue;
            }
            for ( int row = 0; row < 8; row++, bits >>= 1 ) {
                if ( !( bits & 1 ) ) {
                    continue;
                }
                int y0 = y + row * size, y1 = y0 + size;
                if ( y0 < 0 )
                    y0 = 0;
                if ( y1 > height )
                    y1 = height;
                for ( int py = y0; py < y1; py++ ) {
                    uint16_t* at = buffer + py * width;
                    for ( int px = x0; px < x1; px++ ) {
                        at[ px ] = colour;
                    }
                }
            }
        }
    }
}

void fastLine( GFXcanvas16* canvas, int x0, int y0, int x1, int y1, uint16_t colour ) {
    if ( canvas == nullptr ) {
        return;
    }
    uint16_t* buffer = canvas->getBuffer( );
    const int width = canvas->width( ), height = canvas->height( );
    if ( buffer == nullptr ) {
        return;
    }
    // Wholly outside: nothing to do (a line 2000 px off-screen would
    // otherwise still be walked pixel by pixel).
    if ( ( x0 < 0 && x1 < 0 ) || ( y0 < 0 && y1 < 0 ) || ( x0 >= width && x1 >= width ) || ( y0 >= height && y1 >= height ) ) {
        return;
    }
    int dx = x1 - x0, dy = y1 - y0;
    bool steep = ( dy < 0 ? -dy : dy ) > ( dx < 0 ? -dx : dx );
    if ( steep ) {
        int t = x0;
        x0 = y0;
        y0 = t;
        t = x1;
        x1 = y1;
        y1 = t;
    }
    if ( x0 > x1 ) {
        int t = x0;
        x0 = x1;
        x1 = t;
        t = y0;
        y0 = y1;
        y1 = t;
    }
    dx = x1 - x0;
    dy = y1 - y0;
    int yStep = 1;
    if ( dy < 0 ) {
        dy = -dy;
        yStep = -1;
    }
    int err = dx / 2;
    int y = y0;
    for ( int x = x0; x <= x1; x++ ) {
        int px = steep ? y : x, py = steep ? x : y;
        if ( px >= 0 && px < width && py >= 0 && py < height ) {
            buffer[ px + py * width ] = colour;
        }
        err -= dy;
        if ( err < 0 ) {
            y += yStep;
            err += dx;
        }
    }
}

void fastFillRect( GFXcanvas16* canvas, int x, int y, int w, int h, uint16_t colour ) {
    if ( canvas == nullptr ) {
        return;
    }
    uint16_t* buffer = canvas->getBuffer( );
    const int width = canvas->width( ), height = canvas->height( );
    if ( buffer == nullptr ) {
        return;
    }
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = x + w > width ? width : x + w, y1 = y + h > height ? height : y + h;
    for ( int py = y0; py < y1; py++ ) {
        uint16_t* at = buffer + py * width;
        for ( int px = x0; px < x1; px++ ) {
            at[ px ] = colour;
        }
    }
}

void fastRect( GFXcanvas16* canvas, int x, int y, int w, int h, uint16_t colour ) {
    if ( w <= 0 || h <= 0 ) {
        return;
    }
    fastFillRect( canvas, x, y, w, 1, colour );
    fastFillRect( canvas, x, y + h - 1, w, 1, colour );
    fastFillRect( canvas, x, y, 1, h, colour );
    fastFillRect( canvas, x + w - 1, y, 1, h, colour );
}
