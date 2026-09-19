// SPDX-License-Identifier: MIT
#ifndef FASTDRAW_H
#define FASTDRAW_H
// ---------------------------------------------------------------------------
// Drawing into a GFXcanvas16 without Adafruit GFX's per-pixel machinery.
//
// On this chip a GFX line costs about 3 us a pixel and a size-2 character
// 200 us (every pixel goes through startWrite / writePixel / the virtual
// drawPixel / the rotation switch), so a log screen of 280 characters held
// the loop for 45-60 ms and the 3D scene's few thousand line pixels took
// 12-18 ms. The same store from a plain loop measures 40 ns (MagView's
// boot benchmark). These write straight into the canvas buffer: the same
// classic 5x7 font (GFX's own table, glcdfont.c), the same origin (top
// left) and 6 x 8 cell scaled by `size`, the same Bresenham as GFX's
// writeLine, so nothing on the screen moves. Clipped to the canvas;
// rotation 0 only (what the canvases here use).
// ---------------------------------------------------------------------------
#include <Adafruit_GFX.h>

// `text` with its top-left corner at (x, y), each font pixel a size x size
// block, in `colour` (RGB565), no background.
void fastText( GFXcanvas16* canvas, int x, int y, int size, uint16_t colour, const char* text );

// A one-pixel line from (x0, y0) to (x1, y1), ends included.
void fastLine( GFXcanvas16* canvas, int x0, int y0, int x1, int y1, uint16_t colour );

// A filled w x h rectangle with its top-left corner at (x, y), and its outline.
void fastFillRect( GFXcanvas16* canvas, int x, int y, int w, int h, uint16_t colour );
void fastRect( GFXcanvas16* canvas, int x, int y, int w, int h, uint16_t colour );

#endif // FASTDRAW_H
