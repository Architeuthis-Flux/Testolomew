// SPDX-License-Identifier: MIT
#ifndef UILAYOUT_H
#define UILAYOUT_H
// ---------------------------------------------------------------------------
// The numbers the screens share: text sizes (the classic 6 x 8 font scaled),
// how many rows the panels show, the Home grid's cell, and the colours.
// ---------------------------------------------------------------------------
#include "ST7789.h" // LCD_WIDTH, LCD_HEIGHT, RGB565

#define UI_TEXT 2 // size 2: 12 x 16 px characters, 20 to a line on a 240 px panel
#define UI_CHAR_W ( 6 * UI_TEXT )
#define UI_LINE_H ( 8 * UI_TEXT )
#define UI_COLUMNS ( LCD_WIDTH / UI_CHAR_W )
#define UI_MENU_ROWS 9   // items shown at once (18 px rows of size-2 text)
#define UI_LOG_TEXT 1    // the terminal's text size: 1 = 6 x 8 px characters, 40 to a line
#define UI_LOG_ROWS 28   // lines of it above the footer
#define UI_RESULT_ROWS 12 // size-1 lines in the result panel
#define UI_RESULT_COLS 37 // ...of this many characters ((LCD_WIDTH - 16) / 6)
#define UI_HELP_LINES 20  // the most lines a setting's help wraps to (the longest is about nine)
#define UI_HOME_CELL 72  // px per Home cell (three across)
#define UI_ICON_SCALE 2  // a 24 x 24 icon drawn 48 x 48

#define UI_COLOR_BACKGROUND RGB565( 0, 0, 0 )
#define UI_COLOR_PANEL RGB565( 12, 14, 22 )
#define UI_COLOR_FRAME RGB565( 90, 110, 150 )
#define UI_COLOR_TEXT RGB565( 220, 220, 220 )
#define UI_COLOR_DIM RGB565( 120, 120, 120 )
#define UI_COLOR_SELECTED RGB565( 255, 200, 60 )
#define UI_COLOR_ICON RGB565( 150, 190, 240 )
#define UI_COLOR_WARNING RGB565( 255, 140, 0 )

#endif // UILAYOUT_H
