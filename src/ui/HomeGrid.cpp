// SPDX-License-Identifier: MIT
#include "HomeGrid.h"

void homeInit( HomeGrid* h, int count ) {
    h->count = count < 0 ? 0 : ( count > HOME_MAX_CELLS ? HOME_MAX_CELLS : count );
    h->cursor = 0;
}

bool homeKey( HomeGrid* h, MenuKey key ) {
    if ( h->count == 0 )
        return false;
    int was = h->cursor;
    switch ( key ) {
    case MENUKEY_LEFT:
    case MENUKEY_RIGHT: {
        int column = h->cursor % HOME_COLUMNS;
        int rowStart = h->cursor - column;
        int rowCells = h->count - rowStart < HOME_COLUMNS ? h->count - rowStart : HOME_COLUMNS;
        column = ( column + ( key == MENUKEY_RIGHT ? 1 : rowCells - 1 ) ) % rowCells;
        h->cursor = rowStart + column;
        break;
    }
    case MENUKEY_UP:
        if ( h->cursor >= HOME_COLUMNS )
            h->cursor -= HOME_COLUMNS;
        break;
    case MENUKEY_DOWN:
        if ( h->cursor + HOME_COLUMNS < h->count )
            h->cursor += HOME_COLUMNS;
        break;
    default:
        break;
    }
    return h->cursor != was;
}
