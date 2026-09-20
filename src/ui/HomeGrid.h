// SPDX-License-Identifier: MIT
#ifndef HOMEGRID_H
#define HOMEGRID_H
// ---------------------------------------------------------------------------
// The Home screen's cursor: a grid of HOME_COLUMNS across, the cells in
// reading order. Left/right step along the row and wrap within it; up/down
// move a row and stop at the edges (what a grid does everywhere else). Where each cell leads is the shell's
// business. No Arduino in here; host-tested.
// ---------------------------------------------------------------------------
#include <stdbool.h>

#include "Menu.h" // MenuKey

#define HOME_COLUMNS 3
#define HOME_MAX_CELLS 9

struct HomeGrid {
    int count;
    int cursor;
};

void homeInit( HomeGrid* h, int count );
// true if the cursor moved.
bool homeKey( HomeGrid* h, MenuKey key );

#endif // HOMEGRID_H
