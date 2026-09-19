// SPDX-License-Identifier: MIT
#ifndef MENU_H
#define MENU_H
// ---------------------------------------------------------------------------
// A menu tree for a small screen and a four-way switch: a flat table of
// items, each naming its parent, walked with up / down / enter / back. Item
// kinds: a SUBMENU, an ACTION (returns an id for the caller to run; one that
// takes a number pops a number editor first), a TOGGLE on a bool, a NUMBER
// edited in place with the up/down keys (repeats accelerate), a CHOICE among
// named values, and INFO (a line of live text). The drawing is elsewhere
// (Ui.cpp); this is only the state and what the keys do to it, so it can be
// tested on the host.
// ---------------------------------------------------------------------------
#include <stdbool.h>

#define MENU_MAX_ITEMS 128 // ~100 in use on 2026-09-18 (fixed pages + one per console command); a full table drops items, with a line at boot
#define MENU_MAX_DEPTH 4
#define MENU_ROOT -1
#define MENU_FAST_AFTER 8 // repeats before a number editor steps ten at a time

enum MenuKind {
    MENU_SUBMENU,
    MENU_ACTION,
    MENU_TOGGLE,
    MENU_NUMBER,
    MENU_CHOICE,
    MENU_INFO
};

enum MenuKey {
    MENUKEY_UP,
    MENUKEY_DOWN,
    MENUKEY_LEFT,
    MENUKEY_RIGHT,
    MENUKEY_ENTER,
    MENUKEY_BACK
};

typedef void ( *MenuInfoText )( char* buffer, int length );

struct MenuItem {
    const char* label;
    MenuKind kind;
    int parent;       // the submenu it lives in, MENU_ROOT at the top
    int action;       // ACTION: what to run (the caller's id, e.g. a console key)
    bool takesNumber; // ACTION: ask for a number first
    bool* flag;       // TOGGLE
    float* value;     // NUMBER
    float min, max, step;
    const char* unit;
    int* choice; // CHOICE
    const char* const* names;
    int choiceCount;
    MenuInfoText info; // INFO
};

struct Menu {
    MenuItem items[ MENU_MAX_ITEMS ];
    int count;
    bool open;
    int current;                 // the submenu shown
    int cursor;                  // which visible item is selected (0-based)
    int stack[ MENU_MAX_DEPTH ]; // submenus entered, with the cursor to come back to
    int stackCursor[ MENU_MAX_DEPTH ];
    int depth;
    bool editing; // a NUMBER / CHOICE / ACTION-number being edited
    float editValue;
    int repeats; // repeats in a row in the editor
};

void menuInit( Menu* m );

// Builders; each returns the item's index (a SUBMENU's index is the parent
// for what goes in it), or -1 if the table is full.
int menuAddSubmenu( Menu* m, int parent, const char* label );
int menuAddAction( Menu* m, int parent, const char* label, int action, bool takesNumber );
int menuAddToggle( Menu* m, int parent, const char* label, bool* flag );
int menuAddNumber( Menu* m, int parent, const char* label, float* value, float min, float max, float step, const char* unit );
int menuAddChoice( Menu* m, int parent, const char* label, int* choice, const char* const* names, int count );
int menuAddInfo( Menu* m, int parent, const char* label, MenuInfoText info );

// The visible list: how many items the current submenu has, and the index
// of its n-th.
int menuVisibleCount( const Menu* m );
int menuVisibleItem( const Menu* m, int n );
const char* menuTitle( const Menu* m ); // the current submenu's label ("menu" at the root)

// A key. `repeat` marks an auto-repeat while held. Returns an ACTION's id to
// run, with *number set if it took one, or -1 if nothing is to be run.
int menuKey( Menu* m, MenuKey key, bool repeat, float* number );

void menuOpen( Menu* m );
void menuClose( Menu* m );

#endif // MENU_H
