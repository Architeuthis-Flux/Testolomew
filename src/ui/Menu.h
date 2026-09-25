// SPDX-License-Identifier: MIT
#ifndef MENU_H
#define MENU_H
// ---------------------------------------------------------------------------
// A menu tree for a small screen and a four-way switch: a flat table of
// items, each naming its parent, walked with up / down and entered with
// the press; left / right CHANGE THE VALUE IN PLACE - a toggle flips, a
// choice cycles, a number steps (ten steps at a time once the key has
// repeated a while), an action that takes a number steps its argument -
// and back pops to the parent page with its cursor as it was. There is no
// edit mode and nothing to confirm or cancel: what is shown is what is set.
//
// Item kinds: a SUBMENU; an ACTION (the press returns its index; the
// caller runs its callback with its tag and argument, first asking yes/no
// if it is marked confirm); a TOGGLE and a CHOICE, each bound either to a
// plain variable or to a get/set pair (for a value that lives in a module
// and has side effects: the setter compares before it writes); a NUMBER
// bound to a float; and INFO (a line of live text). menuToggleGet/Set and
// menuChoiceGet/Set are the only readers and writers of those two kinds,
// whichever way they are bound - the drawing, the keys and the settings
// module all go through them.
//
// The drawing is elsewhere (Ui.cpp); this is only the state and what the
// keys do to it, so it can be tested on the host.
// ---------------------------------------------------------------------------
#include <stdbool.h>

#define MENU_MAX_ITEMS 128 // ~110 in use on 2026-09-19 (fixed pages + one per console command); a full table drops items, with a line at boot
#define MENU_MAX_DEPTH 4
#define MENU_ROOT -1
#define MENU_AT_ROOT -2   // menuKey's answer to BACK at the root: the caller closes the menu
#define MENU_FAST_AFTER 8 // repeats before a number steps ten at a time

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
typedef void ( *MenuCallback )( int tag, float argument ); // an ACTION: run with its tag (a console key, an id) and its argument
typedef bool ( *MenuGetFlag )( );
typedef void ( *MenuSetFlag )( bool on );
typedef int ( *MenuGetChoice )( );
typedef void ( *MenuSetChoice )( int choice );

struct MenuItem {
    const char* label;
    MenuKind kind;
    int parent; // the submenu it lives in, MENU_ROOT at the top
    // ACTION
    int tag;
    MenuCallback run;
    bool confirm;       // ask yes/no before running
    bool takesArgument; // a number, stepped in place with left/right (min/max/step below), passed to run()
    float argument;
    // TOGGLE: one of the two bindings
    bool* flag;
    MenuGetFlag getFlag;
    MenuSetFlag setFlag;
    const char* onText; // how the two states read ("on"/"off" unless set)
    const char* offText;
    // NUMBER (and an ACTION's argument)
    float* value;
    float min, max, step;
    const char* unit;
    // CHOICE: one of the two bindings
    int* choice;
    MenuGetChoice getChoice;
    MenuSetChoice setChoice;
    const char* const* names;
    int choiceCount;
    // INFO
    MenuInfoText info;
};

struct Menu {
    MenuItem items[ MENU_MAX_ITEMS ];
    int count;
    int current;                 // the submenu shown
    int cursor;                  // which visible item is selected (0-based)
    int stack[ MENU_MAX_DEPTH ]; // submenus entered, with the cursor to come back to
    int stackCursor[ MENU_MAX_DEPTH ];
    int depth;
    int repeats;    // repeats in a row on a number (the fast step)
    int lastPage;   // the page last used (a submenu's index; MENU_ROOT = none yet): the menu opens there again (menuResume)
    int lastCursor; // ...and its cursor
};

void menuInit( Menu* m );

// Builders; each returns the item's index (a SUBMENU's index is the parent
// for what goes in it), or -1 if the table is full.
int menuAddSubmenu( Menu* m, int parent, const char* label );
int menuAddAction( Menu* m, int parent, const char* label, int tag, MenuCallback run, bool confirm );
int menuAddNumberAction( Menu* m, int parent, const char* label, int tag, MenuCallback run, float min, float max, float step, float start );
int menuAddToggle( Menu* m, int parent, const char* label, bool* flag );
int menuAddToggleAccessor( Menu* m, int parent, const char* label, MenuGetFlag get, MenuSetFlag set );
void menuSetToggleText( Menu* m, int index, const char* onText, const char* offText );
int menuAddNumber( Menu* m, int parent, const char* label, float* value, float min, float max, float step, const char* unit );
int menuAddChoice( Menu* m, int parent, const char* label, int* choice, const char* const* names, int count );
int menuAddChoiceAccessor( Menu* m, int parent, const char* label, MenuGetChoice get, MenuSetChoice set, const char* const* names, int count );
int menuAddInfo( Menu* m, int parent, const char* label, MenuInfoText info );
int menuFree( const Menu* m ); // items the table has room for

// The toggle and choice values, whichever way the item is bound. The
// setters write only what differs (an accessor with side effects is not
// run for nothing); a choice is wrapped into range.
bool menuToggleGet( const Menu* m, int index );
void menuToggleSet( Menu* m, int index, bool on );
int menuChoiceGet( const Menu* m, int index );
void menuChoiceSet( Menu* m, int index, int choice );

// The visible list: how many items the current submenu has, and the index
// of its n-th.
int menuVisibleCount( const Menu* m );
int menuVisibleItem( const Menu* m, int n );
int menuCursorItem( const Menu* m );     // the item under the cursor (-1 if the page is empty)
const char* menuTitle( const Menu* m ); // the current submenu's label ("menu" at the root)

// A key. `repeat` marks an auto-repeat while held. Returns an ACTION's item
// index to run (the caller runs it, after a confirmation if item.confirm),
// MENU_AT_ROOT for BACK at the root (the caller closes the menu), else -1.
int menuKey( Menu* m, MenuKey key, bool repeat );

// Back to the root page, cursor at the top.
void menuHome( Menu* m );
// At the root with a page remembered (the last one a key was used on, or
// entered): back into it with its cursor as it was, the stack rebuilt from
// its parents, so the menu opens where it was last used. Nothing otherwise.
void menuResume( Menu* m );
// Note the page and cursor in use (menuKey does; the shell's own steering
// of the cursor does too).
void menuRemember( Menu* m );

#endif // MENU_H
