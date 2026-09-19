// SPDX-License-Identifier: MIT
#include "Menu.h"

void menuInit( Menu* m ) {
    m->count = 0;
    m->current = MENU_ROOT;
    m->cursor = 0;
    m->depth = 0;
    m->repeats = 0;
}

static int add( Menu* m, MenuItem item ) {
    if ( m->count >= MENU_MAX_ITEMS ) {
        return -1;
    }
    m->items[ m->count ] = item;
    return m->count++;
}

int menuAddSubmenu( Menu* m, int parent, const char* label ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_SUBMENU;
    item.parent = parent;
    return add( m, item );
}

int menuAddAction( Menu* m, int parent, const char* label, int tag, MenuCallback run, bool confirm ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_ACTION;
    item.parent = parent;
    item.tag = tag;
    item.run = run;
    item.confirm = confirm;
    return add( m, item );
}

int menuAddNumberAction( Menu* m, int parent, const char* label, int tag, MenuCallback run, float min, float max, float step, float start ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_ACTION;
    item.parent = parent;
    item.tag = tag;
    item.run = run;
    item.takesArgument = true;
    item.argument = start < min ? min : ( start > max ? max : start );
    item.min = min;
    item.max = max;
    item.step = step;
    item.unit = "";
    return add( m, item );
}

int menuAddToggle( Menu* m, int parent, const char* label, bool* flag ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_TOGGLE;
    item.parent = parent;
    item.flag = flag;
    item.onText = "on";
    item.offText = "off";
    return add( m, item );
}

int menuAddToggleAccessor( Menu* m, int parent, const char* label, MenuGetFlag get, MenuSetFlag set ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_TOGGLE;
    item.parent = parent;
    item.getFlag = get;
    item.setFlag = set;
    item.onText = "on";
    item.offText = "off";
    return add( m, item );
}

void menuSetToggleText( Menu* m, int index, const char* onText, const char* offText ) {
    if ( index >= 0 && index < m->count ) {
        m->items[ index ].onText = onText;
        m->items[ index ].offText = offText;
    }
}

int menuAddNumber( Menu* m, int parent, const char* label, float* value, float min, float max, float step, const char* unit ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_NUMBER;
    item.parent = parent;
    item.value = value;
    item.min = min;
    item.max = max;
    item.step = step;
    item.unit = unit;
    return add( m, item );
}

int menuAddChoice( Menu* m, int parent, const char* label, int* choice, const char* const* names, int count ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_CHOICE;
    item.parent = parent;
    item.choice = choice;
    item.names = names;
    item.choiceCount = count;
    return add( m, item );
}

int menuAddChoiceAccessor( Menu* m, int parent, const char* label, MenuGetChoice get, MenuSetChoice set, const char* const* names, int count ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_CHOICE;
    item.parent = parent;
    item.getChoice = get;
    item.setChoice = set;
    item.names = names;
    item.choiceCount = count;
    return add( m, item );
}

int menuAddInfo( Menu* m, int parent, const char* label, MenuInfoText info ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_INFO;
    item.parent = parent;
    item.info = info;
    return add( m, item );
}

int menuFree( const Menu* m ) {
    return MENU_MAX_ITEMS - m->count;
}

// ---- the bound values -----------------------------------------------------------

bool menuToggleGet( const Menu* m, int index ) {
    const MenuItem& item = m->items[ index ];
    if ( item.flag != nullptr )
        return *item.flag;
    return item.getFlag != nullptr ? item.getFlag( ) : false;
}

void menuToggleSet( Menu* m, int index, bool on ) {
    MenuItem& item = m->items[ index ];
    if ( menuToggleGet( m, index ) == on )
        return;
    if ( item.flag != nullptr ) {
        *item.flag = on;
    } else if ( item.setFlag != nullptr ) {
        item.setFlag( on );
    }
}

int menuChoiceGet( const Menu* m, int index ) {
    const MenuItem& item = m->items[ index ];
    int v = item.choice != nullptr ? *item.choice : ( item.getChoice != nullptr ? item.getChoice( ) : 0 );
    if ( item.choiceCount > 0 ) {
        v = ( ( v % item.choiceCount ) + item.choiceCount ) % item.choiceCount;
    }
    return v;
}

void menuChoiceSet( Menu* m, int index, int choice ) {
    MenuItem& item = m->items[ index ];
    if ( item.choiceCount > 0 ) {
        choice = ( ( choice % item.choiceCount ) + item.choiceCount ) % item.choiceCount;
    }
    if ( menuChoiceGet( m, index ) == choice )
        return;
    if ( item.choice != nullptr ) {
        *item.choice = choice;
    } else if ( item.setChoice != nullptr ) {
        item.setChoice( choice );
    }
}

// ---- the page --------------------------------------------------------------------

int menuVisibleCount( const Menu* m ) {
    int n = 0;
    for ( int i = 0; i < m->count; i++ ) {
        n += m->items[ i ].parent == m->current;
    }
    return n;
}

int menuVisibleItem( const Menu* m, int wanted ) {
    int n = 0;
    for ( int i = 0; i < m->count; i++ ) {
        if ( m->items[ i ].parent == m->current ) {
            if ( n == wanted )
                return i;
            n++;
        }
    }
    return -1;
}

int menuCursorItem( const Menu* m ) {
    return menuVisibleItem( m, m->cursor );
}

const char* menuTitle( const Menu* m ) {
    return m->current == MENU_ROOT ? "menu" : m->items[ m->current ].label;
}

void menuHome( Menu* m ) {
    m->current = MENU_ROOT;
    m->cursor = 0;
    m->depth = 0;
    m->repeats = 0;
}

static float clampf( float v, float lo, float hi ) {
    return v < lo ? lo : ( v > hi ? hi : v );
}

// Left/right on a number: a step, ten steps once the key has repeated a while.
static float stepFor( Menu* m, const MenuItem* item, bool repeat ) {
    m->repeats = repeat ? m->repeats + 1 : 0;
    return item->step * ( m->repeats > MENU_FAST_AFTER ? 10.0f : 1.0f );
}

int menuKey( Menu* m, MenuKey key, bool repeat ) {
    if ( !repeat && key != MENUKEY_LEFT && key != MENUKEY_RIGHT ) {
        m->repeats = 0;
    }
    int visible = menuVisibleCount( m );
    if ( visible == 0 ) {
        if ( key == MENUKEY_BACK ) {
            if ( m->depth == 0 )
                return MENU_AT_ROOT;
            m->depth--;
            m->current = m->stack[ m->depth ];
            m->cursor = m->stackCursor[ m->depth ];
        }
        return -1;
    }
    if ( m->cursor >= visible )
        m->cursor = visible - 1;
    int index = menuVisibleItem( m, m->cursor );
    MenuItem* item = &m->items[ index ];

    switch ( key ) {
    case MENUKEY_UP:
        m->cursor = ( m->cursor + visible - 1 ) % visible;
        return -1;
    case MENUKEY_DOWN:
        m->cursor = ( m->cursor + 1 ) % visible;
        return -1;
    case MENUKEY_LEFT:
    case MENUKEY_RIGHT: {
        // The value in place.
        int direction = key == MENUKEY_RIGHT ? 1 : -1;
        switch ( item->kind ) {
        case MENU_TOGGLE:
            menuToggleSet( m, index, !menuToggleGet( m, index ) );
            break;
        case MENU_CHOICE:
            menuChoiceSet( m, index, menuChoiceGet( m, index ) + direction );
            break;
        case MENU_NUMBER:
            *item->value = clampf( *item->value + direction * stepFor( m, item, repeat ), item->min, item->max );
            break;
        case MENU_ACTION:
            if ( item->takesArgument ) {
                item->argument = clampf( item->argument + direction * stepFor( m, item, repeat ), item->min, item->max );
            }
            break;
        default:
            break;
        }
        return -1;
    }
    case MENUKEY_BACK:
        if ( m->depth == 0 ) {
            return MENU_AT_ROOT;
        }
        m->depth--;
        m->current = m->stack[ m->depth ];
        m->cursor = m->stackCursor[ m->depth ];
        return -1;
    case MENUKEY_ENTER:
        switch ( item->kind ) {
        case MENU_SUBMENU:
            if ( m->depth < MENU_MAX_DEPTH ) {
                m->stack[ m->depth ] = m->current;
                m->stackCursor[ m->depth ] = m->cursor;
                m->depth++;
                m->current = index;
                m->cursor = 0;
            }
            return -1;
        case MENU_ACTION:
            return index;
        case MENU_TOGGLE:
            menuToggleSet( m, index, !menuToggleGet( m, index ) );
            return -1;
        case MENU_CHOICE:
            menuChoiceSet( m, index, menuChoiceGet( m, index ) + 1 );
            return -1;
        default:
            return -1;
        }
    }
    return -1;
}
