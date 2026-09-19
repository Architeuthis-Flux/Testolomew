// SPDX-License-Identifier: MIT
#include "Menu.h"

void menuInit( Menu* m ) {
    m->count = 0;
    m->open = false;
    m->current = MENU_ROOT;
    m->cursor = 0;
    m->depth = 0;
    m->editing = false;
    m->editValue = 0.0f;
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

int menuAddAction( Menu* m, int parent, const char* label, int action, bool takesNumber ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_ACTION;
    item.parent = parent;
    item.action = action;
    item.takesNumber = takesNumber;
    item.min = 0.0f;
    item.max = 999999.0f;
    item.step = 1.0f;
    return add( m, item );
}

int menuAddToggle( Menu* m, int parent, const char* label, bool* flag ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_TOGGLE;
    item.parent = parent;
    item.flag = flag;
    return add( m, item );
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

int menuAddInfo( Menu* m, int parent, const char* label, MenuInfoText info ) {
    MenuItem item = { };
    item.label = label;
    item.kind = MENU_INFO;
    item.parent = parent;
    item.info = info;
    return add( m, item );
}

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

const char* menuTitle( const Menu* m ) {
    return m->current == MENU_ROOT ? "menu" : m->items[ m->current ].label;
}

void menuOpen( Menu* m ) {
    m->open = true;
    m->editing = false;
}

void menuClose( Menu* m ) {
    m->open = false;
    m->editing = false;
}

static float clampf( float v, float lo, float hi ) {
    return v < lo ? lo : ( v > hi ? hi : v );
}

// Up/down in an editor: a step, ten steps once the key has repeated a while.
static float stepFor( Menu* m, const MenuItem* item, bool repeat ) {
    m->repeats = repeat ? m->repeats + 1 : 0;
    return item->step * ( m->repeats > MENU_FAST_AFTER ? 10.0f : 1.0f );
}

int menuKey( Menu* m, MenuKey key, bool repeat, float* number ) {
    int visible = menuVisibleCount( m );
    if ( visible == 0 ) {
        if ( key == MENUKEY_BACK ) {
            menuClose( m );
        }
        return -1;
    }
    if ( m->cursor >= visible )
        m->cursor = visible - 1;
    int index = menuVisibleItem( m, m->cursor );
    MenuItem* item = &m->items[ index ];

    if ( m->editing ) {
        switch ( key ) {
        case MENUKEY_UP:
        case MENUKEY_RIGHT:
        case MENUKEY_DOWN:
        case MENUKEY_LEFT: {
            float direction = ( key == MENUKEY_UP || key == MENUKEY_RIGHT ) ? 1.0f : -1.0f;
            if ( item->kind == MENU_CHOICE ) {
                int n = *item->choice + ( direction > 0 ? 1 : -1 );
                *item->choice = ( n + item->choiceCount ) % item->choiceCount;
            } else if ( item->kind == MENU_NUMBER ) {
                *item->value = clampf( *item->value + direction * stepFor( m, item, repeat ), item->min, item->max );
            } else {
                m->editValue = clampf( m->editValue + direction * stepFor( m, item, repeat ), item->min, item->max );
            }
            return -1;
        }
        case MENUKEY_ENTER:
            m->editing = false;
            if ( item->kind == MENU_ACTION ) {
                *number = m->editValue;
                return item->action;
            }
            return -1;
        case MENUKEY_BACK:
            m->editing = false;
            return -1;
        }
        return -1;
    }

    switch ( key ) {
    case MENUKEY_UP:
        m->cursor = ( m->cursor + visible - 1 ) % visible;
        return -1;
    case MENUKEY_DOWN:
        m->cursor = ( m->cursor + 1 ) % visible;
        return -1;
    case MENUKEY_LEFT:
    case MENUKEY_BACK:
        if ( m->depth > 0 ) {
            m->depth--;
            m->current = m->stack[ m->depth ];
            m->cursor = m->stackCursor[ m->depth ];
        } else {
            menuClose( m );
        }
        return -1;
    case MENUKEY_RIGHT:
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
            if ( item->takesNumber ) {
                m->editing = true;
                m->editValue = clampf( m->editValue, item->min, item->max );
                m->repeats = 0;
                return -1;
            }
            return item->action;
        case MENU_TOGGLE:
            *item->flag = !*item->flag;
            return -1;
        case MENU_NUMBER:
        case MENU_CHOICE:
            m->editing = true;
            m->repeats = 0;
            return -1;
        case MENU_INFO:
            return -1;
        }
        return -1;
    }
    return -1;
}
