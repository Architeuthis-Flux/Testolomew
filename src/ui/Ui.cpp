// SPDX-License-Identifier: MIT
#include "Ui.h"

#include <Adafruit_GFX.h>
#include <stdlib.h>
#include <string.h>

#include "Console.h"
#include "FastDraw.h"
#include "Input.h"
#include "UiLayout.h"
#include "UiStream.h"

Ui& ui = Ui::getInstance( );

Ui& Ui::getInstance( ) {
    static Ui instance;
    return instance;
}

static void onScreenVerb( int argc, char** argv, Stream* out );
static void onLogVerb( int argc, char** argv, Stream* out );
static void onUiVerb( int argc, char** argv, Stream* out );

void Ui::begin( const UiApp* apps, int appCount, int firstApp, int settingsCell ) {
    uiShellInit( &shell, apps, appCount, firstApp, settingsCell );
    shell.menuRows = UI_MENU_ROWS;
    consoleAddVerb( "screen", "", "what the screen shows, as text: app, panes, menu page and items, the modules' state", CONSOLE_READS, onScreenVerb );
    consoleAddVerb( "log", "[n]", "the last n lines of the log (20)", CONSOLE_READS, onLogVerb );
    consoleAddVerb( "ui", "open|menu|root|close|back|enter|hold|up|down|left|right|go <label>", "drive the screen: Home, Settings (menu: where it was last used; root: its top page), the panes (hold: the select held - a value tweaked over the app)", CONSOLE_CHANGES, onUiVerb );
}

void Ui::runAction( int index ) {
    if ( index < 0 || index >= shell.menu.count )
        return;
    const MenuItem& item = shell.menu.items[ index ];
    if ( item.run != nullptr ) {
        item.run( item.tag, item.argument );
    }
}

void Ui::showResult( const char* title ) {
    uiShellShowResult( &shell, title, uiStream.logCount( ), UI_RESULT_ROWS );
}

ServiceStatus Ui::service( ) {
    uint32_t nowUs = micros( );
    float dtS = lastUs == 0 ? 0.01f : ( nowUs - lastUs ) * 1e-6f;
    lastUs = nowUs;
    uint32_t now = millis( );

    input.uiWantsEnter = overlayOpen( );
    InputEvent e;
    bool anything = false;
    while ( input.next( &e ) ) {
        anything = true;
        int action = uiShellEvent( &shell, e, now );
        if ( action >= 0 ) {
            runAction( action );
        }
    }
    bool heldNow[ IN_CONTROL_COUNT ];
    uint32_t mask = input.heldMask( );
    for ( int c = 0; c < IN_CONTROL_COUNT; c++ ) {
        heldNow[ c ] = ( mask >> c ) & 1;
    }
    uiShellTick( &shell, now, dtS, input.joyX, input.joyY, input.joyRawX, input.joyRawY, heldNow );
    if ( shell.depth == 0 && ( input.joyX != 0.0f || input.joyY != 0.0f ) ) {
        anything = true;
    }
    lastStatus = anything ? ServiceStatus::BUSY : ServiceStatus::IDLE;
    return lastStatus;
}

// ---- stamps ------------------------------------------------------------------

uint32_t Ui::inputStamp( ) {
    const UiApp* app = uiShellApp( &shell );
    uint32_t g = app != nullptr && app->generation != nullptr ? app->generation( ) : 0;
    return shell.generation * 7919u + g * 3u + 1u;
}

uint32_t Ui::frameStamp( ) {
    const UiApp* app = uiShellApp( &shell );
    bool animated = app != nullptr && app->generation == nullptr;
    if ( animated && uiShellTop( &shell ) != PANE_HOME ) {
        return 0; // the app is moving under the panels: every frame
    }
    uint32_t stamp = inputStamp( );
    for ( int d = 0; d < shell.depth; d++ ) {
        if ( shell.stack[ d ] == PANE_RESULT ) {
            stamp += 104729u * uiStream.logGeneration( ); // the result shows the log as it grows
        }
    }
    return stamp == 0 ? 1 : stamp;
}

// ---- drawing -----------------------------------------------------------------

void uiDrawIcon( GFXcanvas16* canvas, int x, int y, const uint8_t* bits, int scale, uint16_t colour ) {
    if ( bits == nullptr )
        return;
    uint16_t* buffer = canvas->getBuffer( );
    for ( int row = 0; row < 24; row++ ) {
        for ( int col = 0; col < 24; col++ ) {
            if ( !( ( bits[ row * 3 + col / 8 ] >> ( 7 - col % 8 ) ) & 1 ) )
                continue;
            for ( int dy = 0; dy < scale; dy++ ) {
                int py = y + row * scale + dy;
                if ( py < 0 || py >= LCD_HEIGHT )
                    continue;
                for ( int dx = 0; dx < scale; dx++ ) {
                    int px = x + col * scale + dx;
                    if ( px >= 0 && px < LCD_WIDTH )
                        buffer[ py * LCD_WIDTH + px ] = colour;
                }
            }
        }
    }
}

void uiMenuItemValue( const Menu* m, int index, char* value, int size ) {
    const MenuItem& item = m->items[ index ];
    value[ 0 ] = '\0';
    switch ( item.kind ) {
    case MENU_SUBMENU:
        snprintf( value, size, ">" );
        break;
    case MENU_TOGGLE:
        snprintf( value, size, "%s", menuToggleGet( m, index ) ? item.onText : item.offText );
        break;
    case MENU_NUMBER:
        snprintf( value, size, item.step >= 1.0f ? "%.0f%s" : ( item.step >= 0.1f ? "%.1f%s" : "%.2f%s" ), *item.value, item.unit );
        break;
    case MENU_CHOICE:
        snprintf( value, size, "%s", item.names[ menuChoiceGet( m, index ) ] );
        break;
    case MENU_ACTION:
        if ( item.takesArgument ) {
            snprintf( value, size, "%.0f", item.argument );
        }
        break;
    case MENU_INFO:
        if ( item.info != nullptr )
            item.info( value, size );
        break;
    }
}

static const char* homeCellName( const UiShell* s, int cell ) {
    int app = uiShellHomeCellApp( s, cell );
    return app < 0 ? "Settings" : s->apps[ app ].name;
}

// Home: the apps as icons, three across, each named beneath its icon, the
// one under the cursor framed.
void Ui::drawHome( GFXcanvas16* canvas ) {
    canvas->fillScreen( UI_COLOR_BACKGROUND );
    int cells = shell.home.count;
    int rows = ( cells + HOME_COLUMNS - 1 ) / HOME_COLUMNS;
    int x0 = ( LCD_WIDTH - HOME_COLUMNS * UI_HOME_CELL ) / 2;
    int y0 = ( LCD_HEIGHT - rows * UI_HOME_CELL ) / 2;
    if ( y0 < 2 )
        y0 = 2;
    const int iconSize = 24 * UI_ICON_SCALE, labelH = 8;
    for ( int cell = 0; cell < cells; cell++ ) {
        int cx = x0 + ( cell % HOME_COLUMNS ) * UI_HOME_CELL;
        int cy = y0 + ( cell / HOME_COLUMNS ) * UI_HOME_CELL;
        bool selected = cell == shell.home.cursor;
        uint16_t colour = selected ? UI_COLOR_SELECTED : UI_COLOR_ICON;
        if ( selected ) {
            fastFillRect( canvas, cx + 2, cy + 2, UI_HOME_CELL - 4, UI_HOME_CELL - 4, UI_COLOR_PANEL );
            fastRect( canvas, cx + 2, cy + 2, UI_HOME_CELL - 4, UI_HOME_CELL - 4, UI_COLOR_SELECTED );
        }
        int app = uiShellHomeCellApp( &shell, cell );
        const uint8_t* icon = app < 0 ? shell.settingsIcon : shell.apps[ app ].icon;
        const char* name = homeCellName( &shell, cell );
        int ix = cx + ( UI_HOME_CELL - iconSize ) / 2, iy = cy + ( UI_HOME_CELL - iconSize - labelH - 4 ) / 2;
        if ( icon != nullptr ) {
            uiDrawIcon( canvas, ix, iy, icon, UI_ICON_SCALE, colour );
        } else {
            char initial[ 2 ] = { name[ 0 ], '\0' };
            fastText( canvas, cx + UI_HOME_CELL / 2 - 9, iy + iconSize / 2 - 12, 3, colour, initial );
        }
        // The name under the icon, size 1, centred, cut to the cell.
        char label[ 12 ];
        snprintf( label, sizeof( label ), "%.*s", ( UI_HOME_CELL - 4 ) / 6, name );
        int w = (int)strlen( label ) * 6;
        fastText( canvas, cx + ( UI_HOME_CELL - w ) / 2, iy + iconSize + 3, 1, selected ? UI_COLOR_SELECTED : UI_COLOR_TEXT, label );
    }
}

void Ui::drawMenu( GFXcanvas16* canvas ) {
    // Size-2 text: 12 px characters, 20 to a line; a panel of UI_MENU_ROWS
    // rows of 18 px with a title above and a one-line hint below.
    Menu& menu = shell.menu;
    const int T = UI_TEXT, charW = UI_CHAR_W, rowH = UI_LINE_H + 2;
    const int x0 = 4, y0 = 4, w = LCD_WIDTH - 8;
    const int columns = ( w - 8 ) / charW; // characters across the panel
    int visible = menuVisibleCount( &menu );
    int rows = visible < UI_MENU_ROWS ? visible : UI_MENU_ROWS;
    int h = rowH + 2 + rows * rowH + 12;
    fastFillRect( canvas, x0, y0, w, h, UI_COLOR_PANEL );
    fastRect( canvas, x0, y0, w, h, UI_COLOR_FRAME );
    fastText( canvas, x0 + 4, y0 + 3, T, UI_COLOR_FRAME, menuTitle( &menu ) );

    uiShellMenuWindow( &shell );
    int scrollTop = shell.menuScrollTop;

    char text[ 48 ];
    for ( int r = 0; r < rows; r++ ) {
        int n = scrollTop + r;
        int index = menuVisibleItem( &menu, n );
        if ( index < 0 )
            break;
        const MenuItem& item = menu.items[ index ];
        bool selected = n == menu.cursor;
        int y = y0 + rowH + 2 + r * rowH;
        uint16_t colour = selected ? UI_COLOR_SELECTED : UI_COLOR_TEXT;
        if ( selected ) {
            fastText( canvas, x0 + 4, y, T, colour, ">" );
        }
        char value[ 24 ] = "";
        uiMenuItemValue( &menu, index, value, sizeof( value ) );
        // The label gets what the value leaves: the value is right-aligned.
        int valueChars = (int)strlen( value );
        int labelChars = columns - 1 - valueChars - ( valueChars > 0 ? 1 : 0 );
        if ( labelChars < 4 )
            labelChars = 4;
        snprintf( text, sizeof( text ), "%.*s", labelChars, item.label );
        fastText( canvas, x0 + 4 + charW, y, T, colour, text );
        if ( value[ 0 ] ) {
            fastText( canvas, x0 + w - 4 - charW * valueChars, y, T, colour, value );
        }
    }
    if ( visible > rows ) {
        snprintf( text, sizeof( text ), "%d/%d", menu.cursor + 1, visible );
        fastText( canvas, x0 + w - 4 - charW * (int)strlen( text ), y0 + 3, T, UI_COLOR_DIM, text );
    }
    fastText( canvas, x0 + 4, y0 + h - 10, 1, UI_COLOR_DIM, "left/right: change  press: select  B: back" );
}

// A question in the middle: the item's label, yes or no.
void Ui::drawConfirm( GFXcanvas16* canvas ) {
    const int w = LCD_WIDTH - 24, h = 3 * UI_LINE_H + 16;
    const int x0 = 12, y0 = ( LCD_HEIGHT - h ) / 2;
    fastFillRect( canvas, x0, y0, w, h, UI_COLOR_PANEL );
    fastRect( canvas, x0, y0, w, h, UI_COLOR_SELECTED );
    const char* label = shell.confirmItem >= 0 && shell.confirmItem < shell.menu.count ? shell.menu.items[ shell.confirmItem ].label : "?";
    char text[ 24 ];
    snprintf( text, sizeof( text ), "%.*s", ( w - 8 ) / UI_CHAR_W, label );
    fastText( canvas, x0 + 4, y0 + 4, UI_TEXT, UI_COLOR_TEXT, text );
    fastText( canvas, x0 + 4, y0 + 4 + UI_LINE_H + 4, UI_TEXT, UI_COLOR_SELECTED, "really?" );
    fastText( canvas, x0 + 4, y0 + h - 12, 1, UI_COLOR_DIM, "press: yes   B: no" );
}

// The log's tail in a panel: what an action printed, live.
void Ui::drawResult( GFXcanvas16* canvas ) {
    const int T = UI_LOG_TEXT, H = 8 * UI_LOG_TEXT;
    const int x0 = 4, y0 = 4, w = LCD_WIDTH - 8;
    const int h = UI_LINE_H + 4 + UI_RESULT_ROWS * H + 14;
    fastFillRect( canvas, x0, y0, w, h, UI_COLOR_PANEL );
    fastRect( canvas, x0, y0, w, h, UI_COLOR_FRAME );
    fastText( canvas, x0 + 4, y0 + 3, UI_TEXT, UI_COLOR_FRAME, shell.resultTitle );
    int count = uiStream.logCount( );
    int rows = count < UI_RESULT_ROWS ? count : UI_RESULT_ROWS;
    int most = count - rows;
    int scroll = shell.resultScroll > most ? most : shell.resultScroll;
    for ( int r = 0; r < rows; r++ ) {
        int back = rows - 1 - r + scroll;
        const char* line = uiStream.logLine( back );
        if ( line == nullptr )
            continue;
        char text[ 40 ];
        snprintf( text, sizeof( text ), "%.*s", ( w - 8 ) / ( 6 * T ), line );
        fastText( canvas, x0 + 4, y0 + UI_LINE_H + 4 + r * H, T, back == 0 ? UI_COLOR_TEXT : UI_COLOR_DIM, text );
    }
    fastText( canvas, x0 + 4, y0 + h - 10, 1, UI_COLOR_DIM, scroll > 0 ? "up/down: scroll (older)  B: back" : "up/down: scroll  B: back" );
}

// A value tweaked over the app: one line along the bottom - the item's label
// and its value - and the hint under it. The menu beneath is not drawn.
void Ui::drawTweak( GFXcanvas16* canvas ) {
    Menu& menu = shell.menu;
    const int h = UI_LINE_H + 16;
    const int x0 = 4, y0 = LCD_HEIGHT - h - 4, w = LCD_WIDTH - 8;
    fastFillRect( canvas, x0, y0, w, h, UI_COLOR_PANEL );
    fastRect( canvas, x0, y0, w, h, UI_COLOR_SELECTED );
    int index = menuCursorItem( &menu );
    if ( index >= 0 ) {
        char value[ 24 ] = "", text[ 48 ];
        uiMenuItemValue( &menu, index, value, sizeof( value ) );
        const int columns = ( w - 8 ) / UI_CHAR_W;
        int valueChars = (int)strlen( value );
        int labelChars = columns - valueChars - ( valueChars > 0 ? 1 : 0 );
        if ( labelChars < 4 )
            labelChars = 4;
        snprintf( text, sizeof( text ), "%.*s", labelChars, menu.items[ index ].label );
        fastText( canvas, x0 + 4, y0 + 3, UI_TEXT, UI_COLOR_SELECTED, text );
        if ( value[ 0 ] )
            fastText( canvas, x0 + w - 4 - UI_CHAR_W * valueChars, y0 + 3, UI_TEXT, UI_COLOR_TEXT, value );
    }
    fastText( canvas, x0 + 4, y0 + h - 10, 1, UI_COLOR_DIM, "up/down: change  left/right: next  B: menu" );
}

void Ui::draw( GFXcanvas16* canvas ) {
    if ( uiShellTop( &shell ) == PANE_HOME ) {
        drawHome( canvas ); // opaque: nothing underneath is drawn
        return;
    }
    canvas->fillScreen( UI_COLOR_BACKGROUND );
    const UiApp* app = uiShellApp( &shell );
    if ( app != nullptr && app->draw != nullptr ) {
        app->draw( canvas );
    }
    for ( int d = 0; d < shell.depth; d++ ) {
        switch ( shell.stack[ d ] ) {
        case PANE_MENU:
            if ( d + 1 < shell.depth && shell.stack[ d + 1 ] == PANE_TWEAK )
                break; // hidden under the tweak: the app shows
            drawMenu( canvas );
            break;
        case PANE_TWEAK:
            drawTweak( canvas );
            break;
        case PANE_CONFIRM:
            drawConfirm( canvas );
            break;
        case PANE_RESULT:
            drawResult( canvas );
            break;
        default:
            break;
        }
    }
}

// ---- :screen, :log, :ui ------------------------------------------------------------

static const char* const paneNames[ 6 ] = { "app", "home", "menu", "confirm", "result", "tweak" };

void Ui::printScreen( Stream* out ) {
    char line[ 120 ];
    out->println( "screen{" );
    const UiApp* app = uiShellApp( &shell );
    snprintf( line, sizeof( line ), "app: %s", app != nullptr ? app->name : "-" );
    out->println( line );
    int n = snprintf( line, sizeof( line ), "panes:" );
    for ( int d = 0; d < shell.depth; d++ ) {
        n += snprintf( line + n, sizeof( line ) - n, " %s", paneNames[ shell.stack[ d ] ] );
    }
    if ( shell.depth == 0 )
        snprintf( line + n, sizeof( line ) - n, " -" );
    out->println( line );
    for ( int d = 0; d < shell.depth; d++ ) {
        if ( shell.stack[ d ] == PANE_HOME ) {
            snprintf( line, sizeof( line ), "home: cursor %d %s", shell.home.cursor, homeCellName( &shell, shell.home.cursor ) );
            out->println( line );
        } else if ( shell.stack[ d ] == PANE_MENU ) {
            Menu& menu = shell.menu;
            // The page's path from the root, and the cursor.
            char path[ 64 ] = "";
            for ( int k = 0; k < menu.depth; k++ ) {
                int page = k + 1 < menu.depth ? menu.stack[ k + 1 ] : menu.current;
                strncat( path, "/", sizeof( path ) - strlen( path ) - 1 );
                strncat( path, page == MENU_ROOT ? "" : menu.items[ page ].label, sizeof( path ) - strlen( path ) - 1 );
            }
            if ( menu.depth == 0 ) {
                strncpy( path, "/", sizeof( path ) );
            }
            int visible = menuVisibleCount( &menu );
            snprintf( line, sizeof( line ), "menu: %s cursor %d/%d", path, menu.cursor + 1, visible );
            out->println( line );
            for ( int k = 0; k < visible; k++ ) {
                int index = menuVisibleItem( &menu, k );
                if ( index < 0 )
                    break;
                char value[ 24 ];
                uiMenuItemValue( &menu, index, value, sizeof( value ) );
                snprintf( line, sizeof( line ), "%c %-24s %s", k == menu.cursor ? '>' : ' ', menu.items[ index ].label, value );
                out->println( line );
            }
        } else if ( shell.stack[ d ] == PANE_TWEAK ) {
            int index = menuCursorItem( &shell.menu );
            char value[ 24 ] = "";
            if ( index >= 0 )
                uiMenuItemValue( &shell.menu, index, value, sizeof( value ) );
            snprintf( line, sizeof( line ), "tweak: %s %s", index >= 0 ? shell.menu.items[ index ].label : "?", value );
            out->println( line );
        } else if ( shell.stack[ d ] == PANE_CONFIRM ) {
            snprintf( line, sizeof( line ), "confirm: %s", shell.confirmItem >= 0 ? shell.menu.items[ shell.confirmItem ].label : "?" );
            out->println( line );
        } else if ( shell.stack[ d ] == PANE_RESULT ) {
            snprintf( line, sizeof( line ), "result: %s scroll %d of %d lines", shell.resultTitle, shell.resultScroll, uiStream.logCount( ) );
            out->println( line );
        }
    }
    snprintf( line, sizeof( line ), "held: 0x%03lx swallow: 0x%03lx joy %s", (unsigned long)input.heldMask( ), (unsigned long)[]( const UiShell* s ) {
                  unsigned long m = 0;
                  for ( int c = 0; c < IN_CONTROL_COUNT; c++ )
                      if ( s->swallow[ c ] )
                          m |= 1ul << c;
                  return m;
              }( &shell ),
              shell.joyArmed ? "armed" : "waiting" );
    out->println( line );
    if ( screenExtra != nullptr ) {
        screenExtra( out );
    }
    out->println( "}" );
}

static void onScreenVerb( int argc, char** argv, Stream* out ) {
    (void)argc;
    (void)argv;
    bool was = uiStream.logToScreen;
    uiStream.logToScreen = false;
    ui.printScreen( out );
    uiStream.logToScreen = was;
}

static void onLogVerb( int argc, char** argv, Stream* out ) {
    int n = argc >= 2 ? atoi( argv[ 1 ] ) : 20;
    if ( n < 1 )
        n = 1;
    if ( n > uiStream.logCount( ) )
        n = uiStream.logCount( );
    bool was = uiStream.logToScreen;
    uiStream.logToScreen = false;
    out->println( "log{" );
    for ( int back = n - 1; back >= 0; back-- ) {
        const char* line = uiStream.logLine( back );
        out->println( line == nullptr ? "" : line );
    }
    out->println( "}" );
    uiStream.logToScreen = was;
}

static bool labelMatches( const char* label, const char* wanted ) {
    // A case-insensitive prefix, so "go track" finds "tracker".
    for ( int i = 0; wanted[ i ] != '\0'; i++ ) {
        char a = label[ i ], b = wanted[ i ];
        if ( a >= 'A' && a <= 'Z' )
            a = (char)( a - 'A' + 'a' );
        if ( b >= 'A' && b <= 'Z' )
            b = (char)( b - 'A' + 'a' );
        if ( a != b )
            return false;
    }
    return true;
}

// A control's tap or hold, straight into the shell (no emulated key: the
// answer is immediate), running whatever action comes back.
static void shellTap( InputControl c, bool holdIt ) {
    uint32_t now = millis( );
    int action = -1, r;
    r = uiShellEvent( &ui.shell, { c, IN_PRESS }, now );
    if ( r >= 0 )
        action = r;
    r = uiShellEvent( &ui.shell, { c, holdIt ? IN_HOLD : IN_CLICK }, now );
    if ( r >= 0 )
        action = r;
    r = uiShellEvent( &ui.shell, { c, IN_RELEASE }, now );
    if ( r >= 0 )
        action = r;
    if ( action >= 0 ) {
        const MenuItem& item = ui.shell.menu.items[ action ];
        if ( item.run != nullptr )
            item.run( item.tag, item.argument );
    }
}

static void onUiVerb( int argc, char** argv, Stream* out ) {
    if ( argc < 2 ) {
        consoleErr( out, "usage: :ui open|menu|root|close|back|enter|hold|up|down|left|right|go <label>" );
        return;
    }
    const char* what = argv[ 1 ];
    UiShell* s = &ui.shell;
    if ( strcmp( what, "open" ) == 0 ) {
        uiShellOpenHome( s );
        consoleOk( out, "home" );
    } else if ( strcmp( what, "menu" ) == 0 ) {
        uiShellOpenMenu( s );
        consoleOk( out, "menu" );
    } else if ( strcmp( what, "root" ) == 0 ) {
        uiShellOpenMenuRoot( s );
        consoleOk( out, "root" );
    } else if ( strcmp( what, "hold" ) == 0 ) {
        shellTap( IN_NAV_PRESS, true );
        consoleOk( out, "hold" );
    } else if ( strcmp( what, "close" ) == 0 ) {
        uiShellCloseAll( s );
        consoleOk( out, "closed" );
    } else if ( strcmp( what, "back" ) == 0 ) {
        shellTap( IN_BTN_B, false );
        consoleOk( out, "back" );
    } else if ( strcmp( what, "enter" ) == 0 ) {
        shellTap( IN_NAV_PRESS, false );
        consoleOk( out, "enter" );
    } else if ( strcmp( what, "up" ) == 0 ) {
        shellTap( IN_NAV_UP, false );
        consoleOk( out, "up" );
    } else if ( strcmp( what, "down" ) == 0 ) {
        shellTap( IN_NAV_DOWN, false );
        consoleOk( out, "down" );
    } else if ( strcmp( what, "left" ) == 0 ) {
        shellTap( IN_NAV_LEFT, false );
        consoleOk( out, "left" );
    } else if ( strcmp( what, "right" ) == 0 ) {
        shellTap( IN_NAV_RIGHT, false );
        consoleOk( out, "right" );
    } else if ( strcmp( what, "go" ) == 0 && argc >= 3 ) {
        // The label may be several words: join the rest of the line.
        char wanted[ 48 ] = "";
        for ( int i = 2; i < argc; i++ ) {
            if ( i > 2 )
                strncat( wanted, " ", sizeof( wanted ) - strlen( wanted ) - 1 );
            strncat( wanted, argv[ i ], sizeof( wanted ) - strlen( wanted ) - 1 );
        }
        char line[ 80 ];
        PaneKind top = uiShellTop( s );
        if ( top == PANE_HOME ) {
            for ( int cell = 0; cell < s->home.count; cell++ ) {
                if ( labelMatches( homeCellName( s, cell ), wanted ) ) {
                    s->home.cursor = cell;
                    snprintf( line, sizeof( line ), "go %s", homeCellName( s, cell ) );
                    shellTap( IN_NAV_PRESS, false );
                    consoleOk( out, line );
                    return;
                }
            }
            consoleErr( out, "no such app on Home (:screen lists the apps' names; 'settings' is one)" );
        } else if ( top == PANE_MENU ) {
            Menu* m = &s->menu;
            int visible = menuVisibleCount( m );
            for ( int n = 0; n < visible; n++ ) {
                int index = menuVisibleItem( m, n );
                if ( index >= 0 && labelMatches( m->items[ index ].label, wanted ) ) {
                    m->cursor = n;
                    snprintf( line, sizeof( line ), "go %s", m->items[ index ].label );
                    shellTap( IN_NAV_PRESS, false );
                    consoleOk( out, line );
                    return;
                }
            }
            consoleErr( out, "no item with that label on this page (:screen lists them)" );
        } else {
            consoleErr( out, "go works on Home or a menu page (:ui open, :ui menu)" );
        }
    } else {
        consoleErr( out, "usage: :ui open|menu|root|close|back|enter|hold|up|down|left|right|go <label>" );
    }
}
