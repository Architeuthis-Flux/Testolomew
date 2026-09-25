// SPDX-License-Identifier: MIT
// Host-side test of the UI's pure parts: the menu tree (walking it with the
// four-way keys, editing values in place, accessor bindings, actions and
// their arguments) and the camera (modes, the controls, the POV geometry - the camera
// at the probe's point looking down its shaft - and the gliding).
// Run with `pio test -e native`.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "Camera.h"
#include "Menu.h"
#include "UiShell.h"

static Menu menu;
static bool flag;
static float number;
static int choice;
static const char* const choiceNames[ 3 ] = { "under", "pointed", "magnet" };
// An accessor-bound toggle and choice: a module's value with a side effect.
static bool moduleFlag;
static int moduleFlagSets;
static bool lastFlagSet;
static bool getModuleFlag( ) {
    return moduleFlag;
}
static void setModuleFlag( bool on ) {
    moduleFlagSets++;
    lastFlagSet = on;
    moduleFlag = on;
}
static int moduleChoice;
static int moduleChoiceSets;
static int getModuleChoice( ) {
    return moduleChoice;
}
static void setModuleChoice( int c ) {
    moduleChoiceSets++;
    moduleChoice = c;
}
static int ran, ranTag;
static float ranArgument;
static void run( int tag, float argument ) {
    ran++;
    ranTag = tag;
    ranArgument = argument;
}

void setUp( void ) {
    menuInit( &menu );
    flag = false;
    number = 10.0f;
    choice = 0;
    moduleFlag = false;
    moduleFlagSets = 0;
    moduleChoice = 1;
    moduleChoiceSets = 0;
    ran = 0;
}
void tearDown( void ) {}

static int settingsPage, commandsPage, anchorItem, layoutItem, cursorItem;

static void buildMenu( ) {
    settingsPage = menuAddSubmenu( &menu, MENU_ROOT, "settings" );
    menuAddToggle( &menu, settingsPage, "tracker", &flag );
    menuAddNumber( &menu, settingsPage, "surface", &number, 0.0f, 100.0f, 0.5f, "mm" );
    menuAddChoice( &menu, settingsPage, "cursor", &choice, choiceNames, 3 );
    layoutItem = menuAddToggleAccessor( &menu, settingsPage, "layout", getModuleFlag, setModuleFlag );
    menuSetToggleText( &menu, layoutItem, "V5", "V6" );
    cursorItem = menuAddChoiceAccessor( &menu, settingsPage, "mode", getModuleChoice, setModuleChoice, choiceNames, 3 );
    commandsPage = menuAddSubmenu( &menu, MENU_ROOT, "commands" );
    menuAddAction( &menu, commandsPage, "l  latest fix", 'l', run, false );
    anchorItem = menuAddNumberAction( &menu, commandsPage, "anchor at row", 'R', run, 1.0f, 60.0f, 1.0f, 1.0f );
    menuAddAction( &menu, MENU_ROOT, "reset settings", 'Z', run, true );
}

void test_menu_walks_the_tree( void ) {
    buildMenu( );
    TEST_ASSERT_EQUAL( 3, menuVisibleCount( &menu ) );
    TEST_ASSERT_EQUAL_STRING( "menu", menuTitle( &menu ) );
    TEST_ASSERT_EQUAL( -1, menuKey( &menu, MENUKEY_ENTER, false ) ); // into settings
    TEST_ASSERT_EQUAL_STRING( "settings", menuTitle( &menu ) );
    TEST_ASSERT_EQUAL( 5, menuVisibleCount( &menu ) );
    menuKey( &menu, MENUKEY_ENTER, false ); // the press flips a toggle
    TEST_ASSERT_TRUE( flag );
    menuKey( &menu, MENUKEY_UP, false ); // wraps to the last
    TEST_ASSERT_EQUAL( 4, menu.cursor );
    menuKey( &menu, MENUKEY_DOWN, false ); // and round again
    TEST_ASSERT_EQUAL( 0, menu.cursor );
    menuKey( &menu, MENUKEY_DOWN, false );
    TEST_ASSERT_EQUAL( -1, menuKey( &menu, MENUKEY_BACK, false ) ); // up a level, the parent's cursor restored
    TEST_ASSERT_EQUAL_STRING( "menu", menuTitle( &menu ) );
    TEST_ASSERT_EQUAL( 0, menu.cursor );
    TEST_ASSERT_EQUAL( MENU_AT_ROOT, menuKey( &menu, MENUKEY_BACK, false ) ); // at the root: the caller closes
    TEST_ASSERT_EQUAL( MENU_MAX_ITEMS - menu.count, menuFree( &menu ) );
}

// Left and right change the value under the cursor, at once, with no mode.
void test_menu_edits_in_place( void ) {
    buildMenu( );
    menuKey( &menu, MENUKEY_ENTER, false ); // settings
    menuKey( &menu, MENUKEY_RIGHT, false ); // tracker: right flips it
    TEST_ASSERT_TRUE( flag );
    menuKey( &menu, MENUKEY_LEFT, false ); // and left flips it back
    TEST_ASSERT_FALSE( flag );
    menuKey( &menu, MENUKEY_DOWN, false ); // surface
    menuKey( &menu, MENUKEY_RIGHT, false );
    menuKey( &menu, MENUKEY_RIGHT, false );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 11.0f, number );
    menuKey( &menu, MENUKEY_LEFT, false );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 10.5f, number );
    for ( int k = 0; k < 12; k++ )
        menuKey( &menu, MENUKEY_RIGHT, true ); // held: after 8 repeats it steps ten at a time
    TEST_ASSERT_TRUE( number > 10.5f + 12 * 0.5f );
    for ( int k = 0; k < 400; k++ )
        menuKey( &menu, MENUKEY_RIGHT, true );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 100.0f, number ); // clamped
    for ( int k = 0; k < 400; k++ )
        menuKey( &menu, MENUKEY_LEFT, true );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.0f, number ); // clamped the other way
    menuKey( &menu, MENUKEY_DOWN, false ); // cursor (a choice)
    menuKey( &menu, MENUKEY_LEFT, false ); // wraps backwards
    TEST_ASSERT_EQUAL( 2, choice );
    menuKey( &menu, MENUKEY_RIGHT, false );
    TEST_ASSERT_EQUAL( 0, choice );
    menuKey( &menu, MENUKEY_ENTER, false ); // the press cycles too
    TEST_ASSERT_EQUAL( 1, choice );
    menuKey( &menu, MENUKEY_UP, false ); // up/down never change a value
    TEST_ASSERT_EQUAL( 1, choice );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.0f, number );
}

// An accessor-bound item: the setter runs once, with the new value, and
// not at all for a value that is already so.
void test_menu_accessor_set_once_with_the_flipped_value( void ) {
    buildMenu( );
    menuKey( &menu, MENUKEY_ENTER, false ); // settings
    for ( int k = 0; k < 3; k++ )
        menuKey( &menu, MENUKEY_DOWN, false ); // layout
    TEST_ASSERT_EQUAL( layoutItem, menuCursorItem( &menu ) );
    TEST_ASSERT_FALSE( menuToggleGet( &menu, layoutItem ) );
    menuKey( &menu, MENUKEY_RIGHT, false );
    TEST_ASSERT_EQUAL( 1, moduleFlagSets );
    TEST_ASSERT_TRUE( lastFlagSet );
    TEST_ASSERT_TRUE( menuToggleGet( &menu, layoutItem ) );
    menuToggleSet( &menu, layoutItem, true ); // already so: not written
    TEST_ASSERT_EQUAL( 1, moduleFlagSets );
    menuToggleSet( &menu, layoutItem, false );
    TEST_ASSERT_EQUAL( 2, moduleFlagSets );
    TEST_ASSERT_FALSE( moduleFlag );
    menuKey( &menu, MENUKEY_DOWN, false ); // mode (an accessor choice)
    TEST_ASSERT_EQUAL( 1, menuChoiceGet( &menu, cursorItem ) );
    menuKey( &menu, MENUKEY_RIGHT, false );
    TEST_ASSERT_EQUAL( 2, moduleChoice );
    TEST_ASSERT_EQUAL( 1, moduleChoiceSets );
    menuKey( &menu, MENUKEY_RIGHT, false ); // wraps
    TEST_ASSERT_EQUAL( 0, moduleChoice );
    menuChoiceSet( &menu, cursorItem, 3 ); // 3 wraps to 0: already so, not written
    TEST_ASSERT_EQUAL( 2, moduleChoiceSets );
    menuChoiceSet( &menu, cursorItem, -1 ); // -1 wraps to 2
    TEST_ASSERT_EQUAL( 2, moduleChoice );
}

// An action is returned to the caller (with its confirm flag), and one
// that takes a number has it stepped in place beforehand.
void test_menu_returns_actions_with_their_argument( void ) {
    buildMenu( );
    menuKey( &menu, MENUKEY_DOWN, false ); // commands
    menuKey( &menu, MENUKEY_ENTER, false );
    int l = menuKey( &menu, MENUKEY_ENTER, false );
    TEST_ASSERT_TRUE( l >= 0 );
    TEST_ASSERT_EQUAL( 'l', menu.items[ l ].tag );
    TEST_ASSERT_FALSE( menu.items[ l ].confirm );
    menu.items[ l ].run( menu.items[ l ].tag, menu.items[ l ].argument );
    TEST_ASSERT_EQUAL( 1, ran );
    TEST_ASSERT_EQUAL( 'l', ranTag );
    menuKey( &menu, MENUKEY_DOWN, false ); // anchor at row
    TEST_ASSERT_EQUAL( -1, menuKey( &menu, MENUKEY_LEFT, false ) ); // 1 is the floor
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 1.0f, menu.items[ anchorItem ].argument );
    for ( int k = 0; k < 29; k++ )
        menuKey( &menu, MENUKEY_RIGHT, false );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 30.0f, menu.items[ anchorItem ].argument );
    int r = menuKey( &menu, MENUKEY_ENTER, false );
    TEST_ASSERT_EQUAL( anchorItem, r );
    menu.items[ r ].run( menu.items[ r ].tag, menu.items[ r ].argument );
    TEST_ASSERT_EQUAL( 'R', ranTag );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 30.0f, ranArgument );
    // The destructive one at the root says it wants a confirmation.
    menuKey( &menu, MENUKEY_BACK, false );
    menuKey( &menu, MENUKEY_DOWN, false ); // reset settings
    int z = menuKey( &menu, MENUKEY_ENTER, false );
    TEST_ASSERT_EQUAL( 'Z', menu.items[ z ].tag );
    TEST_ASSERT_TRUE( menu.items[ z ].confirm );
    TEST_ASSERT_EQUAL( 2, ran ); // the two the test ran itself: the menu runs nothing
}


// ---- the shell ------------------------------------------------------------------

// Two fake apps that count what reaches them.
struct FakeApp {
    int enters, exits, events, ticks;
    float lastJoyX, lastJoyY;
    InputEvent last;
};
static FakeApp fakes[ 2 ];
static void enter0( ) { fakes[ 0 ].enters++; }
static void exit0( ) { fakes[ 0 ].exits++; }
static void tick0( float, float x, float y ) { fakes[ 0 ].ticks++; fakes[ 0 ].lastJoyX = x; fakes[ 0 ].lastJoyY = y; }
static bool event0( const InputEvent* e ) { fakes[ 0 ].events++; fakes[ 0 ].last = *e; return true; }
static void enter1( ) { fakes[ 1 ].enters++; }
static void exit1( ) { fakes[ 1 ].exits++; }
static void tick1( float, float x, float y ) { fakes[ 1 ].ticks++; fakes[ 1 ].lastJoyX = x; fakes[ 1 ].lastJoyY = y; }
static bool event1( const InputEvent* e ) { fakes[ 1 ].events++; fakes[ 1 ].last = *e; return true; }
static const UiApp fakeApps[ 2 ] = {
    { "View", nullptr, enter0, exit0, tick0, nullptr, event0, nullptr },
    { "LEDs", nullptr, enter1, exit1, tick1, nullptr, event1, nullptr },
};
static UiShell shell;
static uint32_t t;
static bool heldNow[ IN_CONTROL_COUNT ];

static void shellSetUp( ) {
    for ( int k = 0; k < 2; k++ ) {
        FakeApp z = { };
        fakes[ k ] = z;
    }
    for ( int c = 0; c < IN_CONTROL_COUNT; c++ )
        heldNow[ c ] = false;
    t = 1000;
    uiShellInit( &shell, fakeApps, 2, 0, 1 ); // Home: View, Settings, LEDs
    // A settings menu with a page, a toggle and a destructive action.
    ran = 0;
    int page = menuAddSubmenu( &shell.menu, MENU_ROOT, "tracker" );
    menuAddToggle( &shell.menu, page, "tracker on", &flag );
    menuAddNumber( &shell.menu, page, "floor", &number, 0.0f, 100.0f, 0.5f, "mm" );
    menuAddAction( &shell.menu, MENU_ROOT, "reset settings", 'Z', run, true );
    menuAddAction( &shell.menu, MENU_ROOT, "latest fix", 'l', run, false );
}

static int send( InputControl c, InputEventKind k ) {
    t += 10;
    if ( k == IN_PRESS )
        heldNow[ c ] = true;
    if ( k == IN_RELEASE )
        heldNow[ c ] = false;
    return uiShellEvent( &shell, { c, k }, t );
}

// A tap: press, click, release; whatever action any of the three returned.
static int tap( InputControl c ) {
    int r0 = send( c, IN_PRESS );
    int r1 = send( c, IN_CLICK );
    int r2 = send( c, IN_RELEASE );
    return r0 >= 0 ? r0 : ( r1 >= 0 ? r1 : r2 );
}

// A hold: press, hold, release.
static int hold( InputControl c ) {
    send( c, IN_PRESS );
    int r = send( c, IN_HOLD );
    send( c, IN_RELEASE );
    return r;
}

static void tick( float x, float y ) {
    t += 10;
    uiShellTick( &shell, t, 0.01f, x, y, x, y, heldNow ); // the raw stick as the shaped one, for these
}

void test_shell_home_opens_on_settings_and_selects_apps( void ) {
    shellSetUp( );
    TEST_ASSERT_EQUAL( 1, fakes[ 0 ].enters );
    TEST_ASSERT_EQUAL( PANE_APP, uiShellTop( &shell ) );
    tap( IN_BTN_A ); // Home, cursor on Settings (the centre cell), whatever app is up
    TEST_ASSERT_EQUAL( PANE_HOME, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 1, shell.home.cursor );
    send( IN_NAV_RIGHT, IN_PRESS ); // LEDs
    send( IN_NAV_RIGHT, IN_RELEASE );
    TEST_ASSERT_EQUAL( 2, shell.home.cursor );
    send( IN_NAV_RIGHT, IN_PRESS ); // wraps within the row: to its first cell
    send( IN_NAV_RIGHT, IN_RELEASE );
    TEST_ASSERT_EQUAL( 0, shell.home.cursor );
    send( IN_NAV_UP, IN_PRESS ); // clamps: one row only
    send( IN_NAV_UP, IN_RELEASE );
    TEST_ASSERT_EQUAL( 0, shell.home.cursor );
    send( IN_NAV_LEFT, IN_PRESS ); // wraps the other way
    send( IN_NAV_LEFT, IN_RELEASE );
    TEST_ASSERT_EQUAL( 2, shell.home.cursor );
    tap( IN_NAV_PRESS ); // the LEDs app
    TEST_ASSERT_EQUAL( PANE_APP, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 1, shell.app );
    TEST_ASSERT_EQUAL( 1, fakes[ 0 ].exits );
    TEST_ASSERT_EQUAL( 1, fakes[ 1 ].enters );
    TEST_ASSERT_EQUAL( 0, shell.previousApp );
    tap( IN_BTN_B ); // B: the previous app
    TEST_ASSERT_EQUAL( 0, shell.app );
    TEST_ASSERT_EQUAL( 1, shell.previousApp );
    tap( IN_BTN_A ); // Home again: Settings; B closes it
    TEST_ASSERT_EQUAL( 1, shell.home.cursor );
    send( IN_NAV_RIGHT, IN_PRESS );
    send( IN_NAV_RIGHT, IN_RELEASE );
    tap( IN_BTN_B );
    TEST_ASSERT_EQUAL( PANE_APP, uiShellTop( &shell ) );
    tap( IN_BTN_A ); // reopened: Settings again, not where it was left
    TEST_ASSERT_EQUAL( 1, shell.home.cursor );
    tap( IN_NAV_PRESS ); // the Settings cell: the menu in Home's place
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 1, shell.depth );
}

void test_shell_a_and_b_never_reach_an_app( void ) {
    shellSetUp( );
    hold( IN_BTN_A ); // A hold: Settings directly
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    tap( IN_BTN_B ); // at the root: closes
    TEST_ASSERT_EQUAL( PANE_APP, uiShellTop( &shell ) );
    hold( IN_BTN_B ); // B hold in an app: Home
    TEST_ASSERT_EQUAL( PANE_HOME, uiShellTop( &shell ) );
    tap( IN_BTN_B );
    TEST_ASSERT_EQUAL( 0, fakes[ 0 ].events );
    // The app's own controls do reach it: presses, repeats, holds, the joystick's press.
    send( IN_NAV_LEFT, IN_PRESS );
    send( IN_NAV_LEFT, IN_REPEAT );
    send( IN_NAV_LEFT, IN_RELEASE );
    hold( IN_NAV_PRESS );
    tap( IN_JOY_PRESS );
    TEST_ASSERT_EQUAL( 9, fakes[ 0 ].events );
}

// The headline bug: a held nav-left that backs out of the menu must not go
// on into the app; its release clears it; the next press is the app's.
void test_shell_swallows_a_held_control_across_a_focus_change( void ) {
    shellSetUp( );
    hold( IN_BTN_A ); // Settings
    tap( IN_NAV_PRESS ); // into the tracker page
    TEST_ASSERT_EQUAL_STRING( "tracker", menuTitle( &shell.menu ) );
    send( IN_NAV_LEFT, IN_PRESS ); // left on the page's first item: a toggle flips
    TEST_ASSERT_TRUE( flag );
    send( IN_NAV_LEFT, IN_REPEAT );
    TEST_ASSERT_FALSE( flag );
    tap( IN_BTN_B ); // back to the root...
    tap( IN_BTN_B ); // ...and closed, with nav-left still down
    TEST_ASSERT_EQUAL( PANE_APP, uiShellTop( &shell ) );
    TEST_ASSERT_TRUE( shell.swallow[ IN_NAV_LEFT ] );
    send( IN_NAV_LEFT, IN_REPEAT );
    send( IN_NAV_LEFT, IN_REPEAT );
    TEST_ASSERT_EQUAL( 0, fakes[ 0 ].events ); // never reached the app
    send( IN_NAV_LEFT, IN_RELEASE );
    TEST_ASSERT_EQUAL( 0, fakes[ 0 ].events );
    TEST_ASSERT_FALSE( shell.swallow[ IN_NAV_LEFT ] );
    send( IN_NAV_LEFT, IN_PRESS ); // the next press is the app's
    TEST_ASSERT_EQUAL( 1, fakes[ 0 ].events );
    TEST_ASSERT_EQUAL( IN_NAV_LEFT, fakes[ 0 ].last.control );
    send( IN_NAV_LEFT, IN_RELEASE );
    // B held from two panes deep: everything closes (the app alone, not
    // Home: a setting is tweaked, the picture looked at, the menu opened
    // again where it was), and B's own release is swallowed - it does not
    // go on to the previous app.
    hold( IN_BTN_A );
    tap( IN_NAV_PRESS );
    TEST_ASSERT_EQUAL_STRING( "tracker", menuTitle( &shell.menu ) );
    send( IN_BTN_B, IN_PRESS );
    send( IN_BTN_B, IN_HOLD );
    TEST_ASSERT_EQUAL( PANE_APP, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 0, shell.depth );
    TEST_ASSERT_TRUE( shell.swallow[ IN_BTN_B ] );
    send( IN_BTN_B, IN_RELEASE );
    TEST_ASSERT_EQUAL( PANE_APP, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 0, shell.app );
    TEST_ASSERT_EQUAL( 2, fakes[ 0 ].events ); // (the press and its release above)
    // The menu keeps its page: the next open is where it was.
    hold( IN_BTN_A );
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL_STRING( "tracker", menuTitle( &shell.menu ) );
}

// The menu opens where it was last used, even after B backed it out to the
// root and closed it: a page entered is remembered, with its cursor, and
// the root is one B away.
void test_shell_menu_reopens_at_the_last_page( void ) {
    shellSetUp( );
    hold( IN_BTN_A );
    TEST_ASSERT_EQUAL_STRING( "menu", menuTitle( &shell.menu ) ); // nothing visited yet: the root
    tap( IN_NAV_PRESS ); // into tracker
    send( IN_NAV_DOWN, IN_PRESS ); // floor
    send( IN_NAV_DOWN, IN_RELEASE );
    tap( IN_BTN_B ); // the root...
    TEST_ASSERT_EQUAL_STRING( "menu", menuTitle( &shell.menu ) );
    tap( IN_BTN_B ); // ...closed
    TEST_ASSERT_EQUAL( PANE_APP, uiShellTop( &shell ) );
    hold( IN_BTN_A ); // reopened: the tracker page, the cursor on floor
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL_STRING( "tracker", menuTitle( &shell.menu ) );
    TEST_ASSERT_EQUAL( 1, shell.menu.cursor );
    TEST_ASSERT_EQUAL( 1, shell.menu.depth );
    tap( IN_BTN_B ); // the root is one press away, its cursor on the page just left
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL_STRING( "menu", menuTitle( &shell.menu ) );
    TEST_ASSERT_EQUAL( 0, shell.menu.cursor );
    tap( IN_BTN_B );
    tap( IN_BTN_A ); // from Home's Settings cell too
    send( IN_NAV_RIGHT, IN_PRESS );
    send( IN_NAV_RIGHT, IN_RELEASE );
    send( IN_NAV_LEFT, IN_PRESS );
    send( IN_NAV_LEFT, IN_RELEASE );
    TEST_ASSERT_EQUAL( 1, shell.home.cursor );
    tap( IN_NAV_PRESS );
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL_STRING( "tracker", menuTitle( &shell.menu ) );
}

// A select held on a value item: the menu hides and the value is changed
// over the app with left/right (up/down: the neighbouring items), the value
// shown along the bottom; a press or B brings the menu back, B held closes
// everything. The press that started the hold has already acted on a
// toggle or a choice: the hold takes that back. A hold that follows a press
// which entered a page is nothing.
void test_shell_tweak_edits_a_value_over_the_app( void ) {
    shellSetUp( );
    hold( IN_BTN_A );
    tap( IN_NAV_PRESS ); // tracker: [tracker on] [floor]
    send( IN_NAV_DOWN, IN_PRESS );
    send( IN_NAV_DOWN, IN_RELEASE );
    number = 10.0f;
    send( IN_NAV_PRESS, IN_PRESS ); // a press on a number: nothing
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 10.0f, number );
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    send( IN_NAV_PRESS, IN_HOLD ); // held: the value over the app
    TEST_ASSERT_EQUAL( PANE_TWEAK, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 2, shell.depth );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 10.0f, number );
    send( IN_NAV_PRESS, IN_RELEASE );
    TEST_ASSERT_EQUAL( PANE_TWEAK, uiShellTop( &shell ) );
    send( IN_NAV_RIGHT, IN_PRESS );
    send( IN_NAV_RIGHT, IN_RELEASE );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 10.5f, number );
    // The joystick is the app's while the value is tweaked (it orbits the
    // scene being looked at): its four-way is no menu key here.
    send( IN_JOY_RIGHT, IN_PRESS );
    send( IN_JOY_RIGHT, IN_RELEASE );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 10.5f, number );
    tick( 0.0f, 0.0f ); // centred once...
    tick( 0.5f, 0.0f ); // ...then the app's
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.5f, fakes[ 0 ].lastJoyX );
    send( IN_NAV_UP, IN_PRESS ); // the neighbouring item
    send( IN_NAV_UP, IN_RELEASE );
    TEST_ASSERT_EQUAL( 0, shell.menu.cursor );
    flag = false;
    send( IN_NAV_RIGHT, IN_PRESS );
    send( IN_NAV_RIGHT, IN_RELEASE );
    TEST_ASSERT_TRUE( flag );
    tap( IN_BTN_B ); // the menu again, where the cursor is
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 0, shell.menu.cursor );
    // A hold on a toggle: the press flipped it, the hold takes that back.
    send( IN_NAV_PRESS, IN_PRESS );
    TEST_ASSERT_FALSE( flag );
    send( IN_NAV_PRESS, IN_HOLD );
    TEST_ASSERT_TRUE( flag );
    TEST_ASSERT_EQUAL( PANE_TWEAK, uiShellTop( &shell ) );
    send( IN_NAV_PRESS, IN_RELEASE );
    tap( IN_NAV_PRESS ); // a press: the menu again
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    send( IN_NAV_PRESS, IN_PRESS );
    send( IN_NAV_PRESS, IN_HOLD );
    TEST_ASSERT_EQUAL( PANE_TWEAK, uiShellTop( &shell ) );
    send( IN_NAV_PRESS, IN_RELEASE );
    send( IN_BTN_B, IN_PRESS ); // B held: everything closes
    send( IN_BTN_B, IN_HOLD );
    send( IN_BTN_B, IN_RELEASE );
    TEST_ASSERT_EQUAL( PANE_APP, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 0, fakes[ 0 ].events );
    // A press that entered a page, then held: no tweak, and the page's
    // first item is not touched.
    hold( IN_BTN_A );
    tap( IN_BTN_B ); // the root, cursor on tracker
    TEST_ASSERT_EQUAL_STRING( "menu", menuTitle( &shell.menu ) );
    bool was = flag;
    send( IN_NAV_PRESS, IN_PRESS );
    TEST_ASSERT_EQUAL_STRING( "tracker", menuTitle( &shell.menu ) );
    send( IN_NAV_PRESS, IN_HOLD );
    send( IN_NAV_PRESS, IN_RELEASE );
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( was, flag );
}

void test_shell_confirm_and_result( void ) {
    shellSetUp( );
    hold( IN_BTN_A );
    send( IN_NAV_DOWN, IN_PRESS ); // reset settings
    send( IN_NAV_DOWN, IN_RELEASE );
    TEST_ASSERT_EQUAL( -1, tap( IN_NAV_PRESS ) ); // asks first
    TEST_ASSERT_EQUAL( PANE_CONFIRM, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( -1, tap( IN_BTN_B ) ); // no
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( -1, tap( IN_NAV_PRESS ) );
    int action = tap( IN_BTN_A ); // yes: the action comes back to be run
    TEST_ASSERT_TRUE( action >= 0 );
    TEST_ASSERT_EQUAL( 'Z', shell.menu.items[ action ].tag );
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    send( IN_NAV_DOWN, IN_PRESS ); // latest fix: no confirmation
    send( IN_NAV_DOWN, IN_RELEASE );
    action = tap( IN_NAV_PRESS );
    TEST_ASSERT_TRUE( action >= 0 );
    TEST_ASSERT_EQUAL( 'l', shell.menu.items[ action ].tag );
    uiShellShowResult( &shell, "l", 30, 10 ); // the caller ran it and shows the log tail
    TEST_ASSERT_EQUAL( PANE_RESULT, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 2, shell.depth );
    for ( int k = 0; k < 25; k++ ) {
        send( IN_NAV_UP, IN_PRESS );
        send( IN_NAV_UP, IN_RELEASE );
    }
    TEST_ASSERT_EQUAL( 20, shell.resultScroll ); // clamped to what is above the window
    tap( IN_BTN_B ); // dismissed: the page underneath is still there
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 0, fakes[ 0 ].events );
}

// Absolute joystick: the stick's position is the cursor while it is
// deflected - a cell on Home, a row on a menu page - and the four-way's
// events are ignored there; the app's own stick is untouched.
void test_shell_absolute_joystick( void ) {
    shellSetUp( );
    shell.absoluteJoystick = true;
    tap( IN_BTN_A ); // Home: View, Settings, LEDs in one row
    tick( 0.9f, 0.0f );
    TEST_ASSERT_EQUAL( 2, shell.home.cursor ); // right band: the third cell
    tick( 0.5f, 0.0f );                        // let go: back through the middle band...
    tick( 0.2f, 0.0f );
    tick( 0.0f, 0.0f );
    TEST_ASSERT_EQUAL( 2, shell.home.cursor ); // ...and the cursor stays
    tick( 0.1f, 0.1f );
    TEST_ASSERT_EQUAL( 1, shell.home.cursor ); // a small deflection: the middle band
    tick( -0.9f, -0.9f );
    TEST_ASSERT_EQUAL( 0, shell.home.cursor ); // down-left: the last row is the only row
    send( IN_JOY_RIGHT, IN_PRESS ); // the four-way is ignored here
    send( IN_JOY_RIGHT, IN_RELEASE );
    TEST_ASSERT_EQUAL( 0, shell.home.cursor );
    send( IN_NAV_RIGHT, IN_PRESS ); // the nav stick still steps
    send( IN_NAV_RIGHT, IN_RELEASE );
    TEST_ASSERT_EQUAL( 1, shell.home.cursor );
    tap( IN_NAV_PRESS ); // Settings: a page of four rows (tracker, reset settings, latest fix... the root)
    TEST_ASSERT_EQUAL( PANE_MENU, uiShellTop( &shell ) );
    int visible = menuVisibleCount( &shell.menu );
    tick( 0.0f, -0.95f ); // the stick at the bottom: the last row
    TEST_ASSERT_EQUAL( visible - 1, shell.menu.cursor );
    tick( 0.0f, 0.95f ); // at the top: the first
    TEST_ASSERT_EQUAL( 0, shell.menu.cursor );
    tick( 0.0f, 0.0f );
    TEST_ASSERT_EQUAL( 0, shell.menu.cursor );
    // Point at the last row and let go: the stick springs back through the
    // middle rows; the cursor stays on the last.
    tick( 0.0f, -0.95f );
    TEST_ASSERT_EQUAL( visible - 1, shell.menu.cursor );
    tick( 0.0f, -0.6f );
    tick( 0.0f, -0.2f );
    tick( 0.0f, 0.1f );
    tick( 0.0f, 0.0f );
    TEST_ASSERT_EQUAL( visible - 1, shell.menu.cursor );
    tick( 0.0f, 0.95f ); // out again: it follows again
    TEST_ASSERT_EQUAL( 0, shell.menu.cursor );
    tick( 0.0f, 0.0f );
    tap( IN_BTN_B ); // closed: the app has its stick as before
    tick( 0.0f, 0.0f );
    tick( 0.5f, 0.0f );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.5f, fakes[ 0 ].lastJoyX );
}

void test_shell_timeout_and_joystick( void ) {
    shellSetUp( );
    tick( 0.0f, 0.0f );
    TEST_ASSERT_TRUE( shell.joyArmed );
    tick( 0.5f, 0.0f );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.5f, fakes[ 0 ].lastJoyX ); // the app has the stick
    tap( IN_BTN_A ); // Home: the stick is not the app's
    tick( 0.5f, 0.0f );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.0f, fakes[ 0 ].lastJoyX );
    TEST_ASSERT_TRUE( fakes[ 0 ].ticks == 3 ); // but it still ticks
    tap( IN_BTN_B ); // closed while the stick is still over: not the app's until centred
    tick( 0.5f, 0.0f );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.0f, fakes[ 0 ].lastJoyX );
    tick( 0.0f, 0.0f );
    tick( 0.5f, 0.0f );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.5f, fakes[ 0 ].lastJoyX );
    // 20 s idle: the overlays close, cursors kept.
    tap( IN_BTN_A ); // (on Settings, cell 1)
    send( IN_NAV_RIGHT, IN_PRESS );
    send( IN_NAV_RIGHT, IN_RELEASE );
    TEST_ASSERT_EQUAL( 2, shell.home.cursor );
    t += UISHELL_IDLE_MS;
    tick( 0.0f, 0.0f );
    TEST_ASSERT_EQUAL( PANE_APP, uiShellTop( &shell ) );
    TEST_ASSERT_EQUAL( 2, shell.home.cursor );
    // A swallowed control whose release the ring lost: healed from the raw state.
    hold( IN_BTN_A );
    send( IN_NAV_LEFT, IN_PRESS );
    tap( IN_BTN_B );
    TEST_ASSERT_TRUE( shell.swallow[ IN_NAV_LEFT ] );
    heldNow[ IN_NAV_LEFT ] = false; // it went up, unseen
    tick( 0.0f, 0.0f );
    TEST_ASSERT_FALSE( shell.swallow[ IN_NAV_LEFT ] );
    TEST_ASSERT_FALSE( shell.held[ IN_NAV_LEFT ] );
}

// ---- camera ------------------------------------------------------------------

static Camera cam;

static float length( Vec3 v ) {
    return sqrtf( v.x * v.x + v.y * v.y + v.z * v.z );
}

void test_camera_glides_and_orbits( void ) {
    cameraInit( &cam, { 27, 22, 8 }, -25.0f, 32.0f, 300.0f, 2.0f );
    Vec3 none = { 0, 0, 0 };
    cameraOrbit( &cam, 40.0f, 10.0f );
    cameraUpdate( &cam, 0.01f, 0.0f, false, none, none );
    TEST_ASSERT_TRUE( cam.yawDeg > -25.0f && cam.yawDeg < 15.0f ); // on its way, not there yet
    for ( int k = 0; k < 200; k++ )
        cameraUpdate( &cam, 0.01f, k * 0.01f, false, none, none );
    TEST_ASSERT_FLOAT_WITHIN( 0.1f, 15.0f, cam.yawDeg );
    TEST_ASSERT_FLOAT_WITHIN( 0.1f, 42.0f, cam.elevationDeg );
    cameraPan( &cam, 10.0f, 0.0f ); // screen right at yaw 15: mostly +x
    for ( int k = 0; k < 200; k++ )
        cameraUpdate( &cam, 0.01f, k * 0.01f, false, none, none );
    TEST_ASSERT_TRUE( cam.target.x > 27.0f + 9.0f );
    cameraZoom( &cam, 2.0f );
    for ( int k = 0; k < 200; k++ )
        cameraUpdate( &cam, 0.01f, k * 0.01f, false, none, none );
    TEST_ASSERT_FLOAT_WITHIN( 0.05f, 4.0f, cam.zoom );
    cameraReset( &cam );
    for ( int k = 0; k < 300; k++ )
        cameraUpdate( &cam, 0.01f, k * 0.01f, false, none, none );
    TEST_ASSERT_FLOAT_WITHIN( 0.1f, -25.0f, cam.yawDeg );
    TEST_ASSERT_FLOAT_WITHIN( 0.1f, 27.0f, cam.target.x );
}

// The wrap: going from yaw 170 to a goal of -170 turns 20 degrees, not 340.
void test_camera_turns_the_short_way( void ) {
    cameraInit( &cam, { 0, 0, 0 }, 170.0f, 30.0f, 300.0f, 2.0f );
    Vec3 none = { 0, 0, 0 };
    cameraOrbit( &cam, 20.0f, 0.0f ); // user yaw 190 = -170
    cameraUpdate( &cam, 0.02f, 0.0f, false, none, none );
    TEST_ASSERT_TRUE( cam.yawDeg > 170.0f || cam.yawDeg < -170.0f ); // moved past 180, not back through 0
}

// POV: the camera sits at the probe's point and looks down its shaft.
void test_camera_pov_is_at_the_point_looking_down_the_shaft( void ) {
    cameraInit( &cam, { 27, 22, 8 }, -25.0f, 32.0f, 300.0f, 2.0f );
    cameraSetMode( &cam, CAMERA_POV );
    Vec3 tip = { 30.0f, 10.0f, 30.0f };
    float lean = 25.0f * (float)M_PI / 180.0f;
    Vec3 shaft = { sinf( lean ) * cosf( 0.7f ), sinf( lean ) * sinf( 0.7f ), cosf( lean ) };
    for ( int k = 0; k < 800; k++ ) // POV glides slowly on purpose (0.6 s turns): give it time to settle
        cameraUpdate( &cam, 0.01f, k * 0.01f, true, tip, shaft );
    // The camera position is target - distance * lookDirection; it must be the tip.
    Vec3 d = cameraLookDirection( cam.yawDeg, cam.elevationDeg );
    Vec3 camera = { cam.target.x - cam.distance * d.x, cam.target.y - cam.distance * d.y, cam.target.z - cam.distance * d.z };
    Vec3 miss = { camera.x - tip.x, camera.y - tip.y, camera.z - tip.z };
    printf( "  POV camera at (%.1f %.1f %.1f), tip (%.1f %.1f %.1f)\n", camera.x, camera.y, camera.z, tip.x, tip.y, tip.z );
    TEST_ASSERT_TRUE( length( miss ) < 0.5f );
    // ...and looks along -shaft.
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, -shaft.x, d.x );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, -shaft.y, d.y );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, -shaft.z, d.z );
    // An upright probe keeps the last yaw rather than spinning.
    float yawBefore = cam.yawDeg;
    Vec3 upright = { 0.001f, 0.0f, 1.0f };
    for ( int k = 0; k < 600; k++ )
        cameraUpdate( &cam, 0.01f, k * 0.01f, true, tip, upright );
    TEST_ASSERT_FLOAT_WITHIN( 0.5f, yawBefore, cam.yawDeg );
    TEST_ASSERT_FLOAT_WITHIN( 0.5f, 89.9f, cam.elevationDeg );
}

int main( int argc, char** argv ) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN( );
    RUN_TEST( test_menu_walks_the_tree );
    RUN_TEST( test_menu_edits_in_place );
    RUN_TEST( test_menu_accessor_set_once_with_the_flipped_value );
    RUN_TEST( test_menu_returns_actions_with_their_argument );
    RUN_TEST( test_shell_home_opens_on_settings_and_selects_apps );
    RUN_TEST( test_shell_a_and_b_never_reach_an_app );
    RUN_TEST( test_shell_swallows_a_held_control_across_a_focus_change );
    RUN_TEST( test_shell_menu_reopens_at_the_last_page );
    RUN_TEST( test_shell_tweak_edits_a_value_over_the_app );
    RUN_TEST( test_shell_confirm_and_result );
    RUN_TEST( test_shell_timeout_and_joystick );
    RUN_TEST( test_shell_absolute_joystick );
    RUN_TEST( test_camera_glides_and_orbits );
    RUN_TEST( test_camera_turns_the_short_way );
    RUN_TEST( test_camera_pov_is_at_the_point_looking_down_the_shaft );
    return UNITY_END( );
}
