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
    RUN_TEST( test_camera_glides_and_orbits );
    RUN_TEST( test_camera_turns_the_short_way );
    RUN_TEST( test_camera_pov_is_at_the_point_looking_down_the_shaft );
    return UNITY_END( );
}
