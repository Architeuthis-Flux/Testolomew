// SPDX-License-Identifier: MIT
// Host-side test of the UI's pure parts: the menu tree (walking it with the
// four-way keys, editing numbers and choices, running actions, the number
// prompt) and the camera (modes, the controls, the POV geometry - the camera
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

void setUp( void ) {
    menuInit( &menu );
    flag = false;
    number = 10.0f;
    choice = 0;
}
void tearDown( void ) {}

static void buildMenu( ) {
    int settings = menuAddSubmenu( &menu, MENU_ROOT, "settings" );
    menuAddToggle( &menu, settings, "tracker", &flag );
    menuAddNumber( &menu, settings, "surface", &number, 0.0f, 100.0f, 0.5f, "mm" );
    menuAddChoice( &menu, settings, "cursor", &choice, choiceNames, 3 );
    int commands = menuAddSubmenu( &menu, MENU_ROOT, "commands" );
    menuAddAction( &menu, commands, "l  latest fix", 'l', false );
    menuAddAction( &menu, commands, "R  anchor at row <number>", 'R', true );
    menuOpen( &menu );
}

void test_menu_walks_the_tree( void ) {
    buildMenu( );
    float n;
    TEST_ASSERT_EQUAL( 2, menuVisibleCount( &menu ) );
    TEST_ASSERT_EQUAL_STRING( "menu", menuTitle( &menu ) );
    TEST_ASSERT_EQUAL( -1, menuKey( &menu, MENUKEY_ENTER, false, &n ) ); // into settings
    TEST_ASSERT_EQUAL_STRING( "settings", menuTitle( &menu ) );
    TEST_ASSERT_EQUAL( 3, menuVisibleCount( &menu ) );
    menuKey( &menu, MENUKEY_ENTER, false, &n ); // toggle tracker
    TEST_ASSERT_TRUE( flag );
    menuKey( &menu, MENUKEY_UP, false, &n ); // wraps to the last
    TEST_ASSERT_EQUAL( 2, menu.cursor );
    menuKey( &menu, MENUKEY_BACK, false, &n ); // up a level, cursor restored
    TEST_ASSERT_EQUAL_STRING( "menu", menuTitle( &menu ) );
    TEST_ASSERT_EQUAL( 0, menu.cursor );
    menuKey( &menu, MENUKEY_BACK, false, &n ); // closes at the root
    TEST_ASSERT_FALSE( menu.open );
}

void test_menu_edits_a_number_and_a_choice( void ) {
    buildMenu( );
    float n;
    menuKey( &menu, MENUKEY_ENTER, false, &n ); // settings
    menuKey( &menu, MENUKEY_DOWN, false, &n );  // surface
    menuKey( &menu, MENUKEY_ENTER, false, &n ); // edit
    TEST_ASSERT_TRUE( menu.editing );
    menuKey( &menu, MENUKEY_UP, false, &n );
    menuKey( &menu, MENUKEY_UP, false, &n );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 11.0f, number );
    for ( int k = 0; k < 12; k++ )
        menuKey( &menu, MENUKEY_UP, true, &n ); // held: after 8 repeats it steps ten at a time
    TEST_ASSERT_TRUE( number > 11.0f + 12 * 0.5f );
    for ( int k = 0; k < 400; k++ )
        menuKey( &menu, MENUKEY_UP, true, &n );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 100.0f, number ); // clamped
    menuKey( &menu, MENUKEY_ENTER, false, &n );
    TEST_ASSERT_FALSE( menu.editing );
    menuKey( &menu, MENUKEY_DOWN, false, &n );  // cursor
    menuKey( &menu, MENUKEY_ENTER, false, &n ); // edit
    menuKey( &menu, MENUKEY_DOWN, false, &n );  // wraps backwards
    TEST_ASSERT_EQUAL( 2, choice );
    menuKey( &menu, MENUKEY_UP, false, &n );
    TEST_ASSERT_EQUAL( 0, choice );
    menuKey( &menu, MENUKEY_BACK, false, &n ); // leaves the editor, stays in the menu
    TEST_ASSERT_FALSE( menu.editing );
    TEST_ASSERT_TRUE( menu.open );
}

void test_menu_runs_actions_and_asks_for_numbers( void ) {
    buildMenu( );
    float n = -1.0f;
    menuKey( &menu, MENUKEY_DOWN, false, &n ); // commands
    menuKey( &menu, MENUKEY_ENTER, false, &n );
    TEST_ASSERT_EQUAL( 'l', menuKey( &menu, MENUKEY_ENTER, false, &n ) );
    menuKey( &menu, MENUKEY_DOWN, false, &n );                           // R
    TEST_ASSERT_EQUAL( -1, menuKey( &menu, MENUKEY_ENTER, false, &n ) ); // asks first
    TEST_ASSERT_TRUE( menu.editing );
    for ( int k = 0; k < 30; k++ )
        menuKey( &menu, MENUKEY_UP, false, &n );
    TEST_ASSERT_EQUAL( 'R', menuKey( &menu, MENUKEY_ENTER, false, &n ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 30.0f, n );
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
    RUN_TEST( test_menu_edits_a_number_and_a_choice );
    RUN_TEST( test_menu_runs_actions_and_asks_for_numbers );
    RUN_TEST( test_camera_glides_and_orbits );
    RUN_TEST( test_camera_turns_the_short_way );
    RUN_TEST( test_camera_pov_is_at_the_point_looking_down_the_shaft );
    return UNITY_END( );
}
