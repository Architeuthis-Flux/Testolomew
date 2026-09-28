// SPDX-License-Identifier: MIT
// Host test of the surface height map (src/magfit/SurfaceMap.h): a tilted
// board's taps give back the tilt, a bowed one the bow, too few taps give
// nothing, and the map holds its edge beyond the taps. pio test -e native
#include <math.h>
#include <unity.h>

#include "SurfaceMap.h"

void setUp( void ) {}
void tearDown( void ) {}

static SurfaceMap map;

// The twelve calibration taps' places, roughly: rows 1, 15, 30 on one half
// (y 3 and 13), rows 60, 45, 31 on the other (y -3 and -13), x 0, 35, 74.
static int twelveTaps( Vec3* taps, float ( *z )( float x, float y ) ) {
    int n = 0;
    const float xs[ 3 ] = { 0.0f, 35.6f, 73.7f }, ys[ 4 ] = { 3.8f, 13.9f, -3.8f, -13.9f };
    for ( int i = 0; i < 3; i++ )
        for ( int j = 0; j < 4; j++ ) {
            taps[ n ] = { xs[ i ], ys[ j ], z( xs[ i ], ys[ j ] ) };
            n++;
        }
    return n;
}

static float tilted( float x, float y ) {
    return 17.5f + 0.04f * x - 0.05f * y; // 3 mm along the board, 1.4 across
}
static float bowed( float x, float y ) {
    return 17.5f + 0.02f * x - 0.0012f * ( x - 37.0f ) * ( x - 37.0f ) + 0.003f * y * y;
}
static float flat( float x, float y ) {
    (void)x;
    (void)y;
    return 17.5f;
}

void test_a_tilted_board_is_a_plane( void ) {
    Vec3 taps[ 12 ];
    int n = twelveTaps( taps, tilted );
    TEST_ASSERT_EQUAL( 6, surfaceMapFit( &map, taps, n ) ); // twelve taps: the quadratic, whose square terms come out ~0
    TEST_ASSERT_TRUE( map.worstMm < 0.01f );
    // The centre's height and the offsets elsewhere are the board's.
    float zc = tilted( map.centreX, map.centreY );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, zc, surfaceMapCentreZ( &map ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, tilted( 60.0f, 10.0f ) - zc, surfaceMapOffset( &map, 60.0f, 10.0f ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, tilted( 5.0f, -12.0f ) - zc, surfaceMapOffset( &map, 5.0f, -12.0f ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.0f, surfaceMapOffset( &map, map.centreX, map.centreY ) );
    // Four taps spread out: a plane, the same answer.
    Vec3 four[ 4 ] = { taps[ 0 ], taps[ 1 ], taps[ 4 ], taps[ 9 ] };
    TEST_ASSERT_EQUAL( 3, surfaceMapFit( &map, four, 4 ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.02f, tilted( 60.0f, 10.0f ) - tilted( map.centreX, map.centreY ), surfaceMapOffset( &map, 60.0f, 10.0f ) );
}

void test_a_bowed_board_is_a_quadratic( void ) {
    Vec3 taps[ 12 ];
    int n = twelveTaps( taps, bowed );
    TEST_ASSERT_EQUAL( 6, surfaceMapFit( &map, taps, n ) );
    TEST_ASSERT_TRUE( map.worstMm < 0.02f );
    float zc = surfaceMapCentreZ( &map );
    TEST_ASSERT_FLOAT_WITHIN( 0.05f, bowed( 20.0f, 8.0f ) - zc, surfaceMapOffset( &map, 20.0f, 8.0f ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.05f, bowed( 70.0f, -12.0f ) - zc, surfaceMapOffset( &map, 70.0f, -12.0f ) );
}

void test_too_few_or_a_line_is_flat( void ) {
    Vec3 two[ 2 ] = { { 0, 0, 17 }, { 50, 0, 19 } };
    TEST_ASSERT_EQUAL( 0, surfaceMapFit( &map, two, 2 ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, 0.0f, surfaceMapOffset( &map, 30.0f, 0.0f ) );
    Vec3 line[ 4 ] = { { 0, 5, 17 }, { 20, 5, 18 }, { 40, 5, 19 }, { 60, 5, 20 } }; // all on one line: no plane
    TEST_ASSERT_EQUAL( 0, surfaceMapFit( &map, line, 4 ) );
    Vec3 taps[ 12 ];
    int n = twelveTaps( taps, flat );
    TEST_ASSERT_EQUAL( 6, surfaceMapFit( &map, taps, n ) );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.0f, surfaceMapOffset( &map, 70.0f, 10.0f ) ); // a flat board learns nothing
}

void test_the_map_holds_its_edge_beyond_the_taps( void ) {
    Vec3 taps[ 12 ];
    int n = twelveTaps( taps, bowed );
    surfaceMapFit( &map, taps, n );
    float atEdge = surfaceMapOffset( &map, map.maxX, 0.0f );
    TEST_ASSERT_FLOAT_WITHIN( 0.001f, atEdge, surfaceMapOffset( &map, map.maxX + 200.0f, 0.0f ) ); // not the quadratic's tail
}

int main( int argc, char** argv ) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN( );
    RUN_TEST( test_a_tilted_board_is_a_plane );
    RUN_TEST( test_a_bowed_board_is_a_quadratic );
    RUN_TEST( test_too_few_or_a_line_is_flat );
    RUN_TEST( test_the_map_holds_its_edge_beyond_the_taps );
    return UNITY_END( );
}
