// SPDX-License-Identifier: MIT
// Host tests for the breadboard row grid: pio test -e native
#include <math.h>
#include <stdlib.h>
#include <unity.h>

#include "RowGrid.h"

void setUp( void ) {}
void tearDown( void ) {}

// Where a hole really is, for a breadboard with row 1 level with `origin`,
// lying at `angle`, measured by a fit that reads `scale` times too long.
// hole 1 = next to the channel.
static Vec3 holeAt( int row, int hole, float originX, float originY, float angle, float scale ) {
    bool bottom = row > ROWGRID_ROWS_PER_HALF;
    float along = ( ( bottom ? row - ROWGRID_ROWS_PER_HALF : row ) - 1 ) * ROWGRID_PITCH_MM * scale;
    float across = ( ROWGRID_INNER_HOLE_MM + ( hole - 1 ) * ROWGRID_PITCH_MM ) * ( bottom ? -1.0f : 1.0f ) * scale;
    Vec3 p = { originX + along * cosf( angle ) - across * sinf( angle ), originY + along * sinf( angle ) + across * cosf( angle ), 12.0f };
    return p;
}

static void test_both_halves_on_the_default_grid( void ) {
    RowGrid grid;
    rowGridDefault( &grid, -10.0f, 22.0f );
    for ( int row = 1; row <= 60; row++ ) {
        for ( int hole = 1; hole <= 5; hole++ ) {
            RowPlace place = rowGridPlace( &grid, holeAt( row, hole, -10.0f, 22.0f, 0.0f, 1.0f ) );
            TEST_ASSERT_EQUAL_INT( row, rowGridRow( place.along, place.acrossMm < 0.0f ) );
            TEST_ASSERT_EQUAL_INT( hole, rowGridHole( place.acrossMm ) );
        }
    }
    // Row 31 faces row 1, row 60 faces row 30.
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 1.0f, rowGridPlace( &grid, holeAt( 31, 3, -10.0f, 22.0f, 0.0f, 1.0f ) ).along );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 30.0f, rowGridPlace( &grid, holeAt( 60, 3, -10.0f, 22.0f, 0.0f, 1.0f ) ).along );
}

static void test_ends_channel_and_rails( void ) {
    TEST_ASSERT_EQUAL_INT( 11, rowGridRow( 11.49f, false ) );
    TEST_ASSERT_EQUAL_INT( 12, rowGridRow( 11.51f, false ) );
    TEST_ASSERT_EQUAL_INT( 42, rowGridRow( 11.51f, true ) );
    // Up to a row past the end is still the end row; beyond that, off the board.
    TEST_ASSERT_EQUAL_INT( 1, rowGridRow( 0.2f, false ) );
    TEST_ASSERT_EQUAL_INT( 30, rowGridRow( 31.2f, false ) );
    TEST_ASSERT_EQUAL_INT( 60, rowGridRow( 31.2f, true ) );
    TEST_ASSERT_EQUAL_INT( 0, rowGridRow( -0.8f, false ) );
    TEST_ASSERT_EQUAL_INT( 0, rowGridRow( 32.0f, true ) );
    // Over the channel, and out over the rails, is no hole.
    TEST_ASSERT_EQUAL_INT( 0, rowGridHole( 1.0f ) );
    TEST_ASSERT_EQUAL_INT( 0, rowGridHole( -1.0f ) );
    TEST_ASSERT_EQUAL_INT( 1, rowGridHole( -3.0f ) );
    TEST_ASSERT_EQUAL_INT( 5, rowGridHole( 14.5f ) );
    TEST_ASSERT_EQUAL_INT( 0, rowGridHole( 17.0f ) );
}

static float noise( float amplitude ) {
    return amplitude * ( ( rand( ) % 2001 ) - 1000 ) / 1000.0f;
}

static RowAnchor anchorAt( int row, int hole, Vec3 position ) {
    RowAnchor anchor = { row, hole, position, 0.3f };
    return anchor;
}

// Every hole of the breadboard must come out as itself.
static void assertEveryHole( const RowGrid* grid, float originX, float originY, float angle, float scale ) {
    for ( int row = 1; row <= 60; row++ ) {
        for ( int hole = 1; hole <= 5; hole += 2 ) {
            RowPlace place = rowGridPlace( grid, holeAt( row, hole, originX, originY, angle, scale ) );
            TEST_ASSERT_FLOAT_WITHIN( 0.02f, (float)( row > 30 ? row - 30 : row ), place.along );
            TEST_ASSERT_EQUAL_INT( row, rowGridRow( place.along, place.acrossMm < 0.0f ) );
            TEST_ASSERT_EQUAL_INT( hole, rowGridHole( place.acrossMm ) );
        }
    }
}

static void test_one_anchor_slides_the_breadboard( void ) {
    RowGrid grid;
    rowGridDefault( &grid, 0.0f, 0.0f );
    RowAnchor anchor = anchorAt( 44, 2, holeAt( 44, 2, 3.0f, 17.0f, 0.0f, 1.0f ) );
    RowFitReport report = rowGridFit( &grid, &anchor, 1, nullptr );
    TEST_ASSERT_EQUAL_INT( ROWFIT_SHIFT, report.kind );
    assertEveryHole( &grid, 3.0f, 17.0f, 0.0f, 1.0f );
    TEST_ASSERT_EQUAL_INT( ROWFIT_NONE, rowGridFit( &grid, &anchor, 0, nullptr ).kind );
}

// A breadboard lying 5 degrees askew, measured by a fit that reads 2 % short:
// two anchors far apart take both out - on the same half or on opposite
// corners - and a third only makes it better.
static void test_anchors_along_the_board_give_angle_and_scale( void ) {
    float angle = 5.0f * (float)M_PI / 180.0f, scale = 0.98f;
    int anchorRows[ 3 ][ 3 ] = { { 2, 29, 0 }, { 1, 60, 0 }, { 58, 3, 15 } };
    for ( int set = 0; set < 3; set++ ) {
        RowAnchor anchors[ 3 ];
        int count = 0;
        for ( int k = 0; k < 3 && anchorRows[ set ][ k ] > 0; k++ ) {
            anchors[ count++ ] = anchorAt( anchorRows[ set ][ k ], 1, holeAt( anchorRows[ set ][ k ], 1, 3.0f, 20.0f, angle, scale ) );
        }
        RowGrid grid;
        rowGridDefault( &grid, 0.0f, 0.0f );
        RowFitReport report = rowGridFit( &grid, anchors, count, nullptr );
        TEST_ASSERT_EQUAL_INT( ROWFIT_SCALED, report.kind );
        TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.0f, report.rmsMm );
        TEST_ASSERT_FLOAT_WITHIN( 1e-3f, scale, rowGridScaleAlong( &grid ) );
        TEST_ASSERT_FLOAT_WITHIN( 0.05f, 5.0f, rowGridAngleDeg( &grid ) );
        assertEveryHole( &grid, 3.0f, 20.0f, angle, scale );
    }

    // Two anchors on neighbouring rows say nothing about angle or scale: shift only.
    RowAnchor close[ 2 ] = { anchorAt( 7, 1, holeAt( 7, 1, 3, 20, angle, scale ) ), anchorAt( 8, 1, holeAt( 8, 1, 3, 20, angle, scale ) ) };
    RowGrid grid;
    rowGridDefault( &grid, 0.0f, 0.0f );
    TEST_ASSERT_EQUAL_INT( ROWFIT_SHIFT, rowGridFit( &grid, close, 2, nullptr ).kind );
    TEST_ASSERT_FLOAT_WITHIN( 1e-6f, 1.0f, rowGridScaleAlong( &grid ) );
}

// The guided calibration: twelve holes spread both ways. The magnet fit is made
// to read 3 % long along the board, 4 % short across it and a little skewed,
// none of which one scale could describe - and every hole still comes out right.
static void test_calibration_targets_fit_the_full_map( void ) {
    float angle = -4.0f * (float)M_PI / 180.0f;
    RowAnchor anchors[ ROWGRID_CALIBRATION_TARGETS ];
    Vec3 distorted[ 61 ][ 6 ];
    for ( int row = 1; row <= 60; row++ ) {
        for ( int hole = 1; hole <= 5; hole++ ) {
            Vec3 p = holeAt( row, hole, -9.0f, 21.0f, angle, 1.0f );
            distorted[ row ][ hole ] = { 1.03f * p.x + 0.02f * p.y, 0.96f * p.y, p.z };
        }
    }
    for ( int k = 0; k < ROWGRID_CALIBRATION_TARGETS; k++ ) {
        int row, hole;
        rowGridCalibrationTarget( k, &row, &hole );
        anchors[ k ] = anchorAt( row, hole, distorted[ row ][ hole ] );
    }
    RowGrid grid;
    rowGridDefault( &grid, 0.0f, 0.0f );
    float miss[ ROWGRID_CALIBRATION_TARGETS ];
    RowFitReport report = rowGridFit( &grid, anchors, ROWGRID_CALIBRATION_TARGETS, miss );
    TEST_ASSERT_EQUAL_INT( ROWFIT_FULL, report.kind );
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.0f, report.rmsMm );
    for ( int row = 1; row <= 60; row++ ) {
        for ( int hole = 1; hole <= 5; hole++ ) {
            RowPlace place = rowGridPlace( &grid, distorted[ row ][ hole ] );
            TEST_ASSERT_EQUAL_INT( row, rowGridRow( place.along, place.acrossMm < 0.0f ) );
            TEST_ASSERT_EQUAL_INT( hole, rowGridHole( place.acrossMm ) );
            // ...and back again
            Vec3 back = rowGridToBoard( &grid, place.along, place.acrossMm );
            TEST_ASSERT_FLOAT_WITHIN( 0.01f, distorted[ row ][ hole ].x, back.x );
            TEST_ASSERT_FLOAT_WITHIN( 0.01f, distorted[ row ][ hole ].y, back.y );
        }
    }
}

// One anchor that is badly off, and says so with a wide error bar, must not
// drag the fit; and the report must point at it.
static void test_a_poor_anchor_counts_for_less_and_is_reported( void ) {
    srand( 3 );
    RowAnchor anchors[ ROWGRID_CALIBRATION_TARGETS ];
    for ( int k = 0; k < ROWGRID_CALIBRATION_TARGETS; k++ ) {
        int row, hole;
        rowGridCalibrationTarget( k, &row, &hole );
        Vec3 p = holeAt( row, hole, -9.0f, 21.0f, 0.03f, 1.01f );
        p.x += noise( 0.2f );
        p.y += noise( 0.2f );
        anchors[ k ] = anchorAt( row, hole, p );
    }
    anchors[ 5 ].position.x += 4.0f; // row 30's outer hole, out past the sensors
    anchors[ 5 ].sigmaMm = 3.0f;
    RowGrid grid;
    rowGridDefault( &grid, 0.0f, 0.0f );
    float miss[ ROWGRID_CALIBRATION_TARGETS ];
    RowFitReport report = rowGridFit( &grid, anchors, ROWGRID_CALIBRATION_TARGETS, miss );
    TEST_ASSERT_EQUAL_INT( 5, report.worst );
    TEST_ASSERT_TRUE( report.worstMm > 3.0f );
    for ( int k = 0; k < ROWGRID_CALIBRATION_TARGETS; k++ ) {
        if ( k != 5 ) {
            TEST_ASSERT_TRUE( miss[ k ] < 0.6f );
        }
    }
}

static void test_confidence( void ) {
    RowPlace centre = { 14.0f, 8.0f }, edge = { 14.499f, 8.0f };
    // Dead centre: the chance of being within +/-0.5 rows is erf( 0.5 / ( sigma sqrt 2 ) ).
    TEST_ASSERT_FLOAT_WITHIN( 0.002f, 0.9545f, rowGridConfidence( centre, 0.25f, 0.0f ) ); // 2 sigma
    TEST_ASSERT_FLOAT_WITHIN( 0.002f, 0.6827f, rowGridConfidence( centre, 0.5f, 0.0f ) );  // 1 sigma
    // On the boundary it is a coin toss however good the fix.
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.5f, rowGridConfidence( edge, 0.05f, 0.0f ) );
    // No error bar, no doubt; a huge one, no idea.
    TEST_ASSERT_FLOAT_WITHIN( 1e-6f, 1.0f, rowGridConfidence( centre, 0.0f, 0.0f ) );
    TEST_ASSERT_TRUE( rowGridConfidence( centre, 5.0f, 0.0f ) < 0.1f );
    // Which half: 8 mm from the channel with 1 mm of error is certain, over the channel it is a coin toss.
    TEST_ASSERT_FLOAT_WITHIN( 0.002f, 0.9545f, rowGridConfidence( centre, 0.25f, 1.0f ) );
    RowPlace overChannel = { 14.0f, 0.01f };
    TEST_ASSERT_FLOAT_WITHIN( 0.01f, 0.5f, rowGridConfidence( overChannel, 0.0f, 1.0f ) );
}

static void test_sigma_along_and_across( void ) {
    RowGrid grid;
    rowGridDefault( &grid, 0.0f, 0.0f );
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 0.5f, rowGridSigmaAlong( &grid, { 1.27f, 9.0f, 9.0f } ) ); // only x counts along the default grid
    TEST_ASSERT_FLOAT_WITHIN( 1e-4f, 9.0f, rowGridSigmaAcross( &grid, { 1.27f, 9.0f, 9.0f } ) );
}

int main( int, char** ) {
    UNITY_BEGIN( );
    RUN_TEST( test_both_halves_on_the_default_grid );
    RUN_TEST( test_ends_channel_and_rails );
    RUN_TEST( test_one_anchor_slides_the_breadboard );
    RUN_TEST( test_anchors_along_the_board_give_angle_and_scale );
    RUN_TEST( test_calibration_targets_fit_the_full_map );
    RUN_TEST( test_a_poor_anchor_counts_for_less_and_is_reported );
    RUN_TEST( test_confidence );
    RUN_TEST( test_sigma_along_and_across );
    return UNITY_END( );
}
