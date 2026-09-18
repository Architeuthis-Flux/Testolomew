// SPDX-License-Identifier: MIT
#include "RowGrid.h"

#include <math.h>

#define MIN_SPREAD_MM 10.0f    // anchors closer together than this say nothing about angle or scale
#define MIN_SIGMA_MM 0.1f      // no anchor is trusted more than this
#define MIN_ANCHORS_FOR_FULL 4 // the full map has six unknowns; three anchors would fit exactly and prove nothing

void rowGridDefault( RowGrid* grid, float row1X, float channelY ) {
    grid->ax = 1.0f;
    grid->ay = 0.0f;
    grid->a0 = -row1X;
    grid->cx = 0.0f;
    grid->cy = 1.0f;
    grid->c0 = -channelY;
}

RowPlace rowGridPlace( const RowGrid* grid, Vec3 position ) {
    RowPlace place;
    place.along = 1.0f + ( grid->ax * position.x + grid->ay * position.y + grid->a0 ) / ROWGRID_PITCH_MM;
    place.acrossMm = grid->cx * position.x + grid->cy * position.y + grid->c0;
    return place;
}

Vec3 rowGridToBoard( const RowGrid* grid, float along, float acrossMm ) {
    float u = ( along - 1.0f ) * ROWGRID_PITCH_MM - grid->a0;
    float v = acrossMm - grid->c0;
    float det = grid->ax * grid->cy - grid->ay * grid->cx;
    Vec3 position = { ( grid->cy * u - grid->ay * v ) / det, ( -grid->cx * u + grid->ax * v ) / det, 0.0f };
    return position;
}

RowPlace rowGridHolePlace( int row, int hole ) {
    bool bottom = row > ROWGRID_ROWS_PER_HALF;
    RowPlace place;
    place.along = (float)( bottom ? row - ROWGRID_ROWS_PER_HALF : row );
    place.acrossMm = ( ROWGRID_INNER_HOLE_MM + ( hole - 1 ) * ROWGRID_PITCH_MM ) * ( bottom ? -1.0f : 1.0f );
    return place;
}

int rowGridRow( float along, bool bottomHalf ) {
    if ( along < -0.5f || along > ROWGRID_ROWS_PER_HALF + 1.5f ) {
        return 0;
    }
    int row = (int)floorf( along + 0.5f );
    if ( row < 1 ) {
        row = 1;
    }
    if ( row > ROWGRID_ROWS_PER_HALF ) {
        row = ROWGRID_ROWS_PER_HALF;
    }
    return bottomHalf ? row + ROWGRID_ROWS_PER_HALF : row;
}

int rowGridHole( float acrossMm ) {
    float fromFirst = ( fabsf( acrossMm ) - ROWGRID_INNER_HOLE_MM ) / ROWGRID_PITCH_MM; // 0 = hole 1
    int hole = (int)floorf( fromFirst + 0.5f ) + 1;
    return ( hole < 1 || hole > ROWGRID_HOLES_PER_ROW ) ? 0 : hole;
}

// The fit gives one bar per axis and no correlation between them, so these
// treat x and y as independent. On the default grid they are just sigma x and y.
float rowGridSigmaAlong( const RowGrid* grid, Vec3 sigmaMm ) {
    float sx = sigmaMm.x * grid->ax;
    float sy = sigmaMm.y * grid->ay;
    return sqrtf( sx * sx + sy * sy ) / ROWGRID_PITCH_MM;
}

float rowGridSigmaAcross( const RowGrid* grid, Vec3 sigmaMm ) {
    float sx = sigmaMm.x * grid->cx;
    float sy = sigmaMm.y * grid->cy;
    return sqrtf( sx * sx + sy * sy );
}

float rowGridConfidence( RowPlace place, float sigmaRows, float sigmaAcrossMm ) {
    float confidence = 1.0f;
    if ( sigmaRows > 1e-4f ) {
        float offset = place.along - floorf( place.along + 0.5f ); // -0.5 .. 0.5
        float k = 1.0f / ( sigmaRows * sqrtf( 2.0f ) );
        confidence = 0.5f * ( erff( ( 0.5f - offset ) * k ) + erff( ( 0.5f + offset ) * k ) );
    }
    if ( sigmaAcrossMm > 1e-4f ) {
        confidence *= 0.5f * ( 1.0f + erff( fabsf( place.acrossMm ) / ( sigmaAcrossMm * sqrtf( 2.0f ) ) ) );
    }
    return confidence;
}

float rowGridScaleAlong( const RowGrid* grid ) {
    return 1.0f / sqrtf( grid->ax * grid->ax + grid->ay * grid->ay );
}

float rowGridScaleAcross( const RowGrid* grid ) {
    return 1.0f / sqrtf( grid->cx * grid->cx + grid->cy * grid->cy );
}

float rowGridAngleDeg( const RowGrid* grid ) {
    return atan2f( grid->ay, grid->ax ) * 180.0f / (float)M_PI;
}

void rowGridCalibrationTarget( int index, int* row, int* hole ) {
    // Along the far half and back along the near one, so the hand moves least.
    static const int rows[ ROWGRID_CALIBRATION_TARGETS / 2 ] = { 1, 15, 30, 60, 45, 31 };
    *row = rows[ ( index / 2 ) % ( ROWGRID_CALIBRATION_TARGETS / 2 ) ];
    *hole = ( index % 2 == 0 ) ? 1 : ROWGRID_HOLES_PER_ROW;
}

// ---- fitting the grid to anchors ---------------------------------------------

// Solve the n x n system m p = rhs in place (n <= 4), Gaussian elimination with
// pivoting. false if it is singular.
static bool solveSmall( double m[ 4 ][ 4 ], double* rhs, int n ) {
    for ( int col = 0; col < n; col++ ) {
        int pivot = col;
        for ( int r = col + 1; r < n; r++ ) {
            if ( fabs( m[ r ][ col ] ) > fabs( m[ pivot ][ col ] ) ) {
                pivot = r;
            }
        }
        if ( fabs( m[ pivot ][ col ] ) < 1e-12 ) {
            return false;
        }
        for ( int c = 0; c < n; c++ ) {
            double t = m[ col ][ c ];
            m[ col ][ c ] = m[ pivot ][ c ];
            m[ pivot ][ c ] = t;
        }
        double t = rhs[ col ];
        rhs[ col ] = rhs[ pivot ];
        rhs[ pivot ] = t;
        for ( int r = col + 1; r < n; r++ ) {
            double k = m[ r ][ col ] / m[ col ][ col ];
            for ( int c = col; c < n; c++ ) {
                m[ r ][ c ] -= k * m[ col ][ c ];
            }
            rhs[ r ] -= k * rhs[ col ];
        }
    }
    for ( int r = n - 1; r >= 0; r-- ) {
        for ( int c = r + 1; c < n; c++ ) {
            rhs[ r ] -= m[ r ][ c ] * rhs[ c ];
        }
        rhs[ r ] /= m[ r ][ r ];
    }
    return true;
}

static float weightOf( const RowAnchor* anchor ) {
    float sigma = anchor->sigmaMm > MIN_SIGMA_MM ? anchor->sigmaMm : MIN_SIGMA_MM;
    return 1.0f / ( sigma * sigma );
}

// Where the anchor's hole is on the breadboard, mm.
static void anchorTarget( const RowAnchor* anchor, double* along, double* across ) {
    RowPlace place = rowGridHolePlace( anchor->row, anchor->hole );
    *along = ( place.along - 1.0f ) * ROWGRID_PITCH_MM;
    *across = place.acrossMm;
}

RowFitReport rowGridFit( RowGrid* grid, const RowAnchor* anchors, int count, float* residualMm ) {
    RowFitReport report = { ROWFIT_NONE, 0.0f, 0.0f, 0 };
    if ( count < 1 ) {
        return report;
    }

    // Work around the anchors' middle: small numbers keep their digits.
    double sumW = 0, meanX = 0, meanY = 0, meanU = 0, meanV = 0;
    double uMin = 1e9, uMax = -1e9, vMin = 1e9, vMax = -1e9;
    for ( int k = 0; k < count; k++ ) {
        double w = weightOf( &anchors[ k ] ), u, v;
        anchorTarget( &anchors[ k ], &u, &v );
        sumW += w;
        meanX += w * anchors[ k ].position.x;
        meanY += w * anchors[ k ].position.y;
        meanU += w * u;
        meanV += w * v;
        uMin = u < uMin ? u : uMin;
        uMax = u > uMax ? u : uMax;
        vMin = v < vMin ? v : vMin;
        vMax = v > vMax ? v : vMax;
    }
    meanX /= sumW;
    meanY /= sumW;
    meanU /= sumW;
    meanV /= sumW;
    bool spreadAlong = uMax - uMin >= MIN_SPREAD_MM;
    bool spreadAcross = vMax - vMin >= MIN_SPREAD_MM;

    // The straight-line part. Around the middle the offsets drop out, so the
    // full map is two 2-unknown fits ( u = ax x + ay y, v = cx x + cy y ) and
    // the one-scale map is one ( u = a x + b y, v = -b x + a y ).
    double ax = grid->ax, ay = grid->ay, cx = grid->cx, cy = grid->cy;
    report.kind = ROWFIT_SHIFT;
    if ( spreadAlong && spreadAcross && count >= MIN_ANCHORS_FOR_FULL ) {
        double m[ 4 ][ 4 ] = { { 0 } }, n[ 4 ][ 4 ] = { { 0 } }, ru[ 4 ] = { 0 }, rv[ 4 ] = { 0 };
        for ( int k = 0; k < count; k++ ) {
            double w = weightOf( &anchors[ k ] ), u, v;
            anchorTarget( &anchors[ k ], &u, &v );
            double x = anchors[ k ].position.x - meanX, y = anchors[ k ].position.y - meanY;
            m[ 0 ][ 0 ] += w * x * x;
            m[ 0 ][ 1 ] += w * x * y;
            m[ 1 ][ 1 ] += w * y * y;
            ru[ 0 ] += w * x * ( u - meanU );
            ru[ 1 ] += w * y * ( u - meanU );
            rv[ 0 ] += w * x * ( v - meanV );
            rv[ 1 ] += w * y * ( v - meanV );
        }
        m[ 1 ][ 0 ] = m[ 0 ][ 1 ];
        for ( int i = 0; i < 2; i++ ) {
            for ( int j = 0; j < 2; j++ ) {
                n[ i ][ j ] = m[ i ][ j ];
            }
        }
        if ( solveSmall( m, ru, 2 ) && solveSmall( n, rv, 2 ) ) {
            ax = ru[ 0 ];
            ay = ru[ 1 ];
            cx = rv[ 0 ];
            cy = rv[ 1 ];
            report.kind = ROWFIT_FULL;
        }
    } else if ( spreadAlong || spreadAcross ) {
        double sxx = 0, su = 0, sv = 0;
        for ( int k = 0; k < count; k++ ) {
            double w = weightOf( &anchors[ k ] ), u, v;
            anchorTarget( &anchors[ k ], &u, &v );
            double x = anchors[ k ].position.x - meanX, y = anchors[ k ].position.y - meanY;
            sxx += w * ( x * x + y * y );
            su += w * ( x * ( u - meanU ) + y * ( v - meanV ) );
            sv += w * ( y * ( u - meanU ) - x * ( v - meanV ) );
        }
        if ( sxx > 1e-9 ) {
            ax = su / sxx;
            ay = sv / sxx;
            cx = -ay;
            cy = ax;
            report.kind = ROWFIT_SCALED;
        }
    }

    // The offsets put the anchors' middle on the holes' middle.
    grid->ax = (float)ax;
    grid->ay = (float)ay;
    grid->cx = (float)cx;
    grid->cy = (float)cy;
    grid->a0 = (float)( meanU - ax * meanX - ay * meanY );
    grid->c0 = (float)( meanV - cx * meanX - cy * meanY );

    double sumSq = 0;
    for ( int k = 0; k < count; k++ ) {
        double u, v;
        anchorTarget( &anchors[ k ], &u, &v );
        RowPlace place = rowGridPlace( grid, anchors[ k ].position );
        double du = ( place.along - 1.0f ) * ROWGRID_PITCH_MM - u, dv = place.acrossMm - v;
        float miss = (float)sqrt( du * du + dv * dv );
        if ( residualMm != nullptr ) {
            residualMm[ k ] = miss;
        }
        if ( miss > report.worstMm ) {
            report.worstMm = miss;
            report.worst = k;
        }
        sumSq += du * du + dv * dv;
    }
    report.rmsMm = (float)sqrt( sumSq / count );
    return report;
}
