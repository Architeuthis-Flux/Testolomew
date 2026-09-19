// SPDX-License-Identifier: MIT
#include "Camera.h"

#include <math.h>

#define DEG ( (float)M_PI / 180.0f )

const char* const cameraModeNames[ CAMERA_MODE_COUNT ] = { "fixed", "sway", "spin", "top", "follow", "POV" };

static float clampf( float v, float lo, float hi ) {
    return v < lo ? lo : ( v > hi ? hi : v );
}

static float wrapDeg( float a ) {
    while ( a > 180.0f )
        a -= 360.0f;
    while ( a < -180.0f )
        a += 360.0f;
    return a;
}

void cameraInit( Camera* c, Vec3 target, float yawDeg, float elevationDeg, float distance, float zoom ) {
    Camera empty = { };
    *c = empty;
    c->mode = CAMERA_FIXED;
    c->homeTarget = target;
    c->homeYawDeg = yawDeg;
    c->homeElevationDeg = elevationDeg;
    c->homeDistance = distance;
    c->homeZoom = zoom;
    cameraReset( c );
    c->target = c->goalTarget = target;
    c->yawDeg = c->goalYawDeg = yawDeg;
    c->elevationDeg = c->goalElevationDeg = elevationDeg;
    c->distance = c->goalDistance = distance;
    c->zoom = c->goalZoom = zoom;
    c->povYawDeg = yawDeg;
    c->tauS = CAMERA_TAU_S;
    c->povTurnTauS = CAMERA_POV_TURN_TAU_S;
    c->povMoveTauS = CAMERA_POV_MOVE_TAU_S;
}

void cameraReset( Camera* c ) {
    c->userTarget = c->homeTarget;
    c->userYawDeg = c->homeYawDeg;
    c->userElevationDeg = c->homeElevationDeg;
    c->userDistance = c->homeDistance;
    c->userZoom = c->homeZoom;
}

void cameraSetMode( Camera* c, CameraMode mode ) {
    c->mode = mode;
}

void cameraNextMode( Camera* c ) {
    c->mode = (CameraMode)( ( c->mode + 1 ) % CAMERA_MODE_COUNT );
}

void cameraOrbit( Camera* c, float dYawDeg, float dElevationDeg ) {
    c->userYawDeg = wrapDeg( c->userYawDeg + dYawDeg );
    c->userElevationDeg = clampf( c->userElevationDeg + dElevationDeg, 2.0f, 89.9f );
}

// Screen right and up, carried into the board plane by the current yaw:
// right on screen is along x1 = (cos yaw, sin yaw), up on screen (for a
// camera looking down at the board) is along y1 = (-sin yaw, cos yaw).
void cameraPan( Camera* c, float rightMm, float upMm ) {
    float s = sinf( c->yawDeg * DEG ), co = cosf( c->yawDeg * DEG );
    c->userTarget.x += rightMm * co - upMm * s;
    c->userTarget.y += rightMm * s + upMm * co;
}

void cameraZoom( Camera* c, float factor ) {
    c->userZoom = clampf( c->userZoom * factor, CAMERA_MIN_ZOOM, CAMERA_MAX_ZOOM );
}

Vec3 cameraLookDirection( float yawDeg, float elevationDeg ) {
    float sy = sinf( yawDeg * DEG ), cy = cosf( yawDeg * DEG );
    float se = sinf( elevationDeg * DEG ), ce = cosf( elevationDeg * DEG );
    Vec3 d = { -sy * ce, cy * ce, -se };
    return d;
}

static void glide( float* value, float goal, float k ) {
    *value += k * ( goal - *value );
}

void cameraUpdate( Camera* c, float dtS, float tS, bool haveProbe, Vec3 tip, Vec3 shaft ) {
    // The goals.
    c->goalTarget = c->userTarget;
    c->goalYawDeg = c->userYawDeg;
    c->goalElevationDeg = c->userElevationDeg;
    c->goalDistance = c->userDistance;
    c->goalZoom = c->userZoom;
    switch ( c->mode ) {
    case CAMERA_SWAY:
        c->goalYawDeg = c->userYawDeg + CAMERA_SWAY_DEG * sinf( tS * 2.0f * (float)M_PI / CAMERA_SWAY_PERIOD_S );
        break;
    case CAMERA_SPIN:
        c->goalYawDeg = fmodf( tS * CAMERA_SPIN_DEG_PER_S, 360.0f );
        break;
    case CAMERA_TOP:
        c->goalYawDeg = 0.0f;
        c->goalElevationDeg = 89.9f;
        break;
    case CAMERA_FOLLOW:
        if ( haveProbe ) {
            c->goalTarget = tip;
        }
        break;
    case CAMERA_POV:
        if ( haveProbe ) {
            // At the point, looking down the shaft: the target is ahead down
            // the shaft (never below the board), the camera CAMERA_POV_AHEAD
            // back up it - at the point.
            Vec3 ahead = { tip.x - CAMERA_POV_AHEAD_MM * shaft.x, tip.y - CAMERA_POV_AHEAD_MM * shaft.y, tip.z - CAMERA_POV_AHEAD_MM * shaft.z };
            if ( ahead.z < 1.0f )
                ahead.z = 1.0f;
            c->goalTarget = ahead;
            float lean = sqrtf( shaft.x * shaft.x + shaft.y * shaft.y );
            if ( lean > sinf( CAMERA_POV_MIN_TILT_DEG * DEG ) ) {
                c->povYawDeg = atan2f( shaft.x, -shaft.y ) / DEG;
            }
            c->goalYawDeg = c->povYawDeg;
            c->goalElevationDeg = asinf( clampf( shaft.z, -1.0f, 1.0f ) ) / DEG;
            c->goalDistance = CAMERA_POV_AHEAD_MM;
            c->goalZoom = CAMERA_POV_ZOOM;
        }
        break;
    default:
        break;
    }
    c->goalDistance = clampf( c->goalDistance, CAMERA_MIN_DISTANCE_MM, CAMERA_MAX_DISTANCE_MM );

    // Glide there (angles by the short way round). POV moves and turns with
    // its own, slower constants: it is the probe's own noise it is hiding.
    bool pov = c->mode == CAMERA_POV && haveProbe;
    float tau = c->tauS > 0.01f ? c->tauS : 0.01f;
    float moveTau = pov && c->povMoveTauS > 0.01f ? c->povMoveTauS : tau;
    float turnTau = pov && c->povTurnTauS > 0.01f ? c->povTurnTauS : tau;
    float k = c->snapNext ? 1.0f : 1.0f - expf( -dtS / tau );
    float kMove = c->snapNext ? 1.0f : 1.0f - expf( -dtS / moveTau );
    float kTurn = c->snapNext ? 1.0f : 1.0f - expf( -dtS / turnTau );
    c->snapNext = false;
    glide( &c->target.x, c->goalTarget.x, kMove );
    glide( &c->target.y, c->goalTarget.y, kMove );
    glide( &c->target.z, c->goalTarget.z, kMove );
    c->yawDeg = wrapDeg( c->yawDeg + kTurn * wrapDeg( c->goalYawDeg - c->yawDeg ) );
    glide( &c->elevationDeg, c->goalElevationDeg, kTurn );
    glide( &c->distance, c->goalDistance, k );
    glide( &c->zoom, c->goalZoom, k );
}
