// SPDX-License-Identifier: MIT
#ifndef CAMERA_H
#define CAMERA_H
// ---------------------------------------------------------------------------
// The 3D view's camera: where it looks (the target), from which direction
// (yaw round the vertical, elevation above the board), how far, and how many
// pixels a millimetre at the target gets (zoom). MagView's projection takes
// exactly these five numbers; this decides them.
//
// Modes: FIXED (the user's own viewpoint: orbit, pan and zoom by the
// controls), SWAY and SPIN (the fixed view with the yaw moving by itself),
// TOP (straight down), FOLLOW (the user's viewpoint, but the target rides on
// the probe's point), and POV (the camera IS the probe: at its point, looking
// down the shaft at the board, with the horizon kept level so it cannot roll).
// Every change is smoothed toward its goal with one time constant, so a mode
// switch or a jump of the probe glides rather than cuts.
//
// No Arduino in here; host-tested.
// ---------------------------------------------------------------------------
#include "Vec3.h"

#define CAMERA_TAU_S 0.18f           // smoothing time constant (glide of every change; menu: camera)
#define CAMERA_POV_TURN_TAU_S 0.6f   // POV: the direction follows the shaft this slowly (a probe's angle is noisy; menu: POV turn)
#define CAMERA_POV_MOVE_TAU_S 0.3f   // POV: the position follows the point this slowly (menu: POV move)
#define CAMERA_POV_AHEAD_MM 25.0f    // POV: the target is this far down the shaft from the point
#define CAMERA_POV_ZOOM 5.0f         // POV: pixels per mm at the target (wide view)
#define CAMERA_POV_MIN_TILT_DEG 4.0f // below this the shaft's yaw is meaningless: keep the last
#define CAMERA_SWAY_DEG 18.0f
#define CAMERA_SWAY_PERIOD_S 6.0f
#define CAMERA_SPIN_DEG_PER_S 30.0f
#define CAMERA_MIN_DISTANCE_MM 10.0f
#define CAMERA_MAX_DISTANCE_MM 2000.0f
#define CAMERA_MIN_ZOOM 0.5f
#define CAMERA_MAX_ZOOM 12.0f

enum CameraMode {
    CAMERA_FIXED,
    CAMERA_SWAY,
    CAMERA_SPIN,
    CAMERA_TOP,
    CAMERA_FOLLOW,
    CAMERA_POV,
    CAMERA_MODE_COUNT
};

struct Camera {
    CameraMode mode;
    // What the projection uses (smoothed).
    Vec3 target;
    float yawDeg, elevationDeg, distance, zoom;
    // Where it is heading.
    Vec3 goalTarget;
    float goalYawDeg, goalElevationDeg, goalDistance, goalZoom;
    // The user's viewpoint (FIXED / SWAY / SPIN / FOLLOW start from it; orbit,
    // pan and zoom change it), and the home it resets to.
    Vec3 userTarget;
    float userYawDeg, userElevationDeg, userDistance, userZoom;
    Vec3 homeTarget;
    float homeYawDeg, homeElevationDeg, homeDistance, homeZoom;
    float povYawDeg; // the last usable POV yaw
    bool snapNext;   // the next update lands on the goal at once (no glide)
    // Smoothing (menu levers): every change glides with tauS; in POV the
    // direction turns with povTurnTauS and the position moves with povMoveTauS.
    float tauS, povTurnTauS, povMoveTauS;
};

extern const char* const cameraModeNames[ CAMERA_MODE_COUNT ];

// The home viewpoint (what MagView worked out from the array's extent).
void cameraInit( Camera* c, Vec3 target, float yawDeg, float elevationDeg, float distance, float zoom );
void cameraSetMode( Camera* c, CameraMode mode );
void cameraNextMode( Camera* c );

// The controls, on the user's viewpoint.
void cameraOrbit( Camera* c, float dYawDeg, float dElevationDeg );
void cameraPan( Camera* c, float rightMm, float upMm ); // screen-relative, in the board plane
void cameraZoom( Camera* c, float factor );             // > 1 = closer
void cameraReset( Camera* c );                          // back to home

// One frame. tS is time since boot (for sway/spin); haveProbe, tip and shaft
// (unit, up the probe) drive FOLLOW and POV.
void cameraUpdate( Camera* c, float dtS, float tS, bool haveProbe, Vec3 tip, Vec3 shaft );

// The direction the camera looks along, for the given yaw and elevation: the
// same convention as MagView's projection (depth axis). For tests.
Vec3 cameraLookDirection( float yawDeg, float elevationDeg );

#endif // CAMERA_H
