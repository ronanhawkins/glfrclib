#pragma once
#include "gflib/real.hpp"

namespace gflib {

// Field frame: inches, origin at field centre.
// Clockwise positive. 0 forward, 90 right, 180, 270 left
struct Pose {
    real x = 0.0_r;
    real y = 0.0_r;
    real thetaDeg = 0.0_r;
};

// Field frame, same axes as Pose. Inches per second and degrees per second.
// Field frame rather than body frame so a consumer can extrapolate with x += vx * dt
struct Velocity {
    real vx = 0.0_r;
    real vy = 0.0_r;
    real omegaDegPerSec = 0.0_r;
};

} // namespace gflib
