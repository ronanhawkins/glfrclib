#pragma once

namespace gflib {

// Field frame: inches, origin at field centre.
// Clockwise positive. 0 forward, 90 right, 180, 270 left
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double thetaDeg = 0.0;
};

} // namespace gflib
