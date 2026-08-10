#pragma once

namespace gflib {

// Field frame: inches, origin at field centre.
struct Pose {
    double x        = 0.0;
    double y        = 0.0;
    double thetaDeg = 0.0;
};

} // namespace gflib
