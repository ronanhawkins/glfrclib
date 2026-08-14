#pragma once
#include "gflib/pose.hpp"

namespace gflib {

// All robot geometry. Passed in, never hardcoded
struct OdomConfig {
    // Rotation of the whole pod pair about the tracking centreThe pods stay 90 degrees apart: "vert"
    // measures along +podAngleDeg from forward, "horiz" 90 degrees CW of that.
    // 0 is the usual forward/sideways mounting; 45 is a diamond mounting.
    // The names stay vert/horiz at any angle -- read them as pod A and pod B
    double podAngleDeg = 0.0;

    // Signed perpendicular distance from tracking centre to each pod, inches.
    // Measured perpendicular to that pod's own direction of travel, so these
    // rotate with podAngleDeg.
    double vertOffsetInches = 0.0;
    double horizOffsetInches = 0.0;

    //"Count" is whatever the
    // encoder reports: ticks, or degrees on a V5 rotation sensor.
    // Applied by odomStep so one place scales raw readings
    double vertInchesPerCount = 1.0;
    double horizInchesPerCount = 1.0;
};

inline double countsToInches(double counts, double inchesPerCount) {
    return counts * inchesPerCount;
}


Pose odomStep(const Pose& prev, double dVertCounts, double dHorizCounts, double dThetaDeg, const OdomConfig& cfg);

} // namespace gflib
