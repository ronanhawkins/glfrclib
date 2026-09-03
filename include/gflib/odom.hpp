#pragma once
#include "gflib/real.hpp"
#include "gflib/pose.hpp"

namespace gflib {

// All robot geometry. Passed in, never hardcoded
struct OdomConfig {
    // Rotation of the whole pod pair about the tracking centreThe pods stay 90 degrees apart: "vert"
    // measures along +podAngleDeg from forward, "horiz" 90 degrees CW of that.
    // 0 is the usual forward/sideways mounting; 45 is a diamond mounting.
    // The names stay vert/horiz at any angle -- read them as pod A and pod B
    real podAngleDeg = 0.0_r;

    // Signed perpendicular distance from tracking centre to each pod, inches.
    // Measured perpendicular to that pod's own direction of travel, so these
    // rotate with podAngleDeg.
    real vertOffsetInches = 0.0_r;
    real horizOffsetInches = 0.0_r;

    //"Count" is whatever the
    // encoder reports: ticks, or degrees on a V5 rotation sensor.
    // Applied by odomStep so one place scales raw readings
    real vertInchesPerCount = 1.0_r;
    real horizInchesPerCount = 1.0_r;
};

inline real countsToInches(real counts, real inchesPerCount) {
    return counts * inchesPerCount;
}


Pose odomStep(const Pose& prev, real dVertCounts, real dHorizCounts, real dThetaDeg, const OdomConfig& cfg);

} // namespace gflib
