#pragma once
#include "gflib/pose.hpp"

namespace gflib {

// All robot geometry. Passed in, never hardcoded
struct OdomConfig {
    // Signed perpendicular distance from tracking centre to each pod, inches.
    // Signs depend on mounting side
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
