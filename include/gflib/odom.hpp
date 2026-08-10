#pragma once
#include "gflib/pose.hpp"

namespace gflib {

// All robot-specific geometry. Owned by the platform repo and passed in; the
// library never hardcodes any of it. That is what stops you forking this
// library when robot #2 exists.
struct OdomConfig {
    // Signed perpendicular distances from the tracking centre to each pod,
    // inches.
    //
    // THE SIGNS ARE NOT DERIVABLE BY REASONING FROM THIS FILE -- they depend on
    // which side of centre each pod is physically mounted. Get them from the
    // Gate 2 pure-rotation test: if rotating in place walks x/y, an offset is
    // wrong.
    double vertOffsetInches  = 0.0;
    double horizOffsetInches = 0.0;

    // Calibrate from the Gate 2 three-metre run.
    double vertInchesPerCount  = 1.0;
    double horizInchesPerCount = 1.0;
};

inline double countsToInches(double counts, double inchesPerCount) {
    return counts * inchesPerCount;
}

// One integration tick.
//
// Pure: no sensors, no clock, no stored state. The caller owns the sensor task
// and hands this function deltas. That is what makes the whole layer testable
// natively, and it is why porting to the V5 touches nothing in here.
//
// dThetaDeg MUST come from the IMU. Two perpendicular tracking wheels cannot
// resolve heading independently.
Pose odomStep(const Pose& prev,
              double dVertInches, double dHorizInches, double dThetaDeg,
              const OdomConfig& cfg);

} // namespace gflib
