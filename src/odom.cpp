#include "gflib/odom.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

// Public api is degrees. Radians appear only inside this function

Pose odomStep(const Pose& prev, double dVertCounts, double dHorizCounts, double dThetaDeg, const OdomConfig& cfg) {

    // Converts counts to inches
    const double dVertInches = countsToInches(dVertCounts, cfg.vertInchesPerCount);
    const double dHorizInches = countsToInches(dHorizCounts, cfg.horizInchesPerCount);

    //convert degrees to radians
    const double dThetaRad = dThetaDeg * kDegToRad;
    double localX, localY;

    if (std::fabs(dThetaRad) < 1e-9) {
        // Straight line. Without this the formula below divides by zero
        // every time the robot drives straight
        localX = dHorizInches;
        localY = dVertInches;
    } 
    else {
        // Chord of the arc travelled. A pod at offset d sweeps an extra
        // d * dThetaRad during rotation, which is not motion
        const double chord = 2.0 * std::sin(dThetaRad / 2.0);
        localX = chord * (dHorizInches / dThetaRad + cfg.horizOffsetInches);
        localY = chord * (dVertInches / dThetaRad + cfg.vertOffsetInches);
    }

    // Rotate into the field frame using the average heading over the tick.
    const double avgRad = prev.thetaDeg * kDegToRad + dThetaRad / 2.0;
    const double s = std::sin(avgRad), c = std::cos(avgRad);

    Pose out;
    // forward.x + right.x
    out.x = prev.x + localY * s + localX * c;
    // forward.y + right.y
    out.y = prev.y + localY * c - localX * s;
    out.thetaDeg = prev.thetaDeg + dThetaDeg;
    return out;
}

} // namespace gflib
