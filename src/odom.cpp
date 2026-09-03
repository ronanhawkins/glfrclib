#include "gflib/odom.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

// Public api is degrees. Radians appear only inside this function

Pose odomStep(const Pose& prev, real dVertCounts, real dHorizCounts, real dThetaDeg, const OdomConfig& cfg) {

    // Converts counts to inches
    const real dVertInches = countsToInches(dVertCounts, cfg.vertInchesPerCount);
    const real dHorizInches = countsToInches(dHorizCounts, cfg.horizInchesPerCount);

    //convert degrees to radians
    const real dThetaRad = dThetaDeg * kDegToRad;

    const real aRad = cfg.podAngleDeg * kDegToRad;
    const real sa = std::sin(aRad), ca = std::cos(aRad);

    // vert measures along (sa, ca) in (right, forward); horiz along (ca, -sa)
    auto toRobotX = [&](real v, real h) { return v * sa + h * ca; };
    auto toRobotY = [&](real v, real h) { return v * ca - h * sa; };

    real localX, localY;

    if (std::fabs(dThetaRad) < 1e-9_r) {
        // Straight line. Without this the formula below divides by zero
        // every time the robot drives straight
        localX = toRobotX(dVertInches, dHorizInches);
        localY = toRobotY(dVertInches, dHorizInches);
    }
    else {
        // Chord of the arc travelled. A pod at offset d sweeps an extra
        // d * dThetaRad during rotation, which is not motion.
        // Each radius component is resolved in its own pod's direction first,
        // then the pair is rotated into the robot frame. chord is a scalar, so
        // it commutes with that rotation
        const real chord = 2.0_r * std::sin(dThetaRad / 2.0_r);
        const real rVert = dVertInches / dThetaRad + cfg.vertOffsetInches;
        const real rHoriz = dHorizInches / dThetaRad + cfg.horizOffsetInches;
        localX = chord * toRobotX(rVert, rHoriz);
        localY = chord * toRobotY(rVert, rHoriz);
    }

    // Rotate into the field frame using the average heading over the tick.
    const real avgRad = prev.thetaDeg * kDegToRad + dThetaRad / 2.0_r;
    const real s = std::sin(avgRad), c = std::cos(avgRad);

    Pose out;
    // forward.x + right.x
    out.x = prev.x + localY * s + localX * c;
    // forward.y + right.y
    out.y = prev.y + localY * c - localX * s;
    out.thetaDeg = prev.thetaDeg + dThetaDeg;
    return out;
}

} // namespace gflib
