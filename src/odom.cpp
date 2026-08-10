#include "gflib/odom.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

// Public api is degrees. Radians appear only inside this function

Pose odomStep(const Pose& prev, double dVertInches, double dHorizInches, double dThetaDeg, const OdomConfig& cfg) {

    //convert degrees to radians
    const double dThetaRad = dThetaDeg * kDegToRad;
    double localX, localY;

    if (std::fabs(dThetaRad) < 1e-9) {
        // Straight line. Without this branch the formula below divides by zero
        // every time the robot drives straight -- i.e. usually.
        localX = dHorizInches;
        localY = dVertInches;
    } 
    else {
        // Chord of the arc actually travelled. The offset terms account for the
        // tracking wheels not sitting at the centre of rotation: a pod at
        // perpendicular offset d sweeps an extra arc length d * dThetaRad
        // during rotation, which is not robot motion and must come back out.
        const double chord = 2.0 * std::sin(dThetaRad / 2.0);
        localX = chord * (dHorizInches / dThetaRad + cfg.horizOffsetInches);
        localY = chord * (dVertInches  / dThetaRad + cfg.vertOffsetInches);
    }

    // Rotate into the field frame using the AVERAGE heading over the tick.
    // Using prev.thetaDeg alone biases every turning movement.
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
