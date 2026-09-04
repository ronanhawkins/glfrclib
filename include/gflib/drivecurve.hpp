#pragma once
#include "gflib/real.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

// Joystick shaping. kept in the library
// Maintains consistency between different robots
struct DriveCurveConfig {
    // Full-scale joystick reading. 127 on a V5 controller
    real inputMax = 127.0_r;

    // Raw units the stick must leave before anything happens. Covers a stick
    // that does not return to exactly zero; below it the output is hard 0
    real deadband = 5.0_r;

    // Volts commanded the instant the stick leaves the deadband. Without it
    // the first third of stick travel is below the voltage the drivetrain
    // needs to break static friction, and the driver feels dead travel.
    // 0 disables
    real minVolts = 0.0_r;

    real maxVolts = 12.0_r;

    // 0 is linear, 1 is pure cubic.
    real expo = 0.5_r;
};

// Raw joystick in, volts out. Sign preserving, and monotonic in the input for any config a driver can set
inline real driveCurve(real raw, const DriveCurveConfig& cfg) {
    const real span = cfg.inputMax - cfg.deadband;
    if (span <= 0.0_r) return 0.0_r;

    const real mag = std::fabs(raw);
    if (mag <= cfg.deadband) return 0.0_r;

    // Rescaled so the first volt lands exactly at the deadband edge rather
    // than a step below it
    const real u = clamp((mag - cfg.deadband) / span, 0.0_r, 1.0_r);

    const real g = clamp(cfg.expo, 0.0_r, 1.0_r);
    const real shaped = g * u * u * u + (1.0_r - g) * u;

    const real lo = clamp(cfg.minVolts, 0.0_r, cfg.maxVolts);
    const real volts = lo + (cfg.maxVolts - lo) * shaped;

    return raw < 0.0_r ? -volts : volts;
}

} // namespace gflib
