#pragma once
#include <cmath>

namespace gflib {

// k-prefixed because Arduino.h defines PI/DEG_TO_RAD/RAD_TO_DEG as macros, which ignore namespaces
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// [-180, 180). 180 maps to -180
inline double wrapDeg(double a) {
    a = std::fmod(a + 180.0, 360.0);
    if (a < 0) a += 360.0;
    return a - 180.0;
}

inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

} // namespace gflib
