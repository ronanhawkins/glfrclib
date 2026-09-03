#pragma once
#include "gflib/real.hpp"
#include <cmath>

namespace gflib {

// k-prefixed because Arduino.h defines PI/DEG_TO_RAD/RAD_TO_DEG as macros, which ignore namespaces
constexpr real kPi = 3.14159265358979323846_r;
constexpr real kDegToRad = kPi / 180.0_r;
constexpr real kRadToDeg = 180.0_r / kPi;

// [-180, 180). 180 maps to -180
inline real wrapDeg(real a) {
    a = std::fmod(a + 180.0_r, 360.0_r);
    if (a < 0) a += 360.0_r;
    return a - 180.0_r;
}

inline real clamp(real v, real lo, real hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

} // namespace gflib
