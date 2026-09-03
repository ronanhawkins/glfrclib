#pragma once
#include "gflib/real.hpp"
#include <limits>

namespace gflib {

struct PidGains {
    real kP = 0.0_r, kI = 0.0_r, kD = 0.0_r;

    //Initialized to infinity (0 can cause issues)
    real iZone = std::numeric_limits<real>::infinity();
    real iMax = std::numeric_limits<real>::infinity();
    real slewPerSec = std::numeric_limits<real>::infinity();

    // Applied inside pidStep so prevOutput only holds real voltages
    real outMin = -12.0_r, outMax = 12.0_r;
};

// every field needs a default so pidReset works.
struct PidState {
    real integral = 0.0_r;
    real prevError = 0.0_r;
    real prevOutput = 0.0_r;
    bool first = true;
};

real pidStep(PidState& s, const PidGains& g, real error, real dtSec);

inline void pidReset(PidState& s) { s = PidState{}; }

// Seeds the slew limiter, leaving the derivative disarmed. A chained motion
// starts from a moving robot, and prevOutput = 0 would ramp it up from zero
inline void pidResetFrom(PidState& s, real output) {
    s = PidState{};
    s.prevOutput = output;
}
} // namespace gflib
