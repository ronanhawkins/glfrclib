#pragma once
#include <limits>

namespace gflib {

struct PidGains {
    double kP = 0.0, kI = 0.0, kD = 0.0;

    //Initialized to infinity (0 can cause issues)
    double iZone = std::numeric_limits<double>::infinity();
    double iMax = std::numeric_limits<double>::infinity();
    double slewPerSec = std::numeric_limits<double>::infinity();

    // Applied inside pidStep so prevOutput only holds real voltages
    double outMin = -12.0, outMax = 12.0;
};

// every field needs a default so pidReset works.
struct PidState {
    double integral = 0.0;
    double prevError = 0.0;
    double prevOutput = 0.0;
    bool first = true;
};

double pidStep(PidState& s, const PidGains& g, double error, double dtSec);

inline void pidReset(PidState& s) { s = PidState{}; }
} // namespace gflib
