#include "gflib/pid.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

double pidStep(PidState& s, const PidGains& g, double error, double dtSec){
    if (!std::isfinite(error)) return s.prevOutput;

    // Negated on purpose: NaN <= 0 is false, so the obvious form lets a NaN
    // timestep through and poisons integral and deriv permanently
    if (!(dtSec > 0)) return s.prevOutput;

    if (std::fabs(error)<g.iZone) s.integral+= error * dtSec;
    //doesn't act if far away
    else s.integral = 0;

    // !s.first guard: prevError is 0 on the first call and signbit(0) is
    // false, so this would fire whenever the first error is negative
    if (!s.first && std::signbit(error) != std::signbit(s.prevError)){
        s.integral = 0;
    }

    s.integral = clamp(s.integral, -g.iMax, g.iMax);

    const double deriv = s.first ? 0.0 : (error - s.prevError) / dtSec;
    s.first = false;

    double out = g.kP * error + g.kI * s.integral + g.kD * deriv;

    // Slew first, then saturate
    const double maxDelta = g.slewPerSec * dtSec;
    out = clamp(out, s.prevOutput - maxDelta, s.prevOutput + maxDelta);
    out = clamp(out, g.outMin, g.outMax);

    s.prevError = error;
    s.prevOutput = out;
    return out;
}
}
