#include "gflib/motion.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

    bool checkExit(ExitState& s, const ExitConditions& ec, double error, uint32_t nowMs) {
        const double absErr = std::fabs(error);

        // Updated every tick
        const bool smallDone = s.small.update(absErr, ec.smallErr, ec.smallTimeMs, nowMs);
        const bool largeDone = s.large.update(absErr, ec.largeErr, ec.largeTimeMs, nowMs);
        if (smallDone || largeDone) return true;

        // Unsigned subtraction
        if (nowMs - s.startMs >= ec.timeoutMs) {
            s.timedOut = true;
            return true;
        }
        return false;
    }

    TurnToHeading::TurnToHeading(double targetDeg, const PidGains& gains, const ExitConditions& exit)
        : targetDeg_(targetDeg), gains_(gains), ec_(exit) {}

    void TurnToHeading::start(uint32_t nowMs) {
        exitReset(exit_, nowMs);
        pidReset(pid_);
    }

    bool TurnToHeading::tick(const Pose& pose, double dtSec, IDriveOutput& drive, uint32_t nowMs) {
        // wrapDeg is mandatory
        const double errorDeg = wrapDeg(targetDeg_ - pose.thetaDeg);

        if (checkExit(exit_, ec_, errorDeg, nowMs)) {
            drive.stop();
            return true;
        }

        // CW+ : positive error means turn clockwise
        const double out = pidStep(pid_, gains_, errorDeg, dtSec);
        drive.setLeft(out);
        drive.setRight(-out);
        return false;
    }
}
