#include "gflib/motion.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

bool checkExit(ExitState& s, const ExitConditions& ec, double error, uint32_t nowMs) {
    // Avoid NaN errors
    if (!std::isfinite(error)) {
        s.timedOut = true;
        return true;
    }

    const double absErr = std::fabs(error);

    // Updated every tick
    const bool smallDone = s.smallBand.update(absErr, ec.smallErr, ec.smallTimeMs, nowMs);
    const bool largeDone = s.largeBand.update(absErr, ec.largeErr, ec.largeTimeMs, nowMs);

    // Evaluated even when a band has fired. A caller may DECLINE the exit
    // (MoveToPose does, until the heading lands), and then the timeout is the
    // only thing left to end the motion. Returning early here means timedOut
    // never gets set and such a caller loops forever
    // Unsigned subtraction
    if (nowMs - s.startMs >= ec.timeoutMs) s.timedOut = true;

    return smallDone || largeDone || s.timedOut;
}

namespace {
    // Static friction floor, shared by turning and driving.
    double applyFloor(double out, double error, double magnitude, double minVolts, double cutoff) {
        if (minVolts > 0.0 && magnitude >= cutoff && error != 0.0 && std::fabs(out) < minVolts) {
            return std::copysign(minVolts, error);
        }
        return out;
    }
}

TurnToHeading::TurnToHeading(double targetDeg, const PidGains& gains, const ExitConditions& exit, const TurnConfig& cfg)
    : targetDeg_(targetDeg), gains_(gains), ec_(exit), cfg_(cfg) {}

void TurnToHeading::start(uint32_t nowMs) {
    exitReset(exit_, nowMs);
    pidReset(pid_);
}

MotionStatus TurnToHeading::tick(const Pose& pose, double dtSec, IDriveOutput& drive, uint32_t nowMs) {
    // wrapDeg is mandatory
    const double errorDeg = wrapDeg(targetDeg_ - pose.thetaDeg);

    if (checkExit(exit_, ec_, errorDeg, nowMs)) {
        drive.stop();
        return exit_.timedOut ? MotionStatus::TimedOut : MotionStatus::Settled;
    }

    // CW+ : positive error means turn clockwise
    double out = pidStep(pid_, gains_, errorDeg, dtSec);
    out = applyFloor(out, errorDeg, std::fabs(errorDeg), cfg_.minVolts, ec_.smallErr);
    out = clamp(out, -cfg_.maxVolts, cfg_.maxVolts);

    drive.setLeft(out);
    drive.setRight(-out);
    return MotionStatus::Running;
}

namespace {
    // Shared by MoveToPoint and MoveToPose.
    // aimX/aimY is what we steer at: the target, or the carrot.
    // distInches is to the real target, so the linear term winds down on
    // arrival rather than on reaching the carrot.
    // holdHeadingDeg is what to steer to inside the settle radius. nullptr
    // drops steering. MoveToPose passes its final heading, else heading
    // freezes where it crossed the radius
    // Returns the linear volts commanded, which seeds the next chained motion
    double steerToward(double aimX, double aimY, double distInches,
                     const Pose& pose, const MoveConfig& cfg,
                     PidState& linPid, const PidGains& linG,
                     PidState& angPid, const PidGains& angG,
                     double dtSec, IDriveOutput& drive,
                     double floorCutoffInches, double chainRadiusInches,
                     const double* holdHeadingDeg = nullptr) {

        const double dx = aimX - pose.x;
        const double dy = aimY - pose.y;

        // atan2(dx, dy)
        const double bearingDeg = std::atan2(dx, dy) * kRadToDeg;
        double angErrDeg = wrapDeg(bearingDeg - pose.thetaDeg);

        // Target behind, back into it instead of turning around
        bool reversed = false;
        if (cfg.allowReverse && std::fabs(angErrDeg) > 90.0) {
            angErrDeg = wrapDeg(angErrDeg + 180.0);
            reversed = true;
        }

        // Signed projection, not raw distance
        double linErr = distInches * std::cos(angErrDeg * kDegToRad);
        if (reversed) linErr = -linErr;
        // turn, don't back up
        if (!cfg.allowReverse && linErr < 0.0) linErr = 0.0;

        double lin = pidStep(linPid, linG, linErr, dtSec);
        double ang = 0.0;
        double angErrUsed = 0.0;

        if (distInches >= cfg.settleRadiusInches) {
            angErrUsed = angErrDeg;
            ang = pidStep(angPid, angG, angErrUsed, dtSec);
        } else if (holdHeadingDeg) {
            // Close in, hold the final heading. Same either way, backing
            // in still ends facing the same direction
            angErrUsed = wrapDeg(*holdHeadingDeg - pose.thetaDeg);
            ang = pidStep(angPid, angG, angErrUsed, dtSec);
        } else {
            // Bearing is noise here. Reset, not freeze, or re-entering
            // spikes the derivative off a stale prevError
            pidReset(angPid);
        }

        // Higher floor outside the chain radius, so speed survives the handoff
        if (chainRadiusInches > 0.0 && cfg.chainMinVolts > 0.0) {
            lin = applyFloor(lin, linErr, distInches, cfg.chainMinVolts, chainRadiusInches);
        }

        // Magnitude is unsigned distance, sign is the projected error
        lin = applyFloor(lin, linErr, distInches, cfg.minVolts, floorCutoffInches);

        // Same friction floor for steering. Without it a robot with real
        // friction closes the last inches but not the last few degrees
        if (cfg.headingTolDeg > 0.0) {
            ang = applyFloor(ang, angErrUsed, std::fabs(angErrUsed), cfg.angMinVolts, cfg.headingTolDeg);
        }

        // Angular has priority on saturation drop linear speed, not
        // turning authority
        double headroom = cfg.maxVolts - std::fabs(ang);
        if (headroom < 0.0) headroom = 0.0;
        lin = clamp(lin, -headroom, headroom);

        drive.setLeft(clamp(lin + ang, -cfg.maxVolts, cfg.maxVolts));
        drive.setRight(clamp(lin - ang, -cfg.maxVolts, cfg.maxVolts));
        return lin;
    }
}

MoveToPoint::MoveToPoint(double targetX, double targetY, const PidGains& linGains,
                         const PidGains& angGains, const ExitConditions& exit, const MoveConfig& cfg,
                         const ChainParams& chain)
    : targetX_(targetX), targetY_(targetY), linGains_(linGains), angGains_(angGains), ec_(exit), cfg_(cfg), chain_(chain) {
    // A radius inside the settle band would race checkExit. Arm only outside,
    // so a misconfigured one degrades to a normal settle
    chainArmed_ = chain_.radiusInches > 0.0 && chain_.radiusInches > ec_.smallErr;
}

void MoveToPoint::start(uint32_t nowMs) {
    exitReset(exit_, nowMs);
    pidResetFrom(linPid_, chain_.entryLinVolts);
    pidReset(angPid_);

    // Pass through, in case we hand off on the first tick. Otherwise a
    // segment shorter than the radius drops the next motion back to zero
    lastLin_ = chain_.entryLinVolts;
}

MotionStatus MoveToPoint::tick(const Pose& pose, double dtSec, IDriveOutput& drive, uint32_t nowMs) {
    const double dist = std::hypot(targetX_ - pose.x, targetY_ - pose.y);

    // Hand off early, still moving. No stop() here, that is the whole point
    if (chainArmed_ && dist <= chain_.radiusInches) return MotionStatus::EarlyExit;

    // Unsigned on purpose: done when near the point, whichever way we face
    if (checkExit(exit_, ec_, dist, nowMs)) {
        drive.stop();
        return exit_.timedOut ? MotionStatus::TimedOut : MotionStatus::Settled;
    }

    lastLin_ = steerToward(targetX_, targetY_, dist, pose, cfg_, linPid_, linGains_, angPid_, angGains_,
                           dtSec, drive, ec_.smallErr, chainArmed_ ? chain_.radiusInches : 0.0);
    return MotionStatus::Running;
}

MoveToPose::MoveToPose(double targetX, double targetY, double targetThetaDeg,
                       const PidGains& linGains, const PidGains& angGains,
                       const ExitConditions& exit, const MoveConfig& cfg)
    : targetX_(targetX), targetY_(targetY), targetThetaDeg_(targetThetaDeg),
      linGains_(linGains), angGains_(angGains), ec_(exit), cfg_(cfg) {}

void MoveToPose::start(uint32_t nowMs) {
    exitReset(exit_, nowMs);
    pidReset(linPid_);
    pidReset(angPid_);
    carrotX_ = targetX_;
    carrotY_ = targetY_;
}

// No chaining here: the carrot only collapses onto the target as dist -> 0,
// so an early exit means the final heading has not landed
MotionStatus MoveToPose::tick(const Pose& pose, double dtSec, IDriveOutput& drive, uint32_t nowMs) {
    const double dist = std::hypot(targetX_ - pose.x, targetY_ - pose.y);

    if (checkExit(exit_, ec_, dist, nowMs)) {
        // Near the coordinate is not done. Without this the motion reports
        // success pointing anywhere, and competition code that writes
        // moveToPose(x, y, 90) reasonably expects 90
        const double headErrDeg = std::fabs(wrapDeg(targetThetaDeg_ - pose.thetaDeg));

        if (exit_.timedOut || cfg_.headingTolDeg <= 0.0 || headErrDeg <= cfg_.headingTolDeg) {
            drive.stop();
            return exit_.timedOut ? MotionStatus::TimedOut : MotionStatus::Settled;
        }
        // Otherwise fall through. Inside the settle radius steerToward holds
        // the final heading, so this turns on the spot until the heading
        // lands or the timeout gives up
    }

    // Carrot sits lead*dist behind the target along the target heading.
    // forward(theta) = (sin, cos), so behind is a subtraction. The offset
    // scales with dist, so it collapses onto the target as we arrive
    const double tRad = targetThetaDeg_ * kDegToRad;
    carrotX_ = targetX_ - std::sin(tRad) * dist * cfg_.lead;
    carrotY_ = targetY_ - std::cos(tRad) * dist * cfg_.lead;

    steerToward(carrotX_, carrotY_, dist, pose, cfg_,
                linPid_, linGains_, angPid_, angGains_, dtSec, drive,
                ec_.smallErr, 0.0, &targetThetaDeg_);
    return MotionStatus::Running;
}
}
