#include "gflib/drivetrain.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

Drivetrain::Drivetrain(IPoseSource& source, IDriveOutput& drive, IClock& clock, const DrivetrainConfig& cfg)
    : source_(source), drive_(drive), clock_(clock), cfg_(cfg) {}

bool Drivetrain::setPose(real x, real y, real thetaDeg, uint32_t timeoutMs) {
    Pose p;
    p.x = x;
    p.y = y;
    p.thetaDeg = thetaDeg;

    const PoseSetResult r = source_.setPose(p);
    if (r == PoseSetResult::Applied) return true;
    if (r == PoseSetResult::Rejected) return false;

    // the pose owner is another processor and has not confirmed yet
    const uint32_t startMs = clock_.millisNow();
    while (source_.poseSetPending()) {
        if (linkElapsedMs(clock_.millisNow(), startMs) >= timeoutMs) return false;
        clock_.sleepMs(cfg_.loopMs);
        source_.update();
    }
    return true;
}

ExitConditions Drivetrain::withTimeout(const ExitConditions& base, uint32_t timeoutMs) const {
    ExitConditions ec = base;
    ec.timeoutMs = timeoutMs;
    return ec;
}

MotionStatus Drivetrain::runMotion(IMotion& motion) {
    // Not cleared here. A cancel raised between motions would otherwise be
    // wiped by the next one, which is the wrong way for an estop to fail
    if (cancelled_) {
        drive_.stop();
        return MotionStatus::Cancelled;
    }

    // Pose must be current before the first tick, or the motion starts on
    // whatever the last update left behind
    update();

    uint32_t nowMs = clock_.millisNow();
    uint32_t lastMs = nowMs;
    motion.start(nowMs);

    MotionStatus st = MotionStatus::Running;
    while (!cancelled_) {
        update();
        nowMs = clock_.millisNow();

        // Checked every tick, before the pose is used. A dead link means we
        // no longer know where the robot is
        if (!source_.healthy(nowMs)) {
            drive_.stop();
            chainEntryVolts_ = 0.0_r;
            return MotionStatus::PoseUnhealthy;
        }

        // Measured, not nominal. sleepMs(loopMs) yields for AT LEAST loopMs,
        // and update() plus two PIDs are not free, so the real period runs
        // long and jittery on hardware. Feeding the nominal value scales every
        // derivative, integral and slew step by that error
        const real dtSec = (nowMs - lastMs) / 1000.0_r;
        lastMs = nowMs;

        // Identity while poseMaxExtrapMs is 0, which is the default and the
        // only setting the existing tuning has ever seen
        st = motion.tick(getPoseExtrapolated(nowMs), dtSec, drive_, nowMs);
        if (st != MotionStatus::Running) break;
        clock_.sleepMs(cfg_.loopMs);
    }

    // Cancel always stops, chaining or not
    if (cancelled_) {
        drive_.stop();
        return MotionStatus::Cancelled;
    }

    // EarlyExit deliberately leaves the motors running for the next motion
    if (st != MotionStatus::EarlyExit) drive_.stop();
    return st;
}

namespace {
bool ok(MotionStatus st) {
    return st == MotionStatus::Settled || st == MotionStatus::EarlyExit;
}
}

bool Drivetrain::turnToHeading(real thetaDeg, const ExitConditions& exit) {
    TurnToHeading m(thetaDeg, cfg_.angular, exit, cfg_.turn);
    chainEntryVolts_ = 0.0_r;        // a turn ends stopped
    return ok(runMotion(m));
}

bool Drivetrain::turnToHeading(real thetaDeg, uint32_t timeoutMs) {
    return turnToHeading(thetaDeg, withTimeout(cfg_.angularExit, timeoutMs));
}

bool Drivetrain::moveToPoint(real x, real y, const ExitConditions& exit, real chainRadiusInches) {
    MoveToPoint m(x, y, cfg_.lateral, cfg_.angular, exit, cfg_.move,
                  ChainParams{chainRadiusInches, chainEntryVolts_});
    const MotionStatus st = runMotion(m);

    // Only a handoff carries speed forward; anything else left the robot stopped
    chainEntryVolts_ = (st == MotionStatus::EarlyExit) ? m.lastLinearVolts() : 0.0_r;
    return ok(st);
}

bool Drivetrain::moveToPoint(real x, real y, uint32_t timeoutMs, real chainRadiusInches) {
    return moveToPoint(x, y, withTimeout(cfg_.lateralExit, timeoutMs), chainRadiusInches);
}

bool Drivetrain::moveToPose(real x, real y, real thetaDeg, const ExitConditions& exit) {
    MoveToPose m(x, y, thetaDeg, cfg_.lateral, cfg_.angular, exit, cfg_.move);
    chainEntryVolts_ = 0.0_r;        // never chains, ends stopped
    return ok(runMotion(m));
}

bool Drivetrain::moveToPose(real x, real y, real thetaDeg, uint32_t timeoutMs) {
    return moveToPose(x, y, thetaDeg, withTimeout(cfg_.lateralExit, timeoutMs));
}

bool Drivetrain::driveDistance(real inches, uint32_t timeoutMs, real chainRadiusInches) {
    update();

    // Project a point `inches` along the current heading and drive to it.
    // forward = (sin t, cos t). Negative distance lands behind us, and
    // MoveConfig::allowReverse then backs into it rather than turning round
    const Pose p = getPoseExtrapolated(clock_.millisNow());
    const real thRad = p.thetaDeg * kDegToRad;
    const real x = p.x + std::sin(thRad) * inches;
    const real y = p.y + std::cos(thRad) * inches;

    return moveToPoint(x, y, timeoutMs, chainRadiusInches);
}

void Drivetrain::tank(real leftVolts, real rightVolts) {
    const real lim = cfg_.move.maxVolts;
    drive_.setLeft(static_cast<double>(clamp(leftVolts, -lim, lim)));
    drive_.setRight(static_cast<double>(clamp(rightVolts, -lim, lim)));
}

void Drivetrain::arcade(real throttleVolts, real turnVolts) {
    tank(throttleVolts + turnVolts, throttleVolts - turnVolts);
}

void Drivetrain::tankCurved(real leftRaw, real rightRaw) {
    // Both sides share throttleCurve: on a tank stick each side IS throttle,
    // and two different curves would make the robot pull under a straight push
    tank(driveCurve(leftRaw, cfg_.throttleCurve), driveCurve(rightRaw, cfg_.throttleCurve));
}

void Drivetrain::arcadeCurved(real throttleRaw, real turnRaw) {
    arcade(driveCurve(throttleRaw, cfg_.throttleCurve), driveCurve(turnRaw, cfg_.turnCurve));
}

}
