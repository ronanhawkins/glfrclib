#include "gflib/drivetrain.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

Drivetrain::Drivetrain(IEncoder& vert, IEncoder& horiz, IImu& imu, IDriveOutput& drive, IClock& clock, const DrivetrainConfig& cfg)
    : vert_(vert), horiz_(horiz), imu_(imu), drive_(drive), clock_(clock), cfg_(cfg) {}

namespace {

// Never store a non-finite reading
void seed(double& prev, double reading) {
    if (std::isfinite(reading)) prev = reading;
}

// The delta to integrate, or 0 if it is outside bounds. scale puts the limit
// in inches; the IMU passes 1.0 and works in degrees. limit <= 0 drops the
// bound, not the finite check.
//
// The baseline and the reading stay double: encoder counts accumulate, and
// float holds exact integers only to 2^24. Narrowing happens on the delta,
// after the subtraction, where the value is one tick of travel.
real sanify(double& prev, double reading, real scale, real limit) {
    const real delta = static_cast<real>(reading - prev);
    const bool ok = std::isfinite(delta) &&
                    (limit <= 0.0_r || std::fabs(delta * scale) <= limit);

    seed(prev, reading);
    return ok ? delta : 0.0_r;
}

}

void Drivetrain::calibrate() {
    seed(prevVertCounts_, vert_.getCounts());
    seed(prevHorizCounts_, horiz_.getCounts());
    seed(prevHeadingDeg_, imu_.getHeadingDeg());
    calibrated_ = true;
}

void Drivetrain::update() {
    // Before calibrate() the baselines are 0, so the first delta would be
    // the entire boot-time encoder reading. Seed instead of integrating it
    if (!calibrated_) {
        calibrate();
        return;
    }

    // Judged per channel: an IMU glitch should not discard a good tick of
    // encoder travel.
    const real dVertCounts = sanify(prevVertCounts_, vert_.getCounts(), cfg_.odom.vertInchesPerCount, cfg_.maxTravelInchesPerTick);
    const real dHorizCounts = sanify(prevHorizCounts_, horiz_.getCounts(), cfg_.odom.horizInchesPerCount, cfg_.maxTravelInchesPerTick);
    const real dThetaDeg = sanify(prevHeadingDeg_, imu_.getHeadingDeg(), 1.0_r, cfg_.maxDThetaDegPerTick);

    const Pose prev = pose_;
    pose_ = odomStep(pose_, dVertCounts, dHorizCounts, dThetaDeg, cfg_.odom);

    // Measured interval
    const uint32_t nowMs = clock_.millisNow();
    if (!havePrevUpdate_) {
        prevUpdateMs_ = nowMs;
        havePrevUpdate_ = true;
        return;
    }

    // Unsigned, so it is correct across the 49.7-day millis wrap.
    const uint32_t elapsedMs = nowMs - prevUpdateMs_;
    if (elapsedMs == 0) return;
    prevUpdateMs_ = nowMs;

    const real dtSec = static_cast<real>(elapsedMs) / 1000.0_r;
    const real ivx = (pose_.x - prev.x) / dtSec;
    const real ivy = (pose_.y - prev.y) / dtSec;
    const real iw  = (pose_.thetaDeg - prev.thetaDeg) / dtSec;

    // A rejected tick contributes a zero delta, which is the honest reading:
    // it decays every channel toward rest rather than holding the last value.
    const real a = clamp(cfg_.velocityEmaAlpha, 0.0_r, 1.0_r);
    vel_.vx += a * (ivx - vel_.vx);
    vel_.vy += a * (ivy - vel_.vy);
    vel_.omegaDegPerSec += a * (iw - vel_.omegaDegPerSec);
}

void Drivetrain::setPose(real x, real y, real thetaDeg) {
    pose_.x = x;
    pose_.y = y;
    pose_.thetaDeg = thetaDeg;

    // A commanded jump is not motion.
    vel_ = Velocity{};
    havePrevUpdate_ = false;

    // Rebase the sensor baselines, or the next update() applies every count
    // accumulated since the last one as motion away from the new pose
    calibrate();
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

        // Measured, not nominal. sleepMs(loopMs) yields for AT LEAST loopMs,
        // and update() plus two PIDs are not free, so the real period runs
        // long and jittery on hardware. Feeding the nominal value scales every
        // derivative, integral and slew step by that error
        const real dtSec = (nowMs - lastMs) / 1000.0_r;
        lastMs = nowMs;

        st = motion.tick(pose_, dtSec, drive_, nowMs);
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
    const real thRad = pose_.thetaDeg * kDegToRad;
    const real x = pose_.x + std::sin(thRad) * inches;
    const real y = pose_.y + std::cos(thRad) * inches;

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

}
