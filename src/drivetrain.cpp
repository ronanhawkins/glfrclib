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
double sanify(double& prev, double reading, double scale, double limit) {
    const double delta = reading - prev;
    const bool ok = std::isfinite(delta) && (limit <= 0.0 || std::fabs(delta * scale) <= limit);

    seed(prev, reading);
    return ok ? delta : 0.0;
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
    const double dVertCounts = sanify(prevVertCounts_, vert_.getCounts(), cfg_.odom.vertInchesPerCount, cfg_.maxTravelInchesPerTick);
    const double dHorizCounts = sanify(prevHorizCounts_, horiz_.getCounts(), cfg_.odom.horizInchesPerCount, cfg_.maxTravelInchesPerTick);
    const double dThetaDeg = sanify(prevHeadingDeg_, imu_.getHeadingDeg(), 1.0, cfg_.maxDThetaDegPerTick);

    pose_ = odomStep(pose_, dVertCounts, dHorizCounts, dThetaDeg, cfg_.odom);
}

void Drivetrain::setPose(double x, double y, double thetaDeg) {
    pose_.x = x;
    pose_.y = y;
    pose_.thetaDeg = thetaDeg;

    // Rebase the sensor baselines, or the next update() applies every count
    // accumulated since the last one as motion away from the new pose
    calibrate();
}

ExitConditions Drivetrain::withTimeout(const ExitConditions& base, uint32_t timeoutMs) const {
    ExitConditions ec = base;
    ec.timeoutMs = timeoutMs;
    return ec;
}

bool Drivetrain::runMotion(IMotion& motion) {
    // Not cleared here. A cancel raised between motions would otherwise be
    // wiped by the next one, which is the wrong way for an estop to fail
    if (cancelled_) {
        drive_.stop();
        return false;
    }

    // Pose must be current before the first tick, or the motion starts on
    // whatever the last update left behind
    update();

    uint32_t nowMs = clock_.millisNow();
    uint32_t lastMs = nowMs;
    motion.start(nowMs);

    while (!cancelled_) {
        update();
        nowMs = clock_.millisNow();

        // Measured, not nominal. sleepMs(loopMs) yields for AT LEAST loopMs,
        // and update() plus two PIDs are not free, so the real period runs
        // long and jittery on hardware. Feeding the nominal value scales every
        // derivative, integral and slew step by that error
        const double dtSec = (nowMs - lastMs) / 1000.0;
        lastMs = nowMs;

        if (motion.tick(pose_, dtSec, drive_, nowMs)) break;
        clock_.sleepMs(cfg_.loopMs);
    }

    drive_.stop();
    return !cancelled_;
}

bool Drivetrain::turnToHeading(double thetaDeg, const ExitConditions& exit) {
    TurnToHeading m(thetaDeg, cfg_.angular, exit, cfg_.turn);
    const bool finished = runMotion(m);
    return finished && !m.timedOut();
}

bool Drivetrain::turnToHeading(double thetaDeg, uint32_t timeoutMs) {
    return turnToHeading(thetaDeg, withTimeout(cfg_.angularExit, timeoutMs));
}

bool Drivetrain::moveToPoint(double x, double y, const ExitConditions& exit) {
    MoveToPoint m(x, y, cfg_.lateral, cfg_.angular, exit, cfg_.move);
    const bool finished = runMotion(m);
    return finished && !m.timedOut();
}

bool Drivetrain::moveToPoint(double x, double y, uint32_t timeoutMs) {
    return moveToPoint(x, y, withTimeout(cfg_.lateralExit, timeoutMs));
}

bool Drivetrain::moveToPose(double x, double y, double thetaDeg, const ExitConditions& exit) {
    MoveToPose m(x, y, thetaDeg, cfg_.lateral, cfg_.angular, exit, cfg_.move);
    const bool finished = runMotion(m);
    return finished && !m.timedOut();
}

bool Drivetrain::moveToPose(double x, double y, double thetaDeg, uint32_t timeoutMs) {
    return moveToPose(x, y, thetaDeg, withTimeout(cfg_.lateralExit, timeoutMs));
}

bool Drivetrain::driveDistance(double inches, uint32_t timeoutMs) {
    update();

    // Project a point `inches` along the current heading and drive to it.
    // forward = (sin t, cos t). Negative distance lands behind us, and
    // MoveConfig::allowReverse then backs into it rather than turning round
    const double thRad = pose_.thetaDeg * kDegToRad;
    const double x = pose_.x + std::sin(thRad) * inches;
    const double y = pose_.y + std::cos(thRad) * inches;

    return moveToPoint(x, y, timeoutMs);
}

void Drivetrain::tank(double leftVolts, double rightVolts) {
    const double lim = cfg_.move.maxVolts;
    drive_.setLeft(clamp(leftVolts, -lim, lim));
    drive_.setRight(clamp(rightVolts, -lim, lim));
}

void Drivetrain::arcade(double throttleVolts, double turnVolts) {
    tank(throttleVolts + turnVolts, throttleVolts - turnVolts);
}

}
