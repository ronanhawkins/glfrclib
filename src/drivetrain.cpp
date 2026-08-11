#include "gflib/drivetrain.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

Drivetrain::Drivetrain(IEncoder& vert, IEncoder& horiz, IImu& imu, IDriveOutput& drive, IClock& clock, const DrivetrainConfig& cfg)
    : vert_(vert), horiz_(horiz), imu_(imu), drive_(drive), clock_(clock), cfg_(cfg) {}

void Drivetrain::calibrate() {
    prevVertCounts_ = vert_.getCounts();
    prevHorizCounts_ = horiz_.getCounts();
    prevHeadingDeg_ = imu_.getHeadingDeg();
    calibrated_ = true;
}

void Drivetrain::update() {
    // Before calibrate() the baselines are 0, so the first delta would be
    // the entire boot-time encoder reading. Seed instead of integrating it
    if (!calibrated_) {
        calibrate();
        return;
    }

    const double vertCounts = vert_.getCounts();
    const double horizCounts = horiz_.getCounts();
    const double headingDeg = imu_.getHeadingDeg();

    // No wrapDeg here on purpose. IImu is unwrapped, so this
    // subtraction is already the true delta. Wrapping would cap a single
    // tick at 180 degrees and lose the rest
    const double dThetaDeg = headingDeg - prevHeadingDeg_;

    pose_ = odomStep(pose_, vertCounts - prevVertCounts_, horizCounts - prevHorizCounts_, dThetaDeg, cfg_.odom);

    prevVertCounts_ = vertCounts;
    prevHorizCounts_ = horizCounts;
    prevHeadingDeg_ = headingDeg;
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
    cancelled_ = false;

    // Pose must be current before the first tick, or the motion starts on
    // whatever the last update left behind
    update();

    uint32_t nowMs = clock_.millisNow();
    motion.start(nowMs);

    while (!cancelled_) {
        update();
        nowMs = clock_.millisNow();
        if (motion.tick(pose_, cfg_.loopMs / 1000.0, drive_, nowMs)) break;
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
