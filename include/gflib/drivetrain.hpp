#pragma once
#include <cstdint>
#include <atomic>
#include "gflib/hal.hpp"
#include "gflib/odom.hpp"
#include "gflib/pid.hpp"
#include "gflib/motion.hpp"
#include "gflib/pose.hpp"

namespace gflib {

// Everything robot-specific in one place
struct DrivetrainConfig {
    OdomConfig odom;

    PidGains lateral;
    PidGains angular;

    // Defaults used when a motion is called without explicit conditions.
    // Initialised here because ExitConditions itself has no defaults, so a
    // plain `DrivetrainConfig c;` would otherwise leave these indeterminate
    // and hand a motion a garbage timeout
    ExitConditions lateralExit{1.0, 100, 3.0, 500, 3000};
    ExitConditions angularExit{1.0, 100, 3.0, 500, 3000};

    MoveConfig move;
    TurnConfig turn;

    // Motion loop period. 10ms gives the 100Hz odometry the tuning assumes;
    // raising it makes derivatives noisier
    uint32_t loopMs = 10;
};

// NOT thread safe. update() and the motion calls must run on one task.
//
// Blocking motions call update() themselves at cfg.loopMs, so autonomous is
// covered. Driver control must call update() itself, and the rate is now a
// contract nothing enforces: odometry approximates each tick as one arc, so a
// slow loop degrades heading-dependent error exactly when a driver is making
// fast direction changes. Call it at 100Hz, and treat 50Hz as the floor
class Drivetrain {
    public:
        Drivetrain(IEncoder& vert, IEncoder& horiz, IImu& imu, IDriveOutput& drive, IClock& clock, const DrivetrainConfig& cfg);

        // Seeds the sensor baselines. Call once, with the robot still,
        // before anything else. Without it the first update() sees the
        // whole boot-time encoder reading as one enormous delta
        void calibrate();

        // One odometry step from the current sensor readings
        void update();

        Pose getPose() const { return pose_; }
        void setPose(double x, double y, double thetaDeg);

        // Blocking. Each returns false if it gave up on the timeout
        bool turnToHeading(double thetaDeg, const ExitConditions& exit);
        bool turnToHeading(double thetaDeg, uint32_t timeoutMs);

        bool moveToPoint(double x, double y, const ExitConditions& exit);
        bool moveToPoint(double x, double y, uint32_t timeoutMs);

        bool moveToPose(double x, double y, double thetaDeg, const ExitConditions& exit);
        bool moveToPose(double x, double y, double thetaDeg, uint32_t timeoutMs);

        // Straight line relative to where the robot is now. Negative
        // reverses. Built on moveToPoint, so the angular controller holds
        // the current heading for you
        bool driveDistance(double inches, uint32_t timeoutMs);

        // Runs any motion to completion. The other calls are wrappers
        bool runMotion(IMotion& motion);

        // Safe from another task. Ends the running motion at the next tick.
        // The flag LATCHES: every later motion fails immediately until
        // clearCancel(), so an estop raised between motions is not swallowed
        void cancelMotion() { cancelled_ = true; }
        void clearCancel() { cancelled_ = false; }
        bool isCancelled() const { return cancelled_; }

        // Driver control. Volts, clamped to move.maxVolts
        void tank(double leftVolts, double rightVolts);
        void arcade(double throttleVolts, double turnVolts);

        void stop() { drive_.stop(); }

        DrivetrainConfig& config() { return cfg_; }
        const DrivetrainConfig& config() const { return cfg_; }

    private:
        ExitConditions withTimeout(const ExitConditions& base, uint32_t timeoutMs) const;

        IEncoder& vert_;
        IEncoder& horiz_;
        IImu& imu_;
        IDriveOutput& drive_;
        IClock& clock_;
        DrivetrainConfig cfg_;

        Pose pose_{};
        double prevVertCounts_ = 0.0;
        double prevHorizCounts_ = 0.0;
        double prevHeadingDeg_ = 0.0;
        bool calibrated_ = false;

        std::atomic<bool> cancelled_{false};
};

}
