#pragma once
#include "gflib/real.hpp"
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
    ExitConditions lateralExit{1.0_r, 100, 3.0_r, 500, 3000};
    ExitConditions angularExit{1.0_r, 100, 3.0_r, 500, 3000};

    MoveConfig move;
    TurnConfig turn;

    // Motion loop period. 10ms gives the 100Hz odometry the tuning assumes;
    // raising it makes derivatives noisier
    uint32_t loopMs = 10;

    //error bounds per tick
    //max degrees per tick before rejection, at 100Hz this is 9000degrees/s
    real maxDThetaDegPerTick = 90.0_r;

    //max tracking wheel distance per tick
    //6 inches at 100Hz is 50ft/s
    real maxTravelInchesPerTick = 6.0_r;

    // Single-pole EMA on the reported velocity.
    real velocityEmaAlpha = 0.2_r;
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

        // One odometry step from the current sensor readings.
        // Readings failing the plausibility bounds are dropped
        void update();

        Pose getPose() const { return pose_; }
        void setPose(real x, real y, real thetaDeg);

        // Smoothed, field frame. Zero until update() has two timestamps to
        // work from, and reset by setPose(), commanded jump is not motion
        Velocity getVelocity() const { return vel_; }

        // Blocking. Each returns false if it gave up on the timeout
        bool turnToHeading(real thetaDeg, const ExitConditions& exit);
        bool turnToHeading(real thetaDeg, uint32_t timeoutMs);

        // chainRadiusInches > 0 exits that far out WITHOUT stopping, so the
        // next move starts from speed. The last move of a chain must omit it
        // or the motors stay powered. Must exceed exit.smallErr
        bool moveToPoint(real x, real y, const ExitConditions& exit, real chainRadiusInches = 0.0_r);
        bool moveToPoint(real x, real y, uint32_t timeoutMs, real chainRadiusInches = 0.0_r);

        bool moveToPose(real x, real y, real thetaDeg, const ExitConditions& exit);
        bool moveToPose(real x, real y, real thetaDeg, uint32_t timeoutMs);

        // Straight line relative to where the robot is now. Negative
        // reverses. Built on moveToPoint, so the angular controller holds
        // the current heading for you
        bool driveDistance(real inches, uint32_t timeoutMs, real chainRadiusInches = 0.0_r);

        // Runs any motion to completion. The other calls are wrappers.
        // Does not stop the drive on EarlyExit
        MotionStatus runMotion(IMotion& motion);

        // Safe from another task. Ends the running motion at the next tick.
        // The flag LATCHES: every later motion fails immediately until
        // clearCancel(), so an estop raised between motions is not swallowed
        void cancelMotion() { cancelled_ = true; }
        void clearCancel() { cancelled_ = false; }
        bool isCancelled() const { return cancelled_; }

        // Driver control. Volts, clamped to move.maxVolts
        void tank(real leftVolts, real rightVolts);
        void arcade(real throttleVolts, real turnVolts);

        void stop() { drive_.stop(); chainEntryVolts_ = 0.0_r; }

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
        // double, not real: these accumulate raw encoder counts. See sanify()
        double prevVertCounts_ = 0.0;
        double prevHorizCounts_ = 0.0;
        double prevHeadingDeg_ = 0.0;
        bool calibrated_ = false;

        Velocity vel_{};
        uint32_t prevUpdateMs_ = 0;
        bool havePrevUpdate_ = false;

        // Carried across a chained handoff to seed the next linear PID
        real chainEntryVolts_ = 0.0_r;

        std::atomic<bool> cancelled_{false};
};

}
