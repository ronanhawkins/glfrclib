#pragma once
#include "gflib/real.hpp"
#include <cstdint>
#include <atomic>
#include "gflib/hal.hpp"
#include "gflib/posesource.hpp"
#include "gflib/drivecurve.hpp"
#include "gflib/pid.hpp"
#include "gflib/motion.hpp"
#include "gflib/pose.hpp"

namespace gflib {

// Everything robot-specific in one place.
struct DrivetrainConfig {
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

    // Driver control shaping. Here rather than in each opcontrol so both
    // robots answer a stick identically
    DriveCurveConfig throttleCurve;
    DriveCurveConfig turnCurve;

    // Motion loop period. 10ms gives the 100Hz odometry the tuning assumes;
    // raising it makes derivatives noisier
    uint32_t loopMs = 10;

    // Latency compensation, OFF by default. 0 means every motion sees the
    // pose exactly as the source reports it
    //
    // Set poseMaxExtrapMs only once poseTransitLatencyMs has been MEASURED on
    // the bench for your link
    uint32_t poseTransitLatencyMs = 0;
    uint32_t poseMaxExtrapMs = 0;
};

// NOT thread safe. update() and the motion calls must run on one task.
//
// Consumes a pose to run motions. It does not produce one: that is the
// IPoseSource's job, and the split is what lets a V5 Brain with no odometry
// hardware construct a Drivetrain at all.
//
// Blocking motions call update() themselves at cfg.loopMs, so autonomous is
// covered. Driver control must call update() itself, and the rate is now a
// contract nothing enforces: odometry approximates each tick as one arc, so a
// slow loop degrades heading-dependent error exactly when a driver is making
// fast direction changes. Call it at 100Hz, and treat 50Hz as the floor
class Drivetrain {
    public:
        Drivetrain(IPoseSource& source, IDriveOutput& drive, IClock& clock, const DrivetrainConfig& cfg);

        // Brings the pose source up. Call once, with the robot still, before
        // anything else. On odometry this seeds the sensor baselines and
        // returns at once; on a link it waits for frames to start flowing.
        //
        // No default timeout, and the result is worth checking: a link that
        // never comes up is a match that must not drive. There is
        // deliberately no calibrate() wrapper -- one that swallowed this
        // bool would be a silent no-op on the Brain, which is the exact trap
        // the split exists to remove
        bool begin(uint32_t timeoutMs) { return source_.begin(timeoutMs); }

        // One tick of the pose source
        void update() { source_.update(); }

        Pose getPose() const { return source_.getPose(); }

        // What the motions actually steer on: getPose(), projected forward by
        // the configured latency. Identical to getPose() unless
        // poseMaxExtrapMs is set, so it is also what an opcontrol display
        // should read if you want to see what the controller sees
        Pose getPoseExtrapolated(uint32_t nowMs) const {
            return extrapolatePose(source_, nowMs, cfg_.poseTransitLatencyMs, cfg_.poseMaxExtrapMs);
        }

        // Smoothed, field frame
        Velocity getVelocity() const { return source_.getVelocity(); }

        const IPoseSource& poseSource() const { return source_; }
        IPoseSource& poseSource() { return source_; }

        // Blocks until the new pose has actually taken, because on a link the
        // pose belongs to another processor and the write is a request.
        // 
        // Returns false if it never took, which is a reason not to drive.
        //
        // The default is ~12 report intervals at 100Hz, enough for a PoseSet
        // and its echo plus a couple of retries. It must NOT be 0
        bool setPose(real x, real y, real thetaDeg, uint32_t timeoutMs = 250);

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

        // Raw joystick in, shaped and mixed. The curved entry points, so an
        // opcontrol never has to reimplement the feel
        void tankCurved(real leftRaw, real rightRaw);
        void arcadeCurved(real throttleRaw, real turnRaw);

        void stop() { drive_.stop(); chainEntryVolts_ = 0.0_r; }

        DrivetrainConfig& config() { return cfg_; }
        const DrivetrainConfig& config() const { return cfg_; }

    private:
        ExitConditions withTimeout(const ExitConditions& base, uint32_t timeoutMs) const;

        IPoseSource& source_;
        IDriveOutput& drive_;
        IClock& clock_;
        DrivetrainConfig cfg_;

        // Carried across a chained handoff to seed the next linear PID
        real chainEntryVolts_ = 0.0_r;

        std::atomic<bool> cancelled_{false};
};

}
