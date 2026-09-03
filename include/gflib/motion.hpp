#pragma once
#include "gflib/real.hpp"
#include <cstdint>
#include "gflib/pose.hpp"
#include "gflib/hal.hpp"
#include "gflib/pid.hpp"

namespace gflib {
// Why a motion ended. EarlyExit is the chaining case: the robot is still
// moving and must NOT be stopped
enum class MotionStatus {
    Running,
    Settled,
    EarlyExit,
    TimedOut,
    Cancelled,
};

class IMotion{
    public:
        virtual ~IMotion() = default;
        virtual void start(uint32_t nowMs) = 0;
        virtual MotionStatus tick(const Pose&, real dtSec, IDriveOutput&, uint32_t nowMs) = 0;
};

// Per-motion chaining. radiusInches 0 = settle and stop as usual.
// Must exceed ExitConditions::smallErr or it would race the exit bands
struct ChainParams {
    real radiusInches = 0.0_r;
    // Linear volts the previous motion was commanding. Set by Drivetrain
    real entryLinVolts = 0.0_r;
};

// 3 tier exit conditions.
//
// Zero means something different in every field, so set all five:
//   smallErr/largeErr = 0  disables that band, a zero-width band is unenterable
//   smallTimeMs/largeTimeMs = 0  fires the moment you enter the band, which is
//     the "settled while still moving" bug the hold time exists to prevent
//   timeoutMs = 0  gives up before the robot moves
struct ExitConditions {
    real   smallErr;
    uint32_t smallTimeMs;
    real   largeErr;
    uint32_t largeTimeMs;
    uint32_t timeoutMs;
};

// Tracks how long abs(error) has been continuously inside one band.
class BandTimer {
    public:
        void reset() { active_ = false; sinceMs_ = 0; }

        bool update(real absErr, real band, uint32_t holdMs, uint32_t nowMs) {
            if (absErr >= band) { active_ = false; return false; }
            if (!active_) { active_ = true; sinceMs_ = nowMs; }
            return nowMs - sinceMs_ >= holdMs;
        }

    private:
        bool     active_  = false;
        uint32_t sinceMs_ = 0;
};


struct ExitState {
    BandTimer smallBand{};
    BandTimer largeBand{};
    uint32_t startMs  = 0;
    bool timedOut = false;
};

// Delegates to the aggregate defaults, then sets the timer start time.
inline void exitReset(ExitState& s, uint32_t nowMs) {
    s = ExitState{};
    s.startMs = nowMs;
}

// true = stop
bool checkExit(ExitState& s, const ExitConditions& ec, real error, uint32_t nowMs);

struct TurnConfig {
    // Floor until inside the exit band. 0 disables.
    real minVolts = 0.0_r;

    // Lower to stop fast turns slipping the wheels.
    real maxVolts = 12.0_r;
};

class TurnToHeading : public IMotion {
    public:
        TurnToHeading(real targetDeg, const PidGains& gains, const ExitConditions& exit, const TurnConfig& cfg = {});

        void start(uint32_t nowMs) override;
        MotionStatus tick(const Pose& pose, real dtSec, IDriveOutput& drive, uint32_t nowMs) override;

        bool timedOut() const { return exit_.timedOut; }

    private:
        real         targetDeg_;
        PidGains       gains_;
        ExitConditions ec_;
        TurnConfig     cfg_;
        PidState       pid_{};
        ExitState      exit_{};
};

struct MoveConfig {
    // Inside this, bearing is noise: stop steering or the robot orbits.
    real settleRadiusInches = 2.0_r;

    // false = turn around instead of reversing.
    bool allowReverse = true;

    // Ceiling on linear + angular combined.
    real maxVolts = 12.0_r;

    // minSpeed, in volts. 0 disables.
    // Set just above where the drivetrain starts moving
    real minVolts = 0.0_r;

    // Angular tolerance in degrees, used twice:
    //   MoveToPose will not report done until the heading is inside it
    //   the angular floor stops pushing inside it
    // 0 waives the MoveToPose heading guarantee AND disables angMinVolts,
    // since a zero cutoff would floor the turn command forever
    real headingTolDeg = 5.0_r;

    // Floor on the angular command. minVolts covers driving; without this a
    // robot with real friction closes the last inches of a move but not the
    // last few degrees of the course correction. 0 disables
    real angMinVolts = 0.0_r;

    // Linear floor while outside the chain radius. Without it the PID has
    // wound down by the handoff and chaining buys nothing. 0 disables
    real chainMinVolts = 0.0_r;

    // MoveToPose only. Carrot sits lead*distance behind the target.
    // 0 ignores the final heading, higher swings wider to hit it.
    real lead = 0.6_r;
};

// Drives to a coordinate. Final heading is whatever the approach leaves.
class MoveToPoint : public IMotion {
    public:
        MoveToPoint(real targetX, real targetY, const PidGains& linGains, const PidGains& angGains, const ExitConditions& exit, const MoveConfig& cfg = {}, const ChainParams& chain = {});

        void start(uint32_t nowMs) override;
        MotionStatus tick(const Pose& pose, real dtSec, IDriveOutput& drive, uint32_t nowMs) override;

        bool timedOut() const { return exit_.timedOut; }

        // Seeds the next chained motion's slew limiter
        real lastLinearVolts() const { return lastLin_; }

    private:
        real targetX_, targetY_;
        PidGains linGains_, angGains_;
        ExitConditions ec_;
        MoveConfig cfg_;
        ChainParams chain_;
        bool chainArmed_ = false;
        real lastLin_ = 0.0_r;
        PidState linPid_{}, angPid_{};
        ExitState exit_{};
};

// Boomerang: drives to a coordinate and arrives on a heading.
// Chases a carrot behind the target, collapses onto the target
// Final heading is approximate
class MoveToPose : public IMotion {
    public:
        MoveToPose(real targetX, real targetY, real targetThetaDeg,
                   const PidGains& linGains, const PidGains& angGains,
                   const ExitConditions& exit, const MoveConfig& cfg = {});

        void start(uint32_t nowMs) override;
        MotionStatus tick(const Pose& pose, real dtSec, IDriveOutput& drive, uint32_t nowMs) override;

        bool timedOut() const { return exit_.timedOut; }

        // Last tick's carrot
        real carrotX() const { return carrotX_; }
        real carrotY() const { return carrotY_; }

    private:
        real targetX_, targetY_, targetThetaDeg_;
        PidGains linGains_, angGains_;
        ExitConditions ec_;
        MoveConfig cfg_;
        PidState linPid_{}, angPid_{};
        ExitState exit_{};
        real carrotX_ = 0.0_r, carrotY_ = 0.0_r;
};
}
