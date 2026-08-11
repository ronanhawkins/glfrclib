#pragma once
#include <cstdint>
#include <gflib/pose.hpp>
#include <gflib/hal.hpp>
#include <gflib/pid.hpp>

namespace gflib {
class IMotion{
    public:
        virtual ~IMotion() = default;
        virtual void start(uint32_t nowMs) = 0;
        virtual bool tick(const Pose&, double dtSec, IDriveOutput&, uint32_t nowMs) = 0;
};

// 3 tier exit conditions
struct ExitConditions {
    double   smallErr;
    uint32_t smallTimeMs;
    double   largeErr;
    uint32_t largeTimeMs;
    uint32_t timeoutMs;
};

// Tracks how long abs(error) has been continuously inside one band.
class BandTimer {
    public:
        void reset() { active_ = false; sinceMs_ = 0; }

        bool update(double absErr, double band, uint32_t holdMs, uint32_t nowMs) {
            if (absErr >= band) { active_ = false; return false; }
            if (!active_) { active_ = true; sinceMs_ = nowMs; }
            return nowMs - sinceMs_ >= holdMs;
        }

    private:
        bool     active_  = false;
        uint32_t sinceMs_ = 0;
};


struct ExitState {
    BandTimer small{};
    BandTimer large{};
    uint32_t startMs  = 0;
    bool timedOut = false;
};

// Delegates to the aggregate defaults, then sets the timer start time.
inline void exitReset(ExitState& s, uint32_t nowMs) {
    s = ExitState{};
    s.startMs = nowMs;
}

// true = stop
bool checkExit(ExitState& s, const ExitConditions& ec, double error, uint32_t nowMs);

struct TurnConfig {
    // Floor until inside the exit band. 0 disables.
    double minVolts = 0.0;

    // Lower to stop fast turns slipping the wheels.
    double maxVolts = 12.0;
};

class TurnToHeading : public IMotion {
    public:
        TurnToHeading(double targetDeg, const PidGains& gains, const ExitConditions& exit, const TurnConfig& cfg = {});

        void start(uint32_t nowMs) override;
        bool tick(const Pose& pose, double dtSec, IDriveOutput& drive, uint32_t nowMs) override;

        bool timedOut() const { return exit_.timedOut; }

    private:
        double         targetDeg_;
        PidGains       gains_;
        ExitConditions ec_;
        TurnConfig     cfg_;
        PidState       pid_{};
        ExitState      exit_{};
};

struct MoveConfig {
    // Inside this, bearing is noise: stop steering or the robot orbits.
    double settleRadiusInches = 2.0;

    // false = turn around instead of reversing.
    bool allowReverse = true;

    // Ceiling on linear + angular combined.
    double maxVolts = 12.0;

    // minSpeed, in volts. 0 disables.
    // Set just above where the drivetrain starts moving
    double minVolts = 0.0;

    // MoveToPose only. Carrot sits lead*distance behind the target.
    // 0 ignores the final heading, higher swings wider to hit it.
    double lead = 0.6;
};

// Drives to a coordinate. Final heading is whatever the approach leaves.
class MoveToPoint : public IMotion {
    public:
        MoveToPoint(double targetX, double targetY, const PidGains& linGains, const PidGains& angGains, const ExitConditions& exit, const MoveConfig& cfg = {});

        void start(uint32_t nowMs) override;
        bool tick(const Pose& pose, double dtSec, IDriveOutput& drive, uint32_t nowMs) override;

        bool timedOut() const { return exit_.timedOut; }

    private:
        double targetX_, targetY_;
        PidGains linGains_, angGains_;
        ExitConditions ec_;
        MoveConfig cfg_;
        PidState linPid_{}, angPid_{};
        ExitState exit_{};
};

// Boomerang: drives to a coordinate and arrives on a heading.
// Chases a carrot behind the target, collapses onto the target
// Final heading is approximate
class MoveToPose : public IMotion {
    public:
        MoveToPose(double targetX, double targetY, double targetThetaDeg,
                   const PidGains& linGains, const PidGains& angGains,
                   const ExitConditions& exit, const MoveConfig& cfg = {});

        void start(uint32_t nowMs) override;
        bool tick(const Pose& pose, double dtSec, IDriveOutput& drive, uint32_t nowMs) override;

        bool timedOut() const { return exit_.timedOut; }

        // Last tick's carrot
        double carrotX() const { return carrotX_; }
        double carrotY() const { return carrotY_; }

    private:
        double targetX_, targetY_, targetThetaDeg_;
        PidGains linGains_, angGains_;
        ExitConditions ec_;
        MoveConfig cfg_;
        PidState linPid_{}, angPid_{};
        ExitState exit_{};
        double carrotX_ = 0.0, carrotY_ = 0.0;
};
}
