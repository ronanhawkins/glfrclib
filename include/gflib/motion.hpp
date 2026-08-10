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

    // Tracks how long |error| has been continuously inside one band.
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
        uint32_t  startMs  = 0;
        bool      timedOut = false;
    };

    // Delegates to the aggregate defaults, then sets the timer start time.
    inline void exitReset(ExitState& s, uint32_t nowMs) {
        s = ExitState{};
        s.startMs = nowMs;
    }

    // true = stop
    bool checkExit(ExitState& s, const ExitConditions& ec, double error, uint32_t nowMs);

    class TurnToHeading : public IMotion {
        public:
            TurnToHeading(double targetDeg, const PidGains& gains, const ExitConditions& exit);

            void start(uint32_t nowMs) override;
            bool tick(const Pose& pose, double dtSec, IDriveOutput& drive, uint32_t nowMs) override;

            bool timedOut() const { return exit_.timedOut; }

        private:
            double         targetDeg_;
            PidGains       gains_;
            ExitConditions ec_;
            PidState       pid_{};
            ExitState      exit_{};
    };
}
