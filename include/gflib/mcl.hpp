#pragma once
#include "gflib/real.hpp"
#include <cstdint>
#include <cstddef>
#include "gflib/pose.hpp"
#include "gflib/odom.hpp"

// Monte Carlo localisation against the field walls.

namespace gflib {

// Deterministic, self-contained PRNG. Pseudo-random PCG32.
class Rng {
    public:
        explicit Rng(uint64_t seed = 0x853c49e6748fea9bULL) { reseed(seed); }

        void reseed(uint64_t seed);

        uint32_t nextU32();

        // [0, 1). Built from the top 24 bits, which are the well-mixed ones.
        real nextUnit();

        // Standard normal, Marsaglia polar. Generates in pairs and caches the spare
        real nextGaussian();

    private:
        uint64_t state_ = 0;
        uint64_t inc_   = 0;
        real   spare_ = 0.0_r;
        bool     hasSpare_ = false;
};

// Fixed ceiling, no allocation ever. Two buffers of this many particles live
// inside Mcl, so an instance is roughly kMclMaxParticles * 64 bytes.
inline constexpr int kMclMaxParticles = 300;

struct Particle {
    real xInches   = 0.0_r;
    real yInches   = 0.0_r;
    // deg is unwrapped, like Pose
    real thetaDeg  = 0.0_r;
    real weight    = 0.0_r;
};

// A distance sensor's mounting, in the robot frame.
struct SensorMount {
    // Offset from the tracking centre. Compass frame, same as the field:
    // +Y is forward, +X is right.
    real xInches = 0.0_r;
    real yInches = 0.0_r;

    // Direction it looks, relative to robot forward. 0 = straight ahead,
    // 90 = out the right side, CW+.
    real bearingDeg = 0.0_r;
};

// One reading from one mount.
struct SensorReading {
    int    mountIndex = 0;
    real distanceInches = 0.0_r;

    // false for a no-return, an out-of-range reading, or a sensor the ESP32
    // has marked bad. Invalid readings are SKIPPED, not treated as zero --
    // zero is a wall pressed against the lens and would drag the whole cloud.
    bool   valid = false;
};

struct MclConfig {
    // Runtime count, clamped to kMclMaxParticles.
    int particleCount = 200;

    // Half width/height of field, inches. 72 inches on vex field
    real fieldHalfWidthInches  = 72.0_r;
    real fieldHalfHeightInches = 72.0_r;

    // Odometry noise
    // Units: inches/degrees of sigma per inch travelled/per degree turned
    real transNoisePerInch  = 0.02_r;
    real transNoisePerDeg   = 0.01_r;
    real headingNoisePerInch = 0.02_r;
    real headingNoisePerDeg  = 0.02_r;

    // Distance sensor model.
    real sensorSigmaInches   = 1.5_r;
    real sensorMaxRangeInches = 100.0_r;

    // Fraction of the sensor likelihood that is a flat floor rather than the Gaussian. 
    // LOAD-BEARING do NOT REMOVE !!! - RONAN
    real outlierWeight = 0.1_r;

    // Resample when the effective sample size falls below this fraction of particleCount.
    real resampleEssRatio = 0.5_r;

    // Jitter added after a resample.
    real rougheningInches = 0.25_r;
    real rougheningDeg    = 0.5_r;

    // Position spread at which confidence() reads 0. 
    // Below it, confidence ramps to 1 linearly.
    real convergedRadiusInches = 6.0_r;
};

class Mcl {
    public:
        // Scatters particles around `seed`. Call initially and on every PoseSet from the Brain.
        void init(const MclConfig& cfg, const Pose& seed,
                  real posSigmaInches, real headingSigmaDeg,
                  uint64_t rngSeed);

        // Motion update, from the deltas Drivetrain feeds odomStep.
        // Each particle runs the  odomStep with noise added to its own deltas
        void predict(real dVertCounts, real dHorizCounts, real dThetaDeg,
                     const OdomConfig& odom);

        // Measurement update. Reweights against expected wall distances.
        // Resamples internally when the cloud has degenerated.
        void update(const SensorMount* mounts, size_t mountCount,
                    const SensorReading* readings, size_t readingCount);

        // Weighted mean position; heading by circular mean so the estimate
        // does not average 359 and 1 into 180.
        Pose estimate() const;

        // 0 to 1 from position spread against convergedRadiusInches.
        real confidence() const;

        // Weighted position standard deviation, inches, and heading standard
        // deviation, degrees. raw num used for confidence()
        real positionStdDevInches() const;
        real headingStdDevDeg() const;

        // Kish effective sample size. Diagnostic and resample trigger.
        real effectiveSampleSize() const;

        // Re-scatter around `pose` and clear diverged().
        void reseed(const Pose& pose, real posSigmaInches, real headingSigmaDeg);

        // Set when a measurement update found every particle impossible and
        // the weights had to be flattened
        bool diverged() const { return diverged_; }

        // How many times the filter has latched diverged() since init().
        // Monotonic, so a reseed does not erase
        uint32_t divergences() const { return divergences_; }

        int particleCount() const { return count_; }
        const Particle* particles() const { return p_; }

        const MclConfig& config() const { return cfg_; }

    private:
        void  normalise();
        void  resample();
        // Distance from a particle-mounted sensor to the first field wall,
        // or a negative value if the particle is outside the field.
        real raycast(const Particle& p, const SensorMount& m) const;

        MclConfig cfg_{};
        Particle  p_[kMclMaxParticles]{};
        Particle  scratch_[kMclMaxParticles]{};
        int       count_ = 0;
        Rng       rng_{};
        bool      diverged_ = false;
        uint32_t  divergences_ = 0;
        bool      initialised_ = false;
};

} // namespace gflib
