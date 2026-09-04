#include "gflib/mcl.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

// --- rng -------------------------------------------------------------------

void Rng::reseed(uint64_t seed) {
    // SplitMix64 the seed before it reaches PCG. 
    // Consecutive seeds (0, 1, 2) otherwise produce correlated first outputs.
    uint64_t z = seed + 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z =  z ^ (z >> 31);

    state_ = 0;
    // must be odd for a full period
    inc_ = (z << 1u) | 1u;
    nextU32();
    state_ += z;
    nextU32();

    // cached spare from the old stream
    hasSpare_ = false;
    spare_    = 0.0_r;
}

uint32_t Rng::nextU32() {
    const uint64_t old = state_;
    state_ = old * 6364136223846793005ULL + inc_;
    const uint32_t xorshifted = static_cast<uint32_t>(((old >> 18u) ^ old) >> 27u);
    const uint32_t rot = static_cast<uint32_t>(old >> 59u);
    return (xorshifted >> rot) | (xorshifted << ((32u - rot) & 31u));
}

real Rng::nextUnit() {
    // 24 bits into a double: exact, never returns 1.0.
    return (nextU32() >> 8) * (1.0_r / 16777216.0_r);
}

real Rng::nextGaussian() {
    if (hasSpare_) {
        hasSpare_ = false;
        return spare_;
    }

    // Marsaglia polar. No sin/cos
    // rejection loop terminates with probability 1 - (1 - pi/4)^n.
    real u, v, s;
    do {
        u = nextUnit() * 2.0_r - 1.0_r;
        v = nextUnit() * 2.0_r - 1.0_r;
        s = u * u + v * v;
    } while (s >= 1.0_r || s == 0.0_r);

    const real f = std::sqrt(-2.0_r * std::log(s) / s);
    spare_ = v * f;
    hasSpare_ = true;
    return u * f;
}

// monte carlo localization

void Mcl::init(const MclConfig& cfg, const Pose& seed,
               real posSigmaInches, real headingSigmaDeg,
               uint64_t rngSeed) {
    cfg_ = cfg;
    count_ = static_cast<int>(clamp(static_cast<real>(cfg.particleCount), 1.0_r,
                                    static_cast<real>(kMclMaxParticles)));
    rng_.reseed(rngSeed);

    const real w = 1.0_r / count_;
    for (int i = 0; i < count_; ++i) {
        p_[i].xInches  = seed.x + rng_.nextGaussian() * posSigmaInches;
        p_[i].yInches  = seed.y + rng_.nextGaussian() * posSigmaInches;
        p_[i].thetaDeg = seed.thetaDeg + rng_.nextGaussian() * headingSigmaDeg;
        p_[i].weight   = w;
    }

    diverged_ = false;
    initialised_ = true;
}

// Deliberately does NOT reseed the RNG
void Mcl::reseed(const Pose& pose, real posSigmaInches, real headingSigmaDeg) {
    if (!initialised_) return;

    const real w = 1.0_r / count_;
    for (int i = 0; i < count_; ++i) {
        p_[i].xInches  = pose.x + rng_.nextGaussian() * posSigmaInches;
        p_[i].yInches  = pose.y + rng_.nextGaussian() * posSigmaInches;
        p_[i].thetaDeg = pose.thetaDeg + rng_.nextGaussian() * headingSigmaDeg;
        p_[i].weight   = w;
    }

    diverged_ = false;
}

void Mcl::predict(real dVertCounts, real dHorizCounts, real dThetaDeg,
                  const OdomConfig& odom) {
    if (!initialised_) return;

    // Noise scales with how much actually happened.
    const real dVertInches = countsToInches(dVertCounts, odom.vertInchesPerCount);
    const real dHorizInches = countsToInches(dHorizCounts, odom.horizInchesPerCount);
    const real transInches = std::hypot(dVertInches, dHorizInches);
    const real rotDeg = std::fabs(dThetaDeg);

    const real sigmaTrans = cfg_.transNoisePerInch * transInches + cfg_.transNoisePerDeg * rotDeg;
    const real sigmaRot = cfg_.headingNoisePerInch * transInches + cfg_.headingNoisePerDeg * rotDeg;

    // Noise is generated in inches and converted back to counts: the two
    // pods can have different scales.
    const real vScale = (odom.vertInchesPerCount  != 0.0_r) ? odom.vertInchesPerCount  : 1.0_r;
    const real hScale = (odom.horizInchesPerCount != 0.0_r) ? odom.horizInchesPerCount : 1.0_r;

    for (int i = 0; i < count_; ++i) {
        const real nVert = dVertCounts + (rng_.nextGaussian() * sigmaTrans) / vScale;
        const real nHoriz = dHorizCounts + (rng_.nextGaussian() * sigmaTrans) / hScale;
        const real nTheta = dThetaDeg + rng_.nextGaussian() * sigmaRot;

        Pose prev;
        prev.x = p_[i].xInches;
        prev.y = p_[i].yInches;
        prev.thetaDeg = p_[i].thetaDeg;

        // Real odom
        const Pose next = odomStep(prev, nVert, nHoriz, nTheta, odom);

        p_[i].xInches  = next.x;
        p_[i].yInches  = next.y;
        p_[i].thetaDeg = next.thetaDeg;
    }
}

real Mcl::raycast(const Particle& p, const SensorMount& m) const {
    const real halfW = cfg_.fieldHalfWidthInches;
    const real halfH = cfg_.fieldHalfHeightInches;

    // Mount offset is in the robot frame; rotate it into the field frame.
    // Compass: forward = (sin t, cos t), right = (cos t, -sin t).
    const real tRad = p.thetaDeg * kDegToRad;
    const real st = std::sin(tRad), ct = std::cos(tRad);

    const real ox = p.xInches + m.yInches * st + m.xInches * ct;
    const real oy = p.yInches + m.yInches * ct - m.xInches * st;

    // A particle outside the field has no meaningful wall distance.
    if (std::fabs(ox) > halfW || std::fabs(oy) > halfH) return -1.0_r;

    const real bRad = (p.thetaDeg + m.bearingDeg) * kDegToRad;
    const real dx = std::sin(bRad);
    const real dy = std::cos(bRad);

    // Nearest wall the ray reaches. Origin is inside, so exactly one x-wall
    // and one y-wall are ahead; the smaller distance is the one it hits.
    real best = cfg_.sensorMaxRangeInches;
    const real kEps = 1e-9_r;

    if (dx > kEps)       { const real t = ( halfW - ox) / dx; if (t >= 0.0_r && t < best) best = t; }
    else if (dx < -kEps) { const real t = (-halfW - ox) / dx; if (t >= 0.0_r && t < best) best = t; }

    if (dy > kEps)       { const real t = ( halfH - oy) / dy; if (t >= 0.0_r && t < best) best = t; }
    else if (dy < -kEps) { const real t = (-halfH - oy) / dy; if (t >= 0.0_r && t < best) best = t; }

    return best;
}

void Mcl::update(const SensorMount* mounts, size_t mountCount,
                 const SensorReading* readings, size_t readingCount) {
    if (!initialised_ || mounts == nullptr || readings == nullptr) return;

    const real sigma = cfg_.sensorSigmaInches;
    if (!(sigma > 0.0_r)) return;

    const real mix   = clamp(cfg_.outlierWeight, 0.0_r, 1.0_r);
    // flat component
    const real floorL = mix;
    // Gaussian component
    const real gainL  = 1.0_r - mix;

    bool anyReading = false;

    for (int i = 0; i < count_; ++i) {
        real w = p_[i].weight;

        for (size_t r = 0; r < readingCount; ++r) {
            const SensorReading& rd = readings[r];
            if (!rd.valid) continue;
            if (rd.mountIndex < 0 || static_cast<size_t>(rd.mountIndex) >= mountCount) continue;
            anyReading = true;

            const real expected = raycast(p_[i], mounts[rd.mountIndex]);

            if (expected < 0.0_r) {
                // The particle is outside the walls: an impossible pose,
                w = 0.0_r;
                break;
            }

            const real e = rd.distanceInches - expected;
            const real g = std::exp(-0.5_r * (e * e) / (sigma * sigma));
            w *= (gainL * g + floorL);
        }

        p_[i].weight = w;
    }

    // nothing to correct against
    if (!anyReading) return;

    normalise();

    if (effectiveSampleSize() < cfg_.resampleEssRatio * count_) resample();
}

void Mcl::normalise() {
    // Divide by the max
    real maxW = 0.0_r;
    for (int i = 0; i < count_; ++i) if (p_[i].weight > maxW) maxW = p_[i].weight;

    if (!(maxW > 0.0_r) || !std::isfinite(maxW)) {
        // Every particle is impossible. Flatten and latch, fresh init to fix
        const real w = 1.0_r / count_;
        for (int i = 0; i < count_; ++i) p_[i].weight = w;
        if (!diverged_) ++divergences_;      // count episodes, not updates
        diverged_ = true;
        return;
    }

    real sum = 0.0_r;
    for (int i = 0; i < count_; ++i) {
        p_[i].weight /= maxW;
        sum += p_[i].weight;
    }

    for (int i = 0; i < count_; ++i) p_[i].weight /= sum;
}

void Mcl::resample() {
    // Systematic resampling, one random offset, then evenly spaced draws. O(n).
    const real step = 1.0_r / count_;
    real target = rng_.nextUnit() * step;
    real cumulative = p_[0].weight;
    int src = 0;

    for (int i = 0; i < count_; ++i) {
        while (target > cumulative && src < count_ - 1) {
            ++src;
            cumulative += p_[src].weight;
        }
        scratch_[i] = p_[src];
        target += step;
    }

    const real w = 1.0_r / count_;
    for (int i = 0; i < count_; ++i) {
        p_[i] = scratch_[i];
        // Roughening. The copies above are exact duplicates.
        p_[i].xInches  += rng_.nextGaussian() * cfg_.rougheningInches;
        p_[i].yInches  += rng_.nextGaussian() * cfg_.rougheningInches;
        p_[i].thetaDeg += rng_.nextGaussian() * cfg_.rougheningDeg;
        p_[i].weight = w;
    }
}

real Mcl::effectiveSampleSize() const {
    if (count_ <= 0) return 0.0_r;
    real sumSq = 0.0_r;
    for (int i = 0; i < count_; ++i) sumSq += p_[i].weight * p_[i].weight;
    return (sumSq > 0.0_r) ? 1.0_r / sumSq : 0.0_r;
}

Pose Mcl::estimate() const {
    Pose out;
    if (count_ <= 0) return out;

    real sw = 0.0_r, sx = 0.0_r, sy = 0.0_r;
    for (int i = 0; i < count_; ++i) {
        sw += p_[i].weight;
        sx += p_[i].weight * p_[i].xInches;
        sy += p_[i].weight * p_[i].yInches;
    }
    if (!(sw > 0.0_r)) return out;

    out.x = sx / sw;
    out.y = sy / sw;

    // Circular mean, taken relative to particle 0.
    const real ref = p_[0].thetaDeg;
    real ss = 0.0_r, sc = 0.0_r;
    for (int i = 0; i < count_; ++i) {
        const real dRad = wrapDeg(p_[i].thetaDeg - ref) * kDegToRad;
        ss += p_[i].weight * std::sin(dRad);
        sc += p_[i].weight * std::cos(dRad);
    }

    out.thetaDeg = (ss == 0.0_r && sc == 0.0_r)
                       ? ref
                       : ref + std::atan2(ss, sc) * kRadToDeg;
    return out;
}

real Mcl::positionStdDevInches() const {
    if (count_ <= 0) return 0.0_r;
    const Pose m = estimate();

    real sw = 0.0_r, acc = 0.0_r;
    for (int i = 0; i < count_; ++i) {
        const real dx = p_[i].xInches - m.x;
        const real dy = p_[i].yInches - m.y;
        acc += p_[i].weight * (dx * dx + dy * dy);
        sw  += p_[i].weight;
    }
    if (!(sw > 0.0_r)) return 0.0_r;

    // Radial spread, sqrt of the mean squared distance from the estimate
    return std::sqrt(acc / sw);
}

real Mcl::headingStdDevDeg() const {
    if (count_ <= 0) return 0.0_r;
    const Pose m = estimate();

    real sw = 0.0_r, acc = 0.0_r;
    for (int i = 0; i < count_; ++i) {
        const real d = wrapDeg(p_[i].thetaDeg - m.thetaDeg);
        acc += p_[i].weight * d * d;
        sw  += p_[i].weight;
    }
    return (sw > 0.0_r) ? std::sqrt(acc / sw) : 0.0_r;
}

real Mcl::confidence() const {
    if (count_ <= 0 || diverged_) return 0.0_r;
    const real r = cfg_.convergedRadiusInches;
    if (!(r > 0.0_r)) return 0.0_r;
    return clamp(1.0_r - positionStdDevInches() / r, 0.0_r, 1.0_r);
}

} // namespace gflib
