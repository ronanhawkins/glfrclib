#include "gflib/posesource.hpp"
#include "gflib/util.hpp"
#include <cmath>

namespace gflib {

namespace {

// Never store a non-finite reading
void seed(double& prev, double reading) {
    if (std::isfinite(reading)) prev = reading;
}

// The delta to integrate, or 0 if it is outside bounds. scale puts the limit
// in inches; the IMU passes 1.0 and works in degrees.

real sanify(double& prev, double reading, real scale, real limit) {
    const real delta = static_cast<real>(reading - prev);
    const bool ok = std::isfinite(delta) &&
                    (limit <= 0.0_r || std::fabs(delta * scale) <= limit);

    seed(prev, reading);
    return ok ? delta : 0.0_r;
}

}

// Odom Pose Source

OdomPoseSource::OdomPoseSource(IEncoder& vert, IEncoder& horiz, IImu& imu, IClock& clock, const OdomSourceConfig& cfg)
    : vert_(vert), horiz_(horiz), imu_(imu), clock_(clock), cfg_(cfg) {}

bool OdomPoseSource::begin(uint32_t) {
    seed(prevVertCounts_, vert_.getCounts());
    seed(prevHorizCounts_, horiz_.getCounts());
    seed(prevHeadingDeg_, imu_.getHeadingDeg());
    seeded_ = true;
    return true;
}

void OdomPoseSource::update() {
    // Before begin() the baselines are 0, seed instead of integrating it
    if (!seeded_) {
        begin(0);
        return;
    }

    // Judged per channel, an IMU glitch should not discard a good tick
    const real dVertCounts = sanify(prevVertCounts_, vert_.getCounts(), cfg_.odom.vertInchesPerCount, cfg_.maxTravelInchesPerTick);
    const real dHorizCounts = sanify(prevHorizCounts_, horiz_.getCounts(), cfg_.odom.horizInchesPerCount, cfg_.maxTravelInchesPerTick);
    const real dThetaDeg = sanify(prevHeadingDeg_, imu_.getHeadingDeg(), 1.0_r, cfg_.maxDThetaDegPerTick);

    const Pose prev = pose_;
    pose_ = odomStep(pose_, dVertCounts, dHorizCounts, dThetaDeg, cfg_.odom);

    // Measured interval
    const uint32_t nowMs = clock_.millisNow();
    if (!havePrevUpdate_) {
        prevUpdateMs_ = nowMs;
        havePrevUpdate_ = true;
        return;
    }

    // Unsigned, so it is correct across the 49.7-day millis wrap.
    const uint32_t elapsedMs = nowMs - prevUpdateMs_;
    if (elapsedMs == 0) return;
    prevUpdateMs_ = nowMs;

    const real dtSec = static_cast<real>(elapsedMs) / 1000.0_r;
    const real ivx = (pose_.x - prev.x) / dtSec;
    const real ivy = (pose_.y - prev.y) / dtSec;
    const real iw  = (pose_.thetaDeg - prev.thetaDeg) / dtSec;

    // A rejected tick contributes a zero delta, which is the honest reading:
    // it decays every channel toward rest rather than holding the last value.
    const real a = clamp(cfg_.velocityEmaAlpha, 0.0_r, 1.0_r);
    vel_.vx += a * (ivx - vel_.vx);
    vel_.vy += a * (ivy - vel_.vy);
    vel_.omegaDegPerSec += a * (iw - vel_.omegaDegPerSec);
}

PoseSetResult OdomPoseSource::setPose(const Pose& p) {
    pose_ = p;

    // A commanded jump is not motion.
    vel_ = Velocity{};
    havePrevUpdate_ = false;

    // Rebase the sensor baselines, or the next update() applies every count
    // accumulated since the last one as motion away from the new pose
    begin(0);

    // The pose lives here, so it is already true
    return PoseSetResult::Applied;
}

// Link Pose Source

LinkPoseSource::LinkPoseSource(IByteStream& io, IClock& clock, const LinkPoseSourceConfig& cfg)
    : io_(io), clock_(clock), cfg_(cfg) {}

void LinkPoseSource::update() {
    // One frame at a time would leave the rest queued and the pose one tick
    // stale per call, so drain the transport until it stops giving bytes
    uint8_t chunk[kLinkMaxFrame * 2];
    for (;;) {
        const size_t got = io_.read(chunk, sizeof(chunk));
        if (got == 0) break;

        size_t taken = 0;
        while (taken < got) {
            const size_t pushed = parser_.push(chunk + taken, got - taken);
            taken += pushed;

            // One tick of a 100Hz link can carry several frames
            DecodeResult r;
            while (parser_.poll(r)) {
                if (r.err != LinkError::None) continue;

                PoseReport rep;
                if (!decodePoseReport(r, rep)) continue;
                adopt(rep);
            }

            // A short push means the parser was full; the poll above has now
            // drained it. Taking nothing even so would spin forever
            if (pushed == 0) break;
        }

        // A short read means the transport had nothing more to give
        if (got < sizeof(chunk)) break;
    }

    // A PoseSet is one unacknowledged frame on a noisy bus. Resend until the
    // echo arrives rather than assuming the first one survived
    if (poseSetPending_ && linkElapsedMs(clock_.millisNow(), poseSetSentMs_) >= cfg_.poseSetRetryMs) {
        sendPoseSet();
    }
}

void LinkPoseSource::adopt(const PoseReport& rep) {
    // A changed bootId means the far end restarted. Latch it before adopting
    // the report: everything after a restart is measured from an origin
    // nobody agreed to
    if (haveBootId_ && rep.bootId != bootId_) bootIdChanged_ = true;
    bootId_ = rep.bootId;
    haveBootId_ = true;

    last_ = rep;
    haveReport_ = true;

    // LOCAL millis. rep.timestampMs is the sender's clock and shares no epoch
    arrivalMs_ = clock_.millisNow();

    // The far end confirms a reseed by flagging it, not by matching coordinates
    if (poseSetPending_ && (rep.flags & PoseFlags::kPoseReset) != 0) {
        poseSetPending_ = false;

        // A PoseSet re-establishes the origin
        bootIdChanged_ = false;
    }
}

bool LinkPoseSource::begin(uint32_t timeoutMs) {
    const uint32_t startMs = clock_.millisNow();
    for (;;) {
        update();

        // Whatever bootId the first report carries is the one we started
        // with, so it is not a restart
        if (haveReport_) bootIdChanged_ = false;

        const uint32_t nowMs = clock_.millisNow();
        if (healthy(nowMs)) return true;
        if (linkElapsedMs(nowMs, startMs) >= timeoutMs) return false;
        clock_.sleepMs(1);
    }
}

Pose LinkPoseSource::getPose() const {
    // not extrapolated by ageMs
    return poseFromReport(last_);
}

Velocity LinkPoseSource::getVelocity() const {
    Velocity v;
    v.vx = last_.vxInchesPerSec;
    v.vy = last_.vyInchesPerSec;
    v.omegaDegPerSec = last_.omegaDegPerSec;
    return v;
}

void LinkPoseSource::sendPoseSet() {
    uint8_t frame[kLinkMaxFrame];
    pendingSet_.timestampMs = clock_.millisNow();
    const size_t n = writer_.poseSet(pendingSet_, frame, sizeof(frame));
    if (n > 0) io_.write(frame, n);

    // Timed even on a short write
    poseSetSentMs_ = clock_.millisNow();
}

PoseSetResult LinkPoseSource::setPose(const Pose& p) {
    // Rejected only when there is nothing on the other end to hear it.
    if (ageMs(clock_.millisNow()) > cfg_.maxAgeMs) return PoseSetResult::Rejected;

    pendingSet_.xInches      = static_cast<float>(p.x);
    pendingSet_.yInches      = static_cast<float>(p.y);
    pendingSet_.thetaDegrees = static_cast<float>(p.thetaDeg);

    poseSetPending_ = true;
    sendPoseSet();

    // Requested, not Applied: getPose() still reports the old pose until the
    // far end reseeds its particle cloud and echoes back
    return PoseSetResult::Requested;
}

real LinkPoseSource::confidence() const {
    return haveReport_ ? static_cast<real>(last_.confidence) : 0.0_r;
}

uint32_t LinkPoseSource::ageMs(uint32_t nowMs) const {
    // No report at all is infinitely old, not zero
    if (!haveReport_) return UINT32_MAX;

    // Both from OUR clock. See the comment on linkElapsedMs
    return linkElapsedMs(nowMs, arrivalMs_);
}

bool LinkPoseSource::healthy(uint32_t nowMs) const {
    if (!haveReport_) return false;
    if (bootIdChanged_) return false;
    if (ageMs(nowMs) > cfg_.maxAgeMs) return false;
    return confidence() >= cfg_.minConfidence;
}

} // namespace gflib
