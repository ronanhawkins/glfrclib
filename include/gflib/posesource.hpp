#pragma once
#include "gflib/real.hpp"
#include "gflib/pose.hpp"
#include "gflib/hal.hpp"
#include "gflib/odom.hpp"
#include "gflib/link.hpp"
#include <cstdint>

namespace gflib {

// What setPose actually did
enum class PoseSetResult {
    // Local write. getPose() reflects it immediately
    Applied,

    // Sent to the pose owner. getPose() will not reflect it until that owner
    // reseeds and echoes back, roughly 20-40ms later
    Requested,

    // Nothing to send to. The link is unhealthy
    Rejected,
};

// Where a Drivetrain gets its pose. Splits "produce a pose from sensors"
// from "consume a pose to run motions", so a V5 Brain with no odometry
// hardware can still construct a Drivetrain.
class IPoseSource {
    public:
        virtual ~IPoseSource() = default;

        // One tick of whatever keeps the estimate current. NOT thread safe,
        // and must run on the same task as the motions
        virtual void update() = 0;

        // Bring the source to a usable state, blocking up to timeoutMs.
        // Odometry seeds its baselines and returns at once; the link waits
        // for the first healthy frame. Returns false if it gave up.
        virtual bool begin(uint32_t timeoutMs) = 0;

        virtual Pose getPose() const = 0;
        virtual Velocity getVelocity() const = 0;

        virtual PoseSetResult setPose(const Pose& p) = 0;

        // True between a Requested setPose and the pose owner confirming it
        virtual bool poseSetPending() const = 0;

        // 0..1. Dead reckoning has no notion of disagreement, so it says 1
        virtual real confidence() const = 0;

        // How old the estimate is against the CALLER's clock
        virtual uint32_t ageMs(uint32_t nowMs) const = 0;

        virtual bool healthy(uint32_t nowMs) const = 0;
};

// Geometry plus the bounds that decide which readings to believe. Lives here
// rather than in DrivetrainConfig because a Brain with no encoders has no
// use for any of it
struct OdomSourceConfig {
    OdomConfig odom;

    // Error bounds per tick.
    // Max degrees per tick before rejection; at 100Hz this is 9000 deg/s
    real maxDThetaDegPerTick = 90.0_r;

    // Max tracking wheel distance per tick.
    // 6 inches at 100Hz is 50ft/s
    real maxTravelInchesPerTick = 6.0_r;

    // Single-pole EMA on the reported velocity.
    real velocityEmaAlpha = 0.2_r;
};

// Two tracking pods and an IMU, integrated locally
class OdomPoseSource : public IPoseSource {
    public:
        OdomPoseSource(IEncoder& vert, IEncoder& horiz, IImu& imu, IClock& clock, const OdomSourceConfig& cfg);

        // Readings failing the plausibility bounds are dropped
        void update() override;

        // Seeds the sensor baselines from the robot as it stands. timeoutMs
        // is ignored: there is nothing to wait for. Call once, with the robot
        // still, before anything else
        bool begin(uint32_t timeoutMs) override;

        Pose getPose() const override { return pose_; }

        // Smoothed, field frame. Zero until update() has two timestamps to
        // work from, and reset by setPose(), a commanded jump is not motion
        Velocity getVelocity() const override { return vel_; }

        PoseSetResult setPose(const Pose& p) override;
        bool poseSetPending() const override { return false; }

        // Dead reckoning cannot disagree with itself, 
        // the estimate is exactly as current as the
        // last update() call, which the caller controls
        real confidence() const override { return 1.0_r; }
        uint32_t ageMs(uint32_t) const override { return 0; }
        bool healthy(uint32_t) const override { return true; }

        OdomSourceConfig& config() { return cfg_; }
        const OdomSourceConfig& config() const { return cfg_; }

    private:
        IEncoder& vert_;
        IEncoder& horiz_;
        IImu& imu_;
        IClock& clock_;
        OdomSourceConfig cfg_;

        Pose pose_{};
        // double, not real: these accumulate raw encoder counts. See sanify()
        double prevVertCounts_ = 0.0;
        double prevHorizCounts_ = 0.0;
        double prevHeadingDeg_ = 0.0;
        bool seeded_ = false;

        Velocity vel_{};
        uint32_t prevUpdateMs_ = 0;
        bool havePrevUpdate_ = false;
};

struct LinkPoseSourceConfig {
    // Reports arrive at 100Hz. Three missed frames is a link worth aborting
    // a motion over, and short enough that the robot has not travelled far
    uint32_t maxAgeMs = 30;

    // MCL agreement below this is a cloud that has not converged. The pose
    // is a guess and driving on it is worse than stopping
    real minConfidence = 0.2_r;

    // A PoseSet is fire-and-forget on the wire, so retry until the echo
    // arrives rather than hoping the one frame survived
    uint32_t poseSetRetryMs = 20;
};

// The pose arrives over RS-485 from the processor that owns the sensors.
// Holds the last decoded report and the LOCAL millis at which it landed.
//
// It never subtracts PoseReport::timestampMs from local millis: that field
// is the SENDER's clock, and linkElapsedMs requires both arguments from the
// same one. Two free-running millis counters differ by their boot offset,
// which is unbounded
class LinkPoseSource : public IPoseSource {
    public:
        LinkPoseSource(IByteStream& io, IClock& clock, const LinkPoseSourceConfig& cfg = {});

        // Drains the transport and decodes every frame waiting behind it.
        // Frames do not decode themselves, so a tick without this is a tick
        // running on whatever arrived last
        void update() override;

        // Pumps update() until frames are flowing, or timeoutMs expires.
        // Accepts whatever bootId that first report carries.
        //
        // Gates on linked(), NOT on healthy(). The auton sequence is
        // begin() -> setPose(start) -> drive, and the far end's cloud has no
        // reason to have converged before it has been told where the robot
        // is. Waiting on confidence here would block on something that only
        // arrives after the PoseSet that begin() is holding up, and the
        // failure mode is auton never running at all
        bool begin(uint32_t timeoutMs) override;

        // Frames are arriving and the last one is recent. The link works;
        // it says nothing about whether the pose on it is trustworthy.
        //
        // This is the transport question. healthy() is the pose question,
        // and runMotion asks that one every tick
        bool linked(uint32_t nowMs) const;

        Pose getPose() const override;
        Velocity getVelocity() const override;

        PoseSetResult setPose(const Pose& p) override;
        bool poseSetPending() const override { return poseSetPending_; }

        real confidence() const override;
        uint32_t ageMs(uint32_t nowMs) const override;
        bool healthy(uint32_t nowMs) const override;

        // A changed bootId means the far end restarted and no longer knows
        // where it is. Everything it reports is a fresh, wrong origin, so the
        // source stays unhealthy until something with authority says the pose
        // has been re-established: a PoseSet from a known start, or a
        // driver realigning on a field element
        bool bootIdChanged() const { return bootIdChanged_; }
        void acknowledgeBootId() { bootIdChanged_ = false; }
        uint16_t bootId() const { return bootId_; }

        // The whole report, for callers that want the odom fallback or the
        // flags. Meaningless until the first update() lands one
        const PoseReport& lastReport() const { return last_; }
        bool haveReport() const { return haveReport_; }

        // V5 -> ESP32, alongside the pose stream. This side owns the only
        // writer and transport, so nothing else can send one.
        //
        // The far end needs it to weight its MCL: a ToF return taken while
        // the robot is parked and one taken while it is being slammed
        // sideways at full throttle are not the same measurement. Without
        // this it trusts them equally.
        //
        // Returns false on a short write, which means the frame was
        // truncated on the wire. Status is sent fresh every tick, so drop it
        // rather than sending the tail next time
        bool sendStatus(const BrainStatus& status);

        const LinkStats& stats() const { return parser_.stats(); }

        LinkPoseSourceConfig& config() { return cfg_; }
        const LinkPoseSourceConfig& config() const { return cfg_; }

    private:
        void sendPoseSet();
        void adopt(const PoseReport& rep);

        IByteStream& io_;
        IClock& clock_;
        LinkPoseSourceConfig cfg_;

        FrameParser parser_{};
        FrameWriter writer_{};

        PoseReport last_{};
        bool haveReport_ = false;

        // LOCAL millis at which last_ arrived. Never the sender's timestamp
        uint32_t arrivalMs_ = 0;

        uint16_t bootId_ = 0;
        bool haveBootId_ = false;
        bool bootIdChanged_ = false;

        PoseSet pendingSet_{};
        bool poseSetPending_ = false;
        uint32_t poseSetSentMs_ = 0;
};

// Opt-in latency compensationss the field. On
// OdomPoseSource ageMs() is 0, so this is the identity
inline Pose extrapolatePose(const IPoseSource& src, uint32_t nowMs, uint32_t maxExtrapMs) {
    Pose p = src.getPose();

    const uint32_t ageMs = src.ageMs(nowMs);
    const uint32_t useMs = ageMs < maxExtrapMs ? ageMs : maxExtrapMs;
    if (useMs == 0) return p;

    const Velocity v = src.getVelocity();
    const real dtSec = static_cast<real>(useMs) / 1000.0_r;

    p.x += v.vx * dtSec;
    p.y += v.vy * dtSec;

    // Unwrapped, like everything else in this frame. Wrapping here would put
    // a 360 jump into a pose a PID is differencing
    p.thetaDeg += v.omegaDegPerSec * dtSec;
    return p;
}

} // namespace gflib
