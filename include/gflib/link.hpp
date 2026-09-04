#pragma once
#include <cstdint>
#include <cstddef>
#include "gflib/pose.hpp"

// RS-485 link between the ESP32 (owns I/O, odometry, MCL) and the V5 Brain
// (PID and motion) at 100 Hz.

namespace gflib {

// frozen wire prefix
//
// These six header bytes and the trailing CRC are frozen across every
// protocol version.
//
//   [0] 0xA5      sync high
//   [1] 0x5A      sync low
//   [2] version   protocol version
//   [3] type      MsgType
//   [4] len       payload bytes, 0..kLinkMaxPayload
//   [5] seq       sequence counter, wraps 255 -> 0
//   [6 .. 6+len-1] payload, little-endian
//   [6+len, 6+len+1] CRC-16/CCITT-FALSE over bytes [0 .. 6+len-1], LE
//
// The CRC covers the header too, so a corrupted length is caught
inline constexpr uint8_t kLinkSync0 = 0xA5;
inline constexpr uint8_t kLinkSync1 = 0x5A;

// Bump on any payload layout change: a field added, removed, resized or
// reinterpreted. Renaming a field is free.
inline constexpr uint8_t kLinkVersion = 2;

inline constexpr size_t kLinkHeaderBytes = 6;
inline constexpr size_t kLinkCrcBytes    = 2;
inline constexpr size_t kLinkMaxPayload  = 64;
inline constexpr size_t kLinkMaxFrame    = kLinkHeaderBytes + kLinkMaxPayload + kLinkCrcBytes;

// messages

enum class MsgType : uint8_t {
    PoseReport  = 0x01,   // ESP32 -> V5
    BrainStatus = 0x02,   // V5    -> ESP32
    PoseSet     = 0x03,   // V5    -> ESP32
};

// ESP32 -> V5, every 10 ms. The Brain's motion controllers read pose from
// the last one of these that arrived, and age it against their own clock.
struct PoseReport {
    // ESP32 millis at the moment the pose was computed, not at transmit.
    uint32_t timestampMs = 0;

    // Field frame, inches, origin at field centre. Compass: 0 = +Y, CW+.
    float xInches  = 0.0f;
    float yInches  = 0.0f;

    // Unwrapped, as IImu requires. Two turns reads 720, not 0.
    float thetaDegrees = 0.0f;

    // MCL agreement, 0..1. 1 is a tight cloud.
    float confidence = 0.0f;

    uint16_t flags = 0;   // PoseFlags

    // Dead reckoning, uncorrected by MCL. Fallback
    float odomXInches      = 0.0f;
    float odomYInches      = 0.0f;
    float odomThetaDegrees = 0.0f;

    // Changes on every ESP32 boot. A brownout mid-match restarts the ESP32, causing incorrect pose
    uint16_t bootId = 0;

    // Field frame, smoothed. A pose is 15-25 ms old by the time the Brain
    // acts on it; this is what it extrapolates with.
    float vxInchesPerSec = 0.0f;
    float vyInchesPerSec = 0.0f;
    float omegaDegPerSec = 0.0f;
};

// V5 -> ESP32, every 10 ms. Tells the ESP32 what the Brain is doing so it can
// log, drive status LEDs, and decide whether an MCL update is trustworthy.
struct BrainStatus {
    uint32_t timestampMs = 0;

    // What the Brain last commanded, volts.
    float leftVolts  = 0.0f;
    float rightVolts = 0.0f;

    uint8_t  motionState = 0;   // MotionState
    uint16_t flags       = 0;   // BrainFlags
};

// V5 -> ESP32. Forces the ESP32's pose to a known value: auton start, or a
// field-element alignment the driver just performed. The ESP32 reseeds odom
// baselines and the particle cloud, and sets PoseFlags::kPoseReset on its
// next report.
struct PoseSet {
    uint32_t timestampMs = 0;
    float xInches      = 0.0f;
    float yInches      = 0.0f;
    float thetaDegrees = 0.0f;
};

namespace PoseFlags {
inline constexpr uint16_t kImuOk        = 1u << 0;
inline constexpr uint16_t kVertPodOk    = 1u << 1;
inline constexpr uint16_t kHorizPodOk   = 1u << 2;
inline constexpr uint16_t kMclConverged = 1u << 3;
inline constexpr uint16_t kPoseReset    = 1u << 4;   // answers a PoseSet
inline constexpr uint16_t kLinkDegraded = 1u << 5;   // ESP32 is not hearing the Brain
} // namespace PoseFlags

namespace BrainFlags {
inline constexpr uint16_t kAutonomous       = 1u << 0;
inline constexpr uint16_t kDriverControl    = 1u << 1;
inline constexpr uint16_t kDisabled         = 1u << 2;
inline constexpr uint16_t kEstop            = 1u << 3;
inline constexpr uint16_t kRequestPoseReset = 1u << 4;
} // namespace BrainFlags

// Mirrors MotionStatus without depending on motion.hpp. Kept as a plain
// uint8_t on the wire.
enum class MotionState : uint8_t {
    Idle    = 0,
    Running = 1,
    Settled = 2,
    TimedOut = 3,
    Cancelled = 4,
};

// decode

enum class LinkError : uint8_t {
    None = 0,

    // Not an error: not enough bytes yet. Poll again after more arrive.
    Incomplete,

    // A CRC-valid frame whose version byte is not ours. never parsed.
    VersionMismatch,

    // Sync matched but the CRC did not. The parser resyncs past the false sync and keeps going.
    BadCrc,

    // len > kLinkMaxPayload. Cannot be a frame we would ever send.
    BadLength,

    // CRC-valid, version-matched, but a type this build does not know.
    // Forward compatible: ignore it, do not resync.
    UnknownType,
};

// payload points into the parser's own buffer and is invalidated by the next
// push() or poll(). Copy anything you intend to keep, or decode it now.
struct DecodeResult {
    LinkError      err     = LinkError::Incomplete;
    MsgType        type    = MsgType::PoseReport;
    uint8_t        seq     = 0;
    uint8_t        version = 0;   // as seen on the wire, for reporting
    const uint8_t* payload = nullptr;
    uint8_t        payloadLen = 0;
};

// Everything you need to decide the link is unhealthy. Monotonic, never
// reset except by reset().
struct LinkStats {
    uint32_t framesDecoded = 0;
    uint32_t crcErrors = 0;
    uint32_t versionMismatches = 0;
    uint32_t lengthErrors = 0;
    uint32_t unknownTypes = 0;

    // Bytes thrown away hunting for sync. Steadily rising means a baud, wiring, or termination problem, not software.
    uint32_t resyncBytes = 0;

    // Gaps in the sequence counter, i.e. frames that never arrived at all.
    uint32_t droppedFrames = 0;
};

// Byte-stream framer. One per direction, per link. Not thread safe: push() and poll() must be the same task.
class FrameParser {
    public:
        // Copies in what fits and returns how many bytes it took. A short
        // return means the buffer is full because poll() is not being
        // called; drain first, then push the remainder.
        // Retires the previously polled frame first.
        size_t push(const uint8_t* data, size_t len);

        // Resolves at most one frame boundary. Returns true when `out` has
        // been filled, including for a rejected frame, so check out.err.
        // Returns false when the buffer holds no complete frame. Call in a
        // loop until it returns false; one 100 Hz tick can carry several.
        bool poll(DecodeResult& out);

        void reset();

        const LinkStats& stats() const { return stats_; }

        // Bytes waiting to be parsed. Persistently near kLinkMaxFrame * 2
        // means poll() is not keeping up.
        size_t buffered() const { return len_ - pending_; }

    private:
        // Four frames' worth: one being parsed, three arriving behind it.
        uint8_t   buf_[kLinkMaxFrame * 4] = {};
        size_t    len_        = 0;

        // Bytes of an accepted frame still sitting at the front of buf_
        // because the caller holds a payload pointer into them. Dropped by
        // the next push() or poll().
        size_t    pending_    = 0;

        void retirePending();

        LinkStats stats_{};
        uint8_t   lastSeq_    = 0;
        bool      haveSeq_    = false;
};

// encode
//
// Each returns bytes written, or 0 if `cap` was too small. Nothing is
// partially written on failure. `seq` is the caller's counter

size_t encodePoseReport(const PoseReport& msg, uint8_t seq, uint8_t* out, size_t cap);
size_t encodeBrainStatus(const BrainStatus& msg, uint8_t seq, uint8_t* out, size_t cap);
size_t encodePoseSet(const PoseSet& msg, uint8_t seq, uint8_t* out, size_t cap);

// Payload decoders. Each returns false if the result is not that type or the
// payload is the wrong length.
bool decodePoseReport(const DecodeResult& r, PoseReport&  out);
bool decodeBrainStatus(const DecodeResult& r, BrainStatus& out);
bool decodePoseSet(const DecodeResult& r, PoseSet&     out);

// helper functions

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
// Bitwise, so there is no 512-byte table sitting in flash for 22-byte frames.
uint16_t linkCrc16(const uint8_t* data, size_t len);

// Wrap-safe. IClock is 32-bit millis and wraps every ~49.7 days; unsigned
// subtraction is correct through the wrap, a signed one is not.
inline uint32_t linkAgeMs(uint32_t nowMs, uint32_t stampMs) {
    return nowMs - stampMs;
}

inline bool linkIsStale(uint32_t nowMs, uint32_t stampMs, uint32_t maxAgeMs) {
    return linkAgeMs(nowMs, stampMs) > maxAgeMs;
}

// Convenience: fills a Pose from a report. The link carries float; the
// library computes in double.
inline Pose poseFromReport(const PoseReport& r) {
    Pose p;
    p.x        = r.xInches;
    p.y        = r.yInches;
    p.thetaDeg = r.thetaDegrees;
    return p;
}

} // namespace gflib
