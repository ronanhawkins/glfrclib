#include "gflib/link.hpp"
#include <cstring>
#include <limits>

namespace gflib {

// No struct memcpy: it bakes in one compiler's padding and one ABI's float
// layout, and there are three compilers here: clang (native), xtensa-gcc
// (ESP32) and arm-none-eabi-gcc (V5). Bytes go out one at a time, explicitly
static_assert(std::numeric_limits<float>::is_iec559, "wire format is IEEE-754 binary32");
static_assert(sizeof(float) == 4, "wire format is IEEE-754 binary32");

namespace {

void putU8(uint8_t* p, size_t& i, uint8_t v) { p[i++] = v; }

void putU16(uint8_t* p, size_t& i, uint16_t v) {
    p[i++] = static_cast<uint8_t>(v & 0xFF);
    p[i++] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void putU32(uint8_t* p, size_t& i, uint32_t v) {
    p[i++] = static_cast<uint8_t>(v & 0xFF);
    p[i++] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[i++] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[i++] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

// Bit pattern of the float, then that pattern little-endian. memcpy of a
// scalar is the portable spelling of a bit cast, not the struct memcpy the
// comment above forbids.
void putF32(uint8_t* p, size_t& i, float v) {
    uint32_t bits;
    std::memcpy(&bits, &v, sizeof bits);
    putU32(p, i, bits);
}

uint8_t  getU8 (const uint8_t* p, size_t& i) { return p[i++]; }

uint16_t getU16(const uint8_t* p, size_t& i) {
    const uint16_t v = static_cast<uint16_t>(p[i]) |
                       static_cast<uint16_t>(static_cast<uint16_t>(p[i + 1]) << 8);
    i += 2;
    return v;
}

uint32_t getU32(const uint8_t* p, size_t& i) {
    const uint32_t v = static_cast<uint32_t>(p[i]) |
                       (static_cast<uint32_t>(p[i + 1]) << 8) |
                       (static_cast<uint32_t>(p[i + 2]) << 16) |
                       (static_cast<uint32_t>(p[i + 3]) << 24);
    i += 4;
    return v;
}

float getF32(const uint8_t* p, size_t& i) {
    const uint32_t bits = getU32(p, i);
    float v;
    std::memcpy(&v, &bits, sizeof v);
    return v;
}

// Payload sizes. Named so the decoders can reject a frame whose length does
// not match its type.
constexpr uint8_t kPoseReportBytes = 4 + 4 + 4 + 4 + 4 + 2 + 12 + 2 + 12;   // 48
constexpr uint8_t kBrainStatusBytes = 4 + 4 + 4 + 1 + 2;       // 15
constexpr uint8_t kPoseSetBytes = 4 + 4 + 4 + 4;           // 16

static_assert(kPoseReportBytes <= kLinkMaxPayload, "payload exceeds frame");
static_assert(kBrainStatusBytes <= kLinkMaxPayload, "payload exceeds frame");
static_assert(kPoseSetBytes <= kLinkMaxPayload, "payload exceeds frame");

// Writes header, then whatever writePayload emits, then the CRC over both.
// Every encoder goes through here, so the frozen prefix has one
// implementation.
template <typename F>
size_t frame(MsgType type, uint8_t seq, uint8_t payloadLen,
             uint8_t* out, size_t cap, F writePayload) {
    const size_t total = kLinkHeaderBytes + payloadLen + kLinkCrcBytes;
    if (out == nullptr || cap < total) return 0;

    size_t i = 0;
    putU8(out, i, kLinkSync0);
    putU8(out, i, kLinkSync1);
    putU8(out, i, kLinkVersion);
    putU8(out, i, static_cast<uint8_t>(type));
    putU8(out, i, payloadLen);
    putU8(out, i, seq);

    writePayload(out, i);

    const uint16_t crc = linkCrc16(out, kLinkHeaderBytes + payloadLen);
    putU16(out, i, crc);
    return i;
}

} // namespace

uint16_t linkCrc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8);
        for (int b = 0; b < 8; ++b) {
            // Cast back to uint16_t at every step: integer promotion makes
            // crc << 1 an int, and the high bit would survive and corrupt
            // the next round.
            crc = (crc & 0x8000u) ? static_cast<uint16_t>(static_cast<uint16_t>(crc << 1) ^ 0x1021u)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// encode

size_t encodePoseReport(const PoseReport& msg, uint8_t seq, uint8_t* out, size_t cap) {
    return frame(MsgType::PoseReport, seq, kPoseReportBytes, out, cap,
                 [&](uint8_t* p, size_t& i) {
                     putU32(p, i, msg.timestampMs);
                     putF32(p, i, msg.xInches);
                     putF32(p, i, msg.yInches);
                     putF32(p, i, msg.thetaDegrees);
                     putF32(p, i, msg.confidence);
                     putU16(p, i, msg.flags);
                     putF32(p, i, msg.odomXInches);
                     putF32(p, i, msg.odomYInches);
                     putF32(p, i, msg.odomThetaDegrees);
                     putU16(p, i, msg.bootId);
                     putF32(p, i, msg.vxInchesPerSec);
                     putF32(p, i, msg.vyInchesPerSec);
                     putF32(p, i, msg.omegaDegPerSec);
                 });
}

size_t encodeBrainStatus(const BrainStatus& msg, uint8_t seq, uint8_t* out, size_t cap) {
    return frame(MsgType::BrainStatus, seq, kBrainStatusBytes, out, cap,
                 [&](uint8_t* p, size_t& i) {
                     putU32(p, i, msg.timestampMs);
                     putF32(p, i, msg.leftVolts);
                     putF32(p, i, msg.rightVolts);
                     putU8 (p, i, msg.motionState);
                     putU16(p, i, msg.flags);
                 });
}

size_t encodePoseSet(const PoseSet& msg, uint8_t seq, uint8_t* out, size_t cap) {
    return frame(MsgType::PoseSet, seq, kPoseSetBytes, out, cap,
                 [&](uint8_t* p, size_t& i) {
                     putU32(p, i, msg.timestampMs);
                     putF32(p, i, msg.xInches);
                     putF32(p, i, msg.yInches);
                     putF32(p, i, msg.thetaDegrees);
                 });
}

// decode

namespace {

// A payload is only readable if the frame passed CRC, matched our version,
// was a known type, and is exactly the length that type must be.
bool readable(const DecodeResult& r, MsgType type, uint8_t len) {
    return r.err == LinkError::None && r.type == type &&
           r.payload != nullptr && r.payloadLen == len;
}

} // namespace

bool decodePoseReport(const DecodeResult& r, PoseReport& out) {
    if (!readable(r, MsgType::PoseReport, kPoseReportBytes)) return false;
    size_t i = 0;
    out.timestampMs  = getU32(r.payload, i);
    out.xInches      = getF32(r.payload, i);
    out.yInches      = getF32(r.payload, i);
    out.thetaDegrees = getF32(r.payload, i);
    out.confidence   = getF32(r.payload, i);
    out.flags        = getU16(r.payload, i);
    out.odomXInches      = getF32(r.payload, i);
    out.odomYInches      = getF32(r.payload, i);
    out.odomThetaDegrees = getF32(r.payload, i);
    out.bootId           = getU16(r.payload, i);
    out.vxInchesPerSec   = getF32(r.payload, i);
    out.vyInchesPerSec   = getF32(r.payload, i);
    out.omegaDegPerSec   = getF32(r.payload, i);
    return true;
}

bool decodeBrainStatus(const DecodeResult& r, BrainStatus& out) {
    if (!readable(r, MsgType::BrainStatus, kBrainStatusBytes)) return false;
    size_t i = 0;
    out.timestampMs = getU32(r.payload, i);
    out.leftVolts   = getF32(r.payload, i);
    out.rightVolts  = getF32(r.payload, i);
    out.motionState = getU8 (r.payload, i);
    out.flags       = getU16(r.payload, i);
    return true;
}

bool decodePoseSet(const DecodeResult& r, PoseSet& out) {
    if (!readable(r, MsgType::PoseSet, kPoseSetBytes)) return false;
    size_t i = 0;
    out.timestampMs  = getU32(r.payload, i);
    out.xInches      = getF32(r.payload, i);
    out.yInches      = getF32(r.payload, i);
    out.thetaDegrees = getF32(r.payload, i);
    return true;
}

// parser

void FrameParser::retirePending() {
    if (pending_ == 0) return;
    len_ -= pending_;
    std::memmove(buf_, buf_ + pending_, len_);
    pending_ = 0;
}

size_t FrameParser::push(const uint8_t* data, size_t len) {
    if (data == nullptr) return 0;

    // The frame handed out by the last poll() is still occupying the front of the buffer.
    retirePending();

    const size_t room = sizeof buf_ - len_;
    const size_t n = len < room ? len : room;
    std::memcpy(buf_ + len_, data, n);
    len_ += n;
    return n;
}

void FrameParser::reset() {
    len_ = 0;
    pending_ = 0;
    stats_ = LinkStats{};
    lastSeq_ = 0;
    haveSeq_ = false;
}

bool FrameParser::poll(DecodeResult& out) {
    out = DecodeResult{};

    // Retire the frame the previous poll() handed out.
    retirePending();

    for (;;) {
        // Hunt for sync. Anything before it is noise, a half-frame mid-stream, or the tail of a frame we already rejected.
        size_t start = 0;
        while (start + 1 < len_ &&
               !(buf_[start] == kLinkSync0 && buf_[start + 1] == kLinkSync1)) {
            ++start;
        }

        if (start > 0) {
            stats_.resyncBytes += static_cast<uint32_t>(start);
            len_ -= start;
            std::memmove(buf_, buf_ + start, len_);
        }

        // A lone trailing 0xA5 might be the first half of a real frame, so keep it and wait for its partner.
        if (len_ < 2) {
            out.err = LinkError::Incomplete;
            return false;
        }

        if (len_ < kLinkHeaderBytes) {
            out.err = LinkError::Incomplete;
            return false;
        }

        const uint8_t version = buf_[2];
        const uint8_t type    = buf_[3];
        const uint8_t payLen  = buf_[4];
        const uint8_t seq     = buf_[5];

        // Length is checked before we wait on it: an impossible length would
        // otherwise park the parser waiting for bytes that are not coming.
        if (payLen > kLinkMaxPayload) {
            ++stats_.lengthErrors;
            ++stats_.resyncBytes;
            len_ -= 1;
            std::memmove(buf_, buf_ + 1, len_);
            out.err = LinkError::BadLength;
            out.version = version;
            return true;
        }

        const size_t total = kLinkHeaderBytes + payLen + kLinkCrcBytes;
        if (len_ < total) {
            out.err = LinkError::Incomplete;
            return false;
        }

        size_t ci = kLinkHeaderBytes + payLen;
        const uint16_t got = getU16(buf_, ci);
        const uint16_t want = linkCrc16(buf_, kLinkHeaderBytes + payLen);

        if (got != want) {
            // Do not consume the whole frame. If this was a false sync, the
            // real frame may start inside these bytes. Step one byte and rehunt.
            ++stats_.crcErrors;
            ++stats_.resyncBytes;
            len_ -= 1;
            std::memmove(buf_, buf_ + 1, len_);
            out.err = LinkError::BadCrc;
            out.version = version;
            return true;
        }

        // CRC-valid from here on, so the header is trustworthy and the frame
        // can be consumed whole
        auto consume = [&]() {
            len_ -= total;
            std::memmove(buf_, buf_ + total, len_);
        };

        out.version    = version;
        out.seq        = seq;
        out.payloadLen = payLen;

        // Version before payload. A version we do not know may lay its bytes
        // out any way it likes; reading them as ours is parsing garbage.
        if (version != kLinkVersion) {
            ++stats_.versionMismatches;
            consume();
            out.err = LinkError::VersionMismatch;
            return true;
        }

        // Sequence gaps are counted on frames we accept as ours, so link
        // noise shows up as crcErrors and genuinely lost frames as drops.
        if (haveSeq_) {
            const uint8_t expected = static_cast<uint8_t>(lastSeq_ + 1);
            if (seq != expected) {
                stats_.droppedFrames += static_cast<uint8_t>(seq - expected);
            }
        }
        lastSeq_ = seq;
        haveSeq_ = true;

        switch (static_cast<MsgType>(type)) {
            case MsgType::PoseReport:
            case MsgType::BrainStatus:
            case MsgType::PoseSet:
                break;
            default:
                // Same version, unknown type
                ++stats_.unknownTypes;
                consume();
                out.err = LinkError::UnknownType;
                return true;
        }

        out.type = static_cast<MsgType>(type);
        out.err  = LinkError::None;

        // Payload points into buf_, so compacting here would invalidate it.
        // The compaction happens on the next poll() instead.
        out.payload = buf_ + kLinkHeaderBytes;
        pending_ = total;
        ++stats_.framesDecoded;
        return true;
    }
}

} // namespace gflib
