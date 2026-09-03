#pragma once
#include <cstdint>
#include <cstddef>

namespace gflib{
class IEncoder {
    public:
        virtual ~IEncoder() = default;
        virtual double getCounts() const = 0;
};

class IImu {
    public:
        virtual ~IImu() = default;
        // Unwrapped, CW+, 0 = +Y. Two full turns must read 720, not 0.
        // Odometry differences this, so a wrap here becomes a 360 jump
        virtual double getHeadingDeg() const = 0;
};

class IDriveOutput {
    public:
        virtual ~IDriveOutput() = default;
        virtual void setLeft(double volts) = 0;
        virtual void setRight(double volts) = 0;
        virtual void stop() {setLeft(0); setRight(0);}
};

// Raw byte transport, for the RS-485 link. HardwareSerial on the ESP32, a
// PROS smart-port serial on the V5.
//
// Non-blocking both ways. read() returns what is already buffered and 0 is a
// normal answer, not an error. write() returns what it accepted, which may
// be short
class IByteStream {
    public:
        virtual ~IByteStream() = default;

        // Up to `cap` bytes into `dst`. Returns how many were read.
        virtual size_t read(uint8_t* dst, size_t cap) = 0;

        // Returns how many bytes were accepted. A short write means the
        // frame was truncated on the wire: drop the rest, never send the
        // tail on the next tick
        virtual size_t write(const uint8_t* src, size_t len) = 0;
};

class IClock {
    public:
        virtual ~IClock() = default;
        virtual uint32_t millisNow() const = 0;

        // Must yield to the scheduler, not spin. Blocking motions call this
        // between ticks, so a busy-wait starves every other task
        virtual void sleepMs(uint32_t ms) = 0;
};
} //namespace gflib