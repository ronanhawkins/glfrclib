#pragma once
#include <cstdint>

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

class IClock {
    public:
        virtual ~IClock() = default;
        virtual uint32_t millisNow() const = 0;

        // Must yield to the scheduler, not spin. Blocking motions call this
        // between ticks, so a busy-wait starves every other task
        virtual void sleepMs(uint32_t ms) = 0;
};
} //namespace gflib