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
    };
}