#pragma once

// The library's scalar type.
// float rn
// s3 has single precision so float is faster
namespace gflib {

#ifdef GFLIB_DOUBLE_PRECISION
using real = double;
#else
using real = float;
#endif

// Literal suffix. Write 2.0_r
constexpr real operator""_r(long double v) { return static_cast<real>(v); }

} // namespace gflib
