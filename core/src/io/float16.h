#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

namespace bs {

// IEEE 754 half-precision conversion. Depth maps are stored as float16
// meters (matching AVDepthData's native kCVPixelFormatType_DepthFloat16),
// so the codec must round-trip Apple's exact bit patterns.

inline float F16ToF32(uint16_t h) {
  const uint32_t sign = static_cast<uint32_t>(h & 0x8000u) << 16;
  const uint32_t exponent = (h >> 10) & 0x1Fu;
  const uint32_t mantissa = h & 0x3FFu;

  uint32_t bits;
  if (exponent == 0) {
    if (mantissa == 0) {
      bits = sign;  // +-0
    } else {
      // Subnormal half -> normalized float.
      int e = -1;
      uint32_t m = mantissa;
      do {
        ++e;
        m <<= 1;
      } while ((m & 0x400u) == 0);
      bits = sign | ((127 - 15 - e) << 23) | ((m & 0x3FFu) << 13);
    }
  } else if (exponent == 0x1F) {
    bits = sign | 0x7F800000u | (mantissa << 13);  // inf / NaN
  } else {
    bits = sign | ((exponent - 15 + 127) << 23) | (mantissa << 13);
  }

  float out;
  std::memcpy(&out, &bits, sizeof(out));
  return out;
}

inline uint16_t F32ToF16(float f) {
  uint32_t bits;
  std::memcpy(&bits, &f, sizeof(bits));

  const uint16_t sign = static_cast<uint16_t>((bits >> 16) & 0x8000u);
  const int32_t exponent = static_cast<int32_t>((bits >> 23) & 0xFFu) - 127 + 15;
  uint32_t mantissa = bits & 0x7FFFFFu;

  if (((bits >> 23) & 0xFFu) == 0xFFu) {
    // inf / NaN; keep a mantissa bit for NaN.
    return static_cast<uint16_t>(sign | 0x7C00u | (mantissa ? 0x200u : 0));
  }
  if (exponent >= 0x1F) {
    return static_cast<uint16_t>(sign | 0x7C00u);  // overflow -> inf
  }
  if (exponent <= 0) {
    if (exponent < -10) return sign;  // underflow -> 0
    // Subnormal half with round-to-nearest.
    mantissa |= 0x800000u;
    const int shift = 14 - exponent;
    uint32_t sub = mantissa >> shift;
    if ((mantissa >> (shift - 1)) & 1u) ++sub;  // round
    return static_cast<uint16_t>(sign | sub);
  }
  // Normalized with round-to-nearest-even.
  uint32_t half = static_cast<uint32_t>(sign) |
                  (static_cast<uint32_t>(exponent) << 10) | (mantissa >> 13);
  if ((mantissa & 0x1FFFu) > 0x1000u ||
      ((mantissa & 0x1FFFu) == 0x1000u && (half & 1u))) {
    ++half;  // may carry into exponent, which is still correct behavior
  }
  return static_cast<uint16_t>(half);
}

}  // namespace bs
