#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace bs {

// lidar.depth ("BSDP") codec — the binary layout is normative in
// docs/FORMATS.md. Values are float16 meters; invalid samples NaN or 0.
// The Swift SessionWriter implements the identical writer; cross-platform
// byte-compatibility is covered by fixture tests.

struct DepthImage {
  int width = 0;
  int height = 0;
  std::vector<uint16_t> f16;  // row-major, width*height

  // Per-pixel confidence AS REPORTED BY THE SENSOR, 0-255, empty when the
  // capture source does not provide one.
  //
  // AVFoundation exposes no such thing — the depth confidence model in
  // lidar/ exists precisely to reconstruct it from geometry. ARKit does
  // (`ARDepthData.confidenceMap`), so a capture backend that can supply it
  // has somewhere to put it, and one that cannot is unaffected: absent
  // reads as "no opinion", never as "zero confidence".
  std::vector<uint8_t> confidence;

  bool has_confidence() const {
    return confidence.size() == f16.size() && !confidence.empty();
  }

  float MetersAt(int x, int y) const;
  bool ValidAt(int x, int y) const;  // finite and > 0
  std::vector<float> ToFloat() const;
};

enum class DepthCodecError {
  kOk = 0,
  kIo,
  kBadMagic,
  kBadVersion,
  kBadHeader,
  kCrcMismatch,
  kDecompressFailed,
  kSizeMismatch,
};

const char* DepthCodecErrorName(DepthCodecError e);

// Serializes with LZ4 block compression when `compress` (falls back to raw
// storage when compression doesn't shrink the payload).
std::vector<uint8_t> EncodeDepth(const DepthImage& depth, bool compress = true);

DepthCodecError DecodeDepth(const uint8_t* data, size_t size, DepthImage& out);

DepthCodecError WriteDepthFile(const std::string& path, const DepthImage& depth,
                               bool compress = true);
DepthCodecError ReadDepthFile(const std::string& path, DepthImage& out);

// CRC-32 (IEEE 802.3, reflected), used by the depth codec and fixture tests.
uint32_t Crc32(const uint8_t* data, size_t size);

}  // namespace bs
