// Depth codec: exact round-trips, corruption detection, decode robustness.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <random>

#include "bs/bs_api.h"
#include "io/depth_codec.h"
#include "io/float16.h"

namespace bs {
namespace {

namespace fs = std::filesystem;

// Smooth surface-like depth (a tilted plane with gentle waves and NaN
// holes) — representative of real LiDAR maps, which compress well.
DepthImage MakeTestDepth(int w, int h, uint32_t seed) {
  DepthImage d;
  d.width = w;
  d.height = h;
  d.f16.resize(static_cast<size_t>(w) * h);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> chance(0.0f, 1.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      auto& v = d.f16[static_cast<size_t>(y) * w + x];
      if (chance(rng) < 0.03f) {
        v = F32ToF16(std::numeric_limits<float>::quiet_NaN());
      } else {
        const float m = 1.5f + 0.004f * x + 0.006f * y +
                        0.05f * std::sin(0.11f * x) * std::cos(0.07f * y);
        v = F32ToF16(m);
      }
    }
  }
  return d;
}

// Uniformly random f16 payloads are effectively incompressible.
DepthImage MakeIncompressibleDepth(int w, int h, uint32_t seed) {
  DepthImage d;
  d.width = w;
  d.height = h;
  d.f16.resize(static_cast<size_t>(w) * h);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> uni(0.1f, 6.0f);
  for (auto& v : d.f16) v = F32ToF16(uni(rng));
  return d;
}

TEST(Float16, ExhaustiveRoundTrip) {
  // Every finite/NaN/inf half value must survive f16 -> f32 -> f16 exactly.
  int mismatches = 0;
  for (uint32_t h = 0; h <= 0xFFFF; ++h) {
    const uint16_t half = static_cast<uint16_t>(h);
    const float f = F16ToF32(half);
    const uint16_t back = F32ToF16(f);
    if (back != half) {
      // The only tolerated difference: NaN payloads may collapse, but NaN
      // must stay NaN with the same sign.
      const bool src_nan = (half & 0x7C00u) == 0x7C00u && (half & 0x3FFu) != 0;
      const bool dst_nan = (back & 0x7C00u) == 0x7C00u && (back & 0x3FFu) != 0;
      const bool same_sign = (half & 0x8000u) == (back & 0x8000u);
      if (!(src_nan && dst_nan && same_sign)) {
        if (++mismatches < 5) {
          ADD_FAILURE() << "half 0x" << std::hex << half << " -> " << f
                        << " -> 0x" << back;
        }
      }
    }
  }
  EXPECT_EQ(mismatches, 0);
}

TEST(DepthCodec, RoundTripCompressed) {
  const DepthImage src = MakeTestDepth(320, 240, 42);
  const std::vector<uint8_t> bytes = EncodeDepth(src, /*compress=*/true);
  EXPECT_LT(bytes.size(), src.f16.size() * 2 + 24);  // actually compressed

  DepthImage out;
  ASSERT_EQ(DecodeDepth(bytes.data(), bytes.size(), out), DepthCodecError::kOk);
  EXPECT_EQ(out.width, src.width);
  EXPECT_EQ(out.height, src.height);
  EXPECT_EQ(out.f16, src.f16);  // bit-exact
}

TEST(DepthCodec, IncompressibleFallsBackToRawStorage) {
  // When LZ4 cannot shrink the payload the encoder must store raw — and the
  // round-trip must still be exact.
  const DepthImage src = MakeIncompressibleDepth(320, 240, 42);
  const std::vector<uint8_t> bytes = EncodeDepth(src, /*compress=*/true);
  EXPECT_EQ(bytes.size(), 24 + src.f16.size() * 2);

  DepthImage out;
  ASSERT_EQ(DecodeDepth(bytes.data(), bytes.size(), out), DepthCodecError::kOk);
  EXPECT_EQ(out.f16, src.f16);
}

TEST(DepthCodec, RoundTripUncompressed) {
  const DepthImage src = MakeTestDepth(64, 48, 7);
  const std::vector<uint8_t> bytes = EncodeDepth(src, /*compress=*/false);
  EXPECT_EQ(bytes.size(), 24 + src.f16.size() * 2);

  DepthImage out;
  ASSERT_EQ(DecodeDepth(bytes.data(), bytes.size(), out), DepthCodecError::kOk);
  EXPECT_EQ(out.f16, src.f16);
}

TEST(DepthCodec, FileRoundTrip) {
  const auto path = (fs::temp_directory_path() / "bs_codec_test.depth").string();
  const DepthImage src = MakeTestDepth(320, 240, 99);
  ASSERT_EQ(WriteDepthFile(path, src), DepthCodecError::kOk);
  DepthImage out;
  ASSERT_EQ(ReadDepthFile(path, out), DepthCodecError::kOk);
  EXPECT_EQ(out.f16, src.f16);
  fs::remove(path);
}

TEST(DepthCodec, DetectsCorruption) {
  const DepthImage src = MakeTestDepth(80, 60, 3);
  std::vector<uint8_t> bytes = EncodeDepth(src, true);

  // Flip one payload byte -> CRC must catch it.
  std::vector<uint8_t> corrupted = bytes;
  corrupted[corrupted.size() - 1] ^= 0x5A;
  DepthImage out;
  EXPECT_EQ(DecodeDepth(corrupted.data(), corrupted.size(), out),
            DepthCodecError::kCrcMismatch);

  // Truncation -> size mismatch.
  EXPECT_EQ(DecodeDepth(bytes.data(), bytes.size() - 10, out),
            DepthCodecError::kSizeMismatch);

  // Wrong magic.
  corrupted = bytes;
  corrupted[0] = 'X';
  EXPECT_EQ(DecodeDepth(corrupted.data(), corrupted.size(), out),
            DepthCodecError::kBadMagic);

  // Future version is rejected.
  corrupted = bytes;
  corrupted[4] = 2;
  EXPECT_EQ(DecodeDepth(corrupted.data(), corrupted.size(), out),
            DepthCodecError::kBadVersion);
}

TEST(DepthCodec, RandomGarbageNeverCrashes) {
  std::mt19937 rng(1234);
  std::uniform_int_distribution<int> byte(0, 255);
  std::uniform_int_distribution<int> len(0, 4096);
  DepthImage out;
  for (int i = 0; i < 2000; ++i) {
    std::vector<uint8_t> garbage(static_cast<size_t>(len(rng)));
    for (auto& b : garbage) b = static_cast<uint8_t>(byte(rng));
    // Half the runs get a valid magic+version prefix to reach deeper code.
    if (i % 2 == 0 && garbage.size() >= 6) {
      std::memcpy(garbage.data(), "BSDP", 4);
      garbage[4] = 1;
      garbage[5] = 0;
    }
    DecodeDepth(garbage.data(), garbage.size(), out);  // must not crash/UB
  }
  SUCCEED();
}

TEST(DepthCodec, AbiEncodeMatchesNativeCodec) {
  // bs_depth_encode (the Swift writer's path) must produce byte-identical
  // output to the native encoder.
  const DepthImage src = MakeTestDepth(160, 120, 5);
  const std::vector<uint8_t> native = EncodeDepth(src, true);

  size_t len = 0;
  const uint8_t* abi = bs_depth_encode(src.f16.data(), src.width, src.height, &len);
  ASSERT_NE(abi, nullptr);
  ASSERT_EQ(len, native.size());
  EXPECT_EQ(std::memcmp(abi, native.data(), len), 0);
  bs_buffer_release(abi);

  EXPECT_EQ(bs_depth_encode(nullptr, 10, 10, &len), nullptr);
  EXPECT_EQ(bs_depth_encode(src.f16.data(), 0, 10, &len), nullptr);
  bs_buffer_release(nullptr);  // must not crash
}

TEST(DepthCodec, Crc32KnownAnswer) {
  // CRC-32 of "123456789" is the classic check value 0xCBF43926.
  const char* s = "123456789";
  EXPECT_EQ(Crc32(reinterpret_cast<const uint8_t*>(s), 9), 0xCBF43926u);
}


// --- optional confidence plane (ARKit's confidenceMap) ---

TEST(DepthCodecConfidence, RoundTripsBothPlanes) {
  DepthImage in;
  in.width = 40;
  in.height = 30;
  in.f16.resize(40 * 30);
  in.confidence.resize(40 * 30);
  for (size_t i = 0; i < in.f16.size(); ++i) {
    in.f16[i] = F32ToF16(1.0f + 0.001f * static_cast<float>(i));
    in.confidence[i] = static_cast<uint8_t>(i % 256);
  }
  for (bool compress : {false, true}) {
    const auto bytes = EncodeDepth(in, compress);
    DepthImage out;
    ASSERT_EQ(DecodeDepth(bytes.data(), bytes.size(), out),
              DepthCodecError::kOk) << "compress=" << compress;
    EXPECT_EQ(out.f16, in.f16);
    EXPECT_EQ(out.confidence, in.confidence);
    EXPECT_TRUE(out.has_confidence());
  }
}

TEST(DepthCodecConfidence, AbsentStaysAbsentAndBytesAreUnchanged) {
  // The compatibility property: a depth map with no confidence must encode
  // to exactly what it always did, so every session already on disk still
  // decodes and nothing about the format changed for them.
  DepthImage in;
  in.width = 16;
  in.height = 12;
  in.f16.resize(16 * 12);
  for (size_t i = 0; i < in.f16.size(); ++i) {
    in.f16[i] = F32ToF16(2.0f + 0.01f * static_cast<float>(i));
  }
  const auto bytes = EncodeDepth(in, /*compress=*/false);
  // Header flags must have only the (unset) lz4 bit — no confidence bit.
  EXPECT_EQ(bytes[6] & 0x02, 0) << "confidence flag set on a plain frame";
  EXPECT_EQ(bytes.size(), 24u + in.f16.size() * 2);

  DepthImage out;
  ASSERT_EQ(DecodeDepth(bytes.data(), bytes.size(), out), DepthCodecError::kOk);
  EXPECT_EQ(out.f16, in.f16);
  EXPECT_FALSE(out.has_confidence());
  EXPECT_TRUE(out.confidence.empty());
}

TEST(DepthCodecConfidence, TruncationIsRejectedNotHalfRead) {
  // A file cut short must fail, not decode a plausible depth plane with
  // garbage confidence — the CRC and size cover both planes together.
  DepthImage in;
  in.width = 20;
  in.height = 20;
  in.f16.assign(400, F32ToF16(1.5f));
  in.confidence.assign(400, 200);
  auto bytes = EncodeDepth(in, /*compress=*/false);
  bytes.resize(bytes.size() - 50);
  DepthImage out;
  EXPECT_NE(DecodeDepth(bytes.data(), bytes.size(), out),
            DepthCodecError::kOk);
}

}  // namespace
}  // namespace bs
