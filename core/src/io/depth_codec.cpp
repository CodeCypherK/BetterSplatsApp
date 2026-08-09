#include "io/depth_codec.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <lz4/lz4.h>

#include "io/float16.h"

namespace bs {

namespace {

constexpr char kMagic[4] = {'B', 'S', 'D', 'P'};
constexpr uint16_t kVersion = 1;
constexpr uint16_t kFlagLz4 = 1u << 0;
constexpr uint16_t kDtypeF16Meters = 0;
constexpr size_t kHeaderSize = 24;

void PutU16(std::vector<uint8_t>& buf, uint16_t v) {
  buf.push_back(static_cast<uint8_t>(v & 0xFF));
  buf.push_back(static_cast<uint8_t>(v >> 8));
}

void PutU32(std::vector<uint8_t>& buf, uint32_t v) {
  for (int i = 0; i < 4; ++i) buf.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}

uint16_t GetU16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t GetU32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

const std::array<uint32_t, 256>& Crc32Table() {
  static const std::array<uint32_t, 256> table = [] {
    std::array<uint32_t, 256> t{};
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      t[i] = c;
    }
    return t;
  }();
  return table;
}

}  // namespace

uint32_t Crc32(const uint8_t* data, size_t size) {
  const auto& table = Crc32Table();
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < size; ++i) {
    crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
  }
  return crc ^ 0xFFFFFFFFu;
}

float DepthImage::MetersAt(int x, int y) const {
  return F16ToF32(f16[static_cast<size_t>(y) * width + x]);
}

bool DepthImage::ValidAt(int x, int y) const {
  const float m = MetersAt(x, y);
  return std::isfinite(m) && m > 0.0f;
}

std::vector<float> DepthImage::ToFloat() const {
  std::vector<float> out(f16.size());
  for (size_t i = 0; i < f16.size(); ++i) out[i] = F16ToF32(f16[i]);
  return out;
}

const char* DepthCodecErrorName(DepthCodecError e) {
  switch (e) {
    case DepthCodecError::kOk: return "ok";
    case DepthCodecError::kIo: return "io";
    case DepthCodecError::kBadMagic: return "bad magic";
    case DepthCodecError::kBadVersion: return "bad version";
    case DepthCodecError::kBadHeader: return "bad header";
    case DepthCodecError::kCrcMismatch: return "crc mismatch";
    case DepthCodecError::kDecompressFailed: return "decompress failed";
    case DepthCodecError::kSizeMismatch: return "size mismatch";
  }
  return "unknown";
}

std::vector<uint8_t> EncodeDepth(const DepthImage& depth, bool compress) {
  const size_t raw_bytes = depth.f16.size() * sizeof(uint16_t);
  const uint8_t* raw = reinterpret_cast<const uint8_t*>(depth.f16.data());

  std::vector<uint8_t> payload;
  uint16_t flags = 0;
  if (compress && raw_bytes > 0) {
    std::vector<uint8_t> compressed(LZ4_compressBound(static_cast<int>(raw_bytes)));
    const int n = LZ4_compress_default(
        reinterpret_cast<const char*>(raw), reinterpret_cast<char*>(compressed.data()),
        static_cast<int>(raw_bytes), static_cast<int>(compressed.size()));
    if (n > 0 && static_cast<size_t>(n) < raw_bytes) {
      compressed.resize(static_cast<size_t>(n));
      payload = std::move(compressed);
      flags |= kFlagLz4;
    }
  }
  if ((flags & kFlagLz4) == 0) {
    payload.assign(raw, raw + raw_bytes);
  }

  std::vector<uint8_t> out;
  out.reserve(kHeaderSize + payload.size());
  out.insert(out.end(), kMagic, kMagic + 4);
  PutU16(out, kVersion);
  PutU16(out, flags);
  PutU16(out, static_cast<uint16_t>(depth.width));
  PutU16(out, static_cast<uint16_t>(depth.height));
  PutU16(out, kDtypeF16Meters);
  PutU16(out, 0);  // reserved
  PutU32(out, static_cast<uint32_t>(payload.size()));
  PutU32(out, Crc32(payload.data(), payload.size()));
  out.insert(out.end(), payload.begin(), payload.end());
  return out;
}

DepthCodecError DecodeDepth(const uint8_t* data, size_t size, DepthImage& out) {
  if (data == nullptr || size < kHeaderSize) return DepthCodecError::kBadHeader;
  if (std::memcmp(data, kMagic, 4) != 0) return DepthCodecError::kBadMagic;

  const uint16_t version = GetU16(data + 4);
  if (version != kVersion) return DepthCodecError::kBadVersion;

  const uint16_t flags = GetU16(data + 6);
  const uint16_t width = GetU16(data + 8);
  const uint16_t height = GetU16(data + 10);
  const uint16_t dtype = GetU16(data + 12);
  const uint32_t payload_bytes = GetU32(data + 16);
  const uint32_t expected_crc = GetU32(data + 20);

  if (dtype != kDtypeF16Meters) return DepthCodecError::kBadHeader;
  if (width == 0 || height == 0) return DepthCodecError::kBadHeader;
  if (size - kHeaderSize != payload_bytes) return DepthCodecError::kSizeMismatch;

  const uint8_t* payload = data + kHeaderSize;
  if (Crc32(payload, payload_bytes) != expected_crc) {
    return DepthCodecError::kCrcMismatch;
  }

  const size_t raw_bytes = static_cast<size_t>(width) * height * sizeof(uint16_t);
  out.width = width;
  out.height = height;
  out.f16.resize(static_cast<size_t>(width) * height);

  if (flags & kFlagLz4) {
    const int n = LZ4_decompress_safe(
        reinterpret_cast<const char*>(payload),
        reinterpret_cast<char*>(out.f16.data()), static_cast<int>(payload_bytes),
        static_cast<int>(raw_bytes));
    if (n < 0) return DepthCodecError::kDecompressFailed;
    if (static_cast<size_t>(n) != raw_bytes) return DepthCodecError::kSizeMismatch;
  } else {
    if (payload_bytes != raw_bytes) return DepthCodecError::kSizeMismatch;
    std::memcpy(out.f16.data(), payload, raw_bytes);
  }
  return DepthCodecError::kOk;
}

DepthCodecError WriteDepthFile(const std::string& path, const DepthImage& depth,
                               bool compress) {
  const std::vector<uint8_t> bytes = EncodeDepth(depth, compress);
  std::FILE* f = std::fopen(path.c_str(), "wb");
  if (f == nullptr) return DepthCodecError::kIo;
  const size_t written = std::fwrite(bytes.data(), 1, bytes.size(), f);
  const int close_err = std::fclose(f);
  if (written != bytes.size() || close_err != 0) return DepthCodecError::kIo;
  return DepthCodecError::kOk;
}

DepthCodecError ReadDepthFile(const std::string& path, DepthImage& out) {
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (f == nullptr) return DepthCodecError::kIo;
  std::fseek(f, 0, SEEK_END);
  const long len = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  if (len < 0) {
    std::fclose(f);
    return DepthCodecError::kIo;
  }
  std::vector<uint8_t> bytes(static_cast<size_t>(len));
  const size_t read = std::fread(bytes.data(), 1, bytes.size(), f);
  std::fclose(f);
  if (read != bytes.size()) return DepthCodecError::kIo;
  return DecodeDepth(bytes.data(), bytes.size(), out);
}

}  // namespace bs
