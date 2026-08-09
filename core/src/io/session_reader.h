#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "io/depth_codec.h"
#include "io/session_schema.h"

namespace bs {

// Read-only access to a RAW session directory. This type deliberately has no
// mutating API: the RAW layer is immutable and the engine must never write
// to it (docs/ARCHITECTURE.md, "Three data layers").
class SessionReader {
 public:
  // Opens and validates the session. Returns nullopt when the directory is
  // missing, session.json is absent/incompatible, or frames/ is missing.
  static std::optional<SessionReader> Open(const std::string& session_dir);

  const std::string& dir() const { return dir_; }
  const SessionInfo& info() const { return info_; }
  const CalibrationInfo& calibration() const { return calibration_; }

  // Sorted ascending; enumerated from frames/ on open (source of truth is
  // the directory contents, not session.json's frame_count).
  const std::vector<uint32_t>& frame_ids() const { return frame_ids_; }

  std::string FramePath(uint32_t frame_id) const;
  std::string ImagePath(uint32_t frame_id) const;

  std::optional<FrameMeta> ReadMeta(uint32_t frame_id) const;
  std::optional<DepthImage> ReadDepth(uint32_t frame_id) const;
  // Raw JPEG bytes (decode with cv::imdecode where pixels are needed).
  std::optional<std::vector<uint8_t>> ReadImageBytes(uint32_t frame_id) const;

  // Ground truth is present only in synthetic sessions.
  std::optional<GroundTruth> ReadGroundTruth() const;

 private:
  SessionReader() = default;

  std::string dir_;
  SessionInfo info_;
  CalibrationInfo calibration_;
  std::vector<uint32_t> frame_ids_;
};

}  // namespace bs
