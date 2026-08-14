#pragma once

#include <cstdint>
#include <map>
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
  //
  // When this session continues another (session.json's `parent_session`),
  // this spans the WHOLE chain, oldest frames first, and every accessor
  // below resolves an id to whichever session actually holds it. A facility
  // too big for one capture is therefore read exactly like a single session
  // — the final solve needed no changes at all to reconstruct across one.
  //
  // Frame ids are unique across a chain because each session continues the
  // previous one's numbering. A duplicate would be a capture-side bug; it is
  // rejected at open rather than silently resolving to one of the two.
  const std::vector<uint32_t>& frame_ids() const { return frame_ids_; }

  // Session directories in the chain, oldest first. Just this one when there
  // is no parent.
  const std::vector<std::string>& chain() const { return chain_; }
  bool is_chained() const { return chain_.size() > 1; }

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

  // Enumerates frames/ under `dir` into frame_ids_, mapping each to `dir`.
  // Returns false when frames/ is missing or an id already belongs to
  // another session in the chain.
  bool AddFramesFrom(const std::string& dir);

  std::string dir_;
  SessionInfo info_;
  CalibrationInfo calibration_;
  std::vector<uint32_t> frame_ids_;
  std::vector<std::string> chain_;
  // frame id -> the session directory holding it. Single-session sessions
  // pay one small map for the uniformity of never special-casing a chain.
  std::map<uint32_t, std::string> owner_;
};

}  // namespace bs
