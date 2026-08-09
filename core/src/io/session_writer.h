#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "io/depth_codec.h"
#include "io/session_schema.h"

namespace bs {

// RAW-layer writer used by bs_synth and tests ONLY. On device the RAW layer
// is written by the Swift SessionWriter; the engine itself never links a
// code path that mutates RAW (enforced by test_raw_immutability).
class SessionWriter {
 public:
  // Creates <dir>/frames/. Fails (returns false) when the directory already
  // contains a session — RAW is write-once.
  static bool Create(const std::string& session_dir, const SessionInfo& info,
                     const CalibrationInfo& calibration, SessionWriter& out);

  // Writes one immutable frame directory. jpeg_bytes are stored verbatim.
  bool WriteFrame(const FrameMeta& meta, const std::vector<uint8_t>& jpeg_bytes,
                  const DepthImage& depth);

  // Synthetic sessions only: ground truth next to RAW for tests.
  bool WriteGroundTruth(const GroundTruth& gt);

  // Rewrites session.json with final frame_count/keyframes/end time.
  bool Finalize(const std::string& end_utc);

  const std::string& dir() const { return dir_; }
  SessionInfo& info() { return info_; }

 private:
  std::string dir_;
  SessionInfo info_;
  uint32_t frames_written_ = 0;
};

}  // namespace bs
