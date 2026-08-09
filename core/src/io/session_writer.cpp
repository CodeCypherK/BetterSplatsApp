#include "io/session_writer.h"

#include <cstdio>
#include <filesystem>

#include "common/log.h"

namespace bs {

namespace fs = std::filesystem;

namespace {

bool WriteBinaryFile(const std::string& path, const std::vector<uint8_t>& bytes) {
  std::FILE* f = std::fopen(path.c_str(), "wb");
  if (f == nullptr) return false;
  const size_t written = std::fwrite(bytes.data(), 1, bytes.size(), f);
  return std::fclose(f) == 0 && written == bytes.size();
}

}  // namespace

bool SessionWriter::Create(const std::string& session_dir, const SessionInfo& info,
                           const CalibrationInfo& calibration, SessionWriter& out) {
  const fs::path root(session_dir);
  if (fs::exists(root / "session.json")) {
    BS_LOGE("session", "refusing to overwrite existing session at %s",
            session_dir.c_str());
    return false;
  }
  std::error_code ec;
  fs::create_directories(root / "frames", ec);
  if (ec) {
    BS_LOGE("session", "cannot create %s: %s", session_dir.c_str(),
            ec.message().c_str());
    return false;
  }

  out.dir_ = session_dir;
  out.info_ = info;
  out.frames_written_ = 0;

  if (!WriteTextFile((root / "session.json").string(), info.ToJson())) return false;
  if (!WriteTextFile((root / "calibration.json").string(), calibration.ToJson())) {
    return false;
  }
  return true;
}

bool SessionWriter::WriteFrame(const FrameMeta& meta,
                               const std::vector<uint8_t>& jpeg_bytes,
                               const DepthImage& depth) {
  const fs::path frame_dir = fs::path(dir_) / "frames" / FrameDirName(meta.frame_id);
  if (fs::exists(frame_dir)) {
    BS_LOGE("session", "frame %u already exists; RAW is write-once", meta.frame_id);
    return false;
  }
  std::error_code ec;
  fs::create_directories(frame_dir, ec);
  if (ec) return false;

  if (!WriteBinaryFile((frame_dir / "image.jpg").string(), jpeg_bytes)) return false;
  if (WriteDepthFile((frame_dir / "lidar.depth").string(), depth) !=
      DepthCodecError::kOk) {
    return false;
  }
  if (!WriteTextFile((frame_dir / "meta.json").string(), meta.ToJson())) return false;

  ++frames_written_;
  if (meta.is_keyframe) info_.keyframe_ids.push_back(meta.frame_id);
  return true;
}

bool SessionWriter::WriteGroundTruth(const GroundTruth& gt) {
  const fs::path gt_dir = fs::path(dir_) / "ground_truth";
  std::error_code ec;
  fs::create_directories(gt_dir, ec);
  if (ec) return false;
  return WriteTextFile((gt_dir / "poses.json").string(), gt.ToJson());
}

bool SessionWriter::Finalize(const std::string& end_utc) {
  info_.end_utc = end_utc;
  info_.frame_count = frames_written_;
  return WriteTextFile((fs::path(dir_) / "session.json").string(), info_.ToJson());
}

}  // namespace bs
