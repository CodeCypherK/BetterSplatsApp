#include "io/session_reader.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>

#include "common/log.h"

namespace bs {

namespace fs = std::filesystem;

std::optional<SessionReader> SessionReader::Open(const std::string& session_dir) {
  SessionReader r;
  r.dir_ = session_dir;

  const fs::path root(session_dir);
  if (!fs::is_directory(root)) {
    BS_LOGW("session", "not a directory: %s", session_dir.c_str());
    return std::nullopt;
  }

  const std::string session_text = ReadTextFile((root / "session.json").string());
  auto info = SessionInfo::FromJson(session_text);
  if (!info) {
    BS_LOGW("session", "missing/incompatible session.json in %s",
            session_dir.c_str());
    return std::nullopt;
  }
  r.info_ = std::move(*info);

  const std::string calib_text = ReadTextFile((root / "calibration.json").string());
  if (auto calib = CalibrationInfo::FromJson(calib_text)) {
    r.calibration_ = std::move(*calib);
  } else {
    BS_LOGW("session", "missing calibration.json in %s (continuing)",
            session_dir.c_str());
  }

  const fs::path frames = root / "frames";
  if (!fs::is_directory(frames)) {
    BS_LOGW("session", "missing frames/ in %s", session_dir.c_str());
    return std::nullopt;
  }
  for (const auto& entry : fs::directory_iterator(frames)) {
    if (!entry.is_directory()) continue;
    const std::string name = entry.path().filename().string();
    if (name.size() != 6 ||
        !std::all_of(name.begin(), name.end(),
                     [](char c) { return c >= '0' && c <= '9'; })) {
      continue;
    }
    r.frame_ids_.push_back(static_cast<uint32_t>(std::stoul(name)));
  }
  std::sort(r.frame_ids_.begin(), r.frame_ids_.end());
  return r;
}

std::string SessionReader::FramePath(uint32_t frame_id) const {
  return (fs::path(dir_) / "frames" / FrameDirName(frame_id)).string();
}

std::string SessionReader::ImagePath(uint32_t frame_id) const {
  return (fs::path(FramePath(frame_id)) / "image.jpg").string();
}

std::optional<FrameMeta> SessionReader::ReadMeta(uint32_t frame_id) const {
  const std::string text =
      ReadTextFile((fs::path(FramePath(frame_id)) / "meta.json").string());
  if (text.empty()) return std::nullopt;
  return FrameMeta::FromJson(text);
}

std::optional<DepthImage> SessionReader::ReadDepth(uint32_t frame_id) const {
  DepthImage depth;
  const auto err = ReadDepthFile(
      (fs::path(FramePath(frame_id)) / "lidar.depth").string(), depth);
  if (err != DepthCodecError::kOk) {
    BS_LOGW("session", "frame %u depth read failed: %s", frame_id,
            DepthCodecErrorName(err));
    return std::nullopt;
  }
  return depth;
}

std::optional<std::vector<uint8_t>> SessionReader::ReadImageBytes(
    uint32_t frame_id) const {
  std::FILE* f = std::fopen(ImagePath(frame_id).c_str(), "rb");
  if (f == nullptr) return std::nullopt;
  std::fseek(f, 0, SEEK_END);
  const long len = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  if (len < 0) {
    std::fclose(f);
    return std::nullopt;
  }
  std::vector<uint8_t> bytes(static_cast<size_t>(len));
  const size_t read = std::fread(bytes.data(), 1, bytes.size(), f);
  std::fclose(f);
  if (read != bytes.size()) return std::nullopt;
  return bytes;
}

std::optional<GroundTruth> SessionReader::ReadGroundTruth() const {
  const std::string text =
      ReadTextFile((fs::path(dir_) / "ground_truth" / "poses.json").string());
  if (text.empty()) return std::nullopt;
  return GroundTruth::FromJson(text);
}

}  // namespace bs
