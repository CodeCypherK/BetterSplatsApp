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

  // Walk back up the chain first, so the oldest session's frames are
  // enumerated first and `chain()` reads in capture order. Sessions are
  // siblings on disk, which is what makes a chain survive being zipped on a
  // phone and unzipped somewhere else.
  //
  // The parent walk is bounded and cycle-checked. `parent_session` is just a
  // string in a JSON file the app wrote; a bug or a hand-edit could point a
  // session at itself or round a loop, and neither should hang the solve.
  std::vector<std::string> ancestry;
  {
    std::vector<std::string> seen;
    std::string dir = session_dir;
    SessionInfo info_at = r.info_;
    constexpr size_t kMaxChain = 64;
    while (!info_at.parent_session.empty() && ancestry.size() < kMaxChain) {
      const fs::path parent = root.parent_path() / info_at.parent_session;
      const std::string parent_dir = parent.string();
      if (std::find(seen.begin(), seen.end(), parent_dir) != seen.end() ||
          parent_dir == session_dir) {
        BS_LOGW("session", "parent chain loops at %s; stopping",
                parent_dir.c_str());
        break;
      }
      auto parent_info =
          SessionInfo::FromJson(ReadTextFile((parent / "session.json").string()));
      if (!parent_info) {
        // A missing parent is a real possibility — the user may have deleted
        // or not yet copied it. Reconstruct what IS here rather than
        // refusing, and say so loudly, because the result covers less of the
        // building than the chain claims.
        BS_LOGW("session",
                "parent session %s missing or unreadable; reconstructing "
                "this session alone",
                info_at.parent_session.c_str());
        break;
      }
      seen.push_back(parent_dir);
      ancestry.push_back(parent_dir);
      dir = parent_dir;
      info_at = std::move(*parent_info);
    }
  }
  std::reverse(ancestry.begin(), ancestry.end());  // oldest first

  for (const auto& ancestor : ancestry) {
    if (!r.AddFramesFrom(ancestor)) return std::nullopt;
    r.chain_.push_back(ancestor);
  }
  if (!r.AddFramesFrom(session_dir)) return std::nullopt;
  r.chain_.push_back(session_dir);

  std::sort(r.frame_ids_.begin(), r.frame_ids_.end());

  // Collect every rescan declared anywhere in the chain, tagged with where it
  // sits, so a frame can be tested against only the rescans that came after
  // it. Re-reading each session.json is a handful of small files and keeps
  // the walk above concerned with one thing.
  for (size_t i = 0; i < r.chain_.size(); ++i) {
    const SessionInfo* info_at = &r.info_;
    std::optional<SessionInfo> parsed;
    if (r.chain_[i] != session_dir) {
      parsed = SessionInfo::FromJson(
          ReadTextFile((fs::path(r.chain_[i]) / "session.json").string()));
      if (!parsed) continue;
      info_at = &*parsed;
    }
    for (const auto& volume : info_at->supersedes) {
      r.supersessions_.push_back({volume, i});
    }
  }

  if (r.chain_.size() > 1 || !r.supersessions_.empty()) {
    BS_LOGI("session",
            "chained session: %zu sessions, %zu frames, %zu rescan volumes",
            r.chain_.size(), r.frame_ids_.size(), r.supersessions_.size());
  }
  return r;
}

size_t SessionReader::ChainIndexOf(uint32_t frame_id) const {
  const auto it = owner_.find(frame_id);
  if (it == owner_.end()) return chain_.size();
  const auto at = std::find(chain_.begin(), chain_.end(), it->second);
  return static_cast<size_t>(std::distance(chain_.begin(), at));
}

bool SessionReader::IsSuperseded(uint32_t frame_id,
                                 const double world_position[3],
                                 std::string* label) const {
  if (supersessions_.empty()) return false;
  const size_t frame_chain_index = ChainIndexOf(frame_id);
  for (const auto& s : supersessions_) {
    // Strictly later. A session does not supersede its own frames — it IS
    // the rescan — and a rescan cannot invalidate work done after it.
    if (s.chain_index <= frame_chain_index) continue;
    if (s.volume.Contains(world_position[0], world_position[1],
                          world_position[2])) {
      if (label != nullptr) *label = s.volume.label;
      return true;
    }
  }
  return false;
}

bool SessionReader::AddFramesFrom(const std::string& dir) {
  const fs::path frames = fs::path(dir) / "frames";
  if (!fs::is_directory(frames)) {
    BS_LOGW("session", "missing frames/ in %s", dir.c_str());
    return false;
  }
  for (const auto& entry : fs::directory_iterator(frames)) {
    if (!entry.is_directory()) continue;
    const std::string name = entry.path().filename().string();
    if (name.size() != 6 ||
        !std::all_of(name.begin(), name.end(),
                     [](char c) { return c >= '0' && c <= '9'; })) {
      continue;
    }
    const uint32_t frame_id = static_cast<uint32_t>(std::stoul(name));
    const auto [it, inserted] = owner_.emplace(frame_id, dir);
    if (!inserted) {
      // Two sessions claiming one id means the chain's numbering broke, and
      // every downstream id — COLMAP image ids, the export filenames, the
      // per-image report — would silently describe the wrong picture.
      BS_LOGW("session", "frame %06u claimed by both %s and %s", frame_id,
              it->second.c_str(), dir.c_str());
      return false;
    }
    frame_ids_.push_back(frame_id);
  }
  return true;
}

std::string SessionReader::FramePath(uint32_t frame_id) const {
  const auto it = owner_.find(frame_id);
  const std::string& dir = it == owner_.end() ? dir_ : it->second;
  return (fs::path(dir) / "frames" / FrameDirName(frame_id)).string();
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
