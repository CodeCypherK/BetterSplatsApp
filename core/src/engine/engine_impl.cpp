#include "engine/engine_impl.h"

#include <cstring>
#include <filesystem>

#include "common/log.h"

namespace bs {

namespace fs = std::filesystem;

Engine::Engine(EngineConfig config) : config_(config) {
  log_level() = static_cast<LogLevel>(config_.log_level);
  BS_LOGI("engine", "created");
}

Engine::~Engine() {
  if (live_state_ != BS_LIVE_IDLE && live_state_ != BS_LIVE_FINISHED) {
    LiveEnd();
  }
}

bs_result Engine::Fail(bs_result code, const std::string& message) {
  last_error_ = message;
  BS_LOGE("engine", "%s", message.c_str());
  return code;
}

const char* Engine::LastError() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_.c_str();
}

bs_result Engine::LiveBegin(const char* session_dir) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (session_dir == nullptr || session_dir[0] == '\0') {
    return Fail(BS_ERR_INVALID_ARGUMENT, "LiveBegin: empty session_dir");
  }
  if (live_state_ != BS_LIVE_IDLE && live_state_ != BS_LIVE_FINISHED) {
    return Fail(BS_ERR_INVALID_STATE, "LiveBegin: live session already active");
  }

  std::error_code ec;
  fs::create_directories(fs::path(session_dir) / "live", ec);
  if (ec) {
    return Fail(BS_ERR_IO, "LiveBegin: cannot create live/ in " +
                               std::string(session_dir) + ": " + ec.message());
  }

  session_dir_ = session_dir;
  frames_fed_ = 0;
  frames_processed_ = 0;
  last_frame_id_ = 0;
  live_state_ = BS_LIVE_INITIALIZING;
  BS_LOGI("engine", "live session started at %s", session_dir);
  return BS_OK;
}

bs_result Engine::LiveFeed(const bs_frame_in& frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (live_state_ != BS_LIVE_INITIALIZING && live_state_ != BS_LIVE_TRACKING &&
      live_state_ != BS_LIVE_LOST) {
    return Fail(BS_ERR_INVALID_STATE, "LiveFeed: no live session");
  }
  if (frame.luma == nullptr || frame.luma_width <= 0 || frame.luma_height <= 0) {
    return Fail(BS_ERR_INVALID_ARGUMENT, "LiveFeed: missing luma plane");
  }
  if (frame.frame_id <= last_frame_id_ && frames_fed_ > 0) {
    return Fail(BS_ERR_INVALID_ARGUMENT, "LiveFeed: frame_id not increasing");
  }

  // M4 replaces this with the tracker queue; the skeleton just accounts.
  ++frames_fed_;
  ++frames_processed_;
  last_frame_id_ = frame.frame_id;
  return BS_OK;
}

bs_result Engine::LivePollStatus(bs_live_status& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::memset(&out, 0, sizeof(out));
  out.state = live_state_;
  out.last_frame_id = last_frame_id_;
  out.frames_fed = frames_fed_;
  out.frames_processed = frames_processed_;
  out.guidance =
      live_state_ == BS_LIVE_INITIALIZING ? BS_GUIDE_MOVE_SIDEWAYS : BS_GUIDE_NONE;
  out.q[0] = 1.0;  // identity quaternion until the tracker produces poses
  return BS_OK;
}

bs_result Engine::LiveEnd() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (live_state_ == BS_LIVE_IDLE || live_state_ == BS_LIVE_FINISHED) {
    return Fail(BS_ERR_INVALID_STATE, "LiveEnd: no live session");
  }
  live_state_ = BS_LIVE_FINISHED;
  BS_LOGI("engine", "live session ended after %u frames", frames_fed_);
  return BS_OK;
}

bs_result Engine::SnapshotAcquire(bs_snapshot& out) {
  std::memset(&out, 0, sizeof(out));
  // Real snapshots (points/frusta/patches) land with the live map in M4/M5.
  return BS_OK;
}

void Engine::SnapshotRelease(bs_snapshot& snap) {
  std::memset(&snap, 0, sizeof(snap));
}

bs_result Engine::FinalStart(const char* session_dir, const char* preset) {
  (void)preset;
  std::lock_guard<std::mutex> lock(mutex_);
  if (session_dir == nullptr || session_dir[0] == '\0') {
    return Fail(BS_ERR_INVALID_ARGUMENT, "FinalStart: empty session_dir");
  }
  return Fail(BS_ERR_NOT_IMPLEMENTED, "FinalStart: final solve lands in M6");
}

bs_result Engine::FinalPoll(bs_final_progress& out) {
  std::memset(&out, 0, sizeof(out));
  out.stage = BS_STAGE_IDLE;
  return BS_OK;
}

bs_result Engine::FinalCancel() { return BS_OK; }

bs_result Engine::ThermalHint(int32_t level) {
  if (level < 0 || level > 3) {
    std::lock_guard<std::mutex> lock(mutex_);
    return Fail(BS_ERR_INVALID_ARGUMENT, "ThermalHint: level out of range");
  }
  thermal_level_.store(level);
  return BS_OK;
}

bs_result Engine::RegionRename(uint32_t region_id, const char* utf8_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (utf8_name == nullptr) {
    return Fail(BS_ERR_INVALID_ARGUMENT, "RegionRename: null name");
  }
  (void)region_id;
  return Fail(BS_ERR_NOT_IMPLEMENTED, "RegionRename: regions land in M5");
}

}  // namespace bs
