#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include "bs/bs_api.h"
#include "common/config.h"

namespace bs {

// Engine facade behind the C ABI. Owns the live pipeline and the final-solve
// pipeline (added in later milestones); this class is the only place that
// holds cross-module state. All public methods are thread-safe.
class Engine {
 public:
  explicit Engine(EngineConfig config);
  ~Engine();

  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;

  bs_result LiveBegin(const char* session_dir);
  bs_result LiveFeed(const bs_frame_in& frame);
  bs_result LivePollStatus(bs_live_status& out);
  bs_result LiveEnd();

  bs_result SnapshotAcquire(bs_snapshot& out);
  void SnapshotRelease(bs_snapshot& snap);

  bs_result FinalStart(const char* session_dir, const char* preset);
  bs_result FinalPoll(bs_final_progress& out);
  bs_result FinalCancel();

  bs_result ThermalHint(int32_t level);
  bs_result RegionRename(uint32_t region_id, const char* utf8_name);

  const char* LastError() const;

  const EngineConfig& config() const { return config_; }

 private:
  bs_result Fail(bs_result code, const std::string& message);

  EngineConfig config_;

  mutable std::mutex mutex_;
  std::string last_error_;

  // Live-session state. The full tracker/mapper stack lands in M4; the
  // skeleton keeps the ABI honest (state machine + counters) from M0 so the
  // app and replay CLI integrate against real semantics immediately.
  bs_live_state live_state_ = BS_LIVE_IDLE;
  std::string session_dir_;
  uint32_t frames_fed_ = 0;
  uint32_t frames_processed_ = 0;
  uint32_t last_frame_id_ = 0;
  std::atomic<int32_t> thermal_level_{0};
};

}  // namespace bs
