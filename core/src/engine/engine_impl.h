#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "bs/bs_api.h"
#include "common/config.h"
#include "live/live_system.h"

namespace bs {

// Engine facade behind the C ABI. Owns the live pipeline and the final-solve
// pipeline (M6); this class is the only place that holds cross-module state.
// All public methods are thread-safe; live processing currently runs
// synchronously inside LiveFeed (callers feed from their own capture queue).
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
  void MaybeFitDistortion(const bs_frame_in& frame);

  EngineConfig config_;

  mutable std::mutex mutex_;
  std::string last_error_;

  std::unique_ptr<LiveSystem> live_;
  std::string session_dir_;
  uint32_t frames_fed_ = 0;
  uint32_t last_frame_id_ = 0;
  bool distortion_ready_ = false;
  double k1_ = 0, k2_ = 0;

  struct SnapshotStorage {
    std::vector<bs_snap_point> points;
    std::vector<bs_snap_camera> cameras;
    std::vector<bs_snap_patch> patches;
    std::vector<bs_snap_region> regions;
    std::vector<bs_snap_weak_area> weak_areas;
  };

  std::atomic<int32_t> thermal_level_{0};
};

}  // namespace bs
