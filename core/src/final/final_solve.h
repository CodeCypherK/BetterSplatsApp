#pragma once

#include <atomic>
#include <functional>
#include <string>

#include "bs/bs_api.h"
#include "common/config.h"

namespace bs {

// FINAL-layer pipeline: complete global reconstruction from the RAW session,
// using the live solution only as pose initialization. Re-detects features,
// rebuilds tracks, re-triangulates, registers frames the live pass missed,
// runs global BA with confidence-weighted LiDAR regularization, sweeps
// floaters, aligns LiDAR with the FINAL poses only, and exports the COLMAP
// text model + dense.ply + report.json into <session>/final/.
//
// Runs synchronously on the calling thread; the engine wraps it in its own
// worker thread and forwards progress via the C ABI.

struct FinalMetrics {
  uint32_t images_total = 0;
  uint32_t images_registered = 0;
  uint32_t points = 0;
  float reproj_rmse_px = 0;
  float mean_track_len = 0;
  uint32_t ba_round = 0;
  uint32_t floaters_removed = 0;
  uint32_t lidar_residuals = 0;
  // Resume accounting: stages reloaded from final/cache/ instead of
  // recomputed (features per frame; matches whole-stage).
  uint32_t features_cached = 0;
  uint32_t matches_cached = 0;
};

struct FinalOutcome {
  bool ok = false;
  bool cancelled = false;
  std::string error;
  FinalMetrics metrics;
};

using FinalProgressFn =
    std::function<void(bs_final_stage stage, float stage_progress,
                       const FinalMetrics& metrics)>;

FinalOutcome RunFinalSolve(const EngineConfig& config,
                           const std::string& session_dir,
                           const std::string& preset,
                           const FinalProgressFn& progress,
                           const std::atomic<bool>* cancel);

}  // namespace bs
