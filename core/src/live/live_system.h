#pragma once

#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "bs/bs_api.h"
#include "common/config.h"
#include "common/geometry.h"
#include "io/depth_codec.h"
#include "live/live_map.h"

namespace bs {

// Engine-internal copy of one input frame at tracking resolution.
struct LiveFrameInput {
  uint32_t frame_id = 0;
  double t_capture = 0;
  double t_depth = 0;
  cv::Mat gray;          // owned, tracking resolution
  DepthImage depth;      // owned float16 copy
  Intrinsics K;          // for `gray` pixels
  Intrinsics Kd;         // for `depth` pixels
  Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
  bool gyro_valid = false;
};

struct PoseLogEntry {
  uint32_t frame_id = 0;
  double t = 0;
  bool tracked = false;
  SE3 pose;
};

// Live incremental SfM: image-first bootstrap + PnP tracking + keyframe
// mapping with LiDAR-regularized local BA. Single-threaded internally —
// the engine wraps it in a worker thread with a drop-oldest mailbox (live
// processing is lossy by design; RAW storage never routes through here).
class LiveSystem {
 public:
  explicit LiveSystem(const EngineConfig& config);
  ~LiveSystem();

  void Begin(const std::string& session_dir, double k1, double k2);

  // Late distortion injection: the device app delivers the calibration LUT
  // with the first frames, after Begin. Only meaningful before bootstrap
  // completes (keypoints are undistorted at extraction time).
  void SetDistortion(double k1, double k2) {
    k1_ = k1;
    k2_ = k2;
  }

  // Processes one frame synchronously. Returns the post-frame live state.
  bs_live_state Feed(const LiveFrameInput& input);

  void FillStatus(bs_live_status& out) const;
  void FillSnapshot(std::vector<bs_snap_point>& points,
                    std::vector<bs_snap_camera>& cameras) const;

  // Flushes live/poses.jsonl + summary; returns false on IO failure.
  bool End();

  const LiveMap& map() const { return map_; }
  bs_live_state state() const { return state_; }
  bool scale_locked() const { return scale_locked_; }

  // Drains directives accumulated since the last call (max BS_MAX_DIRECTIVES).
  std::vector<bs_store_directive> DrainDirectives();

 private:
  struct FrameFeatures {
    FeatureSet features;
    std::vector<cv::Point2f> undistorted;
    double lap_var = 0;
    double overexp_frac = 0;
  };

  FrameFeatures ExtractFeatures(const LiveFrameInput& input) const;

  // --- bootstrap ---
  void TryBootstrap(const LiveFrameInput& input, FrameFeatures features);
  void ResetBootstrap();

  // --- tracking ---
  bool TrackFrame(const LiveFrameInput& input, const FrameFeatures& features);
  bool Relocalize(const LiveFrameInput& input, const FrameFeatures& features);
  SE3 PredictPose() const;

  // --- mapping ---
  bool ShouldInsertKeyframe(const LiveFrameInput& input,
                            const FrameFeatures& features,
                            int tracked_inliers) const;
  void InsertKeyframe(const LiveFrameInput& input, FrameFeatures features,
                      const std::vector<std::pair<int32_t, int>>& matches);
  void TriangulateNewPoints(Keyframe& kf);
  void LocalBundleAdjustment(uint32_t center_kf_id);
  void CullPoints();

  // --- bookkeeping ---
  void EmitDirective(uint32_t frame_id, bs_store_reason reason, bool keyframe);
  void UpdateStoreGate(const LiveFrameInput& input);
  void LogPose(const LiveFrameInput& input, bool tracked);
  std::shared_ptr<DepthFrame> MakeDepthFrame(const LiveFrameInput& input) const;
  double MeanGradientAt(const cv::Mat& gray, const cv::Point2f& pt) const;

  EngineConfig config_;
  std::string session_dir_;
  double k1_ = 0, k2_ = 0;

  bs_live_state state_ = BS_LIVE_IDLE;
  LiveMap map_;

  // Bootstrap state.
  std::optional<LiveFrameInput> ref_input_;
  std::optional<FrameFeatures> ref_features_;
  std::vector<double> scale_samples_;
  bool scale_locked_ = false;
  int bootstrap_attempts_ = 0;

  // Tracking state.
  SE3 last_pose_;
  SE3 velocity_;  // last_pose ∘ prev_pose^-1
  bool have_prev_pose_ = false;
  uint32_t last_kf_id_ = 0;
  double last_kf_time_ = -1e9;
  int consecutive_lost_ = 0;

  // Storage gating.
  SE3 last_store_pose_;
  bool have_store_pose_ = false;

  // Status/diagnostics (guarded by the engine's lock via const access).
  uint32_t frames_processed_ = 0;
  uint32_t last_frame_id_ = 0;
  int last_inliers_ = 0;
  int last_matches_ = 0;
  double last_lap_var_ = 0;
  double last_gyro_mag_ = 0;
  bs_guidance guidance_ = BS_GUIDE_NONE;

  std::vector<PoseLogEntry> pose_log_;
  std::deque<bs_store_directive> directives_;

  // Rolling sharpness reference: absolute Laplacian variance depends on
  // scene content, so blur gates compare against the recent 80th percentile
  // instead of a fixed number.
  std::deque<double> recent_lap_vars_;
  double BlurThreshold(double fraction) const;
};

}  // namespace bs
