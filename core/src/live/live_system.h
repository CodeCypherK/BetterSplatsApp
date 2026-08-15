#pragma once

#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "bs/bs_api.h"
#include "common/config.h"
#include "common/geometry.h"
#include "io/depth_codec.h"
#include "live/live_map.h"
#include "readiness/patch_grid.h"

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
// Direction from a pose toward the nearest mapped keyframe, in that pose's
// own CAMERA coordinates (+x right, +y down, +z forward).
//
// Free function rather than a LiveSystem method so the geometry can be
// tested against hand-placed keyframes. Sign and frame conventions here are
// exactly the kind that come out backwards and still look plausible on a
// device, and an arrow that confidently points the wrong way is worse than
// no arrow at all.
struct GuideVector {
  bool valid = false;
  Eigen::Vector3d dir_camera = Eigen::Vector3d::Zero();
  double distance_m = 0;
  uint32_t kf_id = 0;
};

GuideVector GuideToNearestKeyframe(const SE3& from_pose,
                                   const std::deque<Keyframe>& keyframes,
                                   double min_distance_m = 0.35);

class LiveSystem {
 public:
  explicit LiveSystem(const EngineConfig& config);
  ~LiveSystem();

  // `pass` selects scout (build a scaffold) or capture (localize into the
  // scaffold left by a previous scout pass, when one exists).
  void Begin(const std::string& session_dir, double k1, double k2,
             bs_pass_kind pass = BS_PASS_CAPTURE);

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
  void FillGuideVector(bs_live_status& out) const;
  void FillSnapshot(std::vector<bs_snap_point>& points,
                    std::vector<bs_snap_camera>& cameras) const;

  const PatchGrid& readiness() const { return readiness_; }
  void RenameRegion(uint32_t region_id, const std::string& name) {
    region_names_[region_id] = name;
  }
  const std::map<uint32_t, std::string>& region_names() const {
    return region_names_;
  }

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
  // Visual loop closure: revisited areas are recognized by appearance and
  // linked by associating current features to the old map points, creating
  // covisibility that the next local BA uses to pull accumulated drift.
  void TryLoopLink(Keyframe& kf);
  void LocalBundleAdjustment(uint32_t center_kf_id);
  void CullPoints();

  // --- bookkeeping ---
  void EmitDirective(uint32_t frame_id, bs_store_reason reason, bool keyframe);
  void UpdateStoreGate(const LiveFrameInput& input,
                       const FrameFeatures& features);
  void LogPose(const LiveFrameInput& input, bool tracked);
  std::shared_ptr<DepthFrame> MakeDepthFrame(const LiveFrameInput& input) const;
  double MeanGradientAt(const cv::Mat& gray, const cv::Point2f& pt) const;

  EngineConfig config_;
  std::string session_dir_;
  // Directory of the session this one continues, empty when standalone.
  std::string parent_dir_;
  double k1_ = 0, k2_ = 0;
  bs_pass_kind pass_ = BS_PASS_CAPTURE;
  // Keyframes at the head of the map that came from a loaded scaffold. They
  // define the session gauge and are never optimized.
  uint32_t scaffold_keyframes_ = 0;

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
  // Capture time of the last accepted tracked frame, for the motion-
  // plausibility gate (-1 = no reference yet).
  double last_track_time_ = -1.0;
  // Rotating start index for the relocalization sweep over older keyframes.
  uint32_t reloc_cursor_ = 0;

  // Storage gating.
  SE3 last_store_pose_;
  bool have_store_pose_ = false;

  // Status/diagnostics (guarded by the engine's lock via const access).
  uint32_t frames_processed_ = 0;
  // Mean reprojection error of the last frame's PnP inliers, px.
  double last_px_error_ = 0.0;
  uint32_t last_frame_id_ = 0;
  int last_inliers_ = 0;
  int last_matches_ = 0;
  double last_lap_var_ = 0;
  // Published through bs_live_status so the app stores on the engine's rule
  // rather than a second copy of it.
  float store_spacing_m_ = 0;
  double last_gyro_mag_ = 0;
  bs_guidance guidance_ = BS_GUIDE_NONE;

  std::vector<PoseLogEntry> pose_log_;
  std::deque<bs_store_directive> directives_;

  // Rolling sharpness reference: absolute Laplacian variance depends on
  // scene content, so blur gates compare against the recent 80th percentile
  // instead of a fixed number.
  std::deque<double> recent_lap_vars_;
  double BlurThreshold(double fraction) const;

  PatchGrid readiness_;
  std::map<uint32_t, std::string> region_names_;
  uint32_t loop_links_ = 0;
};

}  // namespace bs
