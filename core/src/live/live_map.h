#pragma once

#include <deque>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "common/geometry.h"
#include "fusion/residuals.h"
#include "lidar/depth_processing.h"
#include "vision/features.h"

namespace bs {

// LIVE-layer reconstruction state. Approximate by design, capped, and
// disposable — nothing here is ever authoritative (docs/ARCHITECTURE.md).

struct Keyframe {
  uint32_t kf_id = 0;
  uint32_t frame_id = 0;
  double t_capture = 0;
  SE3 pose;  // world-to-camera

  FeatureSet features;
  // Undistorted pixel coordinates per feature (geometry runs undistorted;
  // features.keypoints keep the raw sensor coordinates).
  std::vector<cv::Point2f> undistorted;
  // Map-point id per feature, -1 when unassociated.
  std::vector<int32_t> point_ids;

  Intrinsics K;  // intrinsics of the tracking-resolution image
  // Per-frame LiDAR observation, processed. Shared because DepthLookup
  // borrows the frame's sanitized buffer.
  std::shared_ptr<DepthFrame> depth;
  std::shared_ptr<DepthLookup> depth_lookup;

  // Loaded from a previous pass's map (the scout circuit) rather than
  // tracked in this one. The scaffold defines the session's world frame, so
  // optimization treats it as a constant reference instead of drifting it.
  bool from_scaffold = false;

  int AssociatedCount() const {
    int n = 0;
    for (const int32_t id : point_ids) n += id >= 0;
    return n;
  }
};

struct MapPoint {
  int32_t id = -1;
  Eigen::Vector3d X = Eigen::Vector3d::Zero();
  cv::Mat descriptor;  // 1 x 32 ORB row, refreshed from the newest obs
  std::vector<std::pair<uint32_t, int>> observations;  // (kf_id, feature idx)

  float mean_gradient = 0;       // image texture at the observations
  float max_tri_angle_deg = 0;
  float last_reproj_err_px = 0;
  int visible_count = 0;  // predicted in frustum
  int found_count = 0;    // matched as inlier
  uint8_t rgb[3] = {180, 180, 180};
  bool from_scaffold = false;  // see Keyframe::from_scaffold

  double TextureWeightNow(double tex_floor) const;
};

class LiveMap {
 public:
  // References stay valid across later AddKeyframe calls (deque storage) —
  // callers hold them across map growth.
  Keyframe& AddKeyframe(Keyframe kf);
  MapPoint& AddPoint(MapPoint point);  // assigns id
  void RemovePoint(int32_t id);

  Keyframe* FindKeyframe(uint32_t kf_id);
  const Keyframe* FindKeyframe(uint32_t kf_id) const;
  MapPoint* FindPoint(int32_t id);

  std::deque<Keyframe>& keyframes() { return keyframes_; }
  const std::deque<Keyframe>& keyframes() const { return keyframes_; }
  std::unordered_map<int32_t, MapPoint>& points() { return points_; }
  const std::unordered_map<int32_t, MapPoint>& points() const { return points_; }

  // Keyframes sharing at least `min_shared` map points with `kf_id`,
  // ordered by shared count descending.
  std::vector<std::pair<uint32_t, int>> Covisible(uint32_t kf_id,
                                                 int min_shared) const;

  // Applies a global similarity: X' = s * X, and for every keyframe pose
  // t' = s * t (rotations unchanged). Used once when the metric gauge locks.
  void ApplyScale(double scale);

 private:
  std::deque<Keyframe> keyframes_;
  std::unordered_map<int32_t, MapPoint> points_;
  int32_t next_point_id_ = 1;
  uint32_t next_kf_id_ = 1;
};

}  // namespace bs
