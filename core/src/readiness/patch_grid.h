#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "bs/bs_api.h"
#include "common/geometry.h"
#include "live/live_map.h"

namespace bs {

// Splat-readiness scoring: a world-space voxel patch grid accumulating
// evidence from the live (or final) map, scored on five axes per patch:
//   0 geometry   — point density vs surface area, conditioning, residuals
//   1 pose       — observing-keyframe support
//   2 texture    — gradient energy and pixels-per-cm sampling density
//   3 lidar      — depth sample coverage x confidence
//   4 view       — angular sector diversity and baseline/distance ratio
// Patches aggregate with a min-capped weighted mean; weak clusters produce
// ranked, directional guidance. LIVE-layer: disposable, rebuilt on demand.

struct PatchKey {
  int32_t x = 0, y = 0, z = 0;
  bool operator==(const PatchKey& o) const {
    return x == o.x && y == o.y && z == o.z;
  }
};

struct PatchKeyHash {
  size_t operator()(const PatchKey& k) const {
    return static_cast<size_t>(k.x) * 73856093u ^
           static_cast<size_t>(k.y) * 19349663u ^
           static_cast<size_t>(k.z) * 83492791u;
  }
};

struct Patch {
  PatchKey key;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

  int point_count = 0;
  double err_sum = 0;
  double angle_max_sum = 0;
  double gradient_sum = 0;
  double distance_sum = 0;      // mean observation distance accumulator
  double max_view_angle_deg = 0;  // widest pairwise angular separation
  double max_baseline = 0;
  std::vector<uint32_t> observing_kfs;

  int lidar_samples = 0;
  double lidar_conf_sum = 0;

  uint32_t region_id = 1;
  float sub[5] = {0, 0, 0, 0, 0};
  float score = 0;
};

struct ReadinessOptions {
  double patch_size_m = 0.35;
  double weak_threshold = 70.0;
  double density_target_per_m2 = 300.0;
  // Focal length of the STORED capture images (px) — texture sampling
  // density (px/cm) is about what splat training sees, not the tracking
  // downscale.
  double capture_focal_px = 1450.0;
  double view_angle_full_deg = 25.0;
  int max_weak_areas = 5;
  // Region clustering: keyframes this covisible AND this close share a room.
  int region_min_shared_points = 15;
  double region_max_kf_distance_m = 2.0;
};

struct WeakArea {
  Eigen::Vector3d centroid;
  double radius_m = 0;
  uint32_t region_id = 1;
  int deficiency = 0;     // argmin sub-score index
  int surface_kind = 0;   // 0 wall, 1 floor, 2 ceiling, 3 object
  Eigen::Vector3d move_dir = Eigen::Vector3d::UnitX();
  double move_dist_m = 0.5;
  float score = 0;
  double priority = 0;
  int patch_count = 0;
};

// Region = connected component of the keyframe covisibility graph
// (edges: enough shared points AND spatially close), i.e. "Room 1..N" in
// discovery order. Patches inherit the majority region of their observers.
struct RegionAggregate {
  uint32_t id = 1;
  float score = 0;
  float sub[5] = {0, 0, 0, 0, 0};
  double area_m2 = 0;
  uint32_t patch_count = 0;
};

class PatchGrid {
 public:
  explicit PatchGrid(const ReadinessOptions& options = {});

  // Rebuilds the grid from the map (points + keyframes with depth).
  void Build(const LiveMap& map);

  const std::unordered_map<PatchKey, Patch, PatchKeyHash>& patches() const {
    return patches_;
  }
  const std::vector<WeakArea>& weak_areas() const { return weak_areas_; }
  const std::vector<RegionAggregate>& regions() const { return regions_; }

  // Area-weighted aggregate over all patches, 0-100.
  float OverallScore() const { return overall_; }
  float SubScore(int axis) const { return overall_sub_[axis]; }
  double PatchArea() const;

  // `names`: user renames per region id; unmatched ids get "Room N".
  void FillSnapshot(std::vector<bs_snap_patch>& patches,
                    std::vector<bs_snap_region>& regions,
                    std::vector<bs_snap_weak_area>& weak,
                    const std::map<uint32_t, std::string>& names) const;

 private:
  void ScorePatch(Patch& patch) const;
  void FindWeakAreas();
  // Fills region_of_kf via union-find over the covisibility graph.
  std::map<uint32_t, uint32_t> ClusterRegions(const LiveMap& map) const;

  ReadinessOptions options_;
  std::unordered_map<PatchKey, Patch, PatchKeyHash> patches_;
  std::vector<WeakArea> weak_areas_;
  std::vector<RegionAggregate> regions_;
  float overall_ = 0;
  float overall_sub_[5] = {0, 0, 0, 0, 0};
  // False when no keyframe carries depth (LiDAR-less map): the lidar axis
  // then scores neutrally and never drives deficiency diagnosis.
  bool has_lidar_ = false;
};

}  // namespace bs
