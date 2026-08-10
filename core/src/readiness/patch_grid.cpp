#include "readiness/patch_grid.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <set>

#include <Eigen/Eigenvalues>

namespace bs {

namespace {

double Sat(double x) { return std::min(1.0, std::max(0.0, x)); }

struct KfUnionFind {
  std::map<uint32_t, uint32_t> parent;
  uint32_t Find(uint32_t a) {
    auto it = parent.find(a);
    if (it == parent.end()) {
      parent[a] = a;
      return a;
    }
    while (it->second != a) {
      a = it->second;
      it = parent.find(a);
    }
    return a;
  }
  void Union(uint32_t a, uint32_t b) { parent[Find(a)] = Find(b); }
};

}  // namespace

PatchGrid::PatchGrid(const ReadinessOptions& options) : options_(options) {}

double PatchGrid::PatchArea() const {
  return options_.patch_size_m * options_.patch_size_m;
}

std::map<uint32_t, uint32_t> PatchGrid::ClusterRegions(
    const LiveMap& map) const {
  // Covisibility counted from point observations directly (not the
  // per-keyframe association lists) so clustering works on any map that
  // carries observations, however it was assembled.
  std::unordered_map<uint64_t, int> pair_count;
  for (const auto& [_, mp] : map.points()) {
    for (size_t i = 0; i < mp.observations.size(); ++i) {
      for (size_t j = i + 1; j < mp.observations.size(); ++j) {
        uint32_t a = mp.observations[i].first;
        uint32_t b = mp.observations[j].first;
        if (a == b) continue;
        if (a > b) std::swap(a, b);
        ++pair_count[(static_cast<uint64_t>(a) << 32) | b];
      }
    }
  }

  KfUnionFind uf;
  for (const auto& kf : map.keyframes()) uf.Find(kf.kf_id);
  for (const auto& [key, count] : pair_count) {
    if (count < options_.region_min_shared_points) continue;
    const uint32_t a = static_cast<uint32_t>(key >> 32);
    const uint32_t b = static_cast<uint32_t>(key & 0xffffffffu);
    const Keyframe* ka = map.FindKeyframe(a);
    const Keyframe* kb = map.FindKeyframe(b);
    if (ka == nullptr || kb == nullptr) continue;
    if ((ka->pose.CameraCenter() - kb->pose.CameraCenter()).norm() <
        options_.region_max_kf_distance_m) {
      uf.Union(a, b);
    }
  }
  // Region ids in discovery order (smallest keyframe id first).
  std::map<uint32_t, uint32_t> root_to_region;
  std::map<uint32_t, uint32_t> region_of_kf;
  uint32_t next_region = 1;
  for (const auto& kf : map.keyframes()) {
    const uint32_t root = uf.Find(kf.kf_id);
    auto [it, inserted] = root_to_region.emplace(root, next_region);
    if (inserted) ++next_region;
    region_of_kf[kf.kf_id] = it->second;
  }
  return region_of_kf;
}

void PatchGrid::Build(const LiveMap& map) {
  patches_.clear();
  weak_areas_.clear();
  regions_.clear();
  region_frames_.clear();
  overall_ = 0;
  std::memset(overall_sub_, 0, sizeof(overall_sub_));
  if (map.points().empty()) return;

  const std::map<uint32_t, uint32_t> region_of_kf = ClusterRegions(map);

  has_lidar_ = false;
  for (const auto& kf : map.keyframes()) {
    if (kf.depth) {
      has_lidar_ = true;
      break;
    }
  }

  const double cell = options_.patch_size_m;
  auto key_of = [&](const Eigen::Vector3d& p) {
    return PatchKey{static_cast<int32_t>(std::floor(p.x() / cell)),
                    static_cast<int32_t>(std::floor(p.y() / cell)),
                    static_cast<int32_t>(std::floor(p.z() / cell))};
  };

  // Accumulate point evidence.
  struct Scatter {
    Eigen::Matrix3d outer = Eigen::Matrix3d::Zero();
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int n = 0;
  };
  std::unordered_map<PatchKey, Scatter, PatchKeyHash> scatter;

  for (const auto& [_, mp] : map.points()) {
    const PatchKey key = key_of(mp.X);
    Patch& patch = patches_[key];
    patch.key = key;
    patch.centroid += mp.X;
    ++patch.point_count;
    patch.err_sum += mp.last_reproj_err_px;
    patch.angle_max_sum += mp.max_tri_angle_deg;
    patch.gradient_sum += mp.mean_gradient;

    Scatter& s = scatter[key];
    s.outer += mp.X * mp.X.transpose();
    s.sum += mp.X;
    ++s.n;

    for (const auto& [kf_id, _f] : mp.observations) {
      if (std::find(patch.observing_kfs.begin(), patch.observing_kfs.end(),
                    kf_id) == patch.observing_kfs.end()) {
        patch.observing_kfs.push_back(kf_id);
      }
    }
  }

  // Per-region accumulators for the horizontal reference frame used to name
  // walls (where the user stood vs. where the surfaces are).
  struct RegionAccum {
    Eigen::Vector3d cam_sum = Eigen::Vector3d::Zero();
    int cam_count = 0;
    Eigen::Vector3d cen_sum = Eigen::Vector3d::Zero();
    int cen_count = 0;
  };
  std::map<uint32_t, RegionAccum> region_accum;

  for (auto& [key, patch] : patches_) {
    patch.centroid /= patch.point_count;

    // Normal from the point scatter (smallest eigenvector).
    const Scatter& s = scatter[key];
    if (s.n >= 3) {
      const Eigen::Vector3d mean = s.sum / s.n;
      const Eigen::Matrix3d cov = s.outer / s.n - mean * mean.transpose();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
      patch.normal = eig.eigenvectors().col(0).normalized();
    }

    // View geometry + LiDAR coverage from observing keyframes.
    std::vector<Eigen::Vector3d> centers;
    std::vector<Eigen::Vector3d> view_dirs;
    for (const uint32_t kf_id : patch.observing_kfs) {
      const Keyframe* kf = map.FindKeyframe(kf_id);
      if (kf == nullptr) continue;
      const Eigen::Vector3d center = kf->pose.CameraCenter();
      centers.push_back(center);
      const Eigen::Vector3d to_patch = patch.centroid - center;
      patch.distance_sum += to_patch.norm();
      view_dirs.push_back(-to_patch.normalized());

      // LiDAR: sample confidence around the patch projection.
      if (kf->depth) {
        const Eigen::Vector3d xc = kf->pose.Apply(patch.centroid);
        if (xc.z() > 0.05) {
          const auto& dK = kf->depth->K();
          const int u = static_cast<int>(
              std::lround(dK.fx * xc.x() / xc.z() + dK.cx));
          const int v = static_cast<int>(
              std::lround(dK.fy * xc.y() / xc.z() + dK.cy));
          int valid = 0;
          double conf = 0;
          for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
              if (!kf->depth->Valid(u + dx, v + dy)) continue;
              ++valid;
              conf += kf->depth->ConfidenceAt(u + dx, v + dy);
            }
          }
          patch.lidar_samples += valid;
          patch.lidar_conf_sum += conf;
        }
      }
    }
    for (size_t i = 0; i < centers.size(); ++i) {
      for (size_t j = i + 1; j < centers.size(); ++j) {
        patch.max_baseline =
            std::max(patch.max_baseline, (centers[i] - centers[j]).norm());
        const double cosang = std::clamp(view_dirs[i].dot(view_dirs[j]),
                                         -1.0, 1.0);
        patch.max_view_angle_deg = std::max(
            patch.max_view_angle_deg, RadToDeg(std::acos(cosang)));
      }
    }
    // Mean surface->camera direction: its tangential lean points back toward
    // the viewpoints already captured, so the empty side is the other way.
    Eigen::Vector3d view_sum = Eigen::Vector3d::Zero();
    for (const auto& vd : view_dirs) view_sum += vd;
    if (view_sum.norm() > 1e-9) patch.mean_view_dir = view_sum.normalized();

    // Orient the (sign-ambiguous) PCA normal to face the observing cameras,
    // so normals are consistent across a surface and sum cleanly per cluster.
    if (patch.mean_view_dir.squaredNorm() > 0 &&
        patch.normal.dot(patch.mean_view_dir) < 0) {
      patch.normal = -patch.normal;
    }

    // Region: majority vote over observing keyframes.
    std::map<uint32_t, int> votes;
    for (const uint32_t kf_id : patch.observing_kfs) {
      const auto it = region_of_kf.find(kf_id);
      if (it != region_of_kf.end()) ++votes[it->second];
    }
    if (!votes.empty()) {
      patch.region_id =
          std::max_element(votes.begin(), votes.end(),
                           [](const auto& a, const auto& b) {
                             return a.second < b.second;
                           })
              ->first;
    }

    RegionAccum& acc = region_accum[patch.region_id];
    for (const auto& c : centers) {
      acc.cam_sum += c;
      ++acc.cam_count;
    }
    acc.cen_sum += patch.centroid;
    ++acc.cen_count;

    ScorePatch(patch);
  }

  // Region frames: forward = camera-coverage centroid -> region centroid,
  // flattened onto the world floor plane (Y up).
  for (const auto& [id, acc] : region_accum) {
    if (acc.cam_count == 0 || acc.cen_count == 0) continue;
    RegionFrame frame;
    frame.cam_centroid = acc.cam_sum / acc.cam_count;
    frame.centroid = acc.cen_sum / acc.cen_count;
    Eigen::Vector3d fwd = frame.centroid - frame.cam_centroid;
    fwd.y() = 0;
    if (fwd.norm() > 0.2) {  // need a real depth axis to call back/left/right
      frame.forward = fwd.normalized();
      frame.valid = true;
    }
    region_frames_[id] = frame;
  }

  // Aggregates: overall and per region (uniform patch area).
  double score_sum = 0;
  double sub_sum[5] = {0, 0, 0, 0, 0};
  std::map<uint32_t, RegionAggregate> region_agg;
  std::map<uint32_t, double> region_sub_sums[5];
  for (const auto& [_, patch] : patches_) {
    score_sum += patch.score;
    for (int i = 0; i < 5; ++i) sub_sum[i] += patch.sub[i];
    RegionAggregate& agg = region_agg[patch.region_id];
    agg.id = patch.region_id;
    agg.score += patch.score;
    ++agg.patch_count;
    for (int i = 0; i < 5; ++i) region_sub_sums[i][patch.region_id] += patch.sub[i];
  }
  const double n = static_cast<double>(patches_.size());
  overall_ = static_cast<float>(score_sum / n);
  for (int i = 0; i < 5; ++i) {
    overall_sub_[i] = static_cast<float>(sub_sum[i] / n);
  }
  for (auto& [id, agg] : region_agg) {
    agg.score /= static_cast<float>(agg.patch_count);
    for (int i = 0; i < 5; ++i) {
      agg.sub[i] = static_cast<float>(region_sub_sums[i][id] / agg.patch_count);
    }
    agg.area_m2 = agg.patch_count * PatchArea();
    regions_.push_back(agg);
  }

  FindWeakAreas();
}

void PatchGrid::ScorePatch(Patch& patch) const {
  const double n = patch.point_count;
  const double area = PatchArea();
  const double err_mean = patch.err_sum / n;
  const double angle_mean = patch.angle_max_sum / n;
  const double grad_mean = patch.gradient_sum / n;
  const int kf_count = static_cast<int>(patch.observing_kfs.size());
  const double z_mean =
      kf_count > 0 ? patch.distance_sum / kf_count : 2.0;

  // 0 geometry
  patch.sub[0] = static_cast<float>(
      100.0 * Sat(n / (options_.density_target_per_m2 * area)) *
      Sat(angle_mean / 3.0) * std::exp(-(err_mean / 2.0) * (err_mean / 2.0)));

  // 1 pose support
  patch.sub[1] = static_cast<float>(
      100.0 * Sat(kf_count / 4.0) *
      std::exp(-(err_mean / 2.0) * (err_mean / 2.0)));

  // 2 texture: gradient energy and px-per-cm at CAPTURE resolution — splat
  // training consumes the stored full-res images.
  const double ppcm =
      options_.capture_focal_px * 0.01 / std::max(0.2, z_mean);
  patch.sub[2] =
      static_cast<float>(100.0 * Sat(grad_mean / 25.0) * Sat(ppcm / 8.0));

  // 3 lidar coverage. With no depth frames anywhere the axis is neutral
  // (scored as the mean of the others after they're computed below).
  const double conf_mean =
      patch.lidar_samples > 0 ? patch.lidar_conf_sum / patch.lidar_samples : 0;
  patch.sub[3] = static_cast<float>(
      100.0 * Sat(patch.lidar_samples / 50.0) * conf_mean);

  // 4 view diversity: continuous angular spread + baseline/distance ratio.
  patch.sub[4] = static_cast<float>(
      100.0 * Sat(patch.max_view_angle_deg / options_.view_angle_full_deg) *
      Sat(patch.max_baseline / (0.15 * std::max(0.2, z_mean))));

  if (!has_lidar_) {
    patch.sub[3] = 0.25f * (patch.sub[0] + patch.sub[1] + patch.sub[2] +
                            patch.sub[4]);
  }

  const double weighted = 0.25 * patch.sub[0] + 0.20 * patch.sub[1] +
                          0.20 * patch.sub[2] + 0.15 * patch.sub[3] +
                          0.20 * patch.sub[4];
  const float min_sub = *std::min_element(patch.sub, patch.sub + 5);
  // The cap keeps one catastrophic axis from hiding inside the mean.
  patch.score = static_cast<float>(std::min(weighted, min_sub + 35.0));
}

int PatchGrid::WallSide(const WeakArea& area) const {
  const auto it = region_frames_.find(area.region_id);
  if (it == region_frames_.end() || !it->second.valid) return 0;
  const RegionFrame& f = it->second;
  const Eigen::Vector3d up(0, 1, 0);
  Eigen::Vector3d s = area.centroid - f.cam_centroid;
  s.y() = 0;
  if (s.norm() < 1e-3) return 0;
  const Eigen::Vector3d right = up.cross(f.forward).normalized();  // user's right
  const double along = s.dot(f.forward);
  const double lateral = s.dot(right);
  if (std::abs(lateral) >= std::abs(along)) return lateral > 0 ? 3 : 2;
  return along >= 0 ? 1 : 4;  // back : front (near the region's entry)
}

void PatchGrid::FindWeakAreas() {
  // Cluster weak patches over 6-connectivity.
  std::set<PatchKey, bool (*)(const PatchKey&, const PatchKey&)> visited(
      [](const PatchKey& a, const PatchKey& b) {
        return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
      });

  for (const auto& [key, patch] : patches_) {
    if (patch.score >= options_.weak_threshold) continue;
    if (visited.count(key)) continue;

    // BFS cluster.
    std::vector<const Patch*> cluster;
    std::deque<PatchKey> frontier{key};
    visited.insert(key);
    while (!frontier.empty()) {
      const PatchKey current = frontier.front();
      frontier.pop_front();
      const auto it = patches_.find(current);
      if (it == patches_.end() || it->second.score >= options_.weak_threshold) {
        continue;
      }
      cluster.push_back(&it->second);
      const int32_t dx[6] = {1, -1, 0, 0, 0, 0};
      const int32_t dy[6] = {0, 0, 1, -1, 0, 0};
      const int32_t dz[6] = {0, 0, 0, 0, 1, -1};
      for (int d = 0; d < 6; ++d) {
        const PatchKey next{current.x + dx[d], current.y + dy[d],
                            current.z + dz[d]};
        if (!visited.count(next) && patches_.count(next)) {
          visited.insert(next);
          frontier.push_back(next);
        }
      }
    }
    if (cluster.empty()) continue;

    WeakArea area;
    area.patch_count = static_cast<int>(cluster.size());
    {
      std::map<uint32_t, int> region_votes;
      for (const Patch* p : cluster) ++region_votes[p->region_id];
      area.region_id =
          std::max_element(region_votes.begin(), region_votes.end(),
                           [](const auto& a, const auto& b) {
                             return a.second < b.second;
                           })
              ->first;
    }
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double sub_sum[5] = {0, 0, 0, 0, 0};
    double score_sum = 0;
    double z_sum = 0;
    Eigen::Vector3d view_sum = Eigen::Vector3d::Zero();
    for (const Patch* p : cluster) {
      centroid += p->centroid;
      normal += p->normal;
      score_sum += p->score;
      for (int i = 0; i < 5; ++i) sub_sum[i] += p->sub[i];
      const int kf_count = std::max<size_t>(1, p->observing_kfs.size());
      z_sum += p->distance_sum / kf_count;
      view_sum += p->mean_view_dir;  // mean surface->camera direction
    }
    const double n = static_cast<double>(cluster.size());
    area.centroid = centroid / n;
    area.score = static_cast<float>(score_sum / n);
    area.radius_m =
        options_.patch_size_m * std::sqrt(n) * 0.75;
    const double z_mean = std::max(0.4, z_sum / n);

    int argmin = 0;
    for (int i = 1; i < 5; ++i) {
      if (!has_lidar_ && i == 3) continue;  // absent sensor never diagnosed
      if (sub_sum[i] < sub_sum[argmin]) argmin = i;
    }
    area.deficiency = argmin;

    // Surface kind from the (sign-agnostic, world Y-up) normal.
    Eigen::Vector3d avg_normal = normal.normalized();
    const double up_dot = std::abs(avg_normal.y());
    if (up_dot > 0.75) {
      // Horizontal surface: below eye level = floor, above = ceiling.
      area.surface_kind = area.centroid.y() < 1.1 ? 1 : 2;
    } else if (up_dot < 0.35) {
      area.surface_kind = 0;  // wall
    } else {
      area.surface_kind = 3;  // slanted: object/furniture
    }

    // Suggested move: sideways relative to the surface (perpendicular to the
    // normal, horizontal). The sign points toward the UNDER-observed side —
    // the cluster's mean view direction leans back toward where the cameras
    // already are, so step the other way to add the missing parallax.
    const Eigen::Vector3d up(0, 1, 0);
    Eigen::Vector3d sideways = avg_normal.cross(up);
    if (sideways.norm() < 1e-6) sideways = Eigen::Vector3d::UnitX();
    sideways.normalize();
    if (view_sum.norm() > 1e-9) {
      const double bias = view_sum.normalized().dot(sideways);
      if (bias > 0) sideways = -sideways;  // cameras lean +sideways -> gap is -
    }
    area.move_dir = sideways;
    area.move_dist_m =
        std::clamp(2.0 * z_mean * std::tan(DegToRad(5.0)), 0.3, 1.0);

    // Name the wall relative to where the user has stood in this region.
    area.surface_side = area.surface_kind == 0 ? WallSide(area) : 0;

    area.priority = (100.0 - area.score) * n /
                    (1.0 + area.centroid.norm() / 3.0);
    weak_areas_.push_back(area);
  }

  std::sort(weak_areas_.begin(), weak_areas_.end(),
            [](const WeakArea& a, const WeakArea& b) {
              return a.priority > b.priority;
            });
  if (static_cast<int>(weak_areas_.size()) > options_.max_weak_areas) {
    weak_areas_.resize(options_.max_weak_areas);
  }
}

void PatchGrid::FillSnapshot(std::vector<bs_snap_patch>& patches,
                             std::vector<bs_snap_region>& regions,
                             std::vector<bs_snap_weak_area>& weak,
                             const std::map<uint32_t, std::string>& names) const {
  patches.reserve(patches_.size());
  for (const auto& [_, p] : patches_) {
    bs_snap_patch sp{};
    sp.cx = static_cast<float>(p.centroid.x());
    sp.cy = static_cast<float>(p.centroid.y());
    sp.cz = static_cast<float>(p.centroid.z());
    sp.nx = static_cast<float>(p.normal.x());
    sp.ny = static_cast<float>(p.normal.y());
    sp.nz = static_cast<float>(p.normal.z());
    sp.extent = static_cast<float>(options_.patch_size_m);
    sp.score = p.score;
    for (int i = 0; i < 5; ++i) sp.sub[i] = p.sub[i];
    sp.region_id = p.region_id;
    patches.push_back(sp);
  }

  regions.reserve(regions_.size());
  for (const auto& agg : regions_) {
    bs_snap_region region{};
    region.region_id = agg.id;
    const auto it = names.find(agg.id);
    if (it != names.end()) {
      std::snprintf(region.name, sizeof(region.name), "%s", it->second.c_str());
    } else {
      std::snprintf(region.name, sizeof(region.name), "Room %u", agg.id);
    }
    region.score = agg.score;
    for (int i = 0; i < 5; ++i) region.sub[i] = agg.sub[i];
    region.area_m2 = static_cast<float>(agg.area_m2);
    region.patch_count = agg.patch_count;
    regions.push_back(region);
  }

  weak.reserve(weak_areas_.size());
  for (const auto& area : weak_areas_) {
    bs_snap_weak_area w{};
    w.cx = static_cast<float>(area.centroid.x());
    w.cy = static_cast<float>(area.centroid.y());
    w.cz = static_cast<float>(area.centroid.z());
    w.radius_m = static_cast<float>(area.radius_m);
    w.region_id = area.region_id;
    w.deficiency = area.deficiency;
    w.surface_kind = area.surface_kind;
    w.surface_side = area.surface_side;
    w.move_dir[0] = static_cast<float>(area.move_dir.x());
    w.move_dir[1] = static_cast<float>(area.move_dir.y());
    w.move_dir[2] = static_cast<float>(area.move_dir.z());
    w.move_dist_m = static_cast<float>(area.move_dist_m);
    w.score = area.score;
    weak.push_back(w);
  }
}

}  // namespace bs
