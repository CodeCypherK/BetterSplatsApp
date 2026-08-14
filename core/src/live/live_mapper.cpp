// Mapping half of LiveSystem: keyframe insertion, new-point triangulation,
// local bundle adjustment with LiDAR regularization, and point culling.

#include <algorithm>
#include <cmath>
#include <set>

#include "common/log.h"
#include "fusion/scale.h"
#include "geometry/triangulation.h"
#include "live/live_system.h"
#include "vision/matching.h"

namespace bs {

bool LiveSystem::ShouldInsertKeyframe(const LiveFrameInput& input,
                                      const FrameFeatures& features,
                                      int tracked_inliers) const {
  const Keyframe* last_kf = map_.FindKeyframe(last_kf_id_);
  if (last_kf == nullptr) return false;

  // Rate limit. Deliberately a flat floor: making it tighter while turning
  // (so a turn gets more keyframe pairs to triangulate its leading edge)
  // was measured and is WORSE — see docs/ARCHITECTURE.md.
  const double since_last = input.t_capture - last_kf_time_;
  if (since_last < config_.kf_min_interval_s) return false;

  // Quality gates: a blurred or blown-out frame never becomes a keyframe.
  // The blur threshold is relative to recent sharpness (content-adaptive).
  if (features.lap_var < BlurThreshold(0.5)) return false;
  if (features.overexp_frac > config_.kf_max_overexposed_frac) return false;

  // Geometric need: enough motion since the last keyframe...
  const double translation =
      (last_pose_.CameraCenter() - last_kf->pose.CameraCenter()).norm();
  const double rotation_deg =
      RadToDeg(AngularDistance(last_pose_.q, last_kf->pose.q));

  // Median scene depth from currently tracked points (fallback 2 m).
  double median_depth = 2.0;
  {
    std::vector<double> depths;
    for (const int32_t pid : last_kf->point_ids) {
      if (pid < 0) continue;
      if (const auto it = map_.points().find(pid); it != map_.points().end()) {
        const double z = last_pose_.Apply(it->second.X).z();
        if (z > 0.05) depths.push_back(z);
      }
    }
    if (depths.size() > 10) {
      std::nth_element(depths.begin(), depths.begin() + depths.size() / 2,
                       depths.end());
      median_depth = depths[depths.size() / 2];
    }
  }
  double needed_translation =
      std::max(static_cast<double>(config_.kf_min_translation_m),
               config_.kf_translation_depth_frac * median_depth);
  double needed_rotation = config_.kf_min_rotation_deg;
  if (pass_ == BS_PASS_SCOUT) {
    // A scaffold's whole value is how reliably a later pass can relocalize
    // into it, and that is bounded by how far apart its keyframes are: ORB
    // stops matching well across a large viewpoint change. The scout pass is
    // cheap in every other respect (its frames are never reconstructed), so
    // spend that budget on keyframe density.
    needed_translation *= config_.scout_kf_translation_scale;
    needed_rotation *= config_.scout_kf_rotation_scale;
  }

  const bool moved =
      translation > needed_translation || rotation_deg > needed_rotation;

  // ...or tracked support decaying versus the reference keyframe.
  const double overlap =
      static_cast<double>(tracked_inliers) /
      std::max(1, last_kf->AssociatedCount());
  const bool losing_overlap = overlap < config_.kf_max_overlap ||
                              tracked_inliers < config_.kf_min_tracked_inliers;

  // Forced keyframe when moving continuously without one.
  const bool forced =
      since_last > config_.kf_force_interval_s && translation > 0.02;

  return moved || losing_overlap || forced;
}

void LiveSystem::InsertKeyframe(
    const LiveFrameInput& input, FrameFeatures features,
    const std::vector<std::pair<int32_t, int>>& matches) {
  Keyframe kf;
  kf.frame_id = input.frame_id;
  kf.t_capture = input.t_capture;
  kf.pose = last_pose_;
  kf.K = input.K;
  kf.point_ids.assign(features.features.keypoints.size(), -1);
  kf.depth = MakeDepthFrame(input);
  if (kf.depth) kf.depth_lookup = std::make_shared<DepthLookup>(*kf.depth);
  kf.features = std::move(features.features);
  kf.undistorted = std::move(features.undistorted);

  Keyframe& added = map_.AddKeyframe(std::move(kf));

  // Attach tracked associations and refresh point descriptors/stats.
  for (const auto& [pid, feat_idx] : matches) {
    MapPoint* mp = map_.FindPoint(pid);
    if (mp == nullptr) continue;
    if (feat_idx < 0 ||
        feat_idx >= static_cast<int>(added.point_ids.size())) {
      continue;
    }
    added.point_ids[feat_idx] = pid;
    mp->observations.emplace_back(added.kf_id, feat_idx);
    mp->descriptor = added.features.descriptors.row(feat_idx).clone();

    // Track the strongest baseline this point has seen.
    for (const auto& [other_kf_id, _] : mp->observations) {
      if (other_kf_id == added.kf_id) continue;
      if (const Keyframe* other = map_.FindKeyframe(other_kf_id)) {
        const double angle = TriangulationAngleDeg(
            added.pose.CameraCenter(), other->pose.CameraCenter(), mp->X);
        mp->max_tri_angle_deg =
            std::max(mp->max_tri_angle_deg, static_cast<float>(angle));
      }
    }
  }

  last_kf_id_ = added.kf_id;
  BS_LOGD("live", "keyframe %u at frame %u (%zu kfs, %zu points)", added.kf_id,
          added.frame_id, map_.keyframes().size(), map_.points().size());
  EmitDirective(added.frame_id, BS_STORE_KEYFRAME, true);

  TriangulateNewPoints(added);
  TryLoopLink(added);
  LocalBundleAdjustment(added.kf_id);
  CullPoints();

  ReadinessOptions readiness_options;
  readiness_options.patch_size_m = config_.patch_size_m;
  readiness_options.weak_threshold = config_.readiness_weak_threshold;
  readiness_ = PatchGrid(readiness_options);
  readiness_.Build(map_);

  // LIVE-layer caps: the map is disposable, so bounded memory wins over
  // completeness (the final solve re-derives everything from RAW).
  if (static_cast<int>(map_.points().size()) > config_.live_max_points) {
    std::vector<std::pair<double, int32_t>> ranked;
    ranked.reserve(map_.points().size());
    for (const auto& [id, mp] : map_.points()) {
      const double ratio = mp.visible_count > 3
                               ? static_cast<double>(mp.found_count) /
                                     mp.visible_count
                               : 1.0;
      ranked.emplace_back(ratio, id);
    }
    std::sort(ranked.begin(), ranked.end());
    const int excess =
        static_cast<int>(map_.points().size()) - config_.live_max_points;
    for (int i = 0; i < excess; ++i) map_.RemovePoint(ranked[i].second);
  }
}

void LiveSystem::TriangulateNewPoints(Keyframe& kf) {
  // Pair the new keyframe with its best covisible neighbors (fallback: the
  // previous keyframe). Ranking by covisibility alone favours the most
  // recent, and therefore shortest-baseline, keyframes — but requiring a
  // minimum baseline instead was measured and is WORSE (see
  // docs/ARCHITECTURE.md): in the live map overlap beats parallax, because
  // a distant partner matches fewer features and costs track extensions.
  std::vector<uint32_t> neighbors;
  for (const auto& [kf_id, _] : map_.Covisible(kf.kf_id, 10)) {
    neighbors.push_back(kf_id);
    if (neighbors.size() >= 3) break;
  }
  if (neighbors.empty() && map_.keyframes().size() >= 2) {
    neighbors.push_back(map_.keyframes()[map_.keyframes().size() - 2].kf_id);
  }

  int created = 0;
  int extended = 0;
  int candidates = 0, matched = 0, rej_tri = 0, rej_angle = 0, rej_lidar = 0;
  for (const uint32_t neighbor_id : neighbors) {
    Keyframe* other = map_.FindKeyframe(neighbor_id);
    if (other == nullptr) continue;

    // Match this keyframe's unassociated features against ALL of the
    // neighbor's features: hits on already-associated neighbor features
    // extend existing tracks into this view (vital for continuity as the
    // camera sweeps); hits on unassociated ones triangulate new points.
    FeatureSet unmatched_kf;
    unmatched_kf.type = FeatureType::kOrb;
    std::vector<int> map_kf;
    for (size_t i = 0; i < kf.point_ids.size(); ++i) {
      if (kf.point_ids[i] < 0) map_kf.push_back(static_cast<int>(i));
    }
    if (map_kf.size() < 8) continue;
    candidates += static_cast<int>(map_kf.size());

    unmatched_kf.descriptors.create(static_cast<int>(map_kf.size()), 32, CV_8U);
    for (size_t i = 0; i < map_kf.size(); ++i) {
      kf.features.descriptors.row(map_kf[i])
          .copyTo(unmatched_kf.descriptors.row(static_cast<int>(i)));
      unmatched_kf.keypoints.push_back(kf.features.keypoints[map_kf[i]]);
    }

    MatchOptions options;
    options.ratio = 0.8f;
    options.cross_check = true;
    options.max_distance = 64.0f;
    const std::vector<Match> matches =
        MatchFeatures(unmatched_kf, other->features, options);
    matched += static_cast<int>(matches.size());

    for (const auto& m : matches) {
      const int fi = map_kf[m.idx_a];
      const int fo = m.idx_b;
      if (kf.point_ids[fi] >= 0) continue;

      // Track extension: the neighbor feature already has a map point.
      if (other->point_ids[fo] >= 0) {
        MapPoint* mp = map_.FindPoint(other->point_ids[fo]);
        if (mp == nullptr) continue;
        const Eigen::Vector3d xc = kf.pose.Apply(mp->X);
        if (xc.z() <= 0.05) continue;
        const Eigen::Vector2d proj = kf.K.Project(xc);
        const cv::Point2f& obs = kf.undistorted[fi];
        const double err = std::hypot(proj.x() - obs.x, proj.y() - obs.y);
        if (err > 3.0) continue;
        kf.point_ids[fi] = mp->id;
        mp->observations.emplace_back(kf.kf_id, fi);
        mp->descriptor = kf.features.descriptors.row(fi).clone();
        ++extended;
        continue;
      }

      const auto tri = TriangulateTwoView(kf.pose, other->pose, kf.K,
                                          kf.undistorted[fi],
                                          other->undistorted[fo]);
      if (!tri || !tri->InFrontOfAll()) {
        ++rej_tri;
        continue;
      }
      if (tri->max_angle_deg < 1.0 || tri->max_reproj_error_px > 2.0) {
        ++rej_angle;
        continue;
      }

      // LiDAR plausibility: reject candidates far in FRONT of a confident
      // surface (floaters). Points behind the measured surface may be real
      // geometry the coarse depth missed (doorways, gaps) — keep those.
      bool implausible = false;
      for (const Keyframe* view : {&kf, other}) {
        if (!view->depth) continue;
        const Eigen::Vector3d xc = view->pose.Apply(tri->point);
        if (xc.z() < 0.05) continue;
        const Eigen::Vector2d xn(xc.x() / xc.z(), xc.y() / xc.z());
        const auto& dK = view->depth->K();
        const double u = dK.fx * xn.x() + dK.cx;
        const double v = dK.fy * xn.y() + dK.cy;
        const int xi = static_cast<int>(std::lround(u));
        const int yi = static_cast<int>(std::lround(v));
        if (view->depth->ConfidenceAt(xi, yi) < 0.6) continue;
        const auto d = view->depth->DepthBilinear(u, v);
        if (!d) continue;
        const double sigma = view->depth->Sigma(*d);
        if (*d - xc.z() > std::max(6.0 * sigma, 0.15)) {
          implausible = true;
          break;
        }
      }
      if (implausible) {
        ++rej_lidar;
        continue;
      }

      MapPoint point;
      point.X = tri->point;
      point.descriptor = kf.features.descriptors.row(fi).clone();
      point.observations = {{kf.kf_id, fi}, {other->kf_id, fo}};
      point.max_tri_angle_deg = static_cast<float>(tri->max_angle_deg);
      point.last_reproj_err_px = static_cast<float>(tri->mean_reproj_error_px);
      point.mean_gradient = 20.0f;  // refreshed by BA-time association
      MapPoint& addedp = map_.AddPoint(std::move(point));
      kf.point_ids[fi] = addedp.id;
      other->point_ids[fo] = addedp.id;
      ++created;
    }
  }
  BS_LOGD("live",
          "kf %u: %d new points, %d track extensions (%zu total); cand %d matched %d rejected tri=%d angle=%d lidar=%d",
          kf.kf_id, created, extended, map_.points().size(), candidates,
          matched, rej_tri, rej_angle, rej_lidar);
}

void LiveSystem::TryLoopLink(Keyframe& kf) {
  // Candidates: keyframes spatially near the new one, excluded from the
  // recent window, with no covisibility (i.e., the map doesn't yet know
  // they see the same place).
  const auto covisible = map_.Covisible(kf.kf_id, 1);
  auto is_covisible = [&](uint32_t id) {
    for (const auto& [cid, _] : covisible) {
      if (cid == id) return true;
    }
    return false;
  };

  const auto& kfs = map_.keyframes();
  const int recent_cutoff =
      static_cast<int>(kfs.size()) - config_.loop_exclude_recent;
  const Eigen::Vector3d center = kf.pose.CameraCenter();

  for (int i = 0; i < recent_cutoff; ++i) {
    const Keyframe& candidate = kfs[i];
    if (is_covisible(candidate.kf_id)) continue;
    if ((candidate.pose.CameraCenter() - center).norm() >
        config_.loop_search_radius_m) {
      continue;
    }

    // Appearance check against the candidate's associated features.
    FeatureSet assoc;
    assoc.type = FeatureType::kOrb;
    std::vector<int32_t> pids;
    std::vector<int> rows;
    for (size_t f = 0; f < candidate.point_ids.size(); ++f) {
      if (candidate.point_ids[f] >= 0) {
        pids.push_back(candidate.point_ids[f]);
        rows.push_back(static_cast<int>(f));
      }
    }
    if (static_cast<int>(pids.size()) < config_.loop_min_inliers) continue;
    assoc.descriptors.create(static_cast<int>(rows.size()), 32, CV_8U);
    for (size_t r = 0; r < rows.size(); ++r) {
      candidate.features.descriptors.row(rows[r])
          .copyTo(assoc.descriptors.row(static_cast<int>(r)));
    }
    assoc.keypoints.resize(rows.size());

    MatchOptions options;
    options.ratio = 0.75f;
    options.cross_check = true;
    options.max_distance = 55.0f;
    const std::vector<Match> matches =
        MatchFeatures(assoc, kf.features, options);
    if (static_cast<int>(matches.size()) < config_.loop_min_inliers) continue;

    // Geometric verification: the old points must reproject consistently
    // into the new view under its current pose (drift-widened gate).
    int linked = 0;
    for (const auto& m : matches) {
      MapPoint* mp = map_.FindPoint(pids[m.idx_a]);
      if (mp == nullptr) continue;
      if (kf.point_ids[m.idx_b] >= 0) continue;
      const Eigen::Vector3d xc = kf.pose.Apply(mp->X);
      if (xc.z() <= 0.05) continue;
      const Eigen::Vector2d proj = kf.K.Project(xc);
      const cv::Point2f& obs = kf.undistorted[m.idx_b];
      if (std::hypot(proj.x() - obs.x, proj.y() - obs.y) > 12.0) continue;
      kf.point_ids[m.idx_b] = mp->id;
      mp->observations.emplace_back(kf.kf_id, m.idx_b);
      ++linked;
    }
    if (linked >= config_.loop_min_inliers / 2) {
      loop_links_ += linked;
      BS_LOGI("live", "loop link: kf %u <-> kf %u (%d points joined)",
              kf.kf_id, candidate.kf_id, linked);
      // The follow-up local BA (window now spans the loop) absorbs drift.
    }
  }
}

void LiveSystem::LocalBundleAdjustment(uint32_t center_kf_id) {
  // Window: center + strongest covisible keyframes.
  std::vector<uint32_t> window{center_kf_id};
  for (const auto& [kf_id, _] :
       map_.Covisible(center_kf_id, config_.lba_min_shared_points)) {
    // Scaffold keyframes from a previous pass are the session's reference
    // frame — this pass localizes INTO them, so they never enter the
    // optimization window. Where they observe window points they still join
    // the problem below as fixed observers, which is what pins this pass to
    // the gauge every other pass shares.
    const Keyframe* candidate = map_.FindKeyframe(kf_id);
    if (candidate != nullptr && candidate->from_scaffold) continue;
    window.push_back(kf_id);
    if (static_cast<int>(window.size()) >= config_.lba_window) break;
  }
  std::set<uint32_t> window_set(window.begin(), window.end());

  // Points observed by the window.
  std::set<int32_t> point_ids;
  for (const uint32_t kf_id : window) {
    const Keyframe* kf = map_.FindKeyframe(kf_id);
    if (kf == nullptr) continue;
    for (const int32_t pid : kf->point_ids) {
      if (pid >= 0) point_ids.insert(pid);
    }
  }
  if (point_ids.empty()) return;

  // Fixed keyframes: outside-window observers (anchors the gauge).
  std::set<uint32_t> fixed;
  for (const int32_t pid : point_ids) {
    const auto it = map_.points().find(pid);
    if (it == map_.points().end()) continue;
    for (const auto& [kf_id, _] : it->second.observations) {
      if (window_set.count(kf_id) == 0) fixed.insert(kf_id);
    }
  }
  // The first keyframe is always fixed: it defines the world frame.
  const uint32_t first_kf_id = map_.keyframes().front().kf_id;
  if (window_set.count(first_kf_id)) {
    window_set.erase(first_kf_id);
    window.erase(std::remove(window.begin(), window.end(), first_kf_id),
                 window.end());
    fixed.insert(first_kf_id);
  }
  // With nothing else anchored, also fix the second keyframe's translation
  // scale implicitly by fixing its full pose (two-KF bootstrap case, where
  // LiDAR residuals are the only other scale anchor).
  if (fixed.size() == 1 && window.size() == 1 && !scale_locked_) {
    // leave free: LiDAR terms + first KF fixed is enough in practice
  }

  ceres::Problem problem;
  std::map<uint32_t, std::array<double, 7>> pose_blocks;  // q(4) + t(3)
  auto ensure_pose = [&](uint32_t kf_id) -> double* {
    auto it = pose_blocks.find(kf_id);
    if (it == pose_blocks.end()) {
      const Keyframe* kf = map_.FindKeyframe(kf_id);
      std::array<double, 7> block{};
      PoseToBlocks(kf->pose, block.data(), block.data() + 4);
      it = pose_blocks.emplace(kf_id, block).first;
    }
    return it->second.data();
  };

  std::map<int32_t, std::array<double, 3>> point_blocks;
  for (const int32_t pid : point_ids) {
    const auto it = map_.points().find(pid);
    if (it == map_.points().end()) continue;
    point_blocks[pid] = {it->second.X.x(), it->second.X.y(),
                         it->second.X.z()};
  }

  int reproj_count = 0;
  int lidar_count = 0;
  for (auto& [pid, xyz] : point_blocks) {
    const MapPoint& mp = map_.points().at(pid);
    for (const auto& [kf_id, feat_idx] : mp.observations) {
      const Keyframe* kf = map_.FindKeyframe(kf_id);
      if (kf == nullptr) continue;
      if (window_set.count(kf_id) == 0 && fixed.count(kf_id) == 0) continue;
      double* pose = ensure_pose(kf_id);
      const cv::Point2f& obs = kf->undistorted[feat_idx];
      problem.AddResidualBlock(
          ReprojectionResidual::Create(obs.x, obs.y, kf->K),
          new ceres::HuberLoss(config_.lba_huber_px), pose, pose + 4,
          xyz.data());
      ++reproj_count;
    }

    // LiDAR regularization from window keyframes that observe the point.
    const Eigen::Vector3d X(xyz[0], xyz[1], xyz[2]);
    const double w_tex = mp.TextureWeightNow(config_.lidar_tex_floor);
    for (const uint32_t kf_id : window) {
      const Keyframe* kf = map_.FindKeyframe(kf_id);
      if (kf == nullptr || !kf->depth || !kf->depth_lookup) continue;
      const double gate = scale_locked_ ? config_.lidar_gate_sigmas
                                        : 4.0 * config_.lidar_gate_sigmas;
      const auto assoc =
          MakeLidarAssociation(*kf->depth, kf->pose, X, w_tex, gate);
      if (!assoc) continue;
      double* pose = ensure_pose(kf_id);
      problem.AddResidualBlock(
          LidarDepthResidual::Create(kf->depth_lookup.get(), *assoc),
          new ceres::CauchyLoss(1.0), pose, pose + 4, xyz.data());
      ++lidar_count;
    }
  }
  if (reproj_count < 10) return;

  for (auto& [kf_id, block] : pose_blocks) {
    problem.SetManifold(block.data(), new ceres::EigenQuaternionManifold);
    const Keyframe* kf = map_.FindKeyframe(kf_id);
    if (fixed.count(kf_id) || (kf != nullptr && kf->from_scaffold)) {
      problem.SetParameterBlockConstant(block.data());
      problem.SetParameterBlockConstant(block.data() + 4);
    }
  }
  // Scaffold points are part of the same reference: new observations of them
  // constrain this pass's poses, but must not move the map underneath.
  for (auto& [pid, xyz] : point_blocks) {
    const auto it = map_.points().find(pid);
    if (it != map_.points().end() && it->second.from_scaffold &&
        problem.HasParameterBlock(xyz.data())) {
      problem.SetParameterBlockConstant(xyz.data());
    }
  }

  ceres::Solver::Options options;
  options.max_num_iterations = config_.lba_max_iterations;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Validate before adopting. A solver result is evidence, not truth: an
  // ill-conditioned window can come back with a higher cost or a NaN, and
  // writing that into the map is unrecoverable — the very next frame
  // projects its local points somewhere they are not, so tracking cannot
  // re-acquire. A local refinement should also never teleport a keyframe;
  // if it wants to, the window was not describing the same place.
  double worst_shift = 0.0;
  for (const auto& [kf_id, block] : pose_blocks) {
    if (fixed.count(kf_id)) continue;
    const Keyframe* kf = map_.FindKeyframe(kf_id);
    if (kf == nullptr) continue;
    const SE3 refined = PoseFromBlocks(block.data(), block.data() + 4);
    worst_shift = std::max(
        worst_shift,
        (refined.CameraCenter() - kf->pose.CameraCenter()).norm());
  }
  const bool improved = std::isfinite(summary.final_cost) &&
                        summary.final_cost <= summary.initial_cost;
  if (!improved || worst_shift > config_.lba_max_pose_shift_m) {
    BS_LOGD("live",
            "LBA kf %u rejected: cost %.3g -> %.3g, worst pose shift %.2f m",
            center_kf_id, summary.initial_cost, summary.final_cost,
            worst_shift);
    return;
  }

  // Write back.
  for (const auto& [kf_id, block] : pose_blocks) {
    if (fixed.count(kf_id)) continue;
    if (Keyframe* kf = map_.FindKeyframe(kf_id)) {
      kf->pose = PoseFromBlocks(block.data(), block.data() + 4);
      if (kf_id == last_kf_id_) last_pose_ = kf->pose;
    }
  }
  for (const auto& [pid, xyz] : point_blocks) {
    if (MapPoint* mp = map_.FindPoint(pid)) {
      const Eigen::Vector3d refined(xyz[0], xyz[1], xyz[2]);
      if (!refined.allFinite()) continue;
      mp->X = refined;
    }
  }
  BS_LOGD("live", "LBA kf %u: %d reproj + %d lidar residuals, %d iters",
          center_kf_id, reproj_count, lidar_count,
          static_cast<int>(summary.iterations.size()));
}

void LiveSystem::CullPoints() {
  std::vector<int32_t> to_remove;
  for (const auto& [id, mp] : map_.points()) {
    // Persistently invisible-when-predicted points are tracking noise.
    if (mp.visible_count >= 5) {
      const double ratio =
          static_cast<double>(mp.found_count) / mp.visible_count;
      if (ratio < 0.2) {
        to_remove.push_back(id);
        continue;
      }
    }
    // Points that never gained a second-keyframe observation within two
    // more keyframes were mismatches.
    if (mp.observations.size() < 2) to_remove.push_back(id);
  }
  for (const int32_t id : to_remove) map_.RemovePoint(id);
}

}  // namespace bs
