#include "live/live_system.h"

#include <algorithm>
#include <filesystem>
#include <fstream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "calib/lut_fit.h"
#include "common/log.h"
#include "fusion/scale.h"
#include "geometry/triangulation.h"
#include "geometry/two_view.h"
#include "io/float16.h"
#include "io/session_schema.h"
#include "vision/matching.h"

namespace bs {

namespace fs = std::filesystem;

namespace {

cv::Mat IntrinsicsMatrix(const Intrinsics& K) {
  return (cv::Mat_<double>(3, 3) << K.fx, 0, K.cx, 0, K.fy, K.cy, 0, 0, 1);
}

SE3 PoseFromRvecTvec(const cv::Mat& rvec, const cv::Mat& tvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d rot;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) rot(r, c) = R.at<double>(r, c);
  }
  SE3 pose;
  pose.q = Eigen::Quaterniond(rot).normalized();
  pose.t = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1),
                           tvec.at<double>(2));
  return pose;
}

void PoseToRvecTvec(const SE3& pose, cv::Mat& rvec, cv::Mat& tvec) {
  const Eigen::Matrix3d rot = pose.q.toRotationMatrix();
  cv::Mat R(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) R.at<double>(r, c) = rot(r, c);
  }
  cv::Rodrigues(R, rvec);
  tvec = (cv::Mat_<double>(3, 1) << pose.t.x(), pose.t.y(), pose.t.z());
}

}  // namespace

LiveSystem::LiveSystem(const EngineConfig& config) : config_(config) {}

LiveSystem::~LiveSystem() = default;

void LiveSystem::Begin(const std::string& session_dir, double k1, double k2) {
  session_dir_ = session_dir;
  k1_ = k1;
  k2_ = k2;
  state_ = BS_LIVE_INITIALIZING;
  guidance_ = BS_GUIDE_NONE;
  BS_LOGI("live", "begin (k1=%.4f k2=%.4f)", k1, k2);
}

std::shared_ptr<DepthFrame> LiveSystem::MakeDepthFrame(
    const LiveFrameInput& input) const {
  if (input.depth.f16.empty()) return nullptr;
  LidarConfidenceOptions options;
  options.sigma_base_m = config_.lidar_sigma_base_m;
  options.sigma_quadratic = config_.lidar_sigma_quadratic;
  options.range_min_m = config_.lidar_range_min_m;
  options.range_full_m = config_.lidar_range_full_m;
  options.range_zero_m = config_.lidar_range_zero_m;
  options.max_incidence_deg = config_.lidar_max_incidence_deg;
  return std::make_shared<DepthFrame>(input.depth, input.Kd, options);
}

LiveSystem::FrameFeatures LiveSystem::ExtractFeatures(
    const LiveFrameInput& input) const {
  FrameFeatures out;
  OrbOptions orb;
  orb.max_features = config_.live_orb_features;
  orb.levels = config_.live_orb_levels;
  orb.scale_factor = config_.live_orb_scale;
  orb.fast_threshold = config_.live_fast_threshold;
  orb.fast_threshold_min = config_.live_fast_threshold_min;
  out.features = DetectOrb(input.gray, orb);

  out.undistorted.reserve(out.features.keypoints.size());
  const PinholeIntrinsics pin{input.K.fx, input.K.fy, input.K.cx, input.K.cy,
                              input.K.width, input.K.height};
  for (const auto& kp : out.features.keypoints) {
    if (k1_ == 0.0 && k2_ == 0.0) {
      out.undistorted.push_back(kp.pt);
    } else {
      const Eigen::Vector2d u =
          UndistortPixel({kp.pt.x, kp.pt.y}, pin, k1_, k2_);
      out.undistorted.emplace_back(static_cast<float>(u.x()),
                                   static_cast<float>(u.y()));
    }
  }

  cv::Mat lap;
  cv::Laplacian(input.gray, lap, CV_64F);
  cv::Scalar mean, stddev;
  cv::meanStdDev(lap, mean, stddev);
  out.lap_var = stddev[0] * stddev[0];
  out.overexp_frac =
      static_cast<double>(cv::countNonZero(input.gray >= 250)) /
      (static_cast<double>(input.gray.rows) * input.gray.cols);
  return out;
}

double LiveSystem::MeanGradientAt(const cv::Mat& gray,
                                  const cv::Point2f& pt) const {
  const int x = std::clamp(static_cast<int>(pt.x), 2, gray.cols - 3);
  const int y = std::clamp(static_cast<int>(pt.y), 2, gray.rows - 3);
  double sum = 0;
  int n = 0;
  for (int dy = -2; dy <= 2; ++dy) {
    for (int dx = -2; dx <= 2; ++dx) {
      const int gx = std::abs(gray.at<uint8_t>(y + dy, x + dx + 1) -
                              gray.at<uint8_t>(y + dy, x + dx - 1));
      const int gy = std::abs(gray.at<uint8_t>(y + dy + 1, x + dx) -
                              gray.at<uint8_t>(y + dy - 1, x + dx));
      sum += 0.5 * (gx + gy);
      ++n;
    }
  }
  return sum / n;
}

double LiveSystem::BlurThreshold(double fraction) const {
  if (recent_lap_vars_.size() < 10) return 10.0;
  std::vector<double> sorted(recent_lap_vars_.begin(), recent_lap_vars_.end());
  const size_t p80 = sorted.size() * 4 / 5;
  std::nth_element(sorted.begin(), sorted.begin() + p80, sorted.end());
  // Relative to recent sharpness, floored absolutely, and never demanding
  // more than the nominal "good" value from config.
  return std::max(10.0, std::min(static_cast<double>(config_.kf_min_blur_lapvar),
                                 fraction * sorted[p80]));
}

bs_live_state LiveSystem::Feed(const LiveFrameInput& input) {
  ++frames_processed_;
  last_frame_id_ = input.frame_id;
  last_gyro_mag_ = input.gyro_valid ? input.gyro.norm() : 0.0;

  FrameFeatures features = ExtractFeatures(input);
  last_lap_var_ = features.lap_var;
  recent_lap_vars_.push_back(features.lap_var);
  while (recent_lap_vars_.size() > 90) recent_lap_vars_.pop_front();

  switch (state_) {
    case BS_LIVE_INITIALIZING:
      TryBootstrap(input, std::move(features));
      LogPose(input, state_ == BS_LIVE_TRACKING);
      break;
    case BS_LIVE_TRACKING:
    case BS_LIVE_LOST: {
      const bool tracked = TrackFrame(input, features);
      LogPose(input, tracked);
      break;
    }
    default:
      break;
  }
  return state_;
}

// ------------------------------------------------------------- bootstrap

void LiveSystem::ResetBootstrap() {
  ref_input_.reset();
  ref_features_.reset();
  bootstrap_attempts_ = 0;
}

void LiveSystem::TryBootstrap(const LiveFrameInput& input,
                              FrameFeatures features) {
  if (features.lap_var < BlurThreshold(0.35)) {
    guidance_ = BS_GUIDE_SLOW_DOWN;
    return;
  }

  if (!ref_input_.has_value()) {
    if (features.features.size() >= config_.boot_min_matches) {
      ref_input_ = input;
      ref_features_ = std::move(features);
      guidance_ = BS_GUIDE_MOVE_SIDEWAYS;
    } else {
      guidance_ = BS_GUIDE_MOVE_CLOSER;
    }
    return;
  }

  ++bootstrap_attempts_;
  // A stale or unmatchable reference gets replaced.
  const bool ref_stale = input.t_capture - ref_input_->t_capture > 4.0;

  MatchOptions match_options;
  match_options.ratio = config_.live_match_ratio;
  match_options.cross_check = true;
  match_options.max_distance = 64.0f;
  const std::vector<Match> matches =
      MatchFeatures(ref_features_->features, features.features, match_options);
  last_matches_ = static_cast<int>(matches.size());

  if (static_cast<int>(matches.size()) < config_.boot_min_matches) {
    guidance_ = BS_GUIDE_MOVE_CLOSER;
    if (ref_stale) {
      ref_input_ = input;
      ref_features_ = std::move(features);
    }
    return;
  }

  std::vector<cv::Point2f> pts_ref, pts_cur;
  pts_ref.reserve(matches.size());
  pts_cur.reserve(matches.size());
  for (const auto& m : matches) {
    pts_ref.push_back(ref_features_->undistorted[m.idx_a]);
    pts_cur.push_back(features.undistorted[m.idx_b]);
  }

  TwoViewOptions tv;
  tv.ransac_thresh_px = config_.boot_ransac_px;
  tv.min_median_tri_angle_deg = config_.boot_min_median_tri_deg;
  tv.min_cheirality_frac = config_.boot_min_cheirality;
  const TwoViewResult rel =
      EstimateRelativePose(pts_ref, pts_cur, input.K, tv);

  if (!rel.ok()) {
    guidance_ = rel.failure == TwoViewFailure::kRotationDominant
                    ? BS_GUIDE_MOVE_SIDEWAYS
                    : BS_GUIDE_MOVE_CLOSER;
    if (ref_stale) {
      ref_input_ = input;
      ref_features_ = std::move(features);
    }
    return;
  }

  // Triangulate inliers between ref (identity) and cur (rel pose).
  const SE3 pose_ref = SE3::Identity();
  const SE3& pose_cur = rel.rel_pose;

  struct Candidate {
    int match_idx;
    Eigen::Vector3d X;
    double angle_deg;
    double err_px;
  };
  std::vector<Candidate> candidates;
  for (size_t i = 0; i < matches.size(); ++i) {
    if (i >= rel.inlier_mask.size() || rel.inlier_mask[i] == 0) continue;
    const auto tri = TriangulateTwoView(pose_ref, pose_cur, input.K,
                                        pts_ref[i], pts_cur[i]);
    if (!tri || !tri->InFrontOfAll()) continue;
    if (tri->max_angle_deg < 1.0 || tri->max_reproj_error_px > 2.0) continue;
    candidates.push_back({static_cast<int>(i), tri->point, tri->max_angle_deg,
                          tri->mean_reproj_error_px});
  }
  if (static_cast<int>(candidates.size()) < config_.boot_min_matches / 2) {
    guidance_ = BS_GUIDE_MOVE_SIDEWAYS;
    return;
  }

  // Metric scale from LiDAR depth agreement at both views (gauge only).
  const auto depth_ref = MakeDepthFrame(*ref_input_);
  const auto depth_cur = MakeDepthFrame(input);
  std::vector<double> pair_samples;
  auto sample_ratio = [&](const std::shared_ptr<DepthFrame>& depth,
                          const SE3& pose, const Eigen::Vector3d& X) {
    if (!depth) return;
    const Eigen::Vector3d xc = pose.Apply(X);
    if (xc.z() < 0.1) return;
    // Depth maps live in distorted sensor geometry.
    const Eigen::Vector2d xn(xc.x() / xc.z(), xc.y() / xc.z());
    const Eigen::Vector2d xd = DistortNormalized(xn, k1_, k2_);
    const double u = depth->K().fx * xd.x() + depth->K().cx;
    const double v = depth->K().fy * xd.y() + depth->K().cy;
    const int xi = static_cast<int>(std::lround(u));
    const int yi = static_cast<int>(std::lround(v));
    if (depth->ConfidenceAt(xi, yi) < 0.5) return;
    const auto d = depth->DepthBilinear(u, v);
    if (!d) return;
    pair_samples.push_back(*d / xc.z());
  };
  for (const auto& c : candidates) {
    sample_ratio(depth_ref, pose_ref, c.X);
    sample_ratio(depth_cur, pose_cur, c.X);
  }

  if (rel.planar_ambiguous) {
    // Single-plane views leave the essential matrix with its conjugate-plane
    // two-fold ambiguity. LiDAR disambiguates as *validation*: the wrong
    // branch reconstructs a slanted/mirrored surface whose per-point
    // depth ratios against the measured depth spread wildly, while the
    // right branch clusters tightly. No depth, no bootstrap from a plane.
    const ScaleEstimate branch_check = EstimateScale(pair_samples, {});
    const bool validated = branch_check.samples >= 30 &&
                           branch_check.mad_ratio < 0.10;
    if (!validated) {
      guidance_ = BS_GUIDE_MOVE_SIDEWAYS;
      if (ref_stale) {
        ref_input_ = input;
        ref_features_ = std::move(features);
      }
      return;
    }
  }

  scale_samples_.insert(scale_samples_.end(), pair_samples.begin(),
                        pair_samples.end());
  ScaleOptions scale_options;
  scale_options.min_samples = config_.scale_min_samples;
  scale_options.max_mad_ratio = config_.scale_max_mad_ratio;
  const ScaleEstimate scale = EstimateScale(scale_samples_, scale_options);
  double applied_scale = 1.0;
  if (scale.locked) {
    applied_scale = scale.scale;
    scale_locked_ = true;
  } else if (scale.samples >= 10) {
    applied_scale = scale.scale;  // best effort; final solve re-anchors
  }

  // Build the initial map.
  Keyframe kf_ref;
  kf_ref.frame_id = ref_input_->frame_id;
  kf_ref.t_capture = ref_input_->t_capture;
  kf_ref.pose = pose_ref;
  kf_ref.features = ref_features_->features;
  kf_ref.undistorted = ref_features_->undistorted;
  kf_ref.point_ids.assign(kf_ref.features.keypoints.size(), -1);
  kf_ref.K = ref_input_->K;
  kf_ref.depth = depth_ref;
  if (depth_ref) kf_ref.depth_lookup = std::make_shared<DepthLookup>(*depth_ref);
  Keyframe& ref_kf = map_.AddKeyframe(std::move(kf_ref));

  Keyframe kf_cur;
  kf_cur.frame_id = input.frame_id;
  kf_cur.t_capture = input.t_capture;
  kf_cur.pose = pose_cur;
  kf_cur.pose.t *= applied_scale;
  kf_cur.features = features.features;
  kf_cur.undistorted = features.undistorted;
  kf_cur.point_ids.assign(kf_cur.features.keypoints.size(), -1);
  kf_cur.K = input.K;
  kf_cur.depth = depth_cur;
  if (depth_cur) kf_cur.depth_lookup = std::make_shared<DepthLookup>(*depth_cur);
  Keyframe& cur_kf = map_.AddKeyframe(std::move(kf_cur));

  for (const auto& c : candidates) {
    const Match& m = matches[c.match_idx];
    MapPoint point;
    point.X = c.X * applied_scale;
    point.descriptor = features.features.descriptors.row(m.idx_b).clone();
    point.observations = {{ref_kf.kf_id, m.idx_a}, {cur_kf.kf_id, m.idx_b}};
    point.mean_gradient = static_cast<float>(MeanGradientAt(
        input.gray, features.features.keypoints[m.idx_b].pt));
    point.max_tri_angle_deg = static_cast<float>(c.angle_deg);
    point.last_reproj_err_px = static_cast<float>(c.err_px);
    const uint8_t gray_value = input.gray.at<uint8_t>(
        std::clamp(static_cast<int>(features.features.keypoints[m.idx_b].pt.y),
                   0, input.gray.rows - 1),
        std::clamp(static_cast<int>(features.features.keypoints[m.idx_b].pt.x),
                   0, input.gray.cols - 1));
    point.rgb[0] = point.rgb[1] = point.rgb[2] = gray_value;
    MapPoint& added = map_.AddPoint(std::move(point));
    ref_kf.point_ids[m.idx_a] = added.id;
    cur_kf.point_ids[m.idx_b] = added.id;
  }

  last_pose_ = cur_kf.pose;
  have_prev_pose_ = false;
  velocity_ = SE3::Identity();
  last_kf_id_ = cur_kf.kf_id;
  last_kf_time_ = input.t_capture;
  state_ = BS_LIVE_TRACKING;
  guidance_ = BS_GUIDE_GOOD;

  EmitDirective(ref_kf.frame_id, BS_STORE_KEYFRAME, true);
  EmitDirective(cur_kf.frame_id, BS_STORE_KEYFRAME, true);
  last_store_pose_ = cur_kf.pose;
  have_store_pose_ = true;

  LocalBundleAdjustment(cur_kf.kf_id);

  BS_LOGI("live",
          "bootstrap: frames %u->%u, %zu points, scale=%.3f (%s, %d samples)",
          ref_kf.frame_id, cur_kf.frame_id, map_.points().size(), applied_scale,
          scale_locked_ ? "locked" : "provisional", scale.samples);
  ResetBootstrap();
}

// -------------------------------------------------------------- tracking

SE3 LiveSystem::PredictPose() const {
  if (!have_prev_pose_) return last_pose_;
  return velocity_ * last_pose_;
}

bool LiveSystem::TrackFrame(const LiveFrameInput& input,
                            const FrameFeatures& features) {
  const SE3 predicted = PredictPose();

  // Local map: points of the last keyframe, its covisibles, and recent KFs.
  std::vector<int32_t> local_points;
  {
    std::vector<uint32_t> local_kfs;
    local_kfs.push_back(last_kf_id_);
    for (const auto& [kf_id, _] :
         map_.Covisible(last_kf_id_, config_.lba_min_shared_points / 2)) {
      local_kfs.push_back(kf_id);
      if (local_kfs.size() >= 8) break;
    }
    const auto& kfs = map_.keyframes();
    for (int i = static_cast<int>(kfs.size()) - 1;
         i >= 0 && i + 4 >= static_cast<int>(kfs.size()); --i) {
      local_kfs.push_back(kfs[i].kf_id);
    }
    std::sort(local_kfs.begin(), local_kfs.end());
    local_kfs.erase(std::unique(local_kfs.begin(), local_kfs.end()),
                    local_kfs.end());

    std::vector<char> seen;
    for (const uint32_t kf_id : local_kfs) {
      const Keyframe* kf = map_.FindKeyframe(kf_id);
      if (kf == nullptr) continue;
      for (const int32_t pid : kf->point_ids) {
        if (pid < 0) continue;
        if (static_cast<size_t>(pid) >= seen.size()) seen.resize(pid + 1, 0);
        if (seen[pid]) continue;
        seen[pid] = 1;
        local_points.push_back(pid);
      }
    }
  }

  if (local_points.size() < 20) {
    state_ = BS_LIVE_LOST;
    guidance_ = BS_GUIDE_TRACKING_LOST;
    return Relocalize(input, features);
  }

  // Project into the predicted view; build the guided-match query set.
  FeatureSet query;
  query.type = FeatureType::kOrb;
  std::vector<cv::Point2f> predictions;
  std::vector<int32_t> query_pids;
  std::vector<cv::Mat> desc_rows;
  for (const int32_t pid : local_points) {
    MapPoint* mp = map_.FindPoint(pid);
    if (mp == nullptr) continue;
    const Eigen::Vector3d xc = predicted.Apply(mp->X);
    if (xc.z() < 0.05) continue;
    const Eigen::Vector2d px = input.K.Project(xc);
    const float margin = 40.0f;
    if (px.x() < -margin || px.y() < -margin ||
        px.x() > input.K.width + margin || px.y() > input.K.height + margin) {
      continue;
    }
    ++mp->visible_count;
    predictions.emplace_back(static_cast<float>(px.x()),
                             static_cast<float>(px.y()));
    query_pids.push_back(pid);
    desc_rows.push_back(mp->descriptor);
  }
  if (predictions.size() < 20) {
    state_ = BS_LIVE_LOST;
    guidance_ = BS_GUIDE_TRACKING_LOST;
    return Relocalize(input, features);
  }
  query.descriptors.create(static_cast<int>(desc_rows.size()), 32, CV_8U);
  for (size_t i = 0; i < desc_rows.size(); ++i) {
    desc_rows[i].copyTo(query.descriptors.row(static_cast<int>(i)));
  }
  query.keypoints.resize(desc_rows.size());

  // Match against the frame's features at their UNDISTORTED positions.
  FeatureSet frame_undist;
  frame_undist.type = FeatureType::kOrb;
  frame_undist.descriptors = features.features.descriptors;
  frame_undist.keypoints.reserve(features.undistorted.size());
  for (const auto& pt : features.undistorted) {
    frame_undist.keypoints.emplace_back(pt, 7.0f);
  }

  // Base search radius: fast rotation widens it (gyro magnitude only — the
  // direction is never used; poses stay purely image-derived), and an
  // unknown velocity (first frame after bootstrap/relocalization) widens it
  // further since the constant-velocity prediction is just the last pose.
  const float base_radius =
      config_.live_match_search_px *
      std::min(3.0f, 1.0f + 4.0f * static_cast<float>(last_gyro_mag_)) *
      (have_prev_pose_ ? 1.0f : 2.5f);

  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  std::vector<int32_t> match_pids;
  std::vector<int> match_feat;
  cv::Mat rvec, tvec;
  std::vector<int> inliers;
  bool pnp_ok = false;

  for (const float radius_scale : {1.0f, 3.0f}) {
    MatchOptions guided;
    guided.ratio = config_.live_match_ratio;
    guided.cross_check = false;
    guided.max_distance = 64.0f;
    const std::vector<Match> matches = MatchFeaturesGuided(
        query, frame_undist, predictions, base_radius * radius_scale, guided);
    last_matches_ = static_cast<int>(matches.size());
    if (static_cast<int>(matches.size()) < config_.live_pnp_min_inliers) {
      continue;
    }

    object_points.clear();
    image_points.clear();
    match_pids.clear();
    match_feat.clear();
    for (const auto& m : matches) {
      const MapPoint* mp = map_.FindPoint(query_pids[m.idx_a]);
      if (mp == nullptr) continue;
      object_points.emplace_back(static_cast<float>(mp->X.x()),
                                 static_cast<float>(mp->X.y()),
                                 static_cast<float>(mp->X.z()));
      image_points.push_back(features.undistorted[m.idx_b]);
      match_pids.push_back(query_pids[m.idx_a]);
      match_feat.push_back(m.idx_b);
    }

    PoseToRvecTvec(predicted, rvec, tvec);
    inliers.clear();
    pnp_ok = cv::solvePnPRansac(
        object_points, image_points, IntrinsicsMatrix(input.K), cv::noArray(),
        rvec, tvec, /*useExtrinsicGuess=*/true, /*iterationsCount=*/100,
        static_cast<float>(config_.live_pnp_thresh_px), /*confidence=*/0.999,
        inliers, cv::SOLVEPNP_ITERATIVE);
    last_inliers_ = static_cast<int>(inliers.size());
    if (pnp_ok &&
        static_cast<int>(inliers.size()) >= config_.live_pnp_min_inliers) {
      break;
    }
    pnp_ok = false;
  }

  if (!pnp_ok) {
    ++consecutive_lost_;
    state_ = BS_LIVE_LOST;
    guidance_ = BS_GUIDE_TRACKING_LOST;
    return Relocalize(input, features);
  }

  SE3 pose = PoseFromRvecTvec(rvec, tvec);

  // Pose-only Ceres polish on the PnP inliers.
  {
    double q_param[4], t_param[3];
    PoseToBlocks(pose, q_param, t_param);
    ceres::Problem problem;
    for (const int idx : inliers) {
      problem.AddResidualBlock(
          ReprojectionPoseOnlyResidual::Create(
              image_points[idx].x, image_points[idx].y,
              {object_points[idx].x, object_points[idx].y,
               object_points[idx].z},
              input.K),
          new ceres::HuberLoss(config_.live_pnp_thresh_px), q_param, t_param);
    }
    problem.SetManifold(q_param, new ceres::EigenQuaternionManifold);
    ceres::Solver::Options options;
    options.max_num_iterations = 5;
    options.logging_type = ceres::SILENT;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    pose = PoseFromBlocks(q_param, t_param);
  }

  // Bookkeeping.
  for (const int idx : inliers) {
    if (MapPoint* mp = map_.FindPoint(match_pids[idx])) ++mp->found_count;
  }
  if (have_prev_pose_ || frames_processed_ > 0) {
    velocity_ = pose * last_pose_.Inverse();
    have_prev_pose_ = true;
  }
  last_pose_ = pose;
  consecutive_lost_ = 0;
  state_ = BS_LIVE_TRACKING;

  const double inlier_ratio =
      static_cast<double>(inliers.size()) / std::max(1, last_matches_);
  if (features.lap_var < BlurThreshold(0.4) || last_gyro_mag_ > 0.6) {
    guidance_ = BS_GUIDE_SLOW_DOWN;
  } else if (inlier_ratio < 0.4) {
    guidance_ = BS_GUIDE_RECAPTURE;
  } else {
    guidance_ = BS_GUIDE_GOOD;
  }

  UpdateStoreGate(input);

  if (ShouldInsertKeyframe(input, features, last_inliers_)) {
    std::vector<std::pair<int32_t, int>> associations;
    associations.reserve(inliers.size());
    for (const int idx : inliers) {
      associations.emplace_back(match_pids[idx], match_feat[idx]);
    }
    InsertKeyframe(input, features, associations);
    last_kf_time_ = input.t_capture;
  }
  return true;
}

bool LiveSystem::Relocalize(const LiveFrameInput& input,
                            const FrameFeatures& features) {
  const auto& kfs = map_.keyframes();
  const int start = std::max(0, static_cast<int>(kfs.size()) -
                                    config_.loop_exclude_recent);
  // Search newest-first across up to 20 keyframes.
  for (int i = static_cast<int>(kfs.size()) - 1;
       i >= 0 && i >= static_cast<int>(kfs.size()) - 20; --i) {
    const Keyframe& kf = kfs[i];
    (void)start;

    // Match against the keyframe's associated features only.
    FeatureSet assoc;
    assoc.type = FeatureType::kOrb;
    std::vector<int32_t> pids;
    std::vector<int> rows;
    for (size_t f = 0; f < kf.point_ids.size(); ++f) {
      if (kf.point_ids[f] >= 0) {
        pids.push_back(kf.point_ids[f]);
        rows.push_back(static_cast<int>(f));
      }
    }
    if (pids.size() < 30) continue;
    assoc.descriptors.create(static_cast<int>(rows.size()), 32, CV_8U);
    for (size_t r = 0; r < rows.size(); ++r) {
      kf.features.descriptors.row(rows[r])
          .copyTo(assoc.descriptors.row(static_cast<int>(r)));
    }
    assoc.keypoints.resize(rows.size());

    FeatureSet cur;
    cur.type = FeatureType::kOrb;
    cur.descriptors = features.features.descriptors;
    cur.keypoints = features.features.keypoints;

    MatchOptions options;
    options.ratio = 0.75f;
    options.cross_check = true;
    options.max_distance = 64.0f;
    const std::vector<Match> matches = MatchFeatures(assoc, cur, options);
    if (static_cast<int>(matches.size()) < 30) continue;

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    for (const auto& m : matches) {
      const MapPoint* mp = map_.FindPoint(pids[m.idx_a]);
      if (mp == nullptr) continue;
      object_points.emplace_back(static_cast<float>(mp->X.x()),
                                 static_cast<float>(mp->X.y()),
                                 static_cast<float>(mp->X.z()));
      image_points.push_back(features.undistorted[m.idx_b]);
    }
    if (object_points.size() < 30) continue;

    cv::Mat rvec, tvec;
    std::vector<int> inliers;
    const bool ok = cv::solvePnPRansac(
        object_points, image_points, IntrinsicsMatrix(input.K), cv::noArray(),
        rvec, tvec, false, 200, 4.0f, 0.999, inliers, cv::SOLVEPNP_EPNP);
    if (!ok || static_cast<int>(inliers.size()) < 30) continue;

    last_pose_ = PoseFromRvecTvec(rvec, tvec);
    have_prev_pose_ = false;
    velocity_ = SE3::Identity();
    state_ = BS_LIVE_TRACKING;
    guidance_ = BS_GUIDE_GOOD;
    consecutive_lost_ = 0;
    EmitDirective(input.frame_id, BS_STORE_BURST, false);
    BS_LOGI("live", "relocalized frame %u against kf %u (%zu inliers)",
            input.frame_id, kf.kf_id, inliers.size());
    return true;
  }
  guidance_ = BS_GUIDE_TRACKING_LOST;
  return false;
}

// ------------------------------------------------------------ bookkeeping

void LiveSystem::EmitDirective(uint32_t frame_id, bs_store_reason reason,
                               bool keyframe) {
  bs_store_directive d;
  d.frame_id = frame_id;
  d.reason = reason;
  d.is_keyframe = keyframe ? 1 : 0;
  directives_.push_back(d);
  while (directives_.size() > 4 * BS_MAX_DIRECTIVES) directives_.pop_front();
}

void LiveSystem::UpdateStoreGate(const LiveFrameInput& input) {
  if (!have_store_pose_) {
    last_store_pose_ = last_pose_;
    have_store_pose_ = true;
    EmitDirective(input.frame_id, BS_STORE_GATE, false);
    return;
  }
  const double translation =
      (last_pose_.CameraCenter() - last_store_pose_.CameraCenter()).norm();
  const double rotation_deg =
      RadToDeg(AngularDistance(last_pose_.q, last_store_pose_.q));
  if (translation > config_.store_min_translation_m ||
      rotation_deg > config_.store_min_rotation_deg) {
    last_store_pose_ = last_pose_;
    EmitDirective(input.frame_id, BS_STORE_GATE, false);
  }
}

void LiveSystem::LogPose(const LiveFrameInput& input, bool tracked) {
  PoseLogEntry entry;
  entry.frame_id = input.frame_id;
  entry.t = input.t_capture;
  entry.tracked = tracked;
  if (tracked) entry.pose = last_pose_;
  pose_log_.push_back(entry);
}

std::vector<bs_store_directive> LiveSystem::DrainDirectives() {
  std::vector<bs_store_directive> out;
  while (!directives_.empty() &&
         out.size() < static_cast<size_t>(BS_MAX_DIRECTIVES)) {
    out.push_back(directives_.front());
    directives_.pop_front();
  }
  return out;
}

void LiveSystem::FillStatus(bs_live_status& out) const {
  out.state = state_;
  out.last_frame_id = last_frame_id_;
  out.frames_processed = frames_processed_;
  out.keyframes = static_cast<uint32_t>(map_.keyframes().size());
  out.map_points = static_cast<uint32_t>(map_.points().size());
  out.guidance = guidance_;
  out.scale_locked = scale_locked_ ? 1 : 0;
  out.blur_metric = static_cast<float>(last_lap_var_);
  out.inlier_ratio =
      last_matches_ > 0
          ? static_cast<float>(last_inliers_) / static_cast<float>(last_matches_)
          : 0.0f;
  if (state_ == BS_LIVE_TRACKING) {
    out.pose_valid = 1;
    out.q[0] = last_pose_.q.w();
    out.q[1] = last_pose_.q.x();
    out.q[2] = last_pose_.q.y();
    out.q[3] = last_pose_.q.z();
    out.t[0] = last_pose_.t.x();
    out.t[1] = last_pose_.t.y();
    out.t[2] = last_pose_.t.z();
  }
}

void LiveSystem::FillSnapshot(std::vector<bs_snap_point>& points,
                              std::vector<bs_snap_camera>& cameras) const {
  points.reserve(map_.points().size());
  for (const auto& [_, mp] : map_.points()) {
    bs_snap_point p{};
    p.x = static_cast<float>(mp.X.x());
    p.y = static_cast<float>(mp.X.y());
    p.z = static_cast<float>(mp.X.z());
    p.r = mp.rgb[0];
    p.g = mp.rgb[1];
    p.b = mp.rgb[2];
    p.flags = 0;
    if (mp.max_tri_angle_deg < 1.5f || mp.last_reproj_err_px > 2.0f) {
      p.flags |= 1;  // low confidence
    }
    points.push_back(p);
  }
  cameras.reserve(map_.keyframes().size());
  for (const auto& kf : map_.keyframes()) {
    bs_snap_camera c{};
    c.frame_id = kf.frame_id;
    c.q[0] = static_cast<float>(kf.pose.q.w());
    c.q[1] = static_cast<float>(kf.pose.q.x());
    c.q[2] = static_cast<float>(kf.pose.q.y());
    c.q[3] = static_cast<float>(kf.pose.q.z());
    c.t[0] = static_cast<float>(kf.pose.t.x());
    c.t[1] = static_cast<float>(kf.pose.t.y());
    c.t[2] = static_cast<float>(kf.pose.t.z());
    c.is_keyframe = 1;
    cameras.push_back(c);
  }
}

bool LiveSystem::End() {
  state_ = BS_LIVE_FINISHED;
  if (session_dir_.empty()) return true;

  std::error_code ec;
  fs::create_directories(fs::path(session_dir_) / "live", ec);
  std::ofstream out(fs::path(session_dir_) / "live" / "poses.jsonl",
                    std::ios::trunc);
  if (!out) return false;
  out.precision(12);
  for (const auto& e : pose_log_) {
    out << "{\"frame_id\":" << e.frame_id << ",\"t\":" << e.t << ",\"state\":\""
        << (e.tracked ? "tracking" : "lost") << "\"";
    if (e.tracked) {
      out << ",\"q\":[" << e.pose.q.w() << "," << e.pose.q.x() << ","
          << e.pose.q.y() << "," << e.pose.q.z() << "],\"p\":[" << e.pose.t.x()
          << "," << e.pose.t.y() << "," << e.pose.t.z() << "]";
    }
    out << "}\n";
  }
  BS_LOGI("live", "end: %zu poses, %zu keyframes, %zu points",
          pose_log_.size(), map_.keyframes().size(), map_.points().size());
  return static_cast<bool>(out);
}

}  // namespace bs
