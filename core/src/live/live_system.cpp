#include "live/live_system.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "calib/lut_fit.h"
#include "common/log.h"
#include "fusion/scale.h"
#include "geometry/triangulation.h"
#include "geometry/two_view.h"
#include "io/float16.h"
#include "io/session_schema.h"
#include "live/map_io.h"
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

void LiveSystem::Begin(const std::string& session_dir, double k1, double k2,
                       bs_pass_kind pass) {
  session_dir_ = session_dir;
  k1_ = k1;
  k2_ = k2;
  pass_ = pass;
  state_ = BS_LIVE_INITIALIZING;
  guidance_ = BS_GUIDE_NONE;

  // Resolve the session this one continues, if any. Read here rather than
  // passed in so the C ABI does not have to grow a parameter that only
  // exists to restate what session.json already says.
  parent_dir_ = "";
  if (auto info = SessionInfo::FromJson(
          ReadTextFile((fs::path(session_dir_) / "session.json").string()))) {
    if (!info->parent_session.empty()) {
      parent_dir_ =
          (fs::path(session_dir_).parent_path() / info->parent_session).string();
    }
  }

  // A capture pass starts from the scout circuit's map when there is one.
  // The scaffold already covers the whole space and is already metric, so
  // this pass never has to bootstrap its own gauge: it relocalizes into the
  // session's existing world frame and keeps it. That is the entire point of
  // walking the place first — tracking loss becomes recoverable anywhere,
  // instead of stranding the user in a room nothing has mapped.
  scaffold_keyframes_ = 0;
  if (pass_ == BS_PASS_CAPTURE) {
    // Preference order, most specific first:
    //   1. this session's own scout scaffold
    //   2. the parent session's end-of-capture map  } continuing a facility
    //   3. the parent session's scout scaffold      } too big for one capture
    //
    // The parent's END map before its scaffold, because by the end it has
    // absorbed everything the capture pass added — the parent's scaffold is
    // a strict subset of it.
    std::string map_path =
        (fs::path(session_dir_) / "live" / "map.bin").string();
    if (!fs::exists(map_path) && !parent_dir_.empty()) {
      const fs::path parent(parent_dir_);
      for (const char* name : {"map_end.bin", "map.bin"}) {
        const std::string candidate = (parent / "live" / name).string();
        if (fs::exists(candidate)) {
          map_path = candidate;
          BS_LOGI("live", "continuing from %s", candidate.c_str());
          break;
        }
      }
    }
    bool scaffold_metric = false;
    if (fs::exists(map_path) && ReadLiveMap(map_path, map_, &scaffold_metric)) {
      scaffold_keyframes_ = static_cast<uint32_t>(map_.keyframes().size());
      if (scaffold_keyframes_ > 0) {
        // The scaffold's gauge IS the session gauge — but only inherit a
        // metric claim the scout pass actually earned.
        scale_locked_ = scaffold_metric;
        // Nothing is tracked yet, but there is a map to find ourselves in,
        // so start in the state whose job is exactly that.
        state_ = BS_LIVE_LOST;
        guidance_ = BS_GUIDE_TRACKING_LOST;
        BS_LOGI("live", "loaded scaffold: %u keyframes, %zu points, %s",
                scaffold_keyframes_, map_.points().size(),
                scaffold_metric ? "metric" : "scale NOT locked");
      }
    }
  }

  BS_LOGI("live", "begin %s (k1=%.4f k2=%.4f)",
          pass_ == BS_PASS_SCOUT ? "scout" : "capture", k1, k2);
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
    BS_LOGD("live", "frame %u lost: local map too small (%zu points)",
            input.frame_id, local_points.size());
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
    // Viewing-cone test: a point is only a candidate when seen from a
    // direction close to one it was actually observed from. ORB descriptors
    // are not viewpoint-invariant, so a point mapped from across the room is
    // not matchable from here — including it only crowds the search and
    // wastes the ratio test on a candidate that cannot match.
    {
      const Eigen::Vector3d here =
          (mp->X - predicted.CameraCenter()).normalized();
      double best_cos = -2.0;
      for (const auto& [obs_kf_id, _f] : mp->observations) {
        const Keyframe* okf = map_.FindKeyframe(obs_kf_id);
        if (okf == nullptr) continue;
        const Eigen::Vector3d there =
            (mp->X - okf->pose.CameraCenter()).normalized();
        best_cos = std::max(best_cos, here.dot(there));
      }
      if (best_cos < config_.track_max_view_cos) continue;
    }

    ++mp->visible_count;
    predictions.emplace_back(static_cast<float>(px.x()),
                             static_cast<float>(px.y()));
    query_pids.push_back(pid);
    desc_rows.push_back(mp->descriptor);
  }
  BS_LOGD("live", "frame %u: local %zu -> %zu in view (kf %u, %zu map pts)",
          input.frame_id, local_points.size(), predictions.size(), last_kf_id_,
          map_.points().size());
  if (predictions.size() < 20) {
    const Eigen::Vector3d pc = predicted.CameraCenter();
    const Eigen::Vector3d lc = last_pose_.CameraCenter();
    BS_LOGD("live",
            "frame %u lost: only %zu of %zu local points project; "
            "last C=(%.2f,%.2f,%.2f) pred C=(%.2f,%.2f,%.2f) kf=%u",
            input.frame_id, predictions.size(), local_points.size(), lc.x(),
            lc.y(), lc.z(), pc.x(), pc.y(), pc.z(), last_kf_id_);
    state_ = BS_LIVE_LOST;
    guidance_ = BS_GUIDE_TRACKING_LOST;
    return Relocalize(input, features);
  }
  int empty_desc = 0;
  query.descriptors.create(static_cast<int>(desc_rows.size()), 32, CV_8U);
  for (size_t i = 0; i < desc_rows.size(); ++i) {
    if (desc_rows[i].empty() || desc_rows[i].cols != 32) {
      ++empty_desc;
      query.descriptors.row(static_cast<int>(i)).setTo(0);
      continue;
    }
    desc_rows[i].copyTo(query.descriptors.row(static_cast<int>(i)));
  }
  if (empty_desc > 0) {
    BS_LOGD("live", "frame %u: %d of %zu query descriptors unusable",
            input.frame_id, empty_desc, desc_rows.size());
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
    BS_LOGD("live",
            "frame %u lost: PnP failed (%zu projected, %d matched, %d feats)",
            input.frame_id, predictions.size(), last_matches_,
            features.features.size());
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

  // Motion plausibility. A high inlier count is not proof of a correct pose:
  // in a room of repeated texture, PnP RANSAC can find a large consistent
  // set at a completely wrong place (measured: a 5.9 m single-frame jump
  // accepted with 92/114 inliers). One such pose is unrecoverable — the
  // local map then projects outside the frustum, so no later frame can
  // re-acquire by tracking. A held camera has bounded speed, so a pose that
  // violates it is rejected here and relocalization takes over instead.
  double turn_rate_dps = 0.0;
  if (last_track_time_ >= 0.0 && consecutive_lost_ == 0) {
    const double dt =
        std::clamp(input.t_capture - last_track_time_, 1.0 / 60.0, 0.5);
    turn_rate_dps = TurnRateDps(last_pose_, pose, dt);
    if (!MotionIsPlausible(last_pose_, pose, dt, config_.track_max_speed_mps,
                           config_.track_max_rot_dps)) {
      BS_LOGD("live",
              "frame %u rejected: implausible motion %.2f m / %.1f deg in "
              "%.3f s, %d/%d inliers",
              input.frame_id,
              (pose.CameraCenter() - last_pose_.CameraCenter()).norm(),
              RadToDeg(AngularDistance(pose.q, last_pose_.q)), dt,
              last_inliers_, last_matches_);
      ++consecutive_lost_;
      state_ = BS_LIVE_LOST;
      guidance_ = BS_GUIDE_TRACKING_LOST;
      return Relocalize(input, features);
    }
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
  last_track_time_ = input.t_capture;
  consecutive_lost_ = 0;
  state_ = BS_LIVE_TRACKING;

  // Turning too fast to map. A new point needs two keyframes that both see
  // it, so the map can only grow at the rate keyframes accumulate — while a
  // turn sweeps the leading edge of the view across unmapped space at the
  // rotation rate. Past a certain rate the second one wins: measured on a
  // walking circuit, a 120 deg/s turn round a doorway took new points from
  // 67 to 12 per keyframe while in-view support fell 616 -> 64 over fifteen
  // frames, and tracking ended. This is well inside what a wrist can do and
  // far below the plausibility gate, so it is not a bad pose to reject — it
  // is the one moment the user can still be told to slow down.
  const double inlier_ratio =
      static_cast<double>(inliers.size()) / std::max(1, last_matches_);
  if (features.lap_var < BlurThreshold(0.4) || last_gyro_mag_ > 0.6 ||
      turn_rate_dps > config_.track_warn_rot_dps) {
    guidance_ = BS_GUIDE_SLOW_DOWN;
  } else if (inlier_ratio < 0.4) {
    guidance_ = BS_GUIDE_RECAPTURE;
  } else {
    guidance_ = BS_GUIDE_GOOD;
  }

  UpdateStoreGate(input, features);

  if (ShouldInsertKeyframe(input, features, last_inliers_)) {
    std::vector<std::pair<int32_t, int>> associations;
    associations.reserve(inliers.size());
    for (const int idx : inliers) {
      associations.emplace_back(match_pids[idx], match_feat[idx]);
    }
    InsertKeyframe(input, features, associations);
    last_kf_time_ = input.t_capture;

    // Adopt the keyframe's post-BA pose. InsertKeyframe runs local bundle
    // adjustment, which moves both the keyframe poses and the map points;
    // `last_pose_` was computed before that and is now expressed against a
    // map that has since shifted underneath it. Leaving it stale makes the
    // next frame predict from one gauge and match against another — and a
    // single large correction is then enough to throw every local point out
    // of the predicted frustum, which is exactly how tracking used to die
    // one frame after a keyframe (6 of 492 points projecting, permanently).
    // The constant-velocity estimate is deliberately KEPT. It describes how
    // the camera is moving, which a bundle adjustment of the map does not
    // change; discarding it here made the next frame predict no motion at
    // all, and one frame of real motion (~35 px at walking pace and 2 m
    // depth) is already wider than the guided-match radius — so every
    // keyframe insertion blinded the frame that followed it.
    if (const Keyframe* inserted = map_.FindKeyframe(last_kf_id_)) {
      last_pose_ = inserted->pose;
    }
  }
  return true;
}

bool LiveSystem::Relocalize(const LiveFrameInput& input,
                            const FrameFeatures& features) {
  const auto& kfs = map_.keyframes();
  const int kf_count = static_cast<int>(kfs.size());
  if (kf_count == 0) {
    guidance_ = BS_GUIDE_TRACKING_LOST;
    return false;
  }

  // Candidate selection. Searching only the newest keyframes recovers from a
  // momentary glance away, but it cannot recover a walkthrough: no keyframes
  // are inserted while lost, so that window stays frozen on the very frames
  // that just failed, and the older keyframes covering the rest of the space
  // are never retried — including the ones the user walks back into.
  //
  // So: always try the newest few, then sweep a cursor across the remainder
  // of the map, advancing it every attempt. Every keyframe is retried within
  // a bounded number of frames, at a fixed cost per frame.
  //
  // The sweep widens the longer we stay lost. A fixed eight candidates is a
  // sensible cost while there is a live map to fall back on, but a capture
  // pass that loads a 224-keyframe scaffold is testing 3.6% of it per frame
  // — and a pass that is lost has nothing else to spend its budget on.
  constexpr int kRecentCandidates = 5;
  constexpr int kSweepCandidates = 8;
  constexpr int kMaxSweepCandidates = 64;
  const int sweep_width =
      std::min(kMaxSweepCandidates,
               kSweepCandidates * (1 + std::min(7, consecutive_lost_ / 8)));
  std::vector<int> candidates;
  candidates.reserve(kRecentCandidates + sweep_width);
  for (int i = kf_count - 1; i >= 0 && i > kf_count - 1 - kRecentCandidates;
       --i) {
    candidates.push_back(i);
  }
  const int sweep_span = kf_count - static_cast<int>(candidates.size());
  for (int n = 0; n < std::min(sweep_width, sweep_span); ++n) {
    candidates.push_back(static_cast<int>(
        (reloc_cursor_ + static_cast<uint32_t>(n)) % sweep_span));
  }
  if (sweep_span > 0) {
    reloc_cursor_ = (reloc_cursor_ + std::min(kSweepCandidates, sweep_span)) %
                    static_cast<uint32_t>(sweep_span);
  }

  for (const int i : candidates) {
    const Keyframe& kf = kfs[i];

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
    last_track_time_ = input.t_capture;
    have_prev_pose_ = false;
    velocity_ = SE3::Identity();
    state_ = BS_LIVE_TRACKING;
    guidance_ = BS_GUIDE_GOOD;
    consecutive_lost_ = 0;
    // Move the local-map anchor to where we actually are. TrackFrame builds
    // its candidate set from this keyframe and its covisibles, so leaving it
    // on the pre-loss keyframe hands the next frame a local map for a place
    // the camera is no longer in — and it goes straight back to LOST. That
    // is the reloc-lose-reloc-lose sawtooth: relocalization kept succeeding
    // with 30-70 inliers and every recovery died on the following frame.
    last_kf_id_ = kf.kf_id;
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

void LiveSystem::UpdateStoreGate(const LiveFrameInput& input,
                                 const FrameFeatures& features) {
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
  // How far apart stored frames need to be is a question about the SCENE,
  // not about the room. A capture flown at arm's length around a table and
  // one walked down a wall six metres away want very different spacings for
  // the same overlap between neighbouring images, and a single distance in
  // metres has to be wrong for one of them. At a flat 5 cm the circle-and-
  // orbit walk stores ~944 frames for one room against a 200-500 budget —
  // twice what a project is sized for, and a house ten times over.
  //
  // Scaled by scene depth it is ~4% of what the camera is looking at: 24 cm
  // down a wall at 6 m, 11 cm around a table at 2.7 m, and the 5 cm floor
  // wherever the phone is close enough for that to be the tighter rule.
  // Median depth of the tracked points, the same measure the keyframe gate
  // uses, because it works past the LiDAR's 5 m range where a depth-image
  // median would simply go blank on exactly the wide shots this is for.
  // From the points of the most recent keyframe, which are the ones actually
  // being looked at. Sampling the whole map instead would count anything
  // that happens to lie in front of the camera, including the far room
  // through a doorway, and pull the median out past what is in frame.
  double median_depth = 0;
  if (const Keyframe* kf = map_.FindKeyframe(last_kf_id_)) {
    std::vector<double> depths;
    depths.reserve(kf->point_ids.size());
    for (const int32_t pid : kf->point_ids) {
      if (pid < 0) continue;
      const auto it = map_.points().find(pid);
      if (it == map_.points().end()) continue;
      const double z = last_pose_.Apply(it->second.X).z();
      if (z > 0.05) depths.push_back(z);
    }
    if (depths.size() > 10) {
      std::nth_element(depths.begin(), depths.begin() + depths.size() / 2,
                       depths.end());
      median_depth = depths[depths.size() / 2];
    }
  }
  const double needed_translation =
      std::max(static_cast<double>(config_.store_min_translation_m),
               config_.store_translation_depth_frac * median_depth);
  store_spacing_m_ = static_cast<float>(needed_translation);
  const bool moved = translation > needed_translation ||
                     rotation_deg > config_.store_min_rotation_deg;
  if (!moved) return;

  // Geometry says a frame is due here. Sharpness decides WHICH frame that
  // is. RAW is the layer the final solve reconstructs from and it is never
  // rewritten, so a smeared frame stored now is a smeared frame forever:
  // SIFT finds fewer and worse-localized keypoints on it, and every later
  // stage inherits that. Walking at 1 m/s a good fraction of frames carry
  // motion blur, and the ones that happen to land on the 5 cm boundary are
  // no likelier to be the sharp ones — so wait a few frames for a sharp one
  // instead of taking whatever the geometry lands on.
  //
  // The threshold is relative to recent sharpness, because absolute
  // Laplacian variance is a property of the scene as much as the optics.
  const bool sharp = config_.store_min_sharpness_frac <= 0.0f ||
                     features.lap_var >=
                         BlurThreshold(config_.store_min_sharpness_frac);

  // ...but coverage beats sharpness. If the camera has travelled well past
  // where a frame was due and nothing sharp has come along — a dim room, a
  // continuous pan — store what there is. A gap in coverage cannot be fixed
  // later; a slightly soft frame can at least be down-weighted.
  const bool overdue =
      translation > 2.0 * needed_translation ||
      rotation_deg > 2.0 * config_.store_min_rotation_deg;

  if (!sharp && !overdue) return;

  last_store_pose_ = last_pose_;
  EmitDirective(input.frame_id, BS_STORE_GATE, false);
  if (!sharp) {
    BS_LOGD("live", "frame %u stored soft (lap_var %.0f < %.0f) after %.2f m",
            input.frame_id, features.lap_var,
            BlurThreshold(config_.store_min_sharpness_frac), translation);
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
  out.keyframes = static_cast<uint32_t>(map_.keyframes().size());
  out.map_points = static_cast<uint32_t>(map_.points().size());
  out.guidance = guidance_;
  out.scale_locked = scale_locked_ ? 1 : 0;
  out.blur_metric = static_cast<float>(last_lap_var_);
  out.store_spacing_m = store_spacing_m_;
  out.readiness_overall = readiness_.OverallScore();
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
  FillGuideVector(out);
}

GuideVector GuideToNearestKeyframe(const SE3& from_pose,
                                   const std::deque<Keyframe>& keyframes,
                                   double min_distance_m) {
  GuideVector out;
  // Aim at the NEAREST keyframe, not the most recent. Someone who walked
  // into an unmapped corner should be sent to whichever mapped place is
  // closest, which may well be behind them rather than back along the path
  // they took to get there.
  const Eigen::Vector3d here = from_pose.CameraCenter();
  const Keyframe* nearest = nullptr;
  double best_sq = std::numeric_limits<double>::max();
  for (const auto& kf : keyframes) {
    const double d_sq = (kf.pose.CameraCenter() - here).squaredNorm();
    if (d_sq < best_sq) {
      best_sq = d_sq;
      nearest = &kf;
    }
  }
  if (nearest == nullptr) return out;

  const Eigen::Vector3d to_target = nearest->pose.CameraCenter() - here;
  const double distance = to_target.norm();
  // Standing essentially on top of a keyframe and still lost means the
  // problem is which way the camera is POINTING, not where the user is
  // standing. An arrow would send them away from the very place they need to
  // be looking at, so say nothing and let the wording carry it.
  if (distance < min_distance_m) return out;

  out.valid = true;
  // World -> camera is a rotation by q, so this lands in camera coordinates:
  // +x the user's right, +y down, +z where the camera is pointed.
  out.dir_camera = from_pose.q * to_target.normalized();
  out.distance_m = distance;
  out.kf_id = nearest->kf_id;
  return out;
}

// Where to send the user, as a direction they can act on.
//
// "Tracking lost — return to a mapped area" is close to useless on its own:
// the one thing the person does not know is where the mapped area IS, and it
// is the one thing the engine knows exactly. Every recovered frame is a
// frame that stays in the reconstruction, so this is a data-quality fix as
// much as a wording one.
void LiveSystem::FillGuideVector(bs_live_status& out) const {
  out.guide_dir[0] = out.guide_dir[1] = out.guide_dir[2] = 0.0f;
  out.guide_dist_m = 0.0f;
  out.guide_region_id = 0;
  if (guidance_ != BS_GUIDE_TRACKING_LOST) return;

  // The last tracked pose is the best estimate of where the user is: they
  // lost tracking a moment ago and have not usually gone far since.
  const GuideVector guide =
      GuideToNearestKeyframe(last_pose_, map_.keyframes());
  if (!guide.valid) return;

  out.guide_dir[0] = static_cast<float>(guide.dir_camera.x());
  out.guide_dir[1] = static_cast<float>(guide.dir_camera.y());
  out.guide_dir[2] = static_cast<float>(guide.dir_camera.z());
  out.guide_dist_m = static_cast<float>(guide.distance_m);
  out.guide_region_id = readiness_.RegionOfKeyframe(guide.kf_id);
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

  // A scout pass exists to leave this behind. Everything else in live/ is
  // disposable because it can be recomputed from RAW; the scaffold has to
  // persist because the next pass needs it before it has reconstructed
  // anything of its own.
  if (pass_ == BS_PASS_SCOUT && !map_.keyframes().empty()) {
    const std::string map_path =
        (fs::path(session_dir_) / "live" / "map.bin").string();
    if (WriteLiveMap(map_, map_path, scale_locked_)) {
      BS_LOGI("live", "scout scaffold written: %zu keyframes, %zu points",
              map_.keyframes().size(), map_.points().size());
    } else {
      BS_LOGW("live", "failed to write scout scaffold to %s", map_path.c_str());
    }
  }

  // A capture pass leaves its accumulated map behind too, under a different
  // name, for the NEXT SESSION to localize into. A building bigger than one
  // capture's frame budget is walked as a chain of sessions, and this is
  // what lets session N+1 pick up session N's world frame instead of
  // starting its own.
  //
  // Deliberately not map.bin. Overwriting the scout scaffold would make a
  // re-run of the capture pass start from the previous run's result, so the
  // same session would replay differently every time — and the scaffold is
  // the better reference anyway, having been walked for coverage rather than
  // for detail.
  if (pass_ == BS_PASS_CAPTURE && !map_.keyframes().empty()) {
    const std::string map_path =
        (fs::path(session_dir_) / "live" / "map_end.bin").string();
    if (WriteLiveMap(map_, map_path, scale_locked_)) {
      BS_LOGI("live", "session map written: %zu keyframes, %zu points",
              map_.keyframes().size(), map_.points().size());
    } else {
      BS_LOGW("live", "failed to write session map to %s", map_path.c_str());
    }
  }
  // Each pass gets its own log. They answer different questions — did the
  // scout circuit hold position all the way round, and did the capture pass
  // localize into what it left — and a single file means the second pass
  // erases the evidence for the first.
  const char* log_name =
      pass_ == BS_PASS_SCOUT ? "poses_scout.jsonl" : "poses.jsonl";
  std::ofstream out(fs::path(session_dir_) / "live" / log_name,
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
