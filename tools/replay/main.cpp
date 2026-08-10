// bs_replay — run the engine against a captured session directory on Linux.
//
// This is the primary debugging path for the project: the iPhone app exports
// a full-session zip (RAW layer), and this CLI feeds those frames through the
// same engine binary logic that runs on the phone — live pipeline first, then
// the final solve. No Mac or device required to reproduce engine behavior.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "bs/bs_api.h"
#include "common/geometry.h"
#include "geometry/triangulation.h"
#include "geometry/two_view.h"
#include "io/session_reader.h"
#include "vision/features.h"
#include "vision/matching.h"

namespace {

void PrintUsage() {
  std::fprintf(stderr,
               "usage: bs_replay <session_dir> [--info] [--live] [--config J]\n"
               "                 [--two-view [gap]] [--final [fast|quality]]\n"
               "                 [--check]\n"
               "       bs_replay --version | --selftest\n"
               "  --info      print session summary and exit\n"
               "  --live      feed all frames through the live pipeline\n"
               "  --two-view  match frame pairs (gap apart, default 4), estimate\n"
               "              relative pose + triangulate; compares to ground\n"
               "              truth when present\n"
               "  --final     run the full global reconstruction + COLMAP export\n"
               "  --check     exit nonzero when error bounds are exceeded\n"
               "  --config    engine config JSON string (default {})\n");
}

int RunInfo(const bs::SessionReader& session) {
  const auto& info = session.info();
  std::printf("session      %s\n", info.session_id.c_str());
  std::printf("device       %s (iOS %s)\n", info.device_model.c_str(),
              info.device_ios.c_str());
  std::printf("video        %dx%d @%dfps (%s)\n", info.video_w, info.video_h,
              info.video_fps, info.video_pixel_format.c_str());
  std::printf("depth        %dx%d (%s, filtering=%s)\n", info.depth_w,
              info.depth_h, info.depth_format.c_str(),
              info.depth_filtering ? "on" : "off");
  std::printf("frames       %zu on disk (session.json says %u)\n",
              session.frame_ids().size(), info.frame_count);
  std::printf("keyframes    %zu\n", info.keyframe_ids.size());
  const auto gt = session.ReadGroundTruth();
  std::printf("ground truth %s\n", gt ? "present (synthetic session)" : "absent");

  if (!session.frame_ids().empty()) {
    const uint32_t first = session.frame_ids().front();
    const auto meta = session.ReadMeta(first);
    const auto depth = session.ReadDepth(first);
    if (meta) {
      std::printf("frame %06u  fx=%.1f cx=%.1f  lap_var=%.0f  kf=%d\n", first,
                  meta->intrinsics.fx, meta->intrinsics.cx,
                  meta->quality.lap_var, meta->is_keyframe ? 1 : 0);
    }
    if (depth) {
      int valid = 0;
      for (int y = 0; y < depth->height; ++y) {
        for (int x = 0; x < depth->width; ++x) {
          if (depth->ValidAt(x, y)) ++valid;
        }
      }
      std::printf("frame %06u  depth %dx%d, %.1f%% valid\n", first, depth->width,
                  depth->height,
                  100.0 * valid / (depth->width * depth->height));
    }
  }
  return 0;
}

// Runs feature detection + matching + relative pose + triangulation on
// frame pairs `gap` apart. With ground truth (synthetic sessions) reports
// pose error statistics; with --check, enforces bounds.
int RunTwoView(const bs::SessionReader& session, int gap, bool check) {
  const auto& ids = session.frame_ids();
  if (ids.size() < static_cast<size_t>(gap + 1)) {
    std::fprintf(stderr, "not enough frames for gap %d\n", gap);
    return 1;
  }
  const auto gt = session.ReadGroundTruth();

  auto gt_pose = [&](uint32_t frame_id) -> std::optional<bs::SE3> {
    if (!gt) return std::nullopt;
    for (const auto& p : gt->poses) {
      if (p.frame_id == frame_id) {
        bs::SE3 pose;
        pose.q = Eigen::Quaterniond(p.q[0], p.q[1], p.q[2], p.q[3]);
        pose.t = Eigen::Vector3d(p.t[0], p.t[1], p.t[2]);
        return pose;
      }
    }
    return std::nullopt;
  };

  double worst_rot = 0, worst_tdir = 0;
  int pairs = 0, failures = 0;

  for (size_t i = 0; i + gap < ids.size(); i += gap) {
    const uint32_t id_a = ids[i];
    const uint32_t id_b = ids[i + gap];

    const auto meta_a = session.ReadMeta(id_a);
    const auto jpeg_a = session.ReadImageBytes(id_a);
    const auto jpeg_b = session.ReadImageBytes(id_b);
    if (!meta_a || !jpeg_a || !jpeg_b) continue;
    const cv::Mat gray_a = cv::imdecode(*jpeg_a, cv::IMREAD_GRAYSCALE);
    const cv::Mat gray_b = cv::imdecode(*jpeg_b, cv::IMREAD_GRAYSCALE);
    if (gray_a.empty() || gray_b.empty()) continue;

    const bs::FeatureSet fa = bs::DetectOrb(gray_a, {});
    const bs::FeatureSet fb = bs::DetectOrb(gray_b, {});
    const std::vector<bs::Match> matches = bs::MatchFeatures(fa, fb, {});

    std::vector<cv::Point2f> pts_a, pts_b;
    for (const auto& m : matches) {
      pts_a.push_back(fa.keypoints[m.idx_a].pt);
      pts_b.push_back(fb.keypoints[m.idx_b].pt);
    }

    bs::Intrinsics K;
    K.fx = meta_a->intrinsics.fx;
    K.fy = meta_a->intrinsics.fy;
    K.cx = meta_a->intrinsics.cx;
    K.cy = meta_a->intrinsics.cy;
    K.width = meta_a->intrinsics.ref_w;
    K.height = meta_a->intrinsics.ref_h;

    const bs::TwoViewResult result =
        bs::EstimateRelativePose(pts_a, pts_b, K, {});
    ++pairs;
    if (!result.ok()) {
      ++failures;
      std::printf("pair %06u->%06u: FAILED (%d) matches=%zu\n", id_a, id_b,
                  static_cast<int>(result.failure), matches.size());
      continue;
    }

    // Triangulate inliers, count well-conditioned points.
    int good_points = 0;
    double err_sum = 0;
    int err_count = 0;
    const bs::SE3 identity = bs::SE3::Identity();
    for (size_t m = 0; m < pts_a.size(); ++m) {
      if (m >= result.inlier_mask.size() || result.inlier_mask[m] == 0) continue;
      const auto tri = bs::TriangulateTwoView(identity, result.rel_pose, K,
                                              pts_a[m], pts_b[m]);
      if (!tri || !tri->InFrontOfAll()) continue;
      if (tri->max_angle_deg >= 1.0 && tri->max_reproj_error_px <= 4.0) {
        ++good_points;
        err_sum += tri->mean_reproj_error_px;
        ++err_count;
      }
    }

    std::string gt_report;
    if (const auto pose_a = gt_pose(id_a), pose_b = gt_pose(id_b);
        pose_a && pose_b) {
      const bs::SE3 rel_gt = *pose_b * pose_a->Inverse();
      const double rot_err =
          bs::RadToDeg(bs::AngularDistance(result.rel_pose.q, rel_gt.q));
      const double cosang = std::clamp(
          result.rel_pose.t.normalized().dot(rel_gt.t.normalized()), -1.0, 1.0);
      const double tdir_err = bs::RadToDeg(std::acos(cosang));
      // Planar-ambiguous pairs may sit on the conjugate-plane branch; they
      // are excluded from bootstrap by design, so exclude them from the
      // accuracy bounds too (still printed for inspection).
      if (!result.planar_ambiguous) {
        worst_rot = std::max(worst_rot, rot_err);
        worst_tdir = std::max(worst_tdir, tdir_err);
      }
      char buf[96];
      std::snprintf(buf, sizeof(buf), "  rot_err=%.3fdeg tdir_err=%.2fdeg",
                    rot_err, tdir_err);
      gt_report = buf;
    }

    std::printf(
        "pair %06u->%06u: matches=%zu E-inl=%d H-inl=%d%s points=%d "
        "mean_err=%.2fpx%s\n",
        id_a, id_b, matches.size(), result.inliers_e, result.inliers_h,
        result.planar_ambiguous ? " (planar)" : "", good_points,
        err_count ? err_sum / err_count : 0.0, gt_report.c_str());
  }

  std::printf("two-view summary: %d pairs, %d failures", pairs, failures);
  if (gt) std::printf(", worst rot_err=%.3fdeg tdir_err=%.2fdeg", worst_rot,
                      worst_tdir);
  std::printf("\n");

  if (check) {
    if (failures > pairs / 4) {
      std::fprintf(stderr, "CHECK FAILED: %d/%d pairs failed\n", failures, pairs);
      return 1;
    }
    if (gt && (worst_rot > 1.0 || worst_tdir > 6.0)) {
      std::fprintf(stderr,
                   "CHECK FAILED: pose errors exceed bounds "
                   "(rot %.3f > 1.0 deg or tdir %.2f > 6.0 deg)\n",
                   worst_rot, worst_tdir);
      return 1;
    }
  }
  return 0;
}

// Runs the final solve through the engine ABI (worker thread + polling, the
// same path the app uses), then optionally checks the exported poses
// against ground truth.
int RunFinal(const bs::SessionReader& session, const std::string& config,
             const std::string& preset, bool check) {
  bs_engine* engine = bs_create(config.c_str());
  if (engine == nullptr) return 1;

  if (bs_final_start(engine, session.dir().c_str(), preset.c_str()) != BS_OK) {
    std::fprintf(stderr, "final_start failed: %s\n", bs_last_error(engine));
    bs_destroy(engine);
    return 1;
  }

  int last_stage = -1;
  bs_final_progress progress{};
  while (true) {
    bs_final_poll(engine, &progress);
    if (progress.stage != last_stage) {
      last_stage = progress.stage;
      std::printf("stage %2d  total %3.0f%%  reg %u/%u  points %u  rmse %.2f\n",
                  progress.stage, 100.0f * progress.total_progress,
                  progress.images_registered, progress.images_total,
                  progress.points, progress.reproj_rmse_px);
      std::fflush(stdout);
    }
    if (progress.stage == BS_STAGE_DONE || progress.stage == BS_STAGE_FAILED ||
        (progress.running == 0 && progress.stage != BS_STAGE_IDLE)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }

  const bool solved = progress.stage == BS_STAGE_DONE;
  std::printf("final solve: %s — registered %u/%u, %u points, rmse %.2fpx, "
              "mean track %.1f\n",
              solved ? "DONE" : "FAILED", progress.images_registered,
              progress.images_total, progress.points, progress.reproj_rmse_px,
              progress.mean_track_len);
  if (!solved) {
    std::fprintf(stderr, "error: %s\n", bs_last_error(engine));
    bs_destroy(engine);
    return 1;
  }
  bs_destroy(engine);

  const auto gt = session.ReadGroundTruth();
  if (!gt) return 0;

  // Parse exported poses and compare with ground truth.
  std::ifstream images(session.dir() + "/final/colmap/images.txt");
  if (!images) {
    std::fprintf(stderr, "missing exported images.txt\n");
    return 1;
  }
  struct Exported {
    uint32_t id;
    bs::SE3 pose;
  };
  std::vector<Exported> exported;
  std::string line;
  while (std::getline(images, line)) {
    if (line.empty() || line[0] == '#') continue;
    Exported e;
    double qw, qx, qy, qz, tx, ty, tz;
    int camera_id;
    char name[64];
    if (std::sscanf(line.c_str(), "%u %lf %lf %lf %lf %lf %lf %lf %d %63s",
                    &e.id, &qw, &qx, &qy, &qz, &tx, &ty, &tz, &camera_id,
                    name) != 10) {
      continue;
    }
    e.pose.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
    e.pose.t = Eigen::Vector3d(tx, ty, tz);
    exported.push_back(e);
    std::getline(images, line);  // POINTS2D line
  }

  auto find_gt = [&](uint32_t frame_id) -> std::optional<bs::SE3> {
    for (const auto& gp : gt->poses) {
      if (gp.frame_id == frame_id) {
        bs::SE3 pose;
        pose.q = Eigen::Quaterniond(gp.q[0], gp.q[1], gp.q[2], gp.q[3]);
        pose.t = Eigen::Vector3d(gp.t[0], gp.t[1], gp.t[2]);
        return pose;
      }
    }
    return std::nullopt;
  };

  if (exported.empty()) return 1;
  const auto gt_ref = find_gt(exported.front().id);
  if (!gt_ref) return 1;
  const bs::SE3 align = exported.front().pose.Inverse() * (*gt_ref);

  double ate_sq = 0, rot_sum = 0;
  int compared = 0;
  for (const auto& e : exported) {
    const auto gt_pose = find_gt(e.id);
    if (!gt_pose) continue;
    const bs::SE3 gt_in_final = *gt_pose * align.Inverse();
    const double err =
        (e.pose.CameraCenter() - gt_in_final.CameraCenter()).norm();
    ate_sq += err * err;
    rot_sum += bs::RadToDeg(bs::AngularDistance(e.pose.q, gt_in_final.q));
    ++compared;
  }
  const double ate = std::sqrt(ate_sq / std::max(1, compared));
  const double rot = rot_sum / std::max(1, compared);
  const double reg_frac = static_cast<double>(progress.images_registered) /
                          std::max(1u, progress.images_total);
  std::printf("final ATE: RMSE %.4f m, mean rot %.3f deg, %d poses, "
              "registration %.0f%%\n",
              ate, rot, compared, 100.0 * reg_frac);

  if (check) {
    if (reg_frac < 0.9) {
      std::fprintf(stderr, "CHECK FAILED: registration %.0f%% < 90%%\n",
                   100.0 * reg_frac);
      return 1;
    }
    if (ate > 0.05) {
      std::fprintf(stderr, "CHECK FAILED: final ATE %.4f > 0.05 m\n", ate);
      return 1;
    }
    if (rot > 1.0) {
      std::fprintf(stderr, "CHECK FAILED: final rot err %.3f > 1 deg\n", rot);
      return 1;
    }
  }
  return 0;
}

// Feeds the frames of one pass through a fresh engine. The scout pass runs
// first and leaves live/map.bin behind; the capture pass then loads it, which
// is exactly the sequence the app performs.
int FeedPass(const bs::SessionReader& session, const std::string& config,
             bs_pass_kind pass, const std::vector<uint32_t>& frame_ids,
             bs_live_status& status_out) {
  bs_engine* engine = bs_create(config.c_str());
  if (engine == nullptr) {
    std::fprintf(stderr, "engine creation failed\n");
    return 1;
  }

  if (bs_live_begin(engine, session.dir().c_str(), pass) != BS_OK) {
    std::fprintf(stderr, "live_begin failed: %s\n", bs_last_error(engine));
    bs_destroy(engine);
    return 1;
  }

  uint32_t fed = 0;
  for (const uint32_t frame_id : frame_ids) {
    const auto meta = session.ReadMeta(frame_id);
    const auto depth = session.ReadDepth(frame_id);
    const auto jpeg = session.ReadImageBytes(frame_id);
    if (!meta || !depth || !jpeg) {
      std::fprintf(stderr, "frame %06u unreadable; skipping\n", frame_id);
      continue;
    }
    const cv::Mat img =
        cv::imdecode(*jpeg, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      std::fprintf(stderr, "frame %06u jpeg decode failed; skipping\n", frame_id);
      continue;
    }
    const std::vector<float> depth_f32 = depth->ToFloat();

    bs_frame_in frame{};
    frame.frame_id = frame_id;
    frame.t_capture = meta->t_capture;
    frame.t_depth = meta->t_depth;
    frame.luma = img.data;
    frame.luma_width = img.cols;
    frame.luma_height = img.rows;
    frame.luma_stride = static_cast<int32_t>(img.step);
    frame.depth = depth_f32.data();
    frame.depth_width = depth->width;
    frame.depth_height = depth->height;
    frame.fx = meta->intrinsics.fx;
    frame.fy = meta->intrinsics.fy;
    frame.cx = meta->intrinsics.cx;
    frame.cy = meta->intrinsics.cy;
    frame.dfx = meta->depth_intrinsics.fx;
    frame.dfy = meta->depth_intrinsics.fy;
    frame.dcx = meta->depth_intrinsics.cx;
    frame.dcy = meta->depth_intrinsics.cy;

    const bs_result r = bs_live_feed(engine, &frame);
    if (r == BS_OK) {
      ++fed;
    } else if (r != BS_ERR_BUSY) {
      std::fprintf(stderr, "feed(%06u) error: %s\n", frame_id,
                   bs_last_error(engine));
    }
  }

  bs_live_poll_status(engine, &status_out);
  std::printf("%s pass: fed %u frames, engine processed %u, state=%d, "
              "keyframes=%u, points=%u, scale_locked=%d\n",
              pass == BS_PASS_SCOUT ? "scout" : "live", fed,
              status_out.frames_processed, status_out.state,
              status_out.keyframes, status_out.map_points,
              status_out.scale_locked);

  const bs_result end = bs_live_end(engine);
  if (end != BS_OK) {
    std::fprintf(stderr, "live_end failed: %s\n", bs_last_error(engine));
  }
  bs_destroy(engine);
  return end == BS_OK ? 0 : 1;
}

// One pass's tracked poses, measured against ground truth. Each pass writes
// its own log because they answer different questions: whether the scout
// circuit held position all the way round, and whether the capture pass
// localized into what the scout left behind.
struct PassAccuracy {
  bool have = false;
  int tracked = 0;
  int compared = 0;
  double tracked_frac = 0;
  double ate_rmse = 0;
  double mean_rot = 0;
};

PassAccuracy EvaluatePass(const bs::SessionReader& session,
                          const bs::GroundTruth& gt, const std::string& log,
                          const std::vector<uint32_t>& pass_ids,
                          const char* label) {
  PassAccuracy out;
  std::ifstream poses_file(session.dir() + "/live/" + log);
  if (!poses_file) return out;

  struct LivePose {
    uint32_t frame_id;
    double t;
    bs::SE3 pose;
  };
  std::vector<LivePose> live_poses;
  std::string line;
  while (std::getline(poses_file, line)) {
    // Minimal parse of the fixed jsonl schema.
    if (line.find("\"tracking\"") == std::string::npos) continue;
    LivePose lp;
    double qw, qx, qy, qz, px, py, pz;
    const char* q = std::strstr(line.c_str(), "\"q\":[");
    const char* p = std::strstr(line.c_str(), "\"p\":[");
    if (std::sscanf(line.c_str(), "{\"frame_id\":%u,\"t\":%lf", &lp.frame_id,
                    &lp.t) != 2 ||
        q == nullptr || p == nullptr ||
        std::sscanf(q, "\"q\":[%lf,%lf,%lf,%lf]", &qw, &qx, &qy, &qz) != 4 ||
        std::sscanf(p, "\"p\":[%lf,%lf,%lf]", &px, &py, &pz) != 3) {
      continue;
    }
    lp.pose.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
    lp.pose.t = Eigen::Vector3d(px, py, pz);
    live_poses.push_back(lp);
  }
  out.have = true;
  out.tracked = static_cast<int>(live_poses.size());
  out.tracked_frac = pass_ids.empty()
                         ? 0.0
                         : static_cast<double>(live_poses.size()) /
                               static_cast<double>(pass_ids.size());

  // What this replay actually asked the tracker to follow, measured over
  // consecutive frames only. An exported session holds only STORED frames
  // (~3 fps) while on device the tracker sees every frame (~30 fps), so a
  // replay can be feeding ten times the inter-frame motion the device saw —
  // and a tracking result read without this line is not comparable to
  // on-device behavior.
  {
    double step_sum = 0, turn_sum = 0, dt_sum = 0;
    int n = 0;
    for (size_t i = 1; i < live_poses.size(); ++i) {
      if (live_poses[i].frame_id != live_poses[i - 1].frame_id + 1) continue;
      const double dt = live_poses[i].t - live_poses[i - 1].t;
      if (dt <= 0) continue;
      step_sum += (live_poses[i].pose.CameraCenter() -
                   live_poses[i - 1].pose.CameraCenter())
                      .norm();
      turn_sum += bs::RadToDeg(
          bs::AngularDistance(live_poses[i].pose.q, live_poses[i - 1].pose.q));
      dt_sum += dt;
      ++n;
    }
    if (n > 0 && dt_sum > 0) {
      std::printf(
          "%s motion: %.1f cm/frame, %.2f deg/frame at %.1f fps "
          "(%.2f m/s, %.0f deg/s)\n",
          label, 100.0 * step_sum / n, turn_sum / n, n / dt_sum,
          step_sum / dt_sum, turn_sum / dt_sum);
    }
  }

  auto find_gt = [&](uint32_t frame_id) -> std::optional<bs::SE3> {
    for (const auto& gp : gt.poses) {
      if (gp.frame_id == frame_id) {
        bs::SE3 pose;
        pose.q = Eigen::Quaterniond(gp.q[0], gp.q[1], gp.q[2], gp.q[3]);
        pose.t = Eigen::Vector3d(gp.t[0], gp.t[1], gp.t[2]);
        return pose;
      }
    }
    return std::nullopt;
  };

  if (live_poses.size() < 5) {
    std::printf("%s ATE: only %zu tracked poses\n", label, live_poses.size());
    return out;
  }

  // The live map's world frame is anchored at the first keyframe, so poses
  // compare through T_align = T_live_ref^-1 ∘ T_gt_ref: ATE of camera centers
  // after that SE3 alignment (scale comes from LiDAR and is NOT re-fit — a
  // scale error shows up as trajectory error, which is the point of the test).
  const auto gt_ref = find_gt(live_poses.front().frame_id);
  if (!gt_ref) return out;
  const bs::SE3 align = live_poses.front().pose.Inverse() * (*gt_ref);

  double ate_sq_sum = 0, rot_err_sum = 0;
  for (const auto& lp : live_poses) {
    const auto gt_pose = find_gt(lp.frame_id);
    if (!gt_pose) continue;
    const bs::SE3 gt_in_live = *gt_pose * align.Inverse();
    const double center_err =
        (lp.pose.CameraCenter() - gt_in_live.CameraCenter()).norm();
    ate_sq_sum += center_err * center_err;
    rot_err_sum += bs::RadToDeg(bs::AngularDistance(lp.pose.q, gt_in_live.q));
    ++out.compared;
  }
  if (out.compared == 0) return out;
  out.ate_rmse = std::sqrt(ate_sq_sum / out.compared);
  out.mean_rot = rot_err_sum / out.compared;
  std::printf(
      "%s ATE: %.1f%% tracked, RMSE %.3f m, mean rot err %.2f deg (%d poses)\n",
      label, 100.0 * out.tracked_frac, out.ate_rmse, out.mean_rot,
      out.compared);
  return out;
}

int RunLive(const bs::SessionReader& session, const std::string& config,
            bool check) {
  // Split the session by pass. A scout circuit runs first and writes the
  // scaffold; the capture pass then loads it — the same order the app uses,
  // so replay reproduces on-device behavior rather than approximating it.
  std::vector<uint32_t> scout_ids, capture_ids;
  for (const uint32_t frame_id : session.frame_ids()) {
    const auto meta = session.ReadMeta(frame_id);
    if (meta && meta->is_scout()) {
      scout_ids.push_back(frame_id);
    } else {
      capture_ids.push_back(frame_id);
    }
  }

  bs_live_status status{};
  if (!scout_ids.empty()) {
    bs_live_status scout_status{};
    if (FeedPass(session, config, BS_PASS_SCOUT, scout_ids, scout_status) != 0) {
      return 1;
    }
  }
  if (capture_ids.empty()) {
    std::fprintf(stderr, "session has no capture frames\n");
    return 1;
  }
  if (FeedPass(session, config, BS_PASS_CAPTURE, capture_ids, status) != 0) {
    return 1;
  }

  // Compare live poses against ground truth (synthetic sessions only).
  const auto gt = session.ReadGroundTruth();
  if (!gt) return 0;

  if (!scout_ids.empty()) {
    EvaluatePass(session, *gt, "poses_scout.jsonl", scout_ids, "scout");
  }
  const PassAccuracy live =
      EvaluatePass(session, *gt, "poses.jsonl", capture_ids, "live");
  if (!live.have) {
    std::fprintf(stderr, "missing live/poses.jsonl\n");
    return check ? 1 : 0;
  }
  if (live.compared == 0) return check ? 1 : 0;

  if (check) {
    if (live.tracked_frac < 0.7) {
      std::fprintf(stderr, "CHECK FAILED: tracked %.0f%% < 70%%\n",
                   100.0 * live.tracked_frac);
      return 1;
    }
    if (live.ate_rmse > 0.10) {
      std::fprintf(stderr, "CHECK FAILED: ATE %.3f m > 0.10 m\n",
                   live.ate_rmse);
      return 1;
    }
    if (live.mean_rot > 2.0) {
      std::fprintf(stderr, "CHECK FAILED: rot err %.2f deg > 2 deg\n",
                   live.mean_rot);
      return 1;
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 && std::strcmp(argv[1], "--version") == 0) {
    std::printf("bs_replay %s\n", bs_version());
    return 0;
  }
  if (argc >= 2 && std::strcmp(argv[1], "--selftest") == 0) {
    char buf[512];
    const bs_result r = bs_selftest(buf, sizeof(buf));
    std::printf("%s\n", buf);
    return r == BS_OK ? 0 : 1;
  }
  if (argc < 2 || argv[1][0] == '-') {
    PrintUsage();
    return 2;
  }

  const std::string session_dir = argv[1];
  bool do_info = false;
  bool do_live = false;
  bool do_two_view = false;
  bool do_final = false;
  bool check = false;
  int gap = 4;
  std::string preset = "quality";
  std::string config = "{}";
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--info") do_info = true;
    else if (arg == "--live") do_live = true;
    else if (arg == "--check") check = true;
    else if (arg == "--two-view") {
      do_two_view = true;
      if (i + 1 < argc && argv[i + 1][0] != '-') gap = std::atoi(argv[++i]);
    } else if (arg == "--final") {
      do_final = true;
      if (i + 1 < argc && argv[i + 1][0] != '-') preset = argv[++i];
    } else if (arg == "--config" && i + 1 < argc) {
      config = argv[++i];
    } else {
      std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
      PrintUsage();
      return 2;
    }
  }
  if (!do_info && !do_live && !do_two_view && !do_final) do_info = true;

  auto session = bs::SessionReader::Open(session_dir);
  if (!session) {
    std::fprintf(stderr, "cannot open session: %s\n", session_dir.c_str());
    return 1;
  }

  if (do_info) {
    const int rc = RunInfo(*session);
    if (rc != 0 || (!do_live && !do_two_view && !do_final)) return rc;
  }
  if (do_two_view) {
    const int rc = RunTwoView(*session, std::max(1, gap), check);
    if (rc != 0 || (!do_live && !do_final)) return rc;
  }
  if (do_live) {
    const int rc = RunLive(*session, config, check);
    if (rc != 0 || !do_final) return rc;
  }
  return RunFinal(*session, config, preset, check);
}
