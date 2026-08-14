#include "generate.h"

#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "common/config.h"
#include "common/geometry.h"
#include "io/session_writer.h"
#include "synth_scene.h"

namespace bs::synth {

namespace {

double LaplacianVariance(const cv::Mat& gray) {
  cv::Mat lap;
  cv::Laplacian(gray, lap, CV_64F);
  cv::Scalar mean, stddev;
  cv::meanStdDev(lap, mean, stddev);
  return stddev[0] * stddev[0];
}

// Frames needed to walk `shape`'s path at `speed_mps` without exceeding
// `pan_dps`. The trajectories are arc-length parameterized, so a dense
// sampling measures the true path once, cheaply, before anything renders.
int FramesForSpeed(const std::vector<SE3>& shape, double speed_mps,
                   double pan_dps, double fps) {
  const TrajectoryMotion m = MeasureMotion(shape);
  const double seconds = std::max(m.path_m / std::max(0.05, speed_mps),
                                  m.turn_deg / std::max(1.0, pan_dps));
  const long long n = std::llround(seconds * fps) + 1;
  return static_cast<int>(std::min<long long>(std::max<long long>(n, 2), 20000));
}

// Prints what the generated motion actually is, and says so plainly when it
// is not motion a held phone can produce. Silence here is what let a 9 m/s
// "walkthrough" masquerade as a tracker failure for an entire debugging run.
void ReportMotion(const char* label, const std::vector<SE3>& poses,
                  double fps) {
  const TrajectoryMotion m = MeasureMotion(poses);
  const double speed = m.mean_step_m * fps;
  const double rate = m.mean_turn_deg * fps;
  std::printf(
      "%s: %zu frames, %.1f m path -> %.1f cm/frame, %.2f deg/frame "
      "(%.2f m/s, %.0f deg/s at %.0f fps)\n",
      label, poses.size(), m.path_m, 100.0 * m.mean_step_m, m.mean_turn_deg,
      speed, rate, fps);

  const EngineConfig limits;  // the tracker's own plausibility bounds
  if (speed > limits.track_max_speed_mps || rate > limits.track_max_rot_dps) {
    std::printf(
        "  NOTE: this is store-gate cadence, not capture cadence — %.1f m/s "
        "at %.0f fps exceeds hand-held motion (%.1f m/s, %.0f deg/s), so the "
        "tracker's own motion gate rejects it. Fine as final-solve input "
        "(that stage sees only stored frames anyway); it cannot measure live "
        "tracking. Use --speed for that.\n",
        speed, fps, limits.track_max_speed_mps, limits.track_max_rot_dps);
  }
}

}  // namespace

bool GenerateSession(const GenerateOptions& o) {
  const double hfov = DegToRad(o.hfov_deg);
  Intrinsics K;
  K.width = o.image_width;
  K.height = o.image_height;
  K.fx = 0.5 * o.image_width / std::tan(hfov / 2.0);
  K.fy = K.fx;
  K.cx = 0.5 * (o.image_width - 1);
  K.cy = 0.5 * (o.image_height - 1);
  const Intrinsics Kd = K.ScaledTo(o.depth_width, o.depth_height);

  SessionInfo info;
  info.session_id = "synthetic_" + std::to_string(o.seed);
  info.created_utc = "2026-01-01T00:00:00Z";
  info.device_model = "synthetic";
  info.device_ios = "n/a";
  info.video_w = o.image_width;
  info.video_h = o.image_height;
  info.video_fps = static_cast<int>(std::lround(o.capture_fps));
  info.video_pixel_format = "bgr8-synth";
  info.depth_w = o.depth_width;
  info.depth_h = o.depth_height;
  info.depth_format = "f16-synth";
  info.depth_filtering = false;
  // The synthetic camera is photometrically fixed unless exposure drift is
  // requested, which is exactly what an AE lock would (or would not) give.
  info.ae_locked = o.exposure_drift == 0.0;
  info.awb_locked = true;
  info.regions = o.two_room
                     ? std::vector<RegionEntry>{{1, "Room 1", false},
                                                {2, "Room 2", false}}
                     : std::vector<RegionEntry>{{1, "Room 1", false}};
  info.app_version = "synth";

  CalibrationInfo calib;
  calib.ref_width = o.image_width;
  calib.ref_height = o.image_height;
  calib.intrinsics_session = {K.fx, K.fy, K.cx, K.cy, K.width, K.height};
  // Synthetic camera is distortion-free: empty LUT, exact PINHOLE model.
  calib.colmap_model = ColmapCameraModel{
      "PINHOLE", {K.fx, K.fy, K.cx, K.cy}, 0.0};

  SessionWriter writer;
  if (!SessionWriter::Create(o.out_dir, info, calib, writer)) return false;

  const Scene scene = o.two_room ? MakeTwoRoomScene(o.seed, o.blank_wall)
                                 : MakeRoomScene(o.seed, o.blank_wall);
  // Trajectory shapes are functions of frame count over a fixed path, so a
  // coarse pass measures the path and a second pass renders it at the rate
  // the physics asks for.
  // A declared rotation rate only means something if it binds per frame. The
  // scout circuit's average was a calm 0.7 deg/frame while rounding the
  // doorway corner at 4 — 120 deg/s, faster than new structure can be
  // triangulated, and enough to end tracking.
  const double turn_cap =
      o.speed_mps > 0.0 ? o.pan_dps / std::max(1.0, o.capture_fps) : 0.0;
  auto scout_shape = [&](int n) {
    return ScoutTrajectory(n, 1.5, o.seed ^ 0x5C0Fu, turn_cap);
  };
  auto capture_shape = [&](int n) {
    return o.two_room
               ? WalkthroughTrajectory(n, 1.5, o.seed ^ 0x7777u, turn_cap)
               : OrbitTrajectory(n, 1.6, 2.2, 1.5, o.seed ^ 0x7777u,
                                 o.sweep_deg, turn_cap);
  };

  std::vector<SE3> poses;
  // The scout circuit comes first, so the capture pass can localize against
  // the scaffold it leaves behind — the order is the whole point.
  int scout_count = o.two_room ? std::max(0, o.scout_frames) : 0;
  if (scout_count > 0) {
    if (o.speed_mps > 0.0) {
      scout_count =
          FramesForSpeed(scout_shape(256), o.speed_mps, o.pan_dps, o.capture_fps);
    }
    poses = scout_shape(scout_count);
    ReportMotion("scout", poses, o.capture_fps);
  }
  {
    const int capture_count =
        o.speed_mps > 0.0 ? FramesForSpeed(capture_shape(256), o.speed_mps,
                                           o.pan_dps, o.capture_fps)
                          : o.frame_count;
    const std::vector<SE3> capture = capture_shape(capture_count);
    ReportMotion("capture", capture, o.capture_fps);
    poses.insert(poses.end(), capture.begin(), capture.end());
  }
  const int total_frames = static_cast<int>(poses.size());

  DepthNoise noise;
  noise.sigma_base_m *= o.depth_noise_scale;
  noise.sigma_quadratic *= o.depth_noise_scale;
  if (o.depth_noise_scale <= 0.0) {
    noise.sigma_base_m = 0.0;
    noise.sigma_quadratic = 0.0;
    noise.dropout_frac = 0.0;
  }

  GroundTruth gt;
  const double dt = 1.0 / std::max(1.0, o.capture_fps);

  // Projects a world point into a world-to-camera pose (pixels).
  auto project = [&K](const SE3& p, const Eigen::Vector3d& X) {
    const Eigen::Vector3d xc = p.Apply(X);
    return Eigen::Vector2d(K.fx * xc.x() / xc.z() + K.cx,
                           K.fy * xc.y() / xc.z() + K.cy);
  };

  for (int i = 0; i < total_frames; ++i) {
    const uint32_t frame_id = static_cast<uint32_t>(i + 1);
    const SE3& pose = poses[i];
    const bool is_scout = i < scout_count;

    RenderOptions ropts;
    ropts.noise_sigma = static_cast<float>(1.6 * o.rgb_noise_scale);
    // Auto-exposure drift: a smooth deterministic gain swing per frame.
    const double gain =
        1.0 + o.exposure_drift * std::sin(0.35 * i + 0.5 * (o.seed & 7));
    ropts.gain = static_cast<float>(std::clamp(gain, 0.35, 1.7));
    // Motion blur from the apparent image motion of the scene-centre point
    // between this frame and the last, times a plausible exposure fraction.
    //
    // Deliberately a smooth function of speed, with no footfall or tremor
    // term. One was tried: a real walking capture does jolt at each step,
    // and the intent was to give frame-selection-by-sharpness something to
    // choose between. It did not — rendered blur moves this scene's
    // Laplacian variance by far less than its own texture does, so the
    // spread stayed at 1.2x either way — and its only measurable effect was
    // to break the hard-scene bound through an amplitude that was guessed
    // rather than measured off a device. See docs/ARCHITECTURE.md.
    if (o.motion_blur && i > 0 && i != scout_count) {
      const Eigen::Vector3d fwd =
          pose.Inverse().q * Eigen::Vector3d(0, 0, 1);
      const Eigen::Vector3d mid = pose.CameraCenter() + fwd * 2.5;
      const Eigen::Vector2d flow = project(pose, mid) - project(poses[i - 1], mid);
      const double len = std::clamp(flow.norm() * 0.5, 0.0, 20.0);
      ropts.motion_blur_px = static_cast<float>(len);
      ropts.motion_blur_angle = std::atan2(flow.y(), flow.x());
    }

    const cv::Mat img = RenderImage(scene, pose, K, ropts);
    const DepthImage depth =
        RenderDepth(scene, pose, Kd, noise, o.seed ^ (frame_id * 2654435761u));

    std::vector<uint8_t> jpeg;
    if (!cv::imencode(".jpg", img, jpeg,
                      {cv::IMWRITE_JPEG_QUALITY, o.jpeg_quality})) {
      return false;
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    FrameMeta meta;
    meta.frame_id = frame_id;
    meta.t_capture = i * dt;
    meta.t_depth = i * dt;
    meta.intrinsics = {K.fx, K.fy, K.cx, K.cy, K.width, K.height};
    meta.depth_intrinsics = {Kd.fx, Kd.fy, Kd.cx, Kd.cy, Kd.width, Kd.height};
    // Record the applied gain as exposure duration so a photometric
    // normalizer downstream has an honest signal to work from.
    meta.exposure = {ropts.gain / 120.0, 100.0, 0.0};
    meta.quality.lap_var = LaplacianVariance(gray);
    meta.quality.overexp_frac = 0.0;
    meta.is_keyframe = (o.keyframe_every > 0) && (i % o.keyframe_every == 0);
    meta.store_reason = meta.is_keyframe ? "kf" : "gate";
    meta.pass = is_scout ? "scout" : "capture";

    if (!writer.WriteFrame(meta, jpeg, depth)) return false;

    GroundTruthPose gtp;
    gtp.frame_id = frame_id;
    gtp.q[0] = pose.q.w();
    gtp.q[1] = pose.q.x();
    gtp.q[2] = pose.q.y();
    gtp.q[3] = pose.q.z();
    gtp.t[0] = pose.t.x();
    gtp.t[1] = pose.t.y();
    gtp.t[2] = pose.t.z();
    gt.poses.push_back(gtp);
  }

  if (!writer.WriteGroundTruth(gt)) return false;
  return writer.Finalize("2026-01-01T00:05:00Z");
}

}  // namespace bs::synth
