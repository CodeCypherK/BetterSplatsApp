#include "generate.h"

#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

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
  info.video_fps = 30;
  info.video_pixel_format = "bgr8-synth";
  info.depth_w = o.depth_width;
  info.depth_h = o.depth_height;
  info.depth_format = "f16-synth";
  info.depth_filtering = false;
  // The synthetic camera is photometrically fixed unless exposure drift is
  // requested, which is exactly what an AE lock would (or would not) give.
  info.ae_locked = o.exposure_drift == 0.0;
  info.awb_locked = true;
  info.regions = {{1, "Room 1", false}};
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

  const Scene scene = MakeRoomScene(o.seed, o.blank_wall);
  const std::vector<SE3> poses = OrbitTrajectory(
      o.frame_count, 1.6, 2.2, 1.5, o.seed ^ 0x7777u, o.sweep_deg);

  DepthNoise noise;
  noise.sigma_base_m *= o.depth_noise_scale;
  noise.sigma_quadratic *= o.depth_noise_scale;
  if (o.depth_noise_scale <= 0.0) {
    noise.sigma_base_m = 0.0;
    noise.sigma_quadratic = 0.0;
    noise.dropout_frac = 0.0;
  }

  GroundTruth gt;
  const double dt = 1.0 / 30.0;

  // Projects a world point into a world-to-camera pose (pixels).
  auto project = [&K](const SE3& p, const Eigen::Vector3d& X) {
    const Eigen::Vector3d xc = p.Apply(X);
    return Eigen::Vector2d(K.fx * xc.x() / xc.z() + K.cx,
                           K.fy * xc.y() / xc.z() + K.cy);
  };

  for (int i = 0; i < o.frame_count; ++i) {
    const uint32_t frame_id = static_cast<uint32_t>(i + 1);
    const SE3& pose = poses[i];

    RenderOptions ropts;
    ropts.noise_sigma = static_cast<float>(1.6 * o.rgb_noise_scale);
    // Auto-exposure drift: a smooth deterministic gain swing per frame.
    const double gain =
        1.0 + o.exposure_drift * std::sin(0.35 * i + 0.5 * (o.seed & 7));
    ropts.gain = static_cast<float>(std::clamp(gain, 0.35, 1.7));
    // Motion blur from the apparent image motion of the scene-centre point
    // between this frame and the last, times a plausible exposure fraction.
    if (o.motion_blur && i > 0) {
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
