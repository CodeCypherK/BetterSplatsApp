#include "synth_scene.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include <opencv2/imgproc.hpp>

#include "io/float16.h"

namespace bs::synth {

namespace {

// Deterministic lattice hash -> [0,1).
inline float Hash01(int x, int y, uint32_t seed) {
  uint32_t h = static_cast<uint32_t>(x) * 73856093u ^
               static_cast<uint32_t>(y) * 19349663u ^ seed * 83492791u;
  h ^= h >> 13;
  h *= 0x85EBCA6Bu;
  h ^= h >> 16;
  return static_cast<float>(h & 0xFFFFFFu) / static_cast<float>(0x1000000u);
}

inline float SmoothStep(float t) { return t * t * (3.0f - 2.0f * t); }

// Bilinear value noise at frequency `freq` (cells per meter).
float ValueNoise(double u, double v, float freq, uint32_t seed) {
  const double su = u * freq;
  const double sv = v * freq;
  const int x0 = static_cast<int>(std::floor(su));
  const int y0 = static_cast<int>(std::floor(sv));
  const float tx = SmoothStep(static_cast<float>(su - x0));
  const float ty = SmoothStep(static_cast<float>(sv - y0));
  const float a = Hash01(x0, y0, seed);
  const float b = Hash01(x0 + 1, y0, seed);
  const float c = Hash01(x0, y0 + 1, seed);
  const float d = Hash01(x0 + 1, y0 + 1, seed);
  return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
}

// High-contrast speckle blobs: one jittered blob per lattice cell, smooth
// falloff. These create the corner-like structures feature detectors need.
float Speckle(double u, double v, float density, uint32_t seed) {
  const double su = u * density;
  const double sv = v * density;
  const int cx = static_cast<int>(std::floor(su));
  const int cy = static_cast<int>(std::floor(sv));
  float value = 0.0f;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const int gx = cx + dx;
      const int gy = cy + dy;
      const float jx = Hash01(gx, gy, seed ^ 0xA511u);
      const float jy = Hash01(gx, gy, seed ^ 0x5B3Du);
      const float r = 0.12f + 0.18f * Hash01(gx, gy, seed ^ 0xC0DEu);
      const float sign = Hash01(gx, gy, seed ^ 0xF00Du) < 0.5f ? -1.0f : 1.0f;
      const float px = static_cast<float>(su - gx) - jx;
      const float py = static_cast<float>(sv - gy) - jy;
      const float d2 = px * px + py * py;
      if (d2 < r * r) {
        const float fall = 1.0f - SmoothStep(std::sqrt(d2) / r);
        value += sign * fall;
      }
    }
  }
  return std::clamp(value, -1.0f, 1.0f);
}

}  // namespace

float TextureValue(const TexturedPlane& plane, double u, double v) {
  if (plane.texture_amount <= 0.0f) return 0.5f;
  const uint32_t s = plane.texture_seed;
  float value = 0.5f;
  value += 0.20f * (ValueNoise(u, v, 3.1f, s ^ 1) - 0.5f);
  value += 0.14f * (ValueNoise(u, v, 11.7f, s ^ 2) - 0.5f);
  value += 0.08f * (ValueNoise(u, v, 31.0f, s ^ 3) - 0.5f);
  value += 0.30f * Speckle(u, v, 14.0f, s ^ 4);
  value += 0.18f * Speckle(u, v, 4.5f, s ^ 5);
  return 0.5f + plane.texture_amount * (std::clamp(value, 0.0f, 1.0f) - 0.5f);
}

Scene MakeRoomScene(uint32_t seed, bool blank_wall) {
  // Room extents: x in [-3, 3], z in [-4, 4], y in [0, 2.5] (Y up).
  Scene scene;
  const double W = 6.0, D = 8.0, H = 2.5;
  const Eigen::Vector3d x0(-W / 2, 0, -D / 2);

  auto add = [&](const Eigen::Vector3d& origin, const Eigen::Vector3d& eu,
                 const Eigen::Vector3d& ev, cv::Vec3f color, float tex,
                 uint32_t tseed) {
    scene.planes.push_back({origin, eu, ev, color, tex, tseed});
  };

  // Floor (wood-ish) and ceiling (flat white, low texture).
  add(x0, {W, 0, 0}, {0, 0, D}, {0.35f, 0.52f, 0.62f}, 0.65f, seed ^ 11);
  add({-W / 2, H, -D / 2}, {W, 0, 0}, {0, 0, D}, {0.92f, 0.92f, 0.92f}, 0.10f,
      seed ^ 22);
  // Back wall (z = -D/2): posters/shelves — strongly textured.
  add({-W / 2, 0, -D / 2}, {W, 0, 0}, {0, H, 0}, {0.55f, 0.68f, 0.80f}, 0.85f,
      seed ^ 33);
  // Front wall (z = +D/2).
  add({W / 2, 0, D / 2}, {-W, 0, 0}, {0, H, 0}, {0.62f, 0.72f, 0.78f}, 0.70f,
      seed ^ 44);
  // Left wall (x = -W/2): the (nearly) blank painted wall.
  add({-W / 2, 0, D / 2}, {0, 0, -D}, {0, H, 0}, {0.82f, 0.84f, 0.86f},
      blank_wall ? 0.04f : 0.6f, seed ^ 55);
  // Right wall (x = +W/2).
  add({W / 2, 0, -D / 2}, {0, 0, D}, {0, H, 0}, {0.60f, 0.75f, 0.82f}, 0.75f,
      seed ^ 66);

  // Interior box 1: "sideboard" against the back wall.
  const Eigen::Vector3d b1(-1.4, 0.0, -3.2);
  const double b1w = 1.6, b1h = 0.9, b1d = 0.5;
  add(b1, {b1w, 0, 0}, {0, b1h, 0}, {0.25f, 0.35f, 0.55f}, 0.8f, seed ^ 77);
  add({b1.x(), 0, b1.z() + b1d}, {b1w, 0, 0}, {0, b1h, 0}, {0.25f, 0.35f, 0.55f},
      0.8f, seed ^ 78);
  add({b1.x(), b1h, b1.z()}, {b1w, 0, 0}, {0, 0, b1d}, {0.30f, 0.40f, 0.60f},
      0.75f, seed ^ 79);
  add({b1.x(), 0, b1.z()}, {0, 0, b1d}, {0, b1h, 0}, {0.22f, 0.32f, 0.52f}, 0.8f,
      seed ^ 80);
  add({b1.x() + b1w, 0, b1.z()}, {0, 0, b1d}, {0, b1h, 0}, {0.22f, 0.32f, 0.52f},
      0.8f, seed ^ 81);

  // Interior box 2: "table" mid-room.
  const Eigen::Vector3d b2(0.6, 0.0, 0.8);
  const double b2w = 1.2, b2h = 0.75, b2d = 0.8;
  add({b2.x(), b2h, b2.z()}, {b2w, 0, 0}, {0, 0, b2d}, {0.45f, 0.58f, 0.70f},
      0.85f, seed ^ 91);
  add(b2, {b2w, 0, 0}, {0, b2h, 0}, {0.40f, 0.50f, 0.62f}, 0.7f, seed ^ 92);
  add({b2.x(), 0, b2.z() + b2d}, {b2w, 0, 0}, {0, b2h, 0}, {0.40f, 0.50f, 0.62f},
      0.7f, seed ^ 93);
  add({b2.x(), 0, b2.z()}, {0, 0, b2d}, {0, b2h, 0}, {0.38f, 0.48f, 0.60f}, 0.7f,
      seed ^ 94);
  add({b2.x() + b2w, 0, b2.z()}, {0, 0, b2d}, {0, b2h, 0}, {0.38f, 0.48f, 0.60f},
      0.7f, seed ^ 95);

  return scene;
}

Scene MakeTwoRoomScene(uint32_t seed, bool blank_wall) {
  Scene scene;
  const double H = 2.5;
  const double z0 = -4.0, z1 = 4.0, D = z1 - z0;
  const double ax0 = -3.0, ax1 = 3.0;  // room A
  const double bx0 = 3.0, bx1 = 9.0;   // room B (shares the wall at x = 3)
  const double door_half = 0.55, door_h = 2.1;

  auto add = [&](const Eigen::Vector3d& origin, const Eigen::Vector3d& eu,
                 const Eigen::Vector3d& ev, cv::Vec3f color, float tex,
                 uint32_t tseed) {
    scene.planes.push_back({origin, eu, ev, color, tex, tseed});
  };
  // Axis-aligned box: five visible faces (no underside), one texture seed
  // family so the box reads as a single object across views.
  auto add_box = [&](const Eigen::Vector3d& o, double w, double h, double d,
                     cv::Vec3f color, float tex, uint32_t tseed) {
    add({o.x(), o.y() + h, o.z()}, {w, 0, 0}, {0, 0, d}, color, tex, tseed);
    add(o, {w, 0, 0}, {0, h, 0}, color * 0.9f, tex, tseed ^ 1);
    add({o.x(), o.y(), o.z() + d}, {w, 0, 0}, {0, h, 0}, color * 0.9f, tex,
        tseed ^ 2);
    add(o, {0, 0, d}, {0, h, 0}, color * 0.8f, tex, tseed ^ 3);
    add({o.x() + w, o.y(), o.z()}, {0, 0, d}, {0, h, 0}, color * 0.8f, tex,
        tseed ^ 4);
  };

  // --- room A: floor, ceiling, outer walls ---
  add({ax0, 0, z0}, {ax1 - ax0, 0, 0}, {0, 0, D}, {0.35f, 0.52f, 0.62f}, 0.65f,
      seed ^ 11);
  add({ax0, H, z0}, {ax1 - ax0, 0, 0}, {0, 0, D}, {0.92f, 0.92f, 0.92f}, 0.10f,
      seed ^ 12);
  add({ax0, 0, z0}, {ax1 - ax0, 0, 0}, {0, H, 0}, {0.55f, 0.68f, 0.80f}, 0.85f,
      seed ^ 13);
  add({ax1, 0, z1}, {ax0 - ax1, 0, 0}, {0, H, 0}, {0.62f, 0.72f, 0.78f}, 0.70f,
      seed ^ 14);
  // The (nearly) blank painted wall — LiDAR must carry this surface.
  add({ax0, 0, z1}, {0, 0, -D}, {0, H, 0}, {0.82f, 0.84f, 0.86f},
      blank_wall ? 0.04f : 0.6f, seed ^ 15);

  // --- room B: floor, ceiling, outer walls ---
  add({bx0, 0, z0}, {bx1 - bx0, 0, 0}, {0, 0, D}, {0.40f, 0.48f, 0.58f}, 0.60f,
      seed ^ 21);
  add({bx0, H, z0}, {bx1 - bx0, 0, 0}, {0, 0, D}, {0.90f, 0.91f, 0.92f}, 0.12f,
      seed ^ 22);
  add({bx0, 0, z0}, {bx1 - bx0, 0, 0}, {0, H, 0}, {0.66f, 0.70f, 0.62f}, 0.80f,
      seed ^ 23);
  add({bx1, 0, z1}, {bx0 - bx1, 0, 0}, {0, H, 0}, {0.70f, 0.66f, 0.60f}, 0.75f,
      seed ^ 24);
  add({bx1, 0, z0}, {0, 0, D}, {0, H, 0}, {0.58f, 0.74f, 0.80f}, 0.78f,
      seed ^ 25);

  // --- dividing wall at x = 3, split around an open doorway ---
  add({ax1, 0, z0}, {0, 0, -door_half - z0}, {0, H, 0}, {0.78f, 0.80f, 0.82f},
      0.45f, seed ^ 31);
  add({ax1, 0, door_half}, {0, 0, z1 - door_half}, {0, H, 0},
      {0.78f, 0.80f, 0.82f}, 0.45f, seed ^ 32);
  add({ax1, door_h, -door_half}, {0, 0, 2 * door_half}, {0, H - door_h, 0},
      {0.76f, 0.78f, 0.80f}, 0.40f, seed ^ 33);

  // --- furniture, so both rooms carry parallax-rich near geometry ---
  add_box({-1.4, 0.0, -3.2}, 1.6, 0.9, 0.5, {0.25f, 0.35f, 0.55f}, 0.80f,
          seed ^ 77);
  add_box({0.6, 0.0, 0.8}, 1.2, 0.75, 0.8, {0.45f, 0.58f, 0.70f}, 0.85f,
          seed ^ 91);
  add_box({4.6, 0.0, -3.0}, 1.4, 1.1, 0.6, {0.52f, 0.42f, 0.34f}, 0.82f,
          seed ^ 101);
  add_box({6.4, 0.0, 1.4}, 1.0, 0.55, 1.0, {0.34f, 0.46f, 0.40f}, 0.78f,
          seed ^ 111);

  return scene;
}

RayHit CastRay(const Scene& scene, const Eigen::Vector3d& center,
               const Eigen::Vector3d& dir_world) {
  RayHit best;
  best.t = std::numeric_limits<double>::max();
  for (size_t i = 0; i < scene.planes.size(); ++i) {
    const auto& p = scene.planes[i];
    const Eigen::Vector3d n = p.edge_u.cross(p.edge_v);
    const double denom = dir_world.dot(n);
    if (std::abs(denom) < 1e-12) continue;
    const double t = (p.origin - center).dot(n) / denom;
    if (t <= 1e-6 || t >= best.t) continue;
    const Eigen::Vector3d hit = center + t * dir_world;
    const Eigen::Vector3d rel = hit - p.origin;
    const double lu2 = p.edge_u.squaredNorm();
    const double lv2 = p.edge_v.squaredNorm();
    const double a = rel.dot(p.edge_u) / lu2;
    const double b = rel.dot(p.edge_v) / lv2;
    if (a < 0.0 || a > 1.0 || b < 0.0 || b > 1.0) continue;
    best.t = t;
    best.plane_index = static_cast<int>(i);
    best.u = a * std::sqrt(lu2);
    best.v = b * std::sqrt(lv2);
  }
  if (best.plane_index < 0) best.t = -1.0;
  return best;
}

namespace {

// Linear (directional) motion-blur kernel of the given length and angle.
cv::Mat MotionBlurKernel(double length_px, double angle_rad) {
  const int n = std::max(3, static_cast<int>(std::round(length_px)) | 1);  // odd
  cv::Mat k = cv::Mat::zeros(n, n, CV_32F);
  const double c = n / 2.0;
  const double dx = std::cos(angle_rad), dy = std::sin(angle_rad);
  const int steps = n * 2;
  for (int s = 0; s <= steps; ++s) {
    const double t = static_cast<double>(s) / steps - 0.5;  // [-0.5, 0.5]
    const int x = static_cast<int>(std::round(c + t * (n - 1) * dx));
    const int y = static_cast<int>(std::round(c + t * (n - 1) * dy));
    if (x >= 0 && x < n && y >= 0 && y < n) k.at<float>(y, x) += 1.0f;
  }
  const double sum = cv::sum(k)[0];
  if (sum > 0) k /= sum;
  return k;
}

// Rotates `from` toward `to` by at most `max_deg`.
Eigen::Vector3d SlewToward(const Eigen::Vector3d& from,
                           const Eigen::Vector3d& to, double max_deg) {
  const Eigen::Vector3d a = from.normalized();
  const Eigen::Vector3d b = to.normalized();
  const double angle = RadToDeg(std::acos(std::clamp(a.dot(b), -1.0, 1.0)));
  if (angle <= max_deg || angle < 1e-9) return b;
  const Eigen::Quaterniond full = Eigen::Quaterniond::FromTwoVectors(a, b);
  const Eigen::Quaterniond step =
      Eigen::Quaterniond::Identity().slerp(max_deg / angle, full);
  return (step * a).normalized();
}

// Builds world-to-camera poses from positions and the direction the camera
// WANTS to face, rate-limited so no single frame turns further than a hand
// could. Trajectories express intent ("look at the centre of the room you
// are standing in"); when that intent switches discretely — crossing a
// doorway from one room to the next — the raw result is a single-frame 174
// deg flip. It is physically impossible, it ends tracking outright, and it
// is invisible in mean or 95th-percentile motion. The cap is the larger of a
// fixed hand-held rate and twice the sequence's own average turn, so it
// clips discontinuities without reshaping intended motion (and leaves short,
// deliberately coarse fixtures alone).
std::vector<SE3> PosesFromLookDirections(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<Eigen::Vector3d>& desired, double cap_deg) {
  const size_t n = positions.size();
  std::vector<SE3> poses;
  poses.reserve(n);
  if (n == 0) return poses;

  double turn_sum = 0;
  for (size_t i = 1; i < n; ++i) {
    turn_sum += RadToDeg(std::acos(std::clamp(
        desired[i].normalized().dot(desired[i - 1].normalized()), -1.0, 1.0)));
  }
  const double mean_turn = n > 1 ? turn_sum / static_cast<double>(n - 1) : 0.0;
  // A caller-declared rate wins outright. Without one, fall back to a rate
  // that clips discontinuities without reshaping intended motion (and leaves
  // short, deliberately coarse fixtures alone).
  const double max_turn =
      cap_deg > 0.0 ? cap_deg : std::max(4.0, 2.0 * mean_turn);

  Eigen::Vector3d forward = desired[0].normalized();
  for (size_t i = 0; i < n; ++i) {
    forward = i == 0 ? forward : SlewToward(forward, desired[i], max_turn);
    const Eigen::Vector3d world_up(0, 1, 0);
    // CV camera basis (+X right, +Y down, +Z forward), det(R) = +1.
    const Eigen::Vector3d right = forward.cross(world_up).normalized();
    const Eigen::Vector3d down = forward.cross(right).normalized();
    Eigen::Matrix3d R_cw;  // camera-to-world: columns are camera axes in world
    R_cw.col(0) = right;
    R_cw.col(1) = down;
    R_cw.col(2) = forward;
    poses.push_back(
        SE3::FromCamToWorld(Eigen::Quaterniond(R_cw), positions[i]));
  }
  return poses;
}

}  // namespace

cv::Mat RenderImage(const Scene& scene, const SE3& pose, const Intrinsics& K,
                    const RenderOptions& opts) {
  cv::Mat img(K.height, K.width, CV_8UC3, cv::Scalar(18, 16, 14));
  const SE3 cam_to_world = pose.Inverse();
  const Eigen::Vector3d center = pose.CameraCenter();
  const Eigen::Matrix3d R_cw = cam_to_world.q.toRotationMatrix();

  for (int y = 0; y < K.height; ++y) {
    cv::Vec3b* row = img.ptr<cv::Vec3b>(y);
    for (int x = 0; x < K.width; ++x) {
      const Eigen::Vector3d dir_cam((x - K.cx) / K.fx, (y - K.cy) / K.fy, 1.0);
      const RayHit hit = CastRay(scene, center, R_cw * dir_cam);
      if (hit.plane_index < 0) continue;
      const auto& plane = scene.planes[hit.plane_index];
      const float tex = TextureValue(plane, hit.u, hit.v);
      // Simple distance shading keeps far surfaces slightly darker.
      const float shade = std::clamp(1.15f - 0.05f * static_cast<float>(hit.t),
                                     0.55f, 1.0f);
      for (int c = 0; c < 3; ++c) {
        const float value = 255.0f * plane.base_color[c] * (0.45f + 1.1f * tex) *
                            shade * opts.gain;
        row[x][c] = static_cast<uint8_t>(std::clamp(value, 0.0f, 255.0f));
      }
    }
  }

  // Motion blur (directional) before the optical PSF, matching physics.
  if (opts.motion_blur_px >= 1.0f) {
    const cv::Mat k = MotionBlurKernel(opts.motion_blur_px, opts.motion_blur_angle);
    cv::filter2D(img, img, -1, k, cv::Point(-1, -1), 0, cv::BORDER_REFLECT);
  }

  // Mild optical blur + sensor noise for realism (deterministic noise).
  cv::GaussianBlur(img, img, cv::Size(3, 3), opts.blur_sigma);
  cv::Mat noise(img.size(), CV_16SC3);
  cv::RNG rng(0xBEEF ^ static_cast<uint64_t>(K.width) ^
              static_cast<uint64_t>(pose.t.norm() * 1e6));
  rng.fill(noise, cv::RNG::NORMAL, 0, opts.noise_sigma);
  cv::Mat img16;
  img.convertTo(img16, CV_16SC3);
  img16 += noise;
  img16.convertTo(img, CV_8UC3);
  return img;
}

DepthImage RenderDepth(const Scene& scene, const SE3& pose, const Intrinsics& K,
                       const DepthNoise& noise, uint32_t seed) {
  DepthImage depth;
  depth.width = K.width;
  depth.height = K.height;
  depth.f16.assign(static_cast<size_t>(K.width) * K.height, 0);

  const SE3 cam_to_world = pose.Inverse();
  const Eigen::Vector3d center = pose.CameraCenter();
  const Eigen::Matrix3d R_cw = cam_to_world.q.toRotationMatrix();
  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  const uint16_t nan16 = F32ToF16(std::numeric_limits<float>::quiet_NaN());

  for (int y = 0; y < K.height; ++y) {
    for (int x = 0; x < K.width; ++x) {
      const size_t idx = static_cast<size_t>(y) * K.width + x;
      const Eigen::Vector3d dir_cam((x - K.cx) / K.fx, (y - K.cy) / K.fy, 1.0);
      const Eigen::Vector3d dir_world = R_cw * dir_cam;
      const RayHit hit = CastRay(scene, center, dir_world);
      if (hit.plane_index < 0 || hit.t > noise.max_range_m) {
        depth.f16[idx] = nan16;
        continue;
      }
      // Grazing-angle dropout mimics LiDAR failure on oblique surfaces.
      const auto& plane = scene.planes[hit.plane_index];
      const Eigen::Vector3d n = plane.edge_u.cross(plane.edge_v).normalized();
      const double cos_inc =
          std::abs(n.dot(dir_world.normalized()));
      if (cos_inc < noise.grazing_dropout_cos || uni(rng) < noise.dropout_frac) {
        depth.f16[idx] = nan16;
        continue;
      }
      const double sigma =
          noise.sigma_base_m + noise.sigma_quadratic * hit.t * hit.t;
      const double d = hit.t + sigma * gauss(rng);
      depth.f16[idx] = F32ToF16(static_cast<float>(std::max(0.05, d)));
    }
  }
  return depth;
}

std::vector<SE3> OrbitTrajectory(int frame_count, double radius_x, double radius_z,
                                 double eye_height, uint32_t seed,
                                 double sweep_deg, double max_turn_deg) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);

  // Smooth per-frame jitter accumulated as a slow random walk (handheld feel).
  double jitter_y = 0.0, jitter_heading = 0.0;

  std::vector<Eigen::Vector3d> positions, forwards;
  positions.reserve(frame_count);
  forwards.reserve(frame_count);
  for (int i = 0; i < frame_count; ++i) {
    const double s = static_cast<double>(i) / std::max(1, frame_count - 1);
    const double angle = DegToRad(sweep_deg) * s;
    jitter_y = 0.97 * jitter_y + 0.004 * gauss(rng);
    jitter_heading = 0.97 * jitter_heading + 0.003 * gauss(rng);

    const Eigen::Vector3d position(radius_x * std::sin(angle),
                                   eye_height + jitter_y,
                                   radius_z * std::cos(angle));
    // Look outward past the room center for wall coverage, with heading jitter.
    Eigen::Vector3d target(1.8 * radius_x * std::sin(angle + 0.15 + jitter_heading),
                           eye_height * 0.75,
                           1.8 * radius_z * std::cos(angle + 0.15 + jitter_heading));

    positions.push_back(position);
    forwards.push_back((target - position).normalized());
  }
  return PosesFromLookDirections(positions, forwards, max_turn_deg);
}

std::vector<SE3> ScoutTrajectory(int frame_count, double eye_height,
                                 uint32_t seed, double max_turn_deg) {
  // Hug the perimeter of both rooms, always looking across the space rather
  // than along the wall behind you. The look target is the centre of
  // whichever room you are currently in, so every frame sees a whole room.
  const std::vector<Eigen::Vector3d> waypoints = {
      {-2.3, 0, 3.3},   // room A, front-left
      {-2.3, 0, -3.3},  // room A, back-left  (facing the blank wall's room)
      {2.3, 0, -3.3},   // room A, back-right
      {2.6, 0, -0.2},   // toward the doorway
      {4.0, 0, 0.0},    // through it
      {5.0, 0, -3.3},   // room B, back-left
      {8.3, 0, -3.3},   // room B, back-right
      {8.3, 0, 3.3},    // room B, front-right
      {5.0, 0, 3.3},    // room B, front-left
      {4.0, 0, 0.2},    // back through the doorway
      {2.3, 0, 3.3},    // room A, front-right
      {-2.3, 0, 3.3},   // home
  };

  std::vector<double> arc(waypoints.size(), 0.0);
  for (size_t i = 1; i < waypoints.size(); ++i) {
    arc[i] = arc[i - 1] + (waypoints[i] - waypoints[i - 1]).norm();
  }
  const double total = arc.back();

  auto sample = [&](double s) {
    const double target = std::clamp(s, 0.0, 1.0) * total;
    size_t seg = 1;
    while (seg + 1 < arc.size() && arc[seg] < target) ++seg;
    const double span = std::max(1e-9, arc[seg] - arc[seg - 1]);
    const double f = std::clamp((target - arc[seg - 1]) / span, 0.0, 1.0);
    return waypoints[seg - 1] + f * (waypoints[seg] - waypoints[seg - 1]);
  };

  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  double jitter_y = 0.0;

  std::vector<Eigen::Vector3d> positions, forwards;
  positions.reserve(frame_count);
  forwards.reserve(frame_count);
  for (int i = 0; i < frame_count; ++i) {
    const double s = static_cast<double>(i) / std::max(1, frame_count - 1);
    jitter_y = 0.97 * jitter_y + 0.004 * gauss(rng);
    Eigen::Vector3d position = sample(s);
    position.y() = eye_height + jitter_y;

    // Aim at the centre of the room we are standing in — that is what
    // "back to the wall, scanning inward" produces. Across the doorway the
    // two centres are BLENDED rather than switched: picking whichever room
    // contains the camera swung the view 174 deg between two adjacent frames
    // at x = 3, which read as a tracker failure for a long time and was
    // really a teleporting camera.
    const double blend =
        std::clamp((position.x() - 2.0) / 2.0, 0.0, 1.0);  // x 2->4 m
    const Eigen::Vector3d room_centre(6.0 * blend, eye_height * 0.8, 0.0);
    const Eigen::Vector3d to_centre = room_centre - position;

    // In the doorway that blended centre passes through the camera itself.
    // Looking at a point you are standing on aims the lens at the floor and
    // leaves the roll ill-conditioned — a 4 deg/frame change of view
    // direction came out as a 10 deg/frame change of pose. Close in, the
    // view follows travel instead, which is what walking through a door
    // actually looks like.
    Eigen::Vector3d travel = sample(std::min(1.0, s + 0.01)) - position;
    travel.y() = 0;
    if (travel.norm() < 1e-6) travel = Eigen::Vector3d(1, 0, 0);
    const double weight =
        std::clamp((to_centre.norm() - 0.8) / 1.2, 0.0, 1.0);  // 0.8 -> 2.0 m
    Eigen::Vector3d forward = weight * to_centre.normalized() +
                              (1.0 - weight) * travel.normalized();
    if (forward.norm() < 1e-6) forward = travel.normalized();

    positions.push_back(position);
    forwards.push_back(forward.normalized());
  }
  return PosesFromLookDirections(positions, forwards, max_turn_deg);
}

std::vector<SE3> WalkthroughTrajectory(int frame_count, double eye_height,
                                       uint32_t seed, double max_turn_deg) {
  // Closed loop: sweep room A, through the doorway, around room B, back
  // through the doorway, and home to the starting viewpoint. Ending where it
  // began is the point — that revisit is what loop closure must recognize,
  // and the drift accumulated around the loop is what it must absorb.
  const std::vector<Eigen::Vector3d> waypoints = {
      {0.0, 0, 2.4},    // start, room A
      {-1.8, 0, 0.6},   // toward the blank left wall
      {-1.6, 0, -2.6},  // A back-left corner
      {1.6, 0, -2.6},   // A back-right corner
      {2.2, 0, -0.4},   // approach the doorway
      {4.0, 0, 0.0},    // through into room B
      {5.2, 0, -2.6},   // B back-left
      {7.8, 0, -2.4},   // B back-right
      {7.8, 0, 2.4},    // B front-right
      {5.0, 0, 2.2},    // B front-left
      {4.0, 0, 0.2},    // back to the doorway
      {2.0, 0, 0.4},    // back into room A
      {0.0, 0, 2.4},    // home — same viewpoint as frame 0
  };

  // Cumulative arc length, so frames are spaced by distance travelled rather
  // than by waypoint index (constant walking speed).
  std::vector<double> arc(waypoints.size(), 0.0);
  for (size_t i = 1; i < waypoints.size(); ++i) {
    arc[i] = arc[i - 1] + (waypoints[i] - waypoints[i - 1]).norm();
  }
  const double total = arc.back();

  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  double jitter_y = 0.0;

  auto sample = [&](double s) {  // position at normalized arc length
    const double target = std::clamp(s, 0.0, 1.0) * total;
    size_t seg = 1;
    while (seg + 1 < arc.size() && arc[seg] < target) ++seg;
    const double span = std::max(1e-9, arc[seg] - arc[seg - 1]);
    const double f = std::clamp((target - arc[seg - 1]) / span, 0.0, 1.0);
    return waypoints[seg - 1] + f * (waypoints[seg] - waypoints[seg - 1]);
  };

  std::vector<Eigen::Vector3d> positions, forwards;
  positions.reserve(frame_count);
  forwards.reserve(frame_count);
  Eigen::Vector3d last_ahead(0, 0, -1);
  for (int i = 0; i < frame_count; ++i) {
    const double s = static_cast<double>(i) / std::max(1, frame_count - 1);
    jitter_y = 0.97 * jitter_y + 0.004 * gauss(rng);

    Eigen::Vector3d position = sample(s);
    position.y() = eye_height + jitter_y;

    // Heading follows travel; a slow yaw sweep covers the walls to either
    // side the way a person turns their phone while walking a space. At the
    // very end the forward difference vanishes, so travel direction carries
    // over instead of snapping to a fixed axis — that fallback used to spin
    // the last frame of the loop by 134 deg.
    Eigen::Vector3d ahead = sample(std::min(1.0, s + 0.02)) - sample(s);
    ahead.y() = 0;
    if (ahead.norm() < 1e-6) {
      ahead = last_ahead;
    } else {
      ahead.normalize();
      last_ahead = ahead;
    }
    const double yaw = DegToRad(38.0) * std::sin(2.0 * M_PI * 2.5 * s);
    const Eigen::Vector3d dir(
        ahead.x() * std::cos(yaw) - ahead.z() * std::sin(yaw), 0.0,
        ahead.x() * std::sin(yaw) + ahead.z() * std::cos(yaw));
    const Eigen::Vector3d target =
        position + 3.0 * dir - Eigen::Vector3d(0, eye_height * 0.22, 0);

    positions.push_back(position);
    forwards.push_back((target - position).normalized());
  }
  return PosesFromLookDirections(positions, forwards, max_turn_deg);
}

TrajectoryMotion MeasureMotion(const std::vector<SE3>& poses) {
  TrajectoryMotion m;
  if (poses.size() < 2) return m;

  std::vector<double> steps, turns;
  steps.reserve(poses.size() - 1);
  turns.reserve(poses.size() - 1);
  for (size_t i = 1; i < poses.size(); ++i) {
    steps.push_back(
        (poses[i].CameraCenter() - poses[i - 1].CameraCenter()).norm());
    turns.push_back(RadToDeg(
        poses[i].q.angularDistance(poses[i - 1].q)));
    m.path_m += steps.back();
    m.turn_deg += turns.back();
  }
  m.mean_step_m = m.path_m / static_cast<double>(steps.size());
  m.mean_turn_deg = m.turn_deg / static_cast<double>(turns.size());

  m.max_step_m = *std::max_element(steps.begin(), steps.end());
  m.max_turn_deg = *std::max_element(turns.begin(), turns.end());

  auto percentile = [](std::vector<double>& v, double p) {
    std::sort(v.begin(), v.end());
    const size_t i = std::min(
        v.size() - 1,
        static_cast<size_t>(p * static_cast<double>(v.size() - 1) + 0.5));
    return v[i];
  };
  m.p95_step_m = percentile(steps, 0.95);
  m.p95_turn_deg = percentile(turns, 0.95);
  return m;
}

}  // namespace bs::synth
