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

cv::Mat RenderImage(const Scene& scene, const SE3& pose, const Intrinsics& K) {
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
        const float value =
            255.0f * plane.base_color[c] * (0.45f + 1.1f * tex) * shade;
        row[x][c] = static_cast<uint8_t>(std::clamp(value, 0.0f, 255.0f));
      }
    }
  }

  // Mild optical blur + sensor noise for realism (deterministic noise).
  cv::GaussianBlur(img, img, cv::Size(3, 3), 0.6);
  cv::Mat noise(img.size(), CV_16SC3);
  cv::RNG rng(0xBEEF ^ static_cast<uint64_t>(K.width) ^
              static_cast<uint64_t>(pose.t.norm() * 1e6));
  rng.fill(noise, cv::RNG::NORMAL, 0, 1.6);
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
                                 double eye_height, uint32_t seed) {
  std::vector<SE3> poses;
  poses.reserve(frame_count);
  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);

  // Smooth per-frame jitter accumulated as a slow random walk (handheld feel).
  double jitter_y = 0.0, jitter_heading = 0.0;

  for (int i = 0; i < frame_count; ++i) {
    const double s = static_cast<double>(i) / std::max(1, frame_count - 1);
    const double angle = 2.0 * M_PI * 0.85 * s;  // 85% of a full orbit
    jitter_y = 0.97 * jitter_y + 0.004 * gauss(rng);
    jitter_heading = 0.97 * jitter_heading + 0.006 * gauss(rng);

    const Eigen::Vector3d position(radius_x * std::sin(angle),
                                   eye_height + jitter_y,
                                   radius_z * std::cos(angle));
    // Look outward past the room center for wall coverage, with heading jitter.
    Eigen::Vector3d target(1.8 * radius_x * std::sin(angle + 0.35 + jitter_heading),
                           eye_height * 0.75,
                           1.8 * radius_z * std::cos(angle + 0.35 + jitter_heading));

    const Eigen::Vector3d forward = (target - position).normalized();
    const Eigen::Vector3d world_up(0, 1, 0);
    // CV camera basis (+X right, +Y down, +Z forward), det(R) = +1:
    const Eigen::Vector3d right = forward.cross(world_up).normalized();
    const Eigen::Vector3d down = forward.cross(right).normalized();
    Eigen::Matrix3d R_cw;  // camera-to-world: columns are camera axes in world
    R_cw.col(0) = right;
    R_cw.col(1) = down;
    R_cw.col(2) = forward;

    poses.push_back(SE3::FromCamToWorld(Eigen::Quaterniond(R_cw), position));
  }
  return poses;
}

}  // namespace bs::synth
