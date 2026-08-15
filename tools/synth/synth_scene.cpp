#include "synth_scene.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <functional>
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

const TwoRoomLayout& TwoRoomLayoutSpec() {
  static const TwoRoomLayout layout = [] {
    TwoRoomLayout l;
    l.furniture = {
        // room A: a sideboard against the back wall and a table mid-floor.
        {{-1.4, 0.0, -3.2}, 1.6, 0.9, 0.5, {0.25f, 0.35f, 0.55f}, 0.80f, 77},
        {{0.6, 0.0, 0.8}, 1.2, 0.75, 0.8, {0.45f, 0.58f, 0.70f}, 0.85f, 91},
        // room B: a tall cabinet and a low chest.
        {{4.6, 0.0, -3.0}, 1.4, 1.1, 0.6, {0.52f, 0.42f, 0.34f}, 0.82f, 101},
        {{6.4, 0.0, 1.4}, 1.0, 0.55, 1.0, {0.34f, 0.46f, 0.40f}, 0.78f, 111},
    };
    return l;
  }();
  return layout;
}

Scene MakeTwoRoomScene(uint32_t seed, bool blank_wall) {
  const TwoRoomLayout& L = TwoRoomLayoutSpec();
  Scene scene;
  const double H = L.height;
  const double z0 = L.a.z0, z1 = L.a.z1, D = z1 - z0;
  const double ax0 = L.a.x0, ax1 = L.a.x1;  // room A
  const double bx0 = L.b.x0, bx1 = L.b.x1;  // room B (shares the wall at x = 3)
  const double door_half = L.door_half, door_h = L.door_height;

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

  // --- dividing wall at x = 3: a wall with thickness, not a paper one ---
  // A zero-thickness divider makes the doorway a hole with nothing inside
  // it: no jamb to see from an angle, no soffit overhead, no depth return
  // anywhere in the opening. Both rooms then agree about the wall and about
  // nothing in between, which is exactly where a splat of a house shows its
  // seam. 16 cm is a stud partition boarded on both sides.
  const double face_a = L.face_a(), face_b = L.face_b(), t = L.wall_thickness;
  const cv::Vec3f wall_col{0.78f, 0.80f, 0.82f};
  for (int side = 0; side < 2; ++side) {
    const double xf = side == 0 ? face_a : face_b;
    const uint32_t s = seed ^ (side == 0 ? 31u : 34u);
    add({xf, 0, z0}, {0, 0, -door_half - z0}, {0, H, 0}, wall_col, 0.45f, s);
    add({xf, 0, door_half}, {0, 0, z1 - door_half}, {0, H, 0}, wall_col, 0.45f,
        s ^ 1);
    add({xf, door_h, -door_half}, {0, 0, 2 * door_half}, {0, H - door_h, 0},
        wall_col * 0.97f, 0.40f, s ^ 2);
  }
  // The reveal: two jambs and the soffit. These surfaces exist only because
  // the wall has depth, they are invisible face-on and fully visible from
  // 30 deg off-axis, and they are what orbiting the opening is for.
  const cv::Vec3f jamb_col{0.70f, 0.74f, 0.78f};
  add({face_a, 0, -door_half}, {t, 0, 0}, {0, door_h, 0}, jamb_col, 0.72f,
      seed ^ 41);
  add({face_a, 0, door_half}, {t, 0, 0}, {0, door_h, 0}, jamb_col, 0.72f,
      seed ^ 42);
  add({face_a, door_h, -door_half}, {t, 0, 0}, {0, 0, 2 * door_half}, jamb_col,
      0.60f, seed ^ 43);

  // --- furniture, so both rooms carry parallax-rich near geometry ---
  for (const FurnitureBox& box : L.furniture) {
    add_box(box.origin, box.w, box.h, box.d, box.color, box.texture,
            seed ^ box.seed_xor);
  }

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
//
// The limit is applied to yaw and pitch together, not to the direction as a
// vector. Rotating a direction toward its target along the great circle is
// the shortest path on the sphere, and for a turn of about 180 deg that path
// goes over the pole: the camera obediently pitches down through the floor,
// spins, and comes back up facing the other way, all within the declared
// rotation rate. A hand does not do that — it yaws.
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

  const Eigen::Vector3d first = desired[0].normalized();
  double yaw = std::atan2(first.z(), first.x());
  double pitch = std::asin(std::clamp(first.y(), -1.0, 1.0));
  Eigen::Vector3d forward = first;
  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d want = desired[i].normalized();
    double d_yaw = std::atan2(want.z(), want.x()) - yaw;
    while (d_yaw > M_PI) d_yaw -= 2 * M_PI;
    while (d_yaw < -M_PI) d_yaw += 2 * M_PI;
    double d_pitch = std::asin(std::clamp(want.y(), -1.0, 1.0)) - pitch;
    const double mag = RadToDeg(std::hypot(d_yaw, d_pitch));
    if (mag > max_turn) {
      d_yaw *= max_turn / mag;
      d_pitch *= max_turn / mag;
    }
    yaw += d_yaw;
    pitch += d_pitch;
    forward = {std::cos(pitch) * std::cos(yaw), std::sin(pitch),
               std::cos(pitch) * std::sin(yaw)};
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
  // than along the wall behind you, so every frame sees a whole room.
  //
  // At a doorway the camera looks into the room it is ENTERING, not at the
  // doorway it is crossing. A door frame passed at arm's length sweeps
  // through the view far too fast to match against — near surface, huge
  // apparent motion — and it is not the scaffold's job anyway: the doorway
  // gets proper coverage later, during inside-out capture and orbiting of
  // the regions of interest around it. What the scaffold wants here is the
  // next room's far structure, which is distant, slow-moving, and still
  // there several seconds later.
  // Plan shape: a room-sized loop on each side joined by a straight run
  // through the opening — an H. Nobody crosses a doorway on a diagonal from
  // the far corner; the earlier waypoints did, pinching the path to a point
  // at the threshold and giving the camera no square approach to it.
  const std::vector<Eigen::Vector3d> waypoints = {
      {-2.3, 0, 3.3},    // room A, front-left  (start)
      {-2.3, 0, -3.3},   // room A, back-left   (along the blank wall)
      {2.3, 0, -3.3},    // room A, back-right
      {2.3, 0, -0.25},   // down the divider wall onto the opening's axis
      {3.7, 0, -0.25},   // straight through  ── the H crossbar
      {3.7, 0, -3.3},    // room B, back-left
      {8.3, 0, -3.3},    // room B, back-right
      {8.3, 0, 3.3},     // room B, front-right
      {3.7, 0, 3.3},     // room B, front-left
      {3.7, 0, 0.25},    // back up onto the opening's axis
      {2.3, 0, 0.25},    // straight back through ── offset so it is not a retrace
      {2.3, 0, 3.3},     // room A, front-right
      {-2.3, 0, 3.3},    // home
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

  // Geometry of MakeTwoRoomScene: rooms centred at x = 0 and x = 6, joined
  // by an opening in the wall at x = 3 spanning |z| < 0.55.
  constexpr double kDivider = 3.0;
  const Eigen::Vector3d centre_a(0.0, eye_height * 0.8, 0.0);
  const Eigen::Vector3d centre_b(6.0, eye_height * 0.8, 0.0);

  // How far ahead along the path the doorway decision looks, and it is
  // bounded on BOTH sides. Too short and the turn cannot finish — the
  // camera crosses the threshold still facing backwards. Too long and the
  // switch lands back on the leg that runs along the far wall, where travel
  // is already headed at the doorway: the camera then walks straight toward
  // what it is looking at, which is the degenerate case for parallax, and
  // the scaffold collapses (measured at 5.0 m: scout tracking 90% -> 33%).
  // 3.0 m puts it on the approach leg, where travel is across the new view
  // rather than along it, and still leaves ~120 deg of turn budget.
  const double kLookAheadFrac = 3.0 / total;

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

    // Aim at the centre of the room we are about to be standing in. Near a
    // doorway that is the room being ENTERED, so the view leads through the
    // opening rather than tracking the door frame past at arm's length. A
    // rule keyed on which room CONTAINS the camera switches at the divider
    // instead: too late to turn, and discontinuous — it produced a single
    // 174 deg frame that ended tracking for the following 800.
    const Eigen::Vector3d soon = sample(s + kLookAheadFrac);
    const Eigen::Vector3d target = soon.x() < kDivider ? centre_a : centre_b;

    Eigen::Vector3d forward = target - position;
    if (forward.norm() < 1e-6) forward = Eigen::Vector3d(1, 0, 0);

    positions.push_back(position);
    forwards.push_back(forward.normalized());
  }
  return PosesFromLookDirections(positions, forwards, max_turn_deg);
}

namespace {

// --- capture planning ------------------------------------------------------
//
// The capture walk is not a route between waypoints, it is a plan: circle the
// room, then orbit each large object in it, treating the doorway as one of
// those objects. Waypoints typed by hand encode a plan for one furniture
// arrangement and quietly walk through the sofa when the arrangement changes,
// so the path is computed from the layout instead.

constexpr double kPathStep = 0.025;       // dense path sampling, metres
// How close ANY part of the plan walks to a wall. One number, deliberately:
// giving the lap its own looser wall clearance put the lap outside the
// floorplan the router validates against, so every detour check failed from
// a lap endpoint and routes fell back to straight lines — 35 poses through
// the middle of a table. A path is either walkable or it is not, and two
// answers to that question is one too many.
constexpr double kPlanWallClear = 0.45;
constexpr double kPlanObjClear = 0.35;    // ... and to furniture
constexpr double kDesiredArcDeg = 200.0;  // orbit coverage worth closing in for
constexpr double kMinRunDeg = 25.0;       // shorter arcs are not worth walking
constexpr double kCornerRadius = 0.5;     // rounded circuit corners
// How far off the wall the lap walks, and how far its view is turned along
// the wall from square-on. Both exist to keep the SUBJECT distance sane:
// walking the skirting board and looking sideways puts a flat wall 60 cm
// from the lens, which fills the frame, gives the tracker nothing that
// persists, and records a 70 cm patch of a 6 m surface.
// The lap hugs the perimeter. Inside-out capture wants the camera as far
// from what it is filming as the room allows, and the way to be far from
// the opposite wall is to have your back against this one — so the loop is
// pushed out to the wall, not held off it. Standoff is what a lap gives up
// when furniture forces it, not a distance it aims for.
constexpr double kLapStandoff = 0.0;
// What the lap keeps clear of furniture when it can. kPlanObjClear is the
// hard limit — can a body fit — and it is not the same question as whether
// a person would walk there filming.
constexpr double kLapObjClear = 0.60;
constexpr double kLapSweepDeg = 26.0;  // yaw sweep either side of across

struct Rect {
  double x0, x1, z0, z1;
  bool Contains(const Eigen::Vector3d& p) const {
    return p.x() > x0 && p.x() < x1 && p.z() > z0 && p.z() < z1;
  }
  double width() const { return x1 - x0; }
  double depth() const { return z1 - z0; }
};

struct Floorplan {
  std::vector<Rect> open;     // walkable floor
  std::vector<Rect> blocked;  // furniture footprints, inflated

  bool Walkable(const Eigen::Vector3d& p) const {
    bool inside = false;
    for (const Rect& r : open) {
      if (r.Contains(p)) {
        inside = true;
        break;
      }
    }
    if (!inside) return false;
    for (const Rect& r : blocked) {
      if (r.Contains(p)) return false;
    }
    return true;
  }

  bool WalkableLine(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
    const double len = (b - a).norm();
    const int steps = std::max(2, static_cast<int>(len / kPathStep) + 1);
    for (int i = 0; i <= steps; ++i) {
      const double f = static_cast<double>(i) / steps;
      if (!Walkable(a + f * (b - a))) return false;
    }
    return true;
  }
};

Floorplan MakeFloorplan(const TwoRoomLayout& L, double wall_clear,
                        double obj_clear) {
  Floorplan plan;
  plan.open.push_back({L.a.x0 + wall_clear, L.face_a() - wall_clear,
                       L.a.z0 + wall_clear, L.a.z1 - wall_clear});
  plan.open.push_back({L.face_b() + wall_clear, L.b.x1 - wall_clear,
                       L.b.z0 + wall_clear, L.b.z1 - wall_clear});
  // The opening, as floor rather than as a hole in a wall. It has to overlap
  // both rooms' rectangles or there is no way across at all, and it is the
  // one place the camera passes within half a metre of a solid on both sides.
  const double slot = std::max(0.35, L.door_half - obj_clear);
  plan.open.push_back({L.face_a() - wall_clear - 0.4,
                       L.face_b() + wall_clear + 0.4, -slot, slot});
  for (const FurnitureBox& box : L.furniture) {
    plan.blocked.push_back({box.origin.x() - obj_clear,
                            box.origin.x() + box.w + obj_clear,
                            box.origin.z() - obj_clear,
                            box.origin.z() + box.d + obj_clear});
  }
  return plan;
}

// The lap, one leg at a time. Each starts at kLapStandoff from its wall —
// nobody films a wall from the skirting board, and the camera cannot see
// much of one from there either — and moves only when the furniture makes
// it. Blocked, a leg first tries stepping back TOWARD the wall, down to the
// walkable limit, because that keeps the loop large; only if that fails does
// it come inward, which is what a sideboard standing 1.3 m off the back wall
// forces and is exactly where a person walks around it.
Rect CircuitRect(const Floorplan& roomy, const Floorplan& tight, Rect limit,
                 double standoff, double y) {
  constexpr double kStep = 0.05;
  constexpr double kMinSpan = 1.2;
  const Floorplan* plan = &roomy;
  limit = {limit.x0 + 0.02, limit.x1 - 0.02, limit.z0 + 0.02, limit.z1 - 0.02};
  Rect r = {std::min(limit.x0 + standoff, 0.5 * (limit.x0 + limit.x1)),
            std::max(limit.x1 - standoff, 0.5 * (limit.x0 + limit.x1)),
            std::min(limit.z0 + standoff, 0.5 * (limit.z0 + limit.z1)),
            std::max(limit.z1 - standoff, 0.5 * (limit.z0 + limit.z1))};

  // side: 0 = z0, 1 = z1, 2 = x0, 3 = x1. `out` is toward that side's wall.
  auto leg_clear = [&](int side) {
    const Eigen::Vector3d a(r.x0, y, side == 1 ? r.z1 : r.z0);
    const Eigen::Vector3d b(side == 3 ? r.x1 : r.x0, y,
                            side == 1 ? r.z1 : r.z0);
    if (side < 2) {
      return plan->WalkableLine({r.x0, y, side ? r.z1 : r.z0},
                                {r.x1, y, side ? r.z1 : r.z0});
    }
    return plan->WalkableLine({side == 3 ? r.x1 : r.x0, y, r.z0},
                              {side == 3 ? r.x1 : r.x0, y, r.z1});
  };
  auto edge = [&](int side) -> double& {
    return side == 0 ? r.z0 : side == 1 ? r.z1 : side == 2 ? r.x0 : r.x1;
  };
  auto wall = [&](int side) {
    return side == 0 ? limit.z0 : side == 1 ? limit.z1
           : side == 2 ? limit.x0 : limit.x1;
  };
  auto span_ok = [&](int side) {
    return side < 2 ? r.depth() > kMinSpan : r.width() > kMinSpan;
  };

  for (int side = 0; side < 4; ++side) {
    // Two passes: place the leg with a person's width to spare, and only if
    // that is impossible anywhere on the side, allow the tight clearance.
    // Without this the leg happily threads the 38 cm slot between a chest
    // and the wall — legal by the walkability test, unwalkable by anybody
    // holding a phone, and 40 cm from the only thing it can then see.
    plan = &roomy;
    if (leg_clear(side)) continue;
    const double start = edge(side);
    const double outward = wall(side) > start ? kStep : -kStep;
    bool fixed = false;
    for (int i = 0; i < 60; ++i) {  // back toward the wall first
      const double next = edge(side) + outward;
      if ((outward > 0) != (next < wall(side))) break;  // past the wall
      edge(side) = next;
      if (leg_clear(side)) {
        fixed = true;
        break;
      }
    }
    if (fixed) continue;
    edge(side) = start;
    for (int i = 0; i < 200 && span_ok(side); ++i) {  // then inward
      edge(side) -= outward;
      if (leg_clear(side)) {
        fixed = true;
        break;
      }
    }
    if (fixed) continue;

    plan = &tight;  // nothing roomy exists on this side; take what there is
    edge(side) = start;
    for (int i = 0; i < 60; ++i) {
      const double next = edge(side) + outward;
      if ((outward > 0) != (next < wall(side))) break;
      edge(side) = next;
      if (leg_clear(side)) {
        fixed = true;
        break;
      }
    }
    if (fixed) continue;
    edge(side) = start;
    for (int i = 0; i < 200 && span_ok(side); ++i) {
      edge(side) -= outward;
      if (leg_clear(side)) break;
    }
  }
  return r;
}

struct AngleRun {
  double a0, a1;  // radians, a1 > a0
  double span() const { return a1 - a0; }
};

// Contiguous walkable arcs of the ring of radius `radius` about `centre`.
// `side` restricts to one side of the centre in x (-1 / +1), which is how a
// doorway is orbited: the wall splits its ring into one arc per room, and
// each room's capture owns its own.
std::vector<AngleRun> RingRuns(const Floorplan& plan,
                               const Eigen::Vector3d& centre, double radius,
                               int side) {
  constexpr int kN = 720;
  std::vector<char> ok(kN, 0);
  for (int i = 0; i < kN; ++i) {
    const double a = 2.0 * M_PI * i / kN;
    const Eigen::Vector3d p(centre.x() + radius * std::cos(a), centre.y(),
                            centre.z() + radius * std::sin(a));
    if (side < 0 && p.x() >= centre.x()) continue;
    if (side > 0 && p.x() <= centre.x()) continue;
    ok[i] = plan.Walkable(p) ? 1 : 0;
  }
  std::vector<AngleRun> runs;
  bool all = true;
  for (int i = 0; i < kN; ++i) all = all && ok[i] != 0;
  if (all) {
    runs.push_back({0.0, 2.0 * M_PI});
    return runs;
  }
  // Both ends of a run are pulled in by a hand's width. Untrimmed, a run
  // ends exactly ON the boundary of the walkable set — half a centimetre
  // from a wall — and every route out of it fails on its first sample, so
  // the plan falls back to a straight line through the furniture.
  const double trim = radius > 1e-6 ? 0.1 / radius : 0.0;
  for (int i = 0; i < kN; ++i) {
    if (!ok[i] || ok[(i + kN - 1) % kN]) continue;  // not a run start
    int len = 0;
    while (len < kN && ok[(i + len) % kN]) ++len;
    const double a0 = 2.0 * M_PI * i / kN;
    const AngleRun run{a0 + trim, a0 + 2.0 * M_PI * len / kN - trim};
    if (run.span() > 0) runs.push_back(run);
  }
  return runs;
}

struct OrbitPlan {
  double radius = 0;
  std::vector<AngleRun> runs;
  double coverage_deg = 0;
};

// Orbit distance is half the room's short dimension — far enough that the
// object stays whole in frame with the room behind it for context. Where
// that ring is unwalkable the plan closes in, but only as far as it has to.
//
// Coverage alone picks the wrong radius: a wide ring that clears a table on
// two opposite sides scores 230 deg in two arcs and pays for it by crossing
// the room between them, while a ring 1 m closer sweeps 250 deg in one
// unbroken walk. Each break is charged kGapPenaltyDeg, so a second arc has
// to be worth the trip back.
OrbitPlan ChooseOrbit(const Floorplan& plan, const Eigen::Vector3d& centre,
                      double want_radius, double min_radius, int side) {
  constexpr double kGapPenaltyDeg = 60.0;
  OrbitPlan best;
  double best_score = -1e9;
  for (double r = want_radius; r >= min_radius - 1e-9; r -= 0.1) {
    OrbitPlan p;
    p.radius = r;
    for (const AngleRun& run : RingRuns(plan, centre, r, side)) {
      if (RadToDeg(run.span()) < kMinRunDeg) continue;
      p.runs.push_back(run);
      p.coverage_deg += RadToDeg(run.span());
    }
    if (p.runs.empty()) continue;
    const double score =
        std::min(kDesiredArcDeg, p.coverage_deg) -
        kGapPenaltyDeg * static_cast<double>(p.runs.size() - 1);
    if (score > best_score + 1e-9) {  // strict: larger radii come first
      best_score = score;
      best = std::move(p);
    }
  }
  return best;
}

// A walkable-cell grid over the floorplan, and a breadth-first search on it.
//
// The router used to be "straight line, else one detour point, else straight
// line anyway". One hop is not enough: getting from a room's back corner to
// the doorway has to dodge the sideboard AND the table, no single via clears
// both, and the fallback then drew a 6 m diagonal that clipped the sideboard
// at 9 cm. A grid search either finds a way round or proves there is none —
// and if a plan ever needs the second answer, that is worth failing on
// rather than papering over with a line through the furniture.
struct Grid {
  double step = 0.15;
  double x0 = 0, z0 = 0;
  int nx = 0, nz = 0;
  std::vector<char> free;
  bool At(int i, int j) const {
    return i >= 0 && j >= 0 && i < nx && j < nz &&
           free[static_cast<size_t>(j) * nx + i] != 0;
  }
  Eigen::Vector3d Point(int i, int j, double y) const {
    return {x0 + i * step, y, z0 + j * step};
  }
};

Grid MakeGrid(const Floorplan& plan, double y) {
  Grid g;
  double x1 = -1e9, z1 = -1e9;
  g.x0 = 1e9;
  g.z0 = 1e9;
  for (const Rect& r : plan.open) {
    g.x0 = std::min(g.x0, r.x0);
    g.z0 = std::min(g.z0, r.z0);
    x1 = std::max(x1, r.x1);
    z1 = std::max(z1, r.z1);
  }
  g.nx = static_cast<int>((x1 - g.x0) / g.step) + 1;
  g.nz = static_cast<int>((z1 - g.z0) / g.step) + 1;
  g.free.assign(static_cast<size_t>(g.nx) * g.nz, 0);
  for (int j = 0; j < g.nz; ++j) {
    for (int i = 0; i < g.nx; ++i) {
      g.free[static_cast<size_t>(j) * g.nx + i] =
          plan.Walkable(g.Point(i, j, y)) ? 1 : 0;
    }
  }
  return g;
}

// Waypoints from `from` to `to` that stay walkable, or empty if the search
// found no route at all. The result is string-pulled: the grid path is a
// staircase, and what a person walks is the straight runs between the
// corners it actually had to turn at.
std::vector<Eigen::Vector3d> RoutePath(const Grid& g, const Floorplan& plan,
                                       const Eigen::Vector3d& from,
                                       const Eigen::Vector3d& to, double y) {
  auto nearest = [&](const Eigen::Vector3d& p) {
    int bi = -1, bj = -1;
    double best = std::numeric_limits<double>::max();
    const int ci = static_cast<int>(std::lround((p.x() - g.x0) / g.step));
    const int cj = static_cast<int>(std::lround((p.z() - g.z0) / g.step));
    for (int dj = -3; dj <= 3; ++dj) {
      for (int di = -3; di <= 3; ++di) {
        if (!g.At(ci + di, cj + dj)) continue;
        const double d = (g.Point(ci + di, cj + dj, y) - p).squaredNorm();
        if (d < best) {
          best = d;
          bi = ci + di;
          bj = cj + dj;
        }
      }
    }
    return std::pair<int, int>{bi, bj};
  };
  const auto [si, sj] = nearest(from);
  const auto [ti, tj] = nearest(to);
  if (si < 0 || ti < 0) return {};

  std::vector<int> parent(static_cast<size_t>(g.nx) * g.nz, -2);
  std::vector<int> queue{sj * g.nx + si};
  parent[queue[0]] = -1;
  const int goal = tj * g.nx + ti;
  for (size_t head = 0; head < queue.size() && parent[goal] == -2; ++head) {
    const int cur = queue[head];
    const int i = cur % g.nx, j = cur / g.nx;
    for (int dj = -1; dj <= 1; ++dj) {
      for (int di = -1; di <= 1; ++di) {
        if (di == 0 && dj == 0) continue;
        if (!g.At(i + di, j + dj)) continue;
        // A diagonal step may not cut a corner between two blocked cells.
        if (di && dj && !(g.At(i + di, j) && g.At(i, j + dj))) continue;
        const int next = (j + dj) * g.nx + (i + di);
        if (parent[next] != -2) continue;
        parent[next] = cur;
        queue.push_back(next);
      }
    }
  }
  if (parent[goal] == -2) return {};

  std::vector<Eigen::Vector3d> cells;
  for (int cur = goal; cur != -1; cur = parent[cur]) {
    cells.push_back(g.Point(cur % g.nx, cur / g.nx, y));
  }
  std::reverse(cells.begin(), cells.end());

  std::vector<Eigen::Vector3d> out;
  Eigen::Vector3d anchor = from;
  for (size_t i = 0; i < cells.size(); ++i) {
    if (plan.WalkableLine(anchor, i + 1 < cells.size() ? cells[i + 1] : to)) {
      continue;  // can still see further ahead
    }
    out.push_back(cells[i]);
    anchor = cells[i];
  }
  return out;
}

// Blends two look targets AS SEEN FROM p: the direction rotates, the range
// interpolates. Interpolating the two points instead sweeps a target across
// the room, and on the way it passes through the camera — where it has no
// direction at all, so the view spins on the spot. Measured, that produced a
// 4.3 deg frame while walking a wall at 0.6 deg, pointing almost straight
// down, in the middle of an otherwise unremarkable lap.
Eigen::Vector3d BlendLook(const Eigen::Vector3d& p, const Eigen::Vector3d& la,
                          const Eigen::Vector3d& lb, double f);

// Appends samples every kPathStep along a straight run, turning the view
// from one target to the other as it goes. The first point is dropped when
// it continues an existing path, so joins do not duplicate a sample.
void EmitLine(std::vector<Eigen::Vector3d>& pos,
              std::vector<Eigen::Vector3d>& look, const Eigen::Vector3d& a,
              const Eigen::Vector3d& b, const Eigen::Vector3d& la,
              const Eigen::Vector3d& lb) {
  const int steps =
      std::max(1, static_cast<int>((b - a).norm() / kPathStep + 0.5));
  for (int i = pos.empty() ? 0 : 1; i <= steps; ++i) {
    const double f = static_cast<double>(i) / steps;
    const Eigen::Vector3d p = a + f * (b - a);
    pos.push_back(p);
    look.push_back(BlendLook(p, la, lb, f));
  }
}

Eigen::Vector3d BlendLook(const Eigen::Vector3d& p, const Eigen::Vector3d& la,
                          const Eigen::Vector3d& lb, double f) {
  // Yaw and range blend; height comes from the targets themselves. Rotating
  // the full 3D direction instead carries the near target's steep downward
  // angle out to the far target's range — a metre below the floor — and the
  // camera spends the walk staring at its own feet.
  const Eigen::Vector2d ha(la.x() - p.x(), la.z() - p.z());
  const Eigen::Vector2d hb(lb.x() - p.x(), lb.z() - p.z());
  const double ra = ha.norm(), rb = hb.norm();
  if (ra < 1e-6 || rb < 1e-6) return la + f * (lb - la);
  double turn = std::atan2(hb.y(), hb.x()) - std::atan2(ha.y(), ha.x());
  while (turn > M_PI) turn -= 2 * M_PI;
  while (turn < -M_PI) turn += 2 * M_PI;
  const double angle = std::atan2(ha.y(), ha.x()) + f * turn;
  const double range = ra + f * (rb - ra);
  return {p.x() + range * std::cos(angle), la.y() + f * (lb.y() - la.y()),
          p.z() + range * std::sin(angle)};
}

}  // namespace

bool TwoRoomWalkable(const Eigen::Vector3d& p, double wall_clearance,
                     double object_clearance) {
  return MakeFloorplan(TwoRoomLayoutSpec(), wall_clearance, object_clearance)
      .Walkable(p);
}

CapturePlan BuildCapturePlan(double eye_height) {
  const TwoRoomLayout& L = TwoRoomLayoutSpec();
  const Floorplan plan = MakeFloorplan(L, kPlanWallClear, kPlanObjClear);
  const Floorplan roomy = MakeFloorplan(L, kPlanWallClear, kLapObjClear);
  const Grid grid = MakeGrid(plan, eye_height);
  const double plan_y = eye_height;
  const double look_y = eye_height * 0.72;

  std::vector<Eigen::Vector3d> pos, look;
  std::vector<CapturePhase> phase;
  CapturePhase current = CapturePhase::kLap;
  auto tag = [&] {
    while (phase.size() < pos.size()) phase.push_back(current);
  };

  // Moves to `to` from wherever the path is, going around anything in the
  // way and keeping the eye on whatever it was looking at until it arrives.
  auto route_to = [&](const Eigen::Vector3d& to,
                      const Eigen::Vector3d& look_to) {
    const Eigen::Vector3d from = pos.back();
    const Eigen::Vector3d look_from = look.back();
    if (plan.WalkableLine(from, to)) {
      EmitLine(pos, look, from, to, look_from, look_to);
      return;
    }
    const std::vector<Eigen::Vector3d> via =
        RoutePath(grid, plan, from, to, plan_y);
    if (via.empty()) {
      // No route exists at all. Not something to paper over with a straight
      // line: it means the plan asked to reach somewhere unreachable.
      EmitLine(pos, look, from, to, look_from, look_to);
      return;
    }
    // Looking where you are going while detouring, then back onto the
    // target as you arrive. Held on the old target the whole way, the view
    // ends up pointed through whatever the detour was avoiding.
    Eigen::Vector3d at = from;
    Eigen::Vector3d aim_from = look_from;
    for (size_t i = 0; i < via.size(); ++i) {
      Eigen::Vector3d ahead = via[i] - at;
      ahead.y() = 0;
      const double f = static_cast<double>(i + 1) / (via.size() + 1);
      Eigen::Vector3d aim_to =
          ahead.norm() < 1e-6
              ? look_to
              : Eigen::Vector3d(via[i] + 2.5 * ahead.normalized());
      aim_to.y() = look_y;
      aim_to = aim_to + f * (look_to - aim_to);
      EmitLine(pos, look, at, via[i], aim_from, aim_to);
      at = via[i];
      aim_from = aim_to;
    }
    EmitLine(pos, look, at, to, aim_from, look_to);
  };

  auto ring_point = [](const Eigen::Vector3d& c, double r, double a) {
    return Eigen::Vector3d(c.x() + r * std::cos(a), c.y(),
                           c.z() + r * std::sin(a));
  };

  // One object, orbited: sweep every walkable arc of its ring in one
  // rotational direction, stepping around the blocked stretches without ever
  // taking your eyes off it. `aim` is a function of where the camera is
  // standing, not a fixed point, because a doorway's aim has to slide: an
  // opening centred in frame is a hole, and the ray goes clean through it
  // into the next room.
  double orbit_radius = 0;
  auto orbit = [&](const Eigen::Vector3d& centre,
                   const std::function<Eigen::Vector3d(const Eigen::Vector3d&)>&
                       aim,
                   double want_radius, int side) {
    const OrbitPlan op =
        ChooseOrbit(plan, {centre.x(), plan_y, centre.z()}, want_radius,
                    std::min(0.9, want_radius), side);
    if (op.runs.empty()) return;
    orbit_radius = op.radius;
    const Eigen::Vector3d hub(centre.x(), plan_y, centre.z());

    // Enter at whichever run starts nearest, then keep going the same way
    // round: reversing mid-orbit repeats views instead of adding them.
    size_t first = 0;
    if (!pos.empty()) {
      double best = std::numeric_limits<double>::max();
      for (size_t i = 0; i < op.runs.size(); ++i) {
        const double d =
            (ring_point(hub, op.radius, op.runs[i].a0) - pos.back()).norm();
        if (d < best) {
          best = d;
          first = i;
        }
      }
    }
    for (size_t k = 0; k < op.runs.size(); ++k) {
      const AngleRun& run = op.runs[(first + k) % op.runs.size()];
      const Eigen::Vector3d entry = ring_point(hub, op.radius, run.a0);
      if (pos.empty()) {
        pos.push_back(entry);
        look.push_back(aim(entry));
      } else {
        route_to(entry, aim(entry));
      }
      const int steps = std::max(
          2, static_cast<int>(run.span() * op.radius / kPathStep + 0.5));
      for (int i = 1; i <= steps; ++i) {
        const double a = run.a0 + run.span() * i / steps;
        const Eigen::Vector3d at = ring_point(hub, op.radius, a);
        pos.push_back(at);
        look.push_back(aim(at));
      }
    }
  };

  Eigen::Vector3d home, home_look;
  for (int room = 0; room < 2; ++room) {
    const RoomBounds& R = room == 0 ? L.a : L.b;
    const Rect bounds =
        room == 0 ? Rect{R.x0 + kPlanWallClear, L.face_a() - kPlanWallClear,
                         R.z0 + kPlanWallClear, R.z1 - kPlanWallClear}
                  : Rect{L.face_b() + kPlanWallClear, R.x1 - kPlanWallClear,
                         R.z0 + kPlanWallClear, R.z1 - kPlanWallClear};
    const Rect circuit =
        CircuitRect(roomy, plan, bounds, kLapStandoff, plan_y);
    const Eigen::Vector3d hub(0.5 * (circuit.x0 + circuit.x1), plan_y,
                              0.5 * (circuit.z0 + circuit.z1));

    current = CapturePhase::kApproach;
    // --- 1. circle the room ---
    // Corners in order, walked with the near wall on the outside and the
    // camera facing it: the lap that establishes the room's shell, before
    // any of its contents.
    const std::vector<Eigen::Vector3d> corners = {
        {circuit.x0, plan_y, circuit.z1},
        {circuit.x0, plan_y, circuit.z0},
        {circuit.x1, plan_y, circuit.z0},
        {circuit.x1, plan_y, circuit.z1},
    };
    // Rounding a corner must not cut through what the legs were placed to
    // avoid. The arcs used to be emitted with no walkability test at all —
    // the legs were checked, the curve joining them was not — so a corner
    // could slice a furniture corner off however tightly it liked.
    double cr = std::min({kCornerRadius, circuit.width() / 3,
                          circuit.depth() / 3});
    for (; cr > 0.05; cr -= 0.05) {
      bool ok = true;
      for (int i = 0; i < 4 && ok; ++i) {
        const Eigen::Vector3d here(i == 0 || i == 1 ? circuit.x0 : circuit.x1,
                                   plan_y,
                                   i == 0 || i == 3 ? circuit.z1 : circuit.z0);
        // The arc is inscribed in the corner; its centre is the point the
        // curve stays `cr` from, so testing that centre's disc is enough.
        const Eigen::Vector3d inward(
            here.x() + (i <= 1 ? cr : -cr), plan_y,
            here.z() + (i == 0 || i == 3 ? -cr : cr));
        for (int k = 0; k <= 8 && ok; ++k) {
          const double a = 2.0 * M_PI * k / 8;
          ok = plan.Walkable({inward.x() + cr * std::cos(a), plan_y,
                              inward.z() + cr * std::sin(a)});
        }
      }
      if (ok) break;
    }
    std::vector<Eigen::Vector3d> lap;
    auto lap_line = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
      const int steps =
          std::max(1, static_cast<int>((b - a).norm() / kPathStep + 0.5));
      for (int i = lap.empty() ? 0 : 1; i <= steps; ++i) {
        lap.push_back(a + (static_cast<double>(i) / steps) * (b - a));
      }
    };
    for (int i = 0; i <= 4; ++i) {
      const Eigen::Vector3d& prev = corners[(i + 3) % 4];
      const Eigen::Vector3d& here = corners[i % 4];
      const Eigen::Vector3d& next = corners[(i + 1) % 4];
      const Eigen::Vector3d in = (here - prev).normalized();
      const Eigen::Vector3d out = (next - here).normalized();
      const Eigen::Vector3d arc_start = here - cr * in;
      const Eigen::Vector3d arc_end = here + cr * out;
      const Eigen::Vector3d arc_c = here + cr * (out - in);
      if (i > 0) lap_line(lap.back(), arc_start);
      if (i == 4) break;  // the lap closes where it opened
      const double a0 = std::atan2(arc_start.z() - arc_c.z(),
                                   arc_start.x() - arc_c.x());
      double a1 = std::atan2(arc_end.z() - arc_c.z(), arc_end.x() - arc_c.x());
      while (a1 - a0 > M_PI) a1 -= 2 * M_PI;
      while (a1 - a0 < -M_PI) a1 += 2 * M_PI;
      const int steps =
          std::max(2, static_cast<int>(std::abs(a1 - a0) * cr / kPathStep));
      for (int k = lap.empty() ? 0 : 1; k <= steps; ++k) {
        const double a = a0 + (a1 - a0) * k / steps;
        lap.emplace_back(arc_c.x() + cr * std::cos(a), plan_y,
                         arc_c.z() + cr * std::sin(a));
      }
    }
    // Face ACROSS the room, at the far wall — not at the wall being walked
    // past. This is the whole geometry of the lap and it took two wrong
    // answers to get here. Pointed square at the near wall, the camera is
    // half a metre from a flat surface: it fills the frame, sweeps by at
    // walking speed, records a 70 cm patch of a 6 m wall, and tracked at
    // zero for a whole room's lap. Angled 52 deg along that wall fixed the
    // tracking and still filmed a surface 2 m away that the orbits cover
    // better anyway.
    //
    // Facing the far wall is the answer to both. Every frame sees a whole
    // wall rather than a patch; walking the perimeter views that wall from
    // one end of the room to the other, which is a baseline of metres
    // against a subject metres away — the best triangulation geometry
    // anywhere in the capture. The near wall is not skipped, it is simply
    // filmed from the opposite side of the loop, where it is the far one.
    // It is also why the scout circuit faces inward, and that circuit
    // tracks at 99.4%.
    //
    // On top of it a slow yaw sweep, because straight-anything leaves the
    // ceiling and the corners unseen and nobody holds a phone rigid.
    std::vector<Eigen::Vector3d> lap_look;
    lap_look.reserve(lap.size());
    for (size_t i = 0; i < lap.size(); ++i) {
      Eigen::Vector3d in = hub - lap[i];
      in.y() = 0;
      if (in.norm() < 1e-6) in = Eigen::Vector3d(1, 0, 0);
      in.normalize();
      const double yaw =
          DegToRad(kLapSweepDeg) *
          std::sin(2.0 * M_PI * static_cast<double>(i) * kPathStep / 3.5);
      // Aimed past the centre so the target lands on the far wall rather
      // than in mid-air over the middle of the room.
      const double reach = 2.0 * (lap[i] - hub).norm() + 1.0;
      lap_look.emplace_back(
          lap[i].x() + reach * (in.x() * std::cos(yaw) - in.z() * std::sin(yaw)),
          look_y,
          lap[i].z() + reach * (in.x() * std::sin(yaw) + in.z() * std::cos(yaw)));
    }
    if (pos.empty()) {
      home = lap.front();
      home_look = lap_look.front();
    } else {
      route_to(lap.front(), lap_look.front());
    }
    tag();
    current = CapturePhase::kLap;
    for (size_t i = pos.empty() ? 0 : 1; i < lap.size(); ++i) {
      pos.push_back(lap[i]);
      look.push_back(lap_look[i]);
    }
    tag();

    // --- 2. orbit each object in the room ---
    // Half the room's short side, which is the user-facing rule too: stand
    // back about half the room to go round something. Nearest first, from
    // wherever the lap left off — the order is free, and taking it in
    // declaration order walks the length of the room between objects.
    const double want = 0.5 * R.scale();
    std::vector<const FurnitureBox*> todo;
    for (const FurnitureBox& box : L.furniture) {
      if ((box.centre().x() < L.door_x) == (room == 0)) todo.push_back(&box);
    }
    while (!todo.empty()) {
      auto next = todo.begin();
      double best = std::numeric_limits<double>::max();
      for (auto it = todo.begin(); it != todo.end(); ++it) {
        Eigen::Vector3d d = (*it)->centre() - pos.back();
        d.y() = 0;
        if (d.norm() < best) {
          best = d.norm();
          next = it;
        }
      }
      const Eigen::Vector3d c = (*next)->centre();
      todo.erase(next);
      current = CapturePhase::kApproach;
      const size_t before = pos.size();
      const Eigen::Vector3d at{c.x(), std::max(look_y * 0.8, c.y()), c.z()};
      orbit(c, [at](const Eigen::Vector3d&) { return at; }, want, 0);
      // Everything the orbit emitted past its entry route is the orbit
      // itself; the route in front of it is approach.
      tag();
      for (size_t i = before; i < phase.size(); ++i) {
        if ((pos[i] - Eigen::Vector3d(c.x(), plan_y, c.z())).norm() <
            1.15 * orbit_radius) {
          phase[i] = CapturePhase::kOrbitObject;
        }
      }
    }

    // --- 3. the doorway is an object too ---
    // Orbited from this room's side, which is all the wall leaves of its
    // ring. An opening is where two rooms have to agree about the same
    // surface, so it earns a full orbit from each of them rather than the
    // glance you get walking through.
    current = CapturePhase::kApproach;
    {
      const size_t before = pos.size();
      // Aim at the REVEAL, not through the hole. The target sits on the
      // divider face the camera is on, and slides across the opening with
      // the camera's own z, so the near jamb stays in the middle of frame
      // right through the arc. Aimed at the opening's centre instead, the
      // ray passed through the doorway and landed on the far room's back
      // wall: 7.4 m median subject distance for an orbit whose entire
      // purpose is the 16 cm of surface inside the opening.
      // OUTSIDE the opening by a hand's width, not inside it: aimed at
      // 0.85 x door_half the target is still in the hole and the ray goes
      // straight through. A quarter metre past the jamb puts the reveal,
      // the jamb and the wall around it in the middle of frame, which is
      // what someone photographing a door frame actually points at.
      const double edge = L.door_half + 0.25;
      const double face_a = L.face_a(), face_b = L.face_b();
      const double eye = L.door_height * 0.5;
      orbit(L.door_centre(),
            [edge, face_a, face_b, eye, &L](const Eigen::Vector3d& cam) {
              return Eigen::Vector3d(cam.x() < L.door_x ? face_a : face_b, eye,
                                     std::clamp(cam.z(), -edge, edge));
            },
            want, room == 0 ? -1 : 1);
      tag();
      const Eigen::Vector3d hub(L.door_x, plan_y, 0.0);
      for (size_t i = before; i < phase.size(); ++i) {
        if ((pos[i] - hub).norm() < 1.15 * orbit_radius) {
          phase[i] = CapturePhase::kOrbitDoorway;
        }
      }
    }

    // --- 4. through the opening (and, from room B, home) ---
    const double dir = room == 0 ? 1.0 : -1.0;
    const double lane = room == 0 ? -0.25 : 0.25;  // not a retrace
    const Eigen::Vector3d near_side(L.door_x - dir * 0.95, plan_y, lane);
    const Eigen::Vector3d far_side(L.door_x + dir * 0.95, plan_y, lane);
    const Eigen::Vector3d through(L.door_x + dir * 1.6, look_y, 0.0);
    current = CapturePhase::kApproach;
    route_to(near_side, through);
    tag();
    current = CapturePhase::kThroughDoorway;
    const RoomBounds& next = room == 0 ? L.b : L.a;
    EmitLine(pos, look, near_side, far_side, through, next.centre(look_y));
    // Home, looking the way the first frame looked. A revisit that matches
    // in position but not in view direction is one loop closure has to work
    // to recognize; matching both is the whole point of ending where you
    // started. It also keeps the last target off the room centre, which is
    // where the camera is standing.
    tag();
    current = CapturePhase::kApproach;
    if (room == 1) route_to(home, home_look);
    tag();
  }

  tag();
  return {std::move(pos), std::move(look), std::move(phase)};
}

std::vector<SE3> CaptureTrajectory(int frame_count, double eye_height,
                                   uint32_t seed, double max_turn_deg) {
  const CapturePlan built = BuildCapturePlan(eye_height);
  const std::vector<Eigen::Vector3d>& pos = built.position;
  const std::vector<Eigen::Vector3d>& look = built.look;

  // Resample the plan at constant speed. Every trajectory here covers a
  // fixed physical path, so the frame count IS the walking speed.
  if (pos.size() < 2) return {};
  std::vector<double> arc(pos.size(), 0.0);
  for (size_t i = 1; i < pos.size(); ++i) {
    arc[i] = arc[i - 1] + (pos[i] - pos[i - 1]).norm();
  }
  const double total = arc.back();

  // Smooth the look target over half a metre of walking. Where one move ends
  // and the next begins the target jumps — from a wall to a table, from a
  // table to the doorway — and a jump is a turn no wrist makes. The rate
  // limiter downstream would clip it, but clipping leaves the camera lagging
  // its target for a second afterwards; smoothing turns early instead.
  // Averaged as bearing, range and height about each sample's own position
  // — never as points. Two targets half a metre apart on the path can lie on
  // opposite sides of the camera, and the midpoint of those two points is
  // the camera itself.
  const int kSmooth = static_cast<int>(0.25 / kPathStep);
  std::vector<Eigen::Vector3d> look_s(look.size());
  for (size_t i = 0; i < look.size(); ++i) {
    double cx = 0, cz = 0, range = 0, height = 0;
    int n = 0;
    for (int d = -kSmooth; d <= kSmooth; ++d) {
      const long j = static_cast<long>(i) + d;
      if (j < 0 || j >= static_cast<long>(look.size())) continue;
      const double dx = look[static_cast<size_t>(j)].x() - pos[i].x();
      const double dz = look[static_cast<size_t>(j)].z() - pos[i].z();
      const double r = std::hypot(dx, dz);
      if (r < 1e-6) continue;
      cx += dx / r;
      cz += dz / r;
      range += r;
      height += look[static_cast<size_t>(j)].y();
      ++n;
    }
    if (n == 0 || cx * cx + cz * cz < 1e-9) {
      look_s[i] = look[i];
      continue;
    }
    const double a = std::atan2(cz, cx);
    const double r = range / n;
    look_s[i] = {pos[i].x() + r * std::cos(a), height / n,
                 pos[i].z() + r * std::sin(a)};
  }

  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  double jitter_y = 0.0;

  std::vector<Eigen::Vector3d> positions, forwards;
  positions.reserve(frame_count);
  forwards.reserve(frame_count);
  size_t seg = 1;
  for (int i = 0; i < frame_count; ++i) {
    const double s = static_cast<double>(i) / std::max(1, frame_count - 1);
    const double want = std::clamp(s, 0.0, 1.0) * total;
    while (seg + 1 < arc.size() && arc[seg] < want) ++seg;
    const double span = std::max(1e-9, arc[seg] - arc[seg - 1]);
    const double f = std::clamp((want - arc[seg - 1]) / span, 0.0, 1.0);
    Eigen::Vector3d p = pos[seg - 1] + f * (pos[seg] - pos[seg - 1]);
    const Eigen::Vector3d t = look_s[seg - 1] + f * (look_s[seg] - look_s[seg - 1]);

    jitter_y = 0.97 * jitter_y + 0.004 * gauss(rng);
    p.y() = eye_height + jitter_y;
    Eigen::Vector3d forward = t - p;
    // A look target closer than this carries almost no direction: a 3 cm
    // step past it swings the view through 90 deg, which no rate limiter
    // downstream can make physical because the intent itself is the flip.
    // Hold the previous heading through the near pass instead.
    if (Eigen::Vector2d(forward.x(), forward.z()).norm() < 0.35) {
      forward = forwards.empty() ? Eigen::Vector3d(1, 0, 0) : forwards.back();
    }
    // Never pitch further than a person walking would. Past this the camera
    // frame — which is built by locking roll to world up — turns ill
    // conditioned, and a 1.3 deg pan comes out as an 11 deg roll: the
    // trajectory obeys its own rotation limit and the poses do not.
    const double horiz = std::hypot(forward.x(), forward.z());
    const double max_rise = horiz * std::tan(DegToRad(35.0));
    if (horiz > 1e-9 && std::abs(forward.y()) > max_rise) {
      forward.y() = std::copysign(max_rise, forward.y());
    }
    positions.push_back(p);
    forwards.push_back(forward.normalized());
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

int GatedFrameCount(const std::vector<SE3>& poses, double min_step_m,
                    double min_turn_deg) {
  if (poses.empty()) return 0;
  int stored = 1;
  SE3 last = poses.front();
  for (size_t i = 1; i < poses.size(); ++i) {
    const double step = (poses[i].CameraCenter() - last.CameraCenter()).norm();
    const double turn = RadToDeg(poses[i].q.angularDistance(last.q));
    if (step < min_step_m && turn < min_turn_deg) continue;
    ++stored;
    last = poses[i];
  }
  return stored;
}

}  // namespace bs::synth
