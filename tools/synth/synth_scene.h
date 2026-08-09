#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "common/geometry.h"
#include "io/depth_codec.h"

namespace bs::synth {

// A textured rectangle in world space: corner + two edge vectors. The
// procedural texture is world-anchored (sampled in plane-local meters), so
// multi-view appearance is consistent — SIFT/ORB find real correspondences.
struct TexturedPlane {
  Eigen::Vector3d origin;
  Eigen::Vector3d edge_u;  // full extent along u, meters
  Eigen::Vector3d edge_v;  // full extent along v, meters
  cv::Vec3f base_color{0.72f, 0.72f, 0.72f};  // BGR in [0,1]
  float texture_amount = 0.5f;  // 0 = perfectly blank surface
  uint32_t texture_seed = 1;
};

struct Scene {
  std::vector<TexturedPlane> planes;
};

struct RayHit {
  double t = -1.0;          // camera-space depth (z), <0 = miss
  int plane_index = -1;
  double u = 0, v = 0;      // hit position, plane-local meters
};

// World frame: Y up, floor at y=0. Standard CV camera: +Z forward, +Y down.
// A furnished room with two well-textured walls, one nearly blank wall,
// floor/ceiling, and two interior boxes.
Scene MakeRoomScene(uint32_t seed, bool blank_wall = true);

// Nearest intersection of the pixel ray with the scene. `dir_world` must be
// R_cw * (x_n, y_n, 1) — unnormalized, so `t` is camera-space depth.
RayHit CastRay(const Scene& scene, const Eigen::Vector3d& center,
               const Eigen::Vector3d& dir_world);

// Procedural texture value in [0,1] at plane-local meters (u,v).
float TextureValue(const TexturedPlane& plane, double u, double v);

// Renders a BGR uint8 image of the scene from `pose` (world-to-camera).
cv::Mat RenderImage(const Scene& scene, const SE3& pose, const Intrinsics& K);

struct DepthNoise {
  double sigma_base_m = 0.004;
  double sigma_quadratic = 0.0035;  // sigma = base + q * d^2
  double dropout_frac = 0.01;
  double max_range_m = 5.0;
  double grazing_dropout_cos = 0.25;  // drop when |cos(incidence)| below this
};

// Renders a metric float16 depth map with LiDAR-like noise. Deterministic
// for a given (seed, frame) pair.
DepthImage RenderDepth(const Scene& scene, const SE3& pose, const Intrinsics& K,
                       const DepthNoise& noise, uint32_t seed);

// Smooth orbit inside the room: positions on an ellipse at eye height,
// looking outward, with mild handheld-style height/heading jitter.
// `sweep_deg` is the total orbit arc across the whole sequence — keep the
// per-frame angular change realistic (a phone at 30 fps moves well under
// 1 deg/frame; stored-frame sequences a few deg/frame). Returns
// world-to-camera poses.
std::vector<SE3> OrbitTrajectory(int frame_count, double radius_x, double radius_z,
                                 double eye_height, uint32_t seed,
                                 double sweep_deg = 140.0);

}  // namespace bs::synth
