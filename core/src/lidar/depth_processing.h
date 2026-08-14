#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "common/geometry.h"
#include "io/depth_codec.h"

namespace bs {

// Per-frame LiDAR depth processing. All quantities live in the depth map's
// own camera frame — nothing here ever depends on a camera trajectory, so
// RAW observations can be (re)interpreted under any pose estimate
// (docs/ARCHITECTURE.md, immutability invariant).
//
// Depth values are z-depth in meters (distance along the optical axis, as
// delivered by AVDepthData and produced by bs_synth).

struct LidarConfidenceOptions {
  double sigma_base_m = 0.01;
  double sigma_quadratic = 0.008;  // sigma(d) = base + quad * d^2
  double range_min_m = 0.25;
  double range_full_m = 3.0;
  double range_zero_m = 5.0;
  double max_incidence_deg = 60.0;
  // Relative depth-gradient scale for the edge weight:
  // w_edge = exp(-(|grad d| / (edge_rel * d))^2) per pixel step.
  double edge_rel = 0.05;
};

// Owns a float view of one depth frame plus derived per-pixel products
// (normals, gradient, confidence). Built once per frame, queried many times.
class DepthFrame {
 public:
  DepthFrame(const DepthImage& depth, const Intrinsics& K,
             const LidarConfidenceOptions& options = {});

  int width() const { return width_; }
  int height() const { return height_; }
  const Intrinsics& K() const { return K_; }

  bool Valid(int x, int y) const;
  float DepthAt(int x, int y) const;  // NaN when invalid

  // Bilinear depth at subpixel coords; nullopt when any corner is invalid
  // (never interpolates across holes or map borders).
  std::optional<double> DepthBilinear(double x, double y) const;

  // Camera-space point of pixel (x, y); nullopt when invalid.
  std::optional<Eigen::Vector3d> Unproject(int x, int y) const;

  // Surface normal in camera frame, oriented toward the camera (n.z < 0
  // for surfaces facing the sensor). Computed from central differences of
  // unprojected neighbors; nullopt near holes/borders.
  std::optional<Eigen::Vector3d> NormalAt(int x, int y) const;

  // Measurement noise model.
  double Sigma(double depth_m) const;

  // Per-pixel confidence w = w_edge * w_range * w_angle in [0, 1]:
  //   w_edge  kills interpolated/fabricated depth at discontinuities,
  //   w_range derates near/far range,
  //   w_angle derates oblique incidence.
  // (The adaptive visual term w_tex lives in fusion, not here — it depends
  // on the observing image content, not on the depth map.)
  double ConfidenceAt(int x, int y) const;

  // True when the 4x4 neighborhood needed by bicubic interpolation around
  // (x, y) is fully valid — the association gate for solver residuals.
  bool BicubicSafe(double x, double y) const;

  // Whether this frame carried sensor-reported confidence, as opposed to
  // the geometric model reconstructing it.
  bool has_sensor_confidence() const { return !sensor_confidence_.empty(); }

  // Depth values with NaN/invalid replaced by 0, for solver-side
  // interpolation (only ever sampled where BicubicSafe held).
  const std::vector<double>& SanitizedDepth() const { return sanitized_; }

  const LidarConfidenceOptions& options() const { return options_; }

 private:
  // 0-255 as the sensor reported it, empty when unavailable.
  std::vector<uint8_t> sensor_confidence_;
  int width_ = 0;
  int height_ = 0;
  Intrinsics K_;
  LidarConfidenceOptions options_;
  std::vector<float> depth_;       // meters, NaN = invalid
  std::vector<uint8_t> valid_;
  std::vector<double> sanitized_;  // NaN -> 0
};

}  // namespace bs
