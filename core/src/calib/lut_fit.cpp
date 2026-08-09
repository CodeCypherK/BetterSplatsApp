#include "calib/lut_fit.h"

#include <algorithm>
#include <cmath>

#include "common/log.h"

namespace bs {

namespace {

// Samples an Apple radial LUT at radius fraction rf in [0, 1] with linear
// interpolation between entries.
double SampleLut(const std::vector<float>& table, double rf) {
  if (table.empty()) return 1.0;
  if (table.size() == 1) return table[0];
  const double clamped = std::clamp(rf, 0.0, 1.0);
  const double pos = clamped * (table.size() - 1);
  const size_t i0 = static_cast<size_t>(pos);
  const size_t i1 = std::min(i0 + 1, table.size() - 1);
  const double frac = pos - i0;
  return table[i0] * (1.0 - frac) + table[i1] * frac;
}

}  // namespace

std::optional<LutFitResult> FitOpencvModelFromLut(
    const DistortionLut& lut, const PinholeIntrinsics& intrinsics) {
  if (intrinsics.fx <= 0 || intrinsics.ref_w <= 0 || intrinsics.ref_h <= 0) {
    return std::nullopt;
  }

  LutFitResult result;
  const bool have_inverse = !lut.inverse.empty();
  const bool have_forward = !lut.magnification.empty();
  if (!have_inverse && !have_forward) {
    return result;  // no distortion data: exact pinhole
  }

  // Max radius: distortion center to the farthest image corner, reference px.
  const double cx = lut.center_x > 0 ? lut.center_x : intrinsics.cx;
  const double cy = lut.center_y > 0 ? lut.center_y : intrinsics.cy;
  const double dx = std::max(cx, intrinsics.ref_w - cx);
  const double dy = std::max(cy, intrinsics.ref_h - cy);
  const double r_max_px = std::hypot(dx, dy);
  const double f = 0.5 * (intrinsics.fx + intrinsics.fy);

  // Sample (r_undistorted -> r_distorted) pairs.
  constexpr int kSamples = 64;
  std::vector<double> ru_norm, rd_norm;
  ru_norm.reserve(kSamples);
  rd_norm.reserve(kSamples);
  for (int i = 1; i <= kSamples; ++i) {
    const double rf = static_cast<double>(i) / kSamples;
    double ru_px, rd_px;
    if (have_inverse) {
      // inverse table: undistorted -> distorted, indexed by undistorted rf.
      ru_px = rf * r_max_px;
      rd_px = ru_px * SampleLut(lut.inverse, rf);
    } else {
      // forward table: distorted -> undistorted, indexed by distorted rf.
      rd_px = rf * r_max_px;
      ru_px = rd_px * SampleLut(lut.magnification, rf);
    }
    ru_norm.push_back(ru_px / f);
    rd_norm.push_back(rd_px / f);
  }

  // Linear least squares on rd = ru (1 + k1 ru^2 + k2 ru^4):
  //   (rd - ru) = k1 ru^3 + k2 ru^5.
  Eigen::MatrixXd A(kSamples, 2);
  Eigen::VectorXd b(kSamples);
  for (int i = 0; i < kSamples; ++i) {
    const double ru = ru_norm[i];
    A(i, 0) = ru * ru * ru;
    A(i, 1) = ru * ru * ru * ru * ru;
    b(i) = rd_norm[i] - ru;
  }
  const Eigen::Vector2d k = A.colPivHouseholderQr().solve(b);
  result.k1 = k(0);
  result.k2 = k(1);

  // Residuals in pixels at reference resolution.
  double sq_sum = 0;
  for (int i = 0; i < kSamples; ++i) {
    const double ru = ru_norm[i];
    const double rd_model = ru * (1.0 + result.k1 * ru * ru +
                                  result.k2 * ru * ru * ru * ru);
    const double err_px = std::abs(rd_model - rd_norm[i]) * f;
    result.max_residual_px = std::max(result.max_residual_px, err_px);
    sq_sum += err_px * err_px;
  }
  result.rms_residual_px = std::sqrt(sq_sum / kSamples);
  return result;
}

Eigen::Vector2d DistortNormalized(const Eigen::Vector2d& xy, double k1,
                                  double k2) {
  const double r2 = xy.squaredNorm();
  const double scale = 1.0 + k1 * r2 + k2 * r2 * r2;
  return xy * scale;
}

Eigen::Vector2d UndistortPixel(const Eigen::Vector2d& px,
                               const PinholeIntrinsics& intrinsics, double k1,
                               double k2) {
  const Eigen::Vector2d xy_d((px.x() - intrinsics.cx) / intrinsics.fx,
                             (px.y() - intrinsics.cy) / intrinsics.fy);
  // Fixed-point iteration: xy_u = xy_d / (1 + k1 r_u^2 + k2 r_u^4).
  Eigen::Vector2d xy_u = xy_d;
  for (int i = 0; i < 10; ++i) {
    const double r2 = xy_u.squaredNorm();
    const double scale = 1.0 + k1 * r2 + k2 * r2 * r2;
    if (std::abs(scale) < 1e-9) break;
    xy_u = xy_d / scale;
  }
  return {xy_u.x() * intrinsics.fx + intrinsics.cx,
          xy_u.y() * intrinsics.fy + intrinsics.cy};
}

}  // namespace bs
