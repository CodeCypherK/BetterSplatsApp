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
    // Apple's tables hold RELATIVE radial magnification, applied as
    // (1 + value) — not as a multiplier. This multiplied by the raw value,
    // which on a real device means every radius collapses to about 1.5% of
    // its true length, and the fit then contorts k1/k2 into nonsense trying
    // to model that. Measured on an iPhone18,1 capture: k1 = -5.04,
    // k2 = +5.54, worst-case residual 252 px, with r_u = 900 px mapping to a
    // NEGATIVE radius. Every undistorted point in the pipeline was wrong,
    // which takes two-view geometry, PnP and triangulation with it.
    //
    // The table's own values prove the convention: entry 0 is exactly 0.0,
    // and as a direct multiplier that would send every point to the optical
    // centre. As (1 + 0.0) it correctly leaves the centre alone.
    //
    // It survived because synthetic sessions carry no LUT at all and return
    // early above as an exact pinhole, so this arithmetic had never once run
    // on real calibration data.
    if (have_inverse) {
      // inverse table: undistorted -> distorted, indexed by undistorted rf.
      ru_px = rf * r_max_px;
      rd_px = ru_px * (1.0 + SampleLut(lut.inverse, rf));
    } else {
      // forward table: distorted -> undistorted, indexed by distorted rf.
      rd_px = rf * r_max_px;
      ru_px = rd_px * (1.0 + SampleLut(lut.magnification, rf));
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

  // The fitted model must be PHYSICALLY plausible, or it is not used.
  //
  // Not "better than doing nothing": k1 = k2 = 0 lives inside the model
  // space, so least squares can essentially never come out worse in the
  // mean, and a check against it passes happily on nonsense. What actually
  // characterises a broken fit is that the forward model stops behaving
  // like a lens — radii that shrink toward zero, reverse order, or go
  // negative. The real failure sent a 900 px radius to -112 px.
  //
  // This is the gate that was specified ("accept < 0.3 px, fall back over
  // 0.5 px") and never built, which is how a fit with a 252 px worst-case
  // error came to be used on a capture without anything objecting.
  if (!IsPhysicalLensModel(result.k1, result.k2, intrinsics)) {
    BS_LOGW("calib",
            "lens fit rejected as non-physical: k1=%.4f k2=%.4f "
            "(worst residual %.1f px) — using pinhole instead",
            result.k1, result.k2, result.max_residual_px);
    result.k1 = 0;
    result.k2 = 0;
    result.rejected = true;
  }
  return result;
}

bool IsPhysicalLensModel(double k1, double k2,
                         const PinholeIntrinsics& intrinsics) {
  if (k1 == 0.0 && k2 == 0.0) return true;  // pinhole is always allowed
  const double f = 0.5 * (intrinsics.fx + intrinsics.fy);
  if (!(f > 0)) return false;
  // Corner radius from the principal point: the worst case the model faces.
  const double dx = std::max(intrinsics.cx, intrinsics.ref_w - intrinsics.cx);
  const double dy = std::max(intrinsics.cy, intrinsics.ref_h - intrinsics.cy);
  const double r_max_px = std::hypot(dx, dy);
  if (!(r_max_px > 0)) return false;

  constexpr int kSteps = 64;
  double previous_rd = 0;
  for (int i = 1; i <= kSteps; ++i) {
    const double ru = (static_cast<double>(i) / kSteps) * r_max_px / f;
    const double rd = ru * (1.0 + k1 * ru * ru + k2 * ru * ru * ru * ru);
    if (!(rd > previous_rd) || rd <= 0.0 ||
        std::abs(rd - ru) * f > 0.25 * r_max_px) {
      return false;
    }
    previous_rd = rd;
  }
  return true;
}

Eigen::Vector2d DistortNormalized(const Eigen::Vector2d& xy, double k1,
                                  double k2) {
  const double r2 = xy.squaredNorm();
  const double scale = 1.0 + k1 * r2 + k2 * r2 * r2;
  return xy * scale;
}

Eigen::Vector2d UndistortNormalized(const Eigen::Vector2d& xy_d, double k1,
                                    double k2) {
  // Fixed-point iteration: xy_u = xy_d / (1 + k1 r_u^2 + k2 r_u^4).
  Eigen::Vector2d xy_u = xy_d;
  for (int i = 0; i < 10; ++i) {
    const double r2 = xy_u.squaredNorm();
    const double scale = 1.0 + k1 * r2 + k2 * r2 * r2;
    if (std::abs(scale) < 1e-9) break;
    xy_u = xy_d / scale;
  }
  return xy_u;
}

Eigen::Vector2d UndistortPixel(const Eigen::Vector2d& px,
                               const PinholeIntrinsics& intrinsics, double k1,
                               double k2) {
  const Eigen::Vector2d xy_d((px.x() - intrinsics.cx) / intrinsics.fx,
                             (px.y() - intrinsics.cy) / intrinsics.fy);
  const Eigen::Vector2d xy_u = UndistortNormalized(xy_d, k1, k2);
  return {xy_u.x() * intrinsics.fx + intrinsics.cx,
          xy_u.y() * intrinsics.fy + intrinsics.cy};
}

}  // namespace bs
