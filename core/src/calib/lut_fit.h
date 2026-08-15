#pragma once

#include <optional>
#include <vector>

#include "common/geometry.h"
#include "io/session_schema.h"

namespace bs {

// Fits COLMAP's OPENCV radial model (k1, k2; p1 = p2 = 0 — Apple's LUT is
// purely radial) to the per-session lens distortion lookup table delivered
// by AVCameraCalibrationData.
//
// Apple's tables map a radius fraction (0 at lensDistortionCenter, 1 at the
// far image corner, in the calibration reference pixel space) to a radial
// magnification factor:
//   - lensDistortionLookupTable        : distorted  -> undistorted
//   - inverseLensDistortionLookupTable : undistorted -> distorted
// The OPENCV model maps undistorted normalized coordinates to distorted
// ones, so the fit consumes the inverse table:
//   r_d = r_u * (1 + k1 r_u^2 + k2 r_u^4),  r in normalized units (px / f).

struct LutFitResult {
  double k1 = 0;
  double k2 = 0;
  // Worst absolute pixel error of the fitted model against the table,
  // evaluated across the full radius range at reference resolution.
  double max_residual_px = 0;
  double rms_residual_px = 0;
  // Set when the fitted model was worse than no correction at all and k1/k2
  // were forced to zero. The geometry is then uncorrected rather than
  // wrongly corrected, which is the safe direction.
  bool rejected = false;
};

// `intrinsics` must be in the same reference pixel space as the LUT
// (AVCameraCalibrationData reference dimensions). Empty/degenerate tables
// fit as k1 = k2 = 0 with zero residual (perfect pinhole — synthetic data).
std::optional<LutFitResult> FitOpencvModelFromLut(
    const DistortionLut& lut, const PinholeIntrinsics& intrinsics);

// Does (k1, k2) describe something a lens could do over this frame?
//
// True when the forward model is monotonic and positive across the whole
// radius range and never displaces a point by more than a quarter of it.
// Real phone lenses move corners by a few percent; a broken fit sends a
// 900 px radius to -112 px, and every undistorted point downstream of that
// is wrong in a way nothing else notices.
//
// Applied to fits AND to any model read off disk. A calibration.json is an
// input like any other — it may have been written by an older build whose
// fit was wrong, or by hand — so trusting a persisted model more than a
// freshly computed one has the check backwards.
bool IsPhysicalLensModel(double k1, double k2,
                         const PinholeIntrinsics& intrinsics);

// Applies the fitted forward model: undistorted normalized -> distorted
// normalized (used by tests and the point undistortion below).
Eigen::Vector2d DistortNormalized(const Eigen::Vector2d& xy_undistorted,
                                  double k1, double k2);

// Iteratively inverts the model: distorted pixel -> undistorted pixel
// (Newton fixed-point, converges in < 10 iterations for phone-lens k1/k2).
// Inverts the model in normalized coordinates: distorted -> undistorted.
Eigen::Vector2d UndistortNormalized(const Eigen::Vector2d& xy_distorted,
                                    double k1, double k2);

Eigen::Vector2d UndistortPixel(const Eigen::Vector2d& px_distorted,
                               const PinholeIntrinsics& intrinsics, double k1,
                               double k2);

}  // namespace bs
