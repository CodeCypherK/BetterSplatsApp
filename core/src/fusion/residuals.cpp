#include "fusion/residuals.h"

#include <cmath>

namespace bs {

std::optional<LidarAssociation> MakeLidarAssociation(
    const DepthFrame& frame, const SE3& pose, const Eigen::Vector3d& X,
    double w_tex, double gate_sigmas) {
  const Eigen::Vector3d x_cam = pose.Apply(X);
  if (x_cam.z() < 0.05) return std::nullopt;

  const Eigen::Vector2d px = frame.K().Project(x_cam);
  if (!frame.BicubicSafe(px.x(), px.y())) return std::nullopt;

  const auto measured = frame.DepthBilinear(px.x(), px.y());
  if (!measured) return std::nullopt;

  const int xi = static_cast<int>(std::lround(px.x()));
  const int yi = static_cast<int>(std::lround(px.y()));
  const double w_lidar = frame.ConfidenceAt(xi, yi);
  if (w_lidar <= 1e-4) return std::nullopt;

  const double sigma = frame.Sigma(*measured);
  // Association gate: reject when the candidate is not plausibly the
  // surface this depth pixel measured (occlusion / floater / mismatch).
  if (std::abs(x_cam.z() - *measured) > gate_sigmas * sigma) {
    return std::nullopt;
  }

  LidarAssociation assoc;
  assoc.sqrt_weight = std::sqrt(std::clamp(w_lidar * w_tex, 0.0, 1.0));
  assoc.sigma_m = sigma;
  return assoc;
}

}  // namespace bs
