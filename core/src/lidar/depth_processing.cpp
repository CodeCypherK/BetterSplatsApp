#include "lidar/depth_processing.h"

#include <cmath>

namespace bs {

DepthFrame::DepthFrame(const DepthImage& depth, const Intrinsics& K,
                       const LidarConfidenceOptions& options)
    : width_(depth.width), height_(depth.height), K_(K), options_(options) {
  const size_t n = static_cast<size_t>(width_) * height_;
  depth_.resize(n);
  valid_.resize(n);
  sanitized_.resize(n);
  const std::vector<float> meters = depth.ToFloat();
  for (size_t i = 0; i < n; ++i) {
    const float m = meters[i];
    const bool ok = std::isfinite(m) && m > 0.0f;
    depth_[i] = ok ? m : std::numeric_limits<float>::quiet_NaN();
    valid_[i] = ok ? 1 : 0;
    sanitized_[i] = ok ? static_cast<double>(m) : 0.0;
  }
}

bool DepthFrame::Valid(int x, int y) const {
  if (x < 0 || y < 0 || x >= width_ || y >= height_) return false;
  return valid_[static_cast<size_t>(y) * width_ + x] != 0;
}

float DepthFrame::DepthAt(int x, int y) const {
  if (x < 0 || y < 0 || x >= width_ || y >= height_) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return depth_[static_cast<size_t>(y) * width_ + x];
}

std::optional<double> DepthFrame::DepthBilinear(double x, double y) const {
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  if (x0 < 0 || y0 < 0 || x0 + 1 >= width_ || y0 + 1 >= height_) {
    return std::nullopt;
  }
  if (!Valid(x0, y0) || !Valid(x0 + 1, y0) || !Valid(x0, y0 + 1) ||
      !Valid(x0 + 1, y0 + 1)) {
    return std::nullopt;
  }
  const double fx = x - x0;
  const double fy = y - y0;
  const double d00 = DepthAt(x0, y0);
  const double d10 = DepthAt(x0 + 1, y0);
  const double d01 = DepthAt(x0, y0 + 1);
  const double d11 = DepthAt(x0 + 1, y0 + 1);
  return (d00 * (1 - fx) + d10 * fx) * (1 - fy) +
         (d01 * (1 - fx) + d11 * fx) * fy;
}

std::optional<Eigen::Vector3d> DepthFrame::Unproject(int x, int y) const {
  if (!Valid(x, y)) return std::nullopt;
  return K_.Unproject(x, y, DepthAt(x, y));
}

std::optional<Eigen::Vector3d> DepthFrame::NormalAt(int x, int y) const {
  // Prefer a +-2 px stencil: float16 depth quantization (~1.5 mm at 3 m) is
  // comparable to 1 px depth differences on gentle surfaces, and the wider
  // baseline halves the resulting normal noise. Fall back to +-1 px near
  // holes and borders.
  auto diff = [&](int step) -> std::optional<std::pair<Eigen::Vector3d,
                                                       Eigen::Vector3d>> {
    const auto left = Unproject(x - step, y);
    const auto right = Unproject(x + step, y);
    const auto up = Unproject(x, y - step);
    const auto down = Unproject(x, y + step);
    if (!left || !right || !up || !down) return std::nullopt;
    return std::make_pair(*right - *left, *down - *up);
  };

  auto d = diff(2);
  if (!d) d = diff(1);
  if (!d) return std::nullopt;
  const Eigen::Vector3d& dx = d->first;
  const Eigen::Vector3d& dy = d->second;
  Eigen::Vector3d n = dx.cross(dy);
  const double norm = n.norm();
  if (norm < 1e-12) return std::nullopt;
  n /= norm;
  // Orient toward the camera (surface faces the sensor).
  if (n.z() > 0) n = -n;
  return n;
}

double DepthFrame::Sigma(double depth_m) const {
  return options_.sigma_base_m + options_.sigma_quadratic * depth_m * depth_m;
}

double DepthFrame::ConfidenceAt(int x, int y) const {
  if (!Valid(x, y)) return 0.0;
  const double d = DepthAt(x, y);

  // Range weight.
  double w_range;
  if (d < options_.range_min_m) {
    w_range = 0.0;
  } else if (d <= options_.range_full_m) {
    w_range = 1.0;
  } else if (d >= options_.range_zero_m) {
    w_range = 0.0;
  } else {
    w_range = (options_.range_zero_m - d) /
              (options_.range_zero_m - options_.range_full_m);
  }
  if (w_range <= 0.0) return 0.0;

  // Edge weight from the local depth gradient (relative to depth).
  double w_edge = 0.0;
  if (Valid(x - 1, y) && Valid(x + 1, y) && Valid(x, y - 1) && Valid(x, y + 1)) {
    const double gx = 0.5 * (DepthAt(x + 1, y) - DepthAt(x - 1, y));
    const double gy = 0.5 * (DepthAt(x, y + 1) - DepthAt(x, y - 1));
    const double grad = std::hypot(gx, gy);
    const double rel = grad / (options_.edge_rel * d);
    w_edge = std::exp(-rel * rel);
  }
  if (w_edge <= 0.0) return 0.0;

  // Incidence weight from the local normal.
  double w_angle = 0.0;
  if (const auto n = NormalAt(x, y)) {
    const Eigen::Vector3d ray = K_.Unproject(x, y, 1.0).normalized();
    const double cos_inc = std::abs(n->dot(ray));
    const double cos_max = std::cos(DegToRad(options_.max_incidence_deg));
    w_angle = std::clamp(cos_inc / cos_max, 0.0, 1.0);
    // Full weight only when clearly under the incidence limit.
    if (cos_inc >= cos_max) w_angle = 1.0;
  }

  return w_edge * w_range * w_angle;
}

bool DepthFrame::BicubicSafe(double x, double y) const {
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  if (x0 - 1 < 0 || y0 - 1 < 0 || x0 + 2 >= width_ || y0 + 2 >= height_) {
    return false;
  }
  for (int dy = -1; dy <= 2; ++dy) {
    for (int dx = -1; dx <= 2; ++dx) {
      if (!Valid(x0 + dx, y0 + dy)) return false;
    }
  }
  return true;
}

}  // namespace bs
