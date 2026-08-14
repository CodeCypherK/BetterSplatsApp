#include "lidar/plane_fit.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Eigenvalues>

namespace bs {

DepthPlane FitDepthPlane(const DepthFrame& depth,
                         const DepthPlaneOptions& options) {
  DepthPlane out;

  // Gather usable samples. The confidence model already knows which pixels
  // are fabricated at depth discontinuities and which are too oblique or too
  // far to trust, so this is the same gate every other LiDAR consumer uses
  // rather than a second opinion about the sensor.
  std::vector<Eigen::Vector3d> samples;
  const int stride = std::max(1, options.sample_stride);
  samples.reserve(static_cast<size_t>(depth.width() / stride) *
                  static_cast<size_t>(depth.height() / stride));
  for (int y = 0; y < depth.height(); y += stride) {
    for (int x = 0; x < depth.width(); x += stride) {
      if (depth.ConfidenceAt(x, y) < options.min_confidence) continue;
      const auto point = depth.Unproject(x, y);
      if (!point) continue;
      if (point->z() < options.min_range_m || point->z() > options.max_range_m) {
        continue;
      }
      samples.push_back(*point);
    }
  }
  if (static_cast<int>(samples.size()) < options.min_inliers) return out;

  // Fixed seed: calibrating the same frame twice must give the same plane.
  std::mt19937 rng(0x8E110u);
  std::uniform_int_distribution<size_t> pick(0, samples.size() - 1);

  Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
  double best_offset = 0;
  int best_inliers = 0;
  for (int iter = 0; iter < options.ransac_iterations; ++iter) {
    const size_t a = pick(rng), b = pick(rng), c = pick(rng);
    if (a == b || b == c || a == c) continue;
    const Eigen::Vector3d n =
        (samples[b] - samples[a]).cross(samples[c] - samples[a]);
    if (n.norm() < 1e-9) continue;
    const Eigen::Vector3d normal = n.normalized();
    const double offset = -normal.dot(samples[a]);

    int inliers = 0;
    for (const auto& s : samples) {
      if (std::abs(normal.dot(s) + offset) < options.inlier_m) ++inliers;
    }
    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_normal = normal;
      best_offset = offset;
    }
  }

  const double frac =
      static_cast<double>(best_inliers) / static_cast<double>(samples.size());
  if (best_inliers < options.min_inliers || frac < options.min_inlier_frac) {
    // No single surface dominates: a cluttered corner, a doorway, a table
    // covered in objects. Refusing is the useful answer — the caller can ask
    // the user to aim at a clear patch of floor instead of quietly
    // calibrating against a chair.
    return out;
  }

  // Refit on the inliers so the plane is not hostage to three samples.
  std::vector<Eigen::Vector3d> inlier_points;
  inlier_points.reserve(best_inliers);
  for (const auto& s : samples) {
    if (std::abs(best_normal.dot(s) + best_offset) < options.inlier_m) {
      inlier_points.push_back(s);
    }
  }
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& p : inlier_points) centroid += p;
  centroid /= static_cast<double>(inlier_points.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto& p : inlier_points) {
    const Eigen::Vector3d d = p - centroid;
    cov += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if (solver.info() == Eigen::Success) {
    best_normal = solver.eigenvectors().col(0).normalized();
    best_offset = -best_normal.dot(centroid);
  }

  // Face the camera. The camera sits at the origin of this frame, so the
  // normal should point back toward it — that makes `offset` the camera's
  // distance above the surface, which is the number a calibration is for.
  if (best_offset < 0) {
    best_normal = -best_normal;
    best_offset = -best_offset;
  }

  double sq = 0;
  for (const auto& p : inlier_points) {
    const double d = best_normal.dot(p) + best_offset;
    sq += d * d;
  }

  out.valid = true;
  out.normal = best_normal;
  out.offset = best_offset;
  out.inliers = static_cast<int>(inlier_points.size());
  out.inlier_frac =
      static_cast<double>(inlier_points.size()) / static_cast<double>(samples.size());
  out.rmse_m = std::sqrt(sq / static_cast<double>(inlier_points.size()));
  // The optical axis is +Z in camera coordinates. A phone aimed straight at
  // the surface has the normal pointing back along it.
  out.incidence_deg = RadToDeg(std::acos(
      std::clamp(-best_normal.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0)));
  return out;
}

}  // namespace bs
